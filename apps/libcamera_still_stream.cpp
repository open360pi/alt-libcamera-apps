/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_vid.cpp - libcamera video record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <pigpio.h>

#include "core/frame_info.hpp"
#include "core/libcamera_encoder.hpp"
#include "output/output.hpp"

using namespace std::placeholders;

#define GPIO_TRIGGER_INDEX 20
pthread_mutex_t lock;
pthread_mutexattr_t attr;

void gpioMutexHandler(int gpio, int level, uint32_t tick)
{
   printf("Interrupt level %d at %u\n", level, tick);
   pthread_mutex_unlock(&lock);
}

static int gpio_received;
void gpioHandler(int gpio, int level, uint32_t tick)
{
   printf("Interrupt level %d at %u\n", level, tick);
   gpio_received = gpio;
}

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	std::cerr << "Received signal " << signal_number << std::endl;
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
	int key = 0;
	if (options->keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options->signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if (signal_received == SIGUSR2)
			key = 'x';
		signal_received = 0;
	}

	if (gpio_received)
	{
		key = '\n';
		gpio_received = 0;
	}
	return key;
}

static int get_colourspace_flags(std::string const &codec)
{
	if (codec == "jpeg" || codec == "mjpeg" || codec == "yuv420")
		return LibcameraEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
	else
		return LibcameraEncoder::FLAG_VIDEO_NONE;
}

// The main even loop for the application.

static void event_loop(LibcameraEncoder &app)
{
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->codec));
	app.StartEncoder();

	// Intended for GPIO firing of a capture
	if(options->gpio == 0 || options->gpio == 3) {
		pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL);
		pthread_mutex_init(&lock, &attr);
		gpioInitialise();
		pthread_mutex_lock(&lock);

		gpioSetMode(GPIO_TRIGGER_INDEX, PI_INPUT);
		gpioSetPullUpDown(GPIO_TRIGGER_INDEX, PI_PUD_UP);
		gpioSetISRFunc(GPIO_TRIGGER_INDEX, RISING_EDGE, 0, gpioMutexHandler);

		if (options->verbose)
			fprintf(stderr, "Waiting GPIO signal from master\n");
		pthread_mutex_lock(&lock);
		pthread_mutex_destroy(&lock);

		gpioSleep(PI_TIME_RELATIVE, 0, 100000);
	}
	else {
		gpioInitialise();
		if (options->verbose)
			fprintf(stderr, "Sending GPIO signal to slave\n");
		gpioSetMode(GPIO_TRIGGER_INDEX, PI_OUTPUT);
		gpioWrite(GPIO_TRIGGER_INDEX, 0);
		gpioSleep(PI_TIME_RELATIVE, 0, 10000);
		gpioWrite(GPIO_TRIGGER_INDEX, 1);
		gpioSleep(PI_TIME_RELATIVE, 0, 10000);
		gpioWrite(GPIO_TRIGGER_INDEX, 0);
		gpioSetMode(GPIO_TRIGGER_INDEX, PI_INPUT);
		gpioTerminate();
	}

	if (options->gpio == 3) {
		if (options->verbose)
			fprintf(stderr, "Sleeping a bit\n");
		sleep(0.030);
	}

	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	bool enabled = false;

	gpioInitialise();
	gpioSetMode(GPIO_TRIGGER_INDEX, PI_INPUT);
	gpioSetPullUpDown(GPIO_TRIGGER_INDEX, PI_PUD_UP);
	gpioSetISRFunc(GPIO_TRIGGER_INDEX, RISING_EDGE, 0, gpioHandler);

	for (unsigned int count = 0; ; count++)
	{
		LibcameraEncoder::Msg msg = app.Wait();
		if (msg.type == LibcameraEncoder::MsgType::Quit)
			return;
		else if (msg.type != LibcameraEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		int key = get_key_or_signal(options, p);
		if (key == '\n') {
			enabled = true;
		}

		if (options->verbose)
			std::cerr << "Viewfinder frame " << count << std::endl;
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->frames && options->timeout &&
					   (now - start_time > std::chrono::milliseconds(options->timeout));
		bool frameout = options->frames && count >= options->frames;
		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				std::cerr << "Halting: reached timeout of " << options->timeout << " milliseconds.\n";
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

		FrameInfo frame_info(completed_request->metadata);
		frame_info.fps = completed_request->framerate;
		frame_info.sequence = completed_request->sequence;
		std::string format = "FrameInfo frame=%frame fps=%fps exposure=%exp analog_gain=%ag "
		                     "digital_gain=%dg red_gain=%rg blue_gain=%bg focus=%focus "
		                     "aelock=%aelock colour_temp=%temp frame_duration=%fd lux=%lux";
		std::cerr << frame_info.ToString(format) << std::endl;
		if (enabled) {
			app.EncodeBuffer(completed_request, app.VideoStream());
			app.ShowPreview(completed_request, app.VideoStream());
			enabled = false;
			if (options->verbose)
				std::cerr << "Viewfinder frame " << count << std::endl;
		}
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraEncoder app;
		VideoOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
		return -1;
	}
	return 0;
}
