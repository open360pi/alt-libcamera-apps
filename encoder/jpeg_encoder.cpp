/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>

#include <jpeglib.h>
#include <libexif/exif-data.h>

#include "jpeg_encoder.hpp"

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif

static const ExifByteOrder exif_byte_order = EXIF_BYTE_ORDER_INTEL;
static const unsigned int exif_image_offset = 20; // offset of image in JPEG buffer
static const unsigned char exif_header[] = { 0xff, 0xd8, 0xff, 0xe1 };

JpegEncoder::JpegEncoder(VideoOptions const *options)
	: Encoder(options), abortEncode_(false), abortOutput_(false), index_(0)
{
	output_thread_ = std::thread(&JpegEncoder::outputThread, this);
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i] = std::thread(std::bind(&JpegEncoder::encodeThread, this, i));
	if (options_->verbose)
		std::cerr << "Opened JpegEncoder" << std::endl;
}

JpegEncoder::~JpegEncoder()
{
	abortEncode_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i].join();
	abortOutput_ = true;
	output_thread_.join();
	if (options_->verbose)
		std::cerr << "JpegEncoder closed" << std::endl;
}

void JpegEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, libcamera::ControlList const &metadata, int64_t timestamp_us)
{
	std::lock_guard<std::mutex> lock(encode_mutex_);
	EncodeItem item = { mem, info, metadata, timestamp_us, index_++ };
	encode_queue_.push(item);
	encode_cond_var_.notify_all();
}

void JpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item, uint8_t *&encoded_buffer,
							  size_t &buffer_len)
{
	// Copied from YUV420_to_JPEG_fast in jpeg.cpp.
	cinfo.image_width = item.info.width;
	cinfo.image_height = item.info.height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

	jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = TRUE;
	jpeg_set_quality(&cinfo, options_->quality, TRUE);
	encoded_buffer = nullptr;
	buffer_len = 0;
	jpeg_mem_len_t jpeg_mem_len;
	jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
	jpeg_start_compress(&cinfo, TRUE);

	int stride2 = item.info.stride / 2;
	uint8_t *Y = (uint8_t *)item.mem;
	uint8_t *U = (uint8_t *)Y + item.info.stride * item.info.height;
	uint8_t *V = (uint8_t *)U + stride2 * (item.info.height / 2);
	uint8_t *Y_max = U - item.info.stride;
	uint8_t *U_max = V - stride2;
	uint8_t *V_max = U_max + stride2 * (item.info.height / 2);

	JSAMPROW y_rows[16];
	JSAMPROW u_rows[8];
	JSAMPROW v_rows[8];

	for (uint8_t *Y_row = Y, *U_row = U, *V_row = V; cinfo.next_scanline < item.info.height;)
	{
		for (int i = 0; i < 16; i++, Y_row += item.info.stride)
			y_rows[i] = std::min(Y_row, Y_max);
		for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
			u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}

	jpeg_finish_compress(&cinfo);
	buffer_len = jpeg_mem_len;
}

ExifEntry *exif_create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
	ExifEntry *entry = exif_content_get_entry(exif->ifd[ifd], tag);
	if (entry)
		return entry;
	entry = exif_entry_new();
	if (!entry)
		throw std::runtime_error("failed to allocate EXIF entry");
	entry->tag = tag;
	exif_content_add_entry(exif->ifd[ifd], entry);
	exif_entry_initialize(entry, entry->tag);
	exif_entry_unref(entry);
	return entry;
}

void exif_set_string(ExifEntry *entry, char const *s)
{
	if (entry->data)
		free(entry->data);
	entry->size = entry->components = strlen(s);
	entry->data = (unsigned char *)strdup(s);
	if (!entry->data)
		throw std::runtime_error("failed to copy exif string");
	entry->format = EXIF_FORMAT_ASCII;
}

static void create_exif_data(uint8_t * const &mem,
							 StreamInfo const &info, libcamera::ControlList const &metadata, std::string const &cam_name,
							 VideoOptions const *options, uint8_t *&exif_buffer, unsigned int &exif_len,
							 uint8_t *&thumb_buffer, jpeg_mem_len_t &thumb_len)
{
	exif_buffer = nullptr;
	ExifData *exif = nullptr;

	try
	{
		exif = exif_data_new();
		if (!exif)
			throw std::runtime_error("failed to allocate EXIF data");
		exif_data_set_byte_order(exif, exif_byte_order);

		// First add some fixed EXIF tags.

		ExifEntry *entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_MAKE);
		exif_set_string(entry, "Raspberry Pi");
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_MODEL);
		exif_set_string(entry, cam_name.c_str());
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_SOFTWARE);
		exif_set_string(entry, "libcamera-still");
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME);
		std::time_t raw_time;
		std::time(&raw_time);
		std::tm *time_info;
		char time_string[32];
		time_info = std::localtime(&raw_time);
		std::strftime(time_string, sizeof(time_string), "%Y:%m:%d %H:%M:%S", time_info);
		exif_set_string(entry, time_string);

		// Now add some tags filled in from the image metadata.
		if (metadata.contains(libcamera::controls::ExposureTime))
		{
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_EXPOSURE_TIME);
			int32_t exposure_time = metadata.get(libcamera::controls::ExposureTime);
			if (options->verbose)
				std::cerr << "Exposure time: " << exposure_time << std::endl;
			ExifRational exposure = { (ExifLong)exposure_time, 1000000 };
			exif_set_rational(entry->data, exif_byte_order, exposure);
		}
		if (metadata.contains(libcamera::controls::AnalogueGain))
		{
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_ISO_SPEED_RATINGS);
			float ag = metadata.get(libcamera::controls::AnalogueGain), dg = 1.0, gain;
			if (metadata.contains(libcamera::controls::DigitalGain))
				dg = metadata.get(libcamera::controls::DigitalGain);
			gain = ag * dg;
			if (options->verbose)
				std::cerr << "Ag " << ag << " Dg " << dg << " Total " << gain << std::endl;
			exif_set_short(entry->data, exif_byte_order, 100 * gain);
		}

		// Command-line supplied tags.
		/*for (auto &exif_item : options->exif)
		{
			if (options->verbose)
				std::cerr << "Processing EXIF item: " << exif_item << std::endl;
			exif_read_tag(exif, exif_item.c_str());
		}*/

		/*if (options->thumb_quality)
		{
			// Add some tags for the thumbnail. We put in dummy values for the thumbnail
			// offset/length to occupy the right amount of space, and fill them in later.

			if (options->verbose)
				std::cerr << "Thumbnail dimensions are " << options->thumb_width << " x " << options->thumb_height
						  << std::endl;
			entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_IMAGE_WIDTH);
			exif_set_short(entry->data, exif_byte_order, options->thumb_width);
			entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_IMAGE_LENGTH);
			exif_set_short(entry->data, exif_byte_order, options->thumb_height);
			entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_COMPRESSION);
			exif_set_short(entry->data, exif_byte_order, 6);
			ExifEntry *thumb_offset_entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_JPEG_INTERCHANGE_FORMAT);
			exif_set_long(thumb_offset_entry->data, exif_byte_order, 0);
			ExifEntry *thumb_length_entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_JPEG_INTERCHANGE_FORMAT_LENGTH);
			exif_set_long(thumb_length_entry->data, exif_byte_order, 0);

			// We actually have to write out an EXIF buffer to find out how long it is.

			exif_len = 0;
			exif_data_save_data(exif, &exif_buffer, &exif_len);
			free(exif_buffer);
			exif_buffer = nullptr;

			// Next create the JPEG for the thumbnail, we need to do this now so that we can
			// go back and fill in the correct values for the thumbnail offsets/length.

			int q = options->thumb_quality;
			for (; q > 0; q -= 5)
			{
				YUV_to_JPEG(mem, info, options->thumb_width,
							options->thumb_height, q, 0, thumb_buffer, thumb_len);
				if (thumb_len < 60000) // entire EXIF data must be < 65536, so this should be safe
					break;
				free(thumb_buffer);
				thumb_buffer = nullptr;
			}
			if (options->verbose)
				std::cerr << "Thumbnail size " << thumb_len << std::endl;
			if (q <= 0)
				throw std::runtime_error("failed to make acceptable thumbnail");

			// Now fill in the correct offsets and length.

			unsigned int offset = exif_len - 6; // do not ask me why "- 6", I have no idea
			exif_set_long(thumb_offset_entry->data, exif_byte_order, offset);
			exif_set_long(thumb_length_entry->data, exif_byte_order, thumb_len);
		}*/

		// And create the EXIF data buffer *again*.

		exif_data_save_data(exif, &exif_buffer, &exif_len);
		exif_data_unref(exif);
		exif = nullptr;
	}
	catch (std::exception const &e)
	{
		if (exif)
			exif_data_unref(exif);
		if (exif_buffer)
			free(exif_buffer);
		if (thumb_buffer)
			free(thumb_buffer);
		throw;
	}
}

void JpegEncoder::encodeThread(int num)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	std::chrono::duration<double> encode_time(0);
	uint32_t frames = 0;

	char *output_buf = nullptr;
	size_t output_len;

	EncodeItem encode_item;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(encode_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				if (abortEncode_ && encode_queue_.empty())
				{
					if (frames && options_->verbose)
						std::cerr << "Encode " << frames << " frames, average time "
								  << encode_time.count() * 1000 / frames << "ms" << std::endl;
					jpeg_destroy_compress(&cinfo);
					return;
				}
				if (!encode_queue_.empty())
				{
					encode_item = encode_queue_.front();
					encode_queue_.pop();
					break;
				}
				else
					encode_cond_var_.wait_for(lock, 200ms);
			}
		}

		// Encode the buffer.
		uint8_t *jpeg_buffer = nullptr;
		size_t jpeg_len = 0;
		auto start_time = std::chrono::high_resolution_clock::now();
		encodeJPEG(cinfo, encode_item, jpeg_buffer, jpeg_len);

		uint8_t *thumb_buffer = nullptr;
		unsigned char *exif_buffer = nullptr;

		jpeg_mem_len_t thumb_len = 0; // stays zero if no thumbnail
		unsigned int exif_len;
		create_exif_data((uint8_t *)(encode_item.mem), encode_item.info,
						 encode_item.metadata, "test camera name", options_, exif_buffer, exif_len,
						 thumb_buffer, thumb_len);

		encode_time += (std::chrono::high_resolution_clock::now() - start_time);
		frames++;

		FILE *fp;
		output_buf = nullptr;

		fp = open_memstream (&output_buf, &output_len);

		if (fwrite(exif_header, sizeof(exif_header), 1, fp) != 1 || fputc((exif_len + thumb_len + 2) >> 8, fp) == EOF ||
			fputc((exif_len + thumb_len + 2) & 0xff, fp) == EOF || fwrite(exif_buffer, exif_len, 1, fp) != 1 ||
			(thumb_len && fwrite(thumb_buffer, thumb_len, 1, fp) != 1) ||
			fwrite(jpeg_buffer + exif_image_offset, jpeg_len - exif_image_offset, 1, fp) != 1)
			throw std::runtime_error("failed to write file - output probably corrupt");

		fclose (fp);

		free(exif_buffer);
		exif_buffer = nullptr;
		free(thumb_buffer);
		thumb_buffer = nullptr;
		free(jpeg_buffer);
		jpeg_buffer = nullptr;

		// Don't return buffers until the output thread as that's where they're
		// in order again.

		// We push this encoded buffer to another thread so that our
		// application can take its time with the data without blocking the
		// encode process.
		OutputItem output_item = { output_buf, output_len, encode_item.timestamp_us, encode_item.index };
		std::lock_guard<std::mutex> lock(output_mutex_);
		output_queue_[num].push(output_item);
		output_cond_var_.notify_one();
	}
}

void JpegEncoder::outputThread()
{
	OutputItem item;
	uint64_t index = 0;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(output_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				// We look for the thread that's completed the frame we want next.
				// If we don't find it, we wait.
				//
				// Must also check for an abort signal, and if set, all queues must
				// be empty. This is done first to ensure all frame callbacks have
				// had a chance to run.
				bool abort = abortOutput_ ? true : false;
				for (auto &q : output_queue_)
				{
					if (abort && !q.empty())
						abort = false;

					if (!q.empty() && q.front().index == index)
					{
						item = q.front();
						q.pop();
						goto got_item;
					}
				}
				if (abort)
					return;

				output_cond_var_.wait_for(lock, 200ms);
			}
		}
	got_item:
		input_done_callback_(nullptr);

		output_ready_callback_(item.mem, item.bytes_used, item.timestamp_us, true);
		free(item.mem);
		index++;
	}
}
