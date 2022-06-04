/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * frame_info.hpp - Frame info class for libcamera apps
 */
#include <array>
#include <iomanip>
#include <sstream>
#include <string>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

struct FrameInfo
{
	FrameInfo(libcamera::ControlList &ctrls)
		: color_temperature(0), frame_duration(0),
		  exposure_time(0.0), digital_gain(0.0),
		  colour_gains({ { 0.0f, 0.0f } }), focus(0.0), lux(0.0), aelock(false)
	{
		if (ctrls.contains(libcamera::controls::ColourTemperature))
			color_temperature = ctrls.get<int32_t>(libcamera::controls::ColourTemperature);

		if (ctrls.contains(libcamera::controls::ExposureTime))
			exposure_time = ctrls.get<int32_t>(libcamera::controls::ExposureTime);

		if (ctrls.contains(libcamera::controls::FrameDuration))
			frame_duration = ctrls.get<int64_t>(libcamera::controls::FrameDuration);

		if (ctrls.contains(libcamera::controls::AnalogueGain))
			analogue_gain = ctrls.get(libcamera::controls::AnalogueGain);

		if (ctrls.contains(libcamera::controls::DigitalGain))
			digital_gain = ctrls.get(libcamera::controls::DigitalGain);

		if (ctrls.contains(libcamera::controls::ColourGains))
		{
			libcamera::Span<const float> gains = ctrls.get(libcamera::controls::ColourGains);
			colour_gains[0] = gains[0], colour_gains[1] = gains[1];
		}

		if (ctrls.contains(libcamera::controls::FocusFoM))
			focus = ctrls.get(libcamera::controls::FocusFoM);

		if (ctrls.contains(libcamera::controls::Lux))
			lux = ctrls.get(libcamera::controls::Lux);

		if (ctrls.contains(libcamera::controls::AeLocked))
			aelock = ctrls.get(libcamera::controls::AeLocked);
	}

	std::string ToString(std::string &info_string) const
	{
		std::string parsed(info_string);

		for (auto const &t : tokens)
		{
			std::size_t pos = parsed.find(t);
			if (pos != std::string::npos)
			{
				std::stringstream value;
				value << std::fixed << std::setprecision(2);

				if (t == "%frame")
					value << sequence;
				else if (t == "%fps")
					value << fps;
				else if (t == "%exp")
					value << exposure_time;
				else if (t == "%temp")
					value << color_temperature;
				else if (t == "%fd")
					value << frame_duration;
				else if (t == "%lux")
					value << lux;
				else if (t == "%ag")
					value << analogue_gain;
				else if (t == "%dg")
					value << digital_gain;
				else if (t == "%rg")
					value << colour_gains[0];
				else if (t == "%bg")
					value << colour_gains[1];
				else if (t == "%focus")
					value << focus;
				else if (t == "%aelock")
					value << aelock;

				parsed.replace(pos, t.length(), value.str());
			}
		}

		return parsed;
	}

	unsigned int sequence;
	unsigned int color_temperature;
	unsigned long frame_duration;
	float exposure_time;
	float analogue_gain;
	float digital_gain;
	std::array<float, 2> colour_gains;
	float focus;
	float fps;
	float lux;
	bool aelock;

private:
	// Info text tokens.
	inline static const std::string tokens[] = { "%frame", "%fps", "%exp",	 "%ag",	   "%dg",
												 "%rg",	   "%bg",  "%focus", "%aelock", "%temp",
												 "%fd",    "%lux" };
};
