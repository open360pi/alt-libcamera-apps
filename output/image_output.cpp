/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */

#include "image_output.hpp"

ImageOutput::ImageOutput(VideoOptions const *options)
	: Output(options), fp_(nullptr), count_(0), file_start_time_ms_(0)
{
}

ImageOutput::~ImageOutput()
{
	closeFile();
}

void ImageOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	// We need to open a new file
	openFile(timestamp_us);

	if (options_->verbose)
		std::cerr << "ImageOutput: output buffer " << mem << " size " << size << "\n";
	if (fp_ && size)
	{
		if (fwrite(mem, size, 1, fp_) != 1)
			throw std::runtime_error("failed to write output bytes");
		if (options_->flush)
			fflush(fp_);
	}

	closeFile();
}

void ImageOutput::openFile(int64_t timestamp_us)
{
	if (options_->output == "-")
		fp_ = stdout;
	else if (!options_->output.empty())
	{
		// Generate the next output file name.
		char filename[256];
		int n = snprintf(filename, sizeof(filename), options_->output.c_str() + 6, count_);
		count_++;
		if (options_->wrap)
			count_ = count_ % options_->wrap;
		if (n < 0)
			throw std::runtime_error("failed to generate filename");

		fp_ = fopen(filename, "w");
		if (!fp_)
			throw std::runtime_error("failed to open output file " + std::string(filename));
		if (options_->verbose)
			std::cerr << "ImageOutput: opened output file " << filename << std::endl;

		file_start_time_ms_ = timestamp_us / 1000;
	}
}

void ImageOutput::closeFile()
{
	if (fp_ && fp_ != stdout)
		fclose(fp_);
	fp_ = nullptr;
}
