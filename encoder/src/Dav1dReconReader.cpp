// The copyright in this software is being made available under the BSD
// License, included below. This software may be subject to other third party
// and contributor rights, including patent rights, and no such rights are
// granted under this license.
//
// Copyright (c) 2022, ISO/IEC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  * Neither the name of the ISO/IEC nor the names of its contributors may
//    be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
//
// Dav1dReconReader.cpp
//
#include "Dav1dReconReader.hpp"

#include <dav1d/dav1d.h>

#include "Diagnostics.hpp"
#include "Misc.hpp"
#include "Surface.hpp"

#include <chrono>
#include <cstring>
#include <thread>

namespace lctm {

Dav1dReconReader::Dav1dReconReader(const ImageDescription &description) : description_(description) {
	Dav1dSettings settings;
	dav1d_default_settings(&settings);
	settings.n_threads = 0;       // automatic
	settings.max_frame_delay = 0; // automatic

	const int err = dav1d_open(&context_, &settings);
	if (err != 0)
		ERR("dav1d_open() failed (%d)", err);
}

Dav1dReconReader::~Dav1dReconReader() {
	if (context_ != nullptr) {
		dav1d_close(&context_);
		context_ = nullptr;
	}
}

void Dav1dReconReader::feed(const uint8_t *data, size_t size) {
	if (size == 0)
		return;

	Dav1dData dav1d_data;
	uint8_t *dest = dav1d_data_create(&dav1d_data, size);
	if (!dest)
		ERR("libdav1d could not allocate input buffer");
	std::memcpy(dest, data, size);

	// Keep resending until the chunk has been consumed, draining pictures to make room in the
	// decoder's frame queue when it reports EAGAIN
	int res = 0;
	while (dav1d_data.sz > 0) {
		res = dav1d_send_data(context_, &dav1d_data);
		if (res == DAV1D_ERR(EAGAIN))
			drain();
		else
			break;
	}
	if (res < 0 && res != DAV1D_ERR(EAGAIN))
		ERR("libdav1d send failed (%d)", res);

	drain();
}

void Dav1dReconReader::drain() {
	for (;;) {
		Dav1dPicture picture;
		const int res = dav1d_get_picture(context_, &picture);
		if (res == DAV1D_ERR(EAGAIN))
			return;
		if (res < 0)
			ERR("libdav1d decode failed (%d)", res);

		const Dav1dPictureParameters &p = picture.p;
		if (!p.w || !p.h)
			ERR("libdav1d produced an invalid picture");

		std::vector<Surface> surfaces;

		const unsigned bpp = description_.byte_depth(); // bytes per sample
		for (unsigned plane = 0; plane < description_.num_planes(); ++plane) {
			// luma uses stride[0], chroma uses stride[1]
			const ptrdiff_t src_stride = picture.stride[(plane == 0) ? 0 : 1];
			const ptrdiff_t dst_stride = description_.row_stride(plane);
			const unsigned width = description_.width(plane);
			const unsigned height = description_.height(plane);

			auto b = Surface::build_from<int8_t>();
			b.reserve_bpp(width, height, bpp, dst_stride);

			const uint8_t *src = static_cast<const uint8_t *>(picture.data[plane]);
			for (unsigned y = 0; y < height; ++y)
				std::memcpy(b.data(0, y), src + (ptrdiff_t)y * src_stride, (size_t)width * bpp);
			surfaces.push_back(b.finish());
		}

		dav1d_picture_unref(&picture);

		queue_.push_back(Image(format("recon:%u", count_++), description_, 0, surfaces));
	}
}

Image Dav1dReconReader::read(unsigned position, uint64_t timestamp) {
	(void)position;
	(void)timestamp;

	// Block until the next picture has been decoded. feed() already drained whatever was ready,
	// so this only waits for the decoder's frame threads to catch up.
	while (queue_.empty()) {
		drain();
		if (queue_.empty())
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	Image image = std::move(queue_.front());
	queue_.pop_front();
	return image;
}

} // namespace lctm