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
// Dav1dReconReader.hpp
//
#pragma once

#include "Image.hpp"

#include <cstdint>
#include <deque>
#include <memory>

struct Dav1dContext;

namespace lctm {

// Reconstructs the base (AV1) stream with libdav1d in-process and returns each decoded
// reconstruction as an Image. Used by the single-pass streaming path where the base stream
// only exists in transit and cannot be handed to the dav1d command line (unseekable input).
//
class Dav1dReconReader {
public:
	Dav1dReconReader(const ImageDescription &description);
	~Dav1dReconReader();

	Dav1dReconReader(const Dav1dReconReader &) = delete;
	Dav1dReconReader &operator=(const Dav1dReconReader &) = delete;

	// Feed any number of bytes of the base bitstream (partial chunks are buffered). This may
	// be called incrementally as access units arrive.
	void feed(const uint8_t *data, size_t size);

	// Return the next decoded picture as an Image matching the reconstruction description,
	// blocking until it is available.
	Image read(unsigned position, uint64_t timestamp = 0);

private:
	void drain();

	Dav1dContext *context_ = nullptr;
	ImageDescription description_;
	std::deque<Image> queue_;
	unsigned count_ = 0;
};

} // namespace lctm