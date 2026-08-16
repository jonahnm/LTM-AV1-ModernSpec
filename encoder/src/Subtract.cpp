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
// Contributors: Sam Littlewood (sam.littlewood@v-nova.com)
//
// Subtract.cpp
//

#include <cstdint>
#include "Subtract.hpp"

namespace lctm {

// Generate a new plane as difference of two 16 bit planes
//
Surface Subtract::process(const Surface &plane_a, const Surface &plane_b) {
	const auto a = plane_a.view_as<int16_t>();
	const auto b = plane_b.view_as<int16_t>();

	CHECK(a.width() == b.width() && a.height() == b.height());

	const unsigned width = a.width();
	const unsigned height = a.height();
	const ptrdiff_t a_stride = a.stride() / sizeof(int16_t);
	const ptrdiff_t b_stride = b.stride() / sizeof(int16_t);
	const int16_t *ap = a.data();
	const int16_t *bp = b.data();

	auto dest = Surface::build_from<int16_t>();
	dest.reserve(width, height);
	int16_t *dst = dest.data();
	const ptrdiff_t dst_stride = dest.stride() / sizeof(int16_t);

	for (unsigned y = 0; y < height; ++y) {
		const int16_t *ar = ap + (ptrdiff_t)y * a_stride;
		const int16_t *br = bp + (ptrdiff_t)y * b_stride;
		int16_t *dr = dst + (ptrdiff_t)y * dst_stride;
		for (unsigned x = 0; x < width; ++x)
			dr[x] = (int16_t)(ar[x] - br[x]);
	}

	return dest.finish();
}
} // namespace lctm
