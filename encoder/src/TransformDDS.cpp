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
//               Stefano Battista (bautz66@gmail.com)
//
// TransformDDS.cpp
//

#include <cstdint>
#include <algorithm>
#include <thread>
#include <vector>
#include "TransformDDS.hpp"
#include "Config.hpp"

namespace lctm {

void TransformDDS::process(const Surface &residuals, EncodingMode mode, Surface layers[]) {

	const LayerEncodeFlags encode_flags(TransformType_DDS, mode);
#if defined __OPT_MODULO__
	CHECK((residuals.width() & 0x03) == 0);
	CHECK((residuals.height() & 0x03) == 0);
#else
	CHECK((residuals.width() % 4) == 0);
	CHECK((residuals.height() % 4) == 0);
#endif
#if defined __OPT_DIVISION__
	unsigned width = residuals.width() >> 2;
	unsigned height = residuals.height() >> 2;
#else
	unsigned width = residuals.width() / 4;
	unsigned height = residuals.height() / 4;
#endif

	SurfaceView<int16_t> src(residuals);

	static const int32_t basis[16][16] = {
	    {+1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1}, // 0,0
	    {+1, +1, -1, -1, +1, +1, -1, -1, +1, +1, -1, -1, +1, +1, -1, -1}, // 1,0
	    {+1, +1, +1, +1, +1, +1, +1, +1, -1, -1, -1, -1, -1, -1, -1, -1}, // 2,0
	    {+1, +1, -1, -1, +1, +1, -1, -1, -1, -1, +1, +1, -1, -1, +1, +1}, // 3,0

	    {+1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1}, // 0,1
	    {+1, -1, -1, +1, +1, -1, -1, +1, +1, -1, -1, +1, +1, -1, -1, +1}, // 1,1
	    {+1, -1, +1, -1, +1, -1, +1, -1, -1, +1, -1, +1, -1, +1, -1, +1}, // 2,1
	    {+1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1}, // 3.1

	    {+1, +1, +1, +1, -1, -1, -1, -1, +1, +1, +1, +1, -1, -1, -1, -1}, // 0,2
	    {+1, +1, -1, -1, -1, -1, +1, +1, +1, +1, -1, -1, -1, -1, +1, +1}, // 1,2
	    {+1, +1, +1, +1, -1, -1, -1, -1, -1, -1, -1, -1, +1, +1, +1, +1}, // 2,2
	    {+1, +1, -1, -1, -1, -1, +1, +1, -1, -1, +1, +1, +1, +1, -1, -1}, // 3,2

	    {+1, -1, +1, -1, -1, +1, -1, +1, +1, -1, +1, -1, -1, +1, -1, +1}, // 0,3
	    {+1, -1, -1, +1, -1, +1, +1, -1, +1, -1, -1, +1, -1, +1, +1, -1}, // 1,3
	    {+1, -1, +1, -1, -1, +1, -1, +1, -1, +1, -1, +1, +1, -1, +1, -1}, // 2,3
	    {+1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1}, // 3,3
	};

#if defined __OPT_MATRIX__

	// Process each layer in parallel - every layer only reads the source surface and writes
	// its own output layer, so the 16 layer transforms are independent
	{
		std::vector<std::thread> threads;
		const unsigned n_layers = 16;
		const unsigned n_threads = std::min(n_layers, std::max(1u, std::thread::hardware_concurrency()));
		auto transform_range = [&](unsigned begin, unsigned end) {
			const int16_t *srcp = src.data();
			const ptrdiff_t src_stride = src.stride() / sizeof(int16_t); // stride is in bytes
			for (unsigned l = begin; l < end; ++l) {
				if (encode_flags.encode_residual(l)) {
					const int32_t *b = basis[l];
					auto dest = Surface::build_from<int16_t>();
					dest.reserve(width, height);
					int16_t *dst = dest.data();
					for (unsigned y = 0; y < height; ++y) {
						const int16_t *r0 = srcp + (ptrdiff_t)(4 * y) * src_stride;
						const int16_t *r1 = r0 + src_stride;
						const int16_t *r2 = r1 + src_stride;
						const int16_t *r3 = r2 + src_stride;
						for (unsigned x = 0; x < width; ++x, dst += 1) {
							const int16_t *c0 = r0 + 4 * x;
							const int16_t *c1 = r1 + 4 * x;
							const int16_t *c2 = r2 + 4 * x;
							const int16_t *c3 = r3 + 4 * x;
							const int32_t coef =
							    (c0[0] * b[0] + c0[1] * b[1] + c0[2] * b[2] + c0[3] * b[3] + c1[0] * b[4] + c1[1] * b[5] +
							     c1[2] * b[6] + c1[3] * b[7] + c2[0] * b[8] + c2[1] * b[9] + c2[2] * b[10] + c2[3] * b[11] +
							     c3[0] * b[12] + c3[1] * b[13] + c3[2] * b[14] + c3[3] * b[15]) /
							    16;
							*dst = (int16_t)coef;
						}
					}
					layers[l] = dest.finish();
				} else {
					layers[l] = Surface::build_from<int16_t>()
					                .generate(width, height, [](unsigned x, unsigned y) -> int16_t { return 0; })
					                .finish();
				}
			}
		};
		for (unsigned t = 0; t < n_threads; ++t) {
			const unsigned begin = (n_layers * t) / n_threads;
			const unsigned end = (n_layers * (t + 1)) / n_threads;
			threads.emplace_back(transform_range, begin, end);
		}
		for (auto &t : threads)
			t.join();
	}

#else

	for (unsigned l = 0; l < 16; ++l) {
		if (encode_flags.encode_residual(l)) {
			layers[l] =
			    Surface::build_from<int16_t>()
			        .generate(
			            width, height,
			            [&](unsigned x, unsigned y) -> int16_t {
				            return (src.read(x * 4 + 0, y * 4 + 0) * basis[l][0] + src.read(x * 4 + 1, y * 4 + 0) * basis[l][1] +
				                    src.read(x * 4 + 2, y * 4 + 0) * basis[l][2] + src.read(x * 4 + 3, y * 4 + 0) * basis[l][3] +

				                    src.read(x * 4 + 0, y * 4 + 1) * basis[l][4] + src.read(x * 4 + 1, y * 4 + 1) * basis[l][5] +
				                    src.read(x * 4 + 2, y * 4 + 1) * basis[l][6] + src.read(x * 4 + 3, y * 4 + 1) * basis[l][7] +

				                    src.read(x * 4 + 0, y * 4 + 2) * basis[l][8] + src.read(x * 4 + 1, y * 4 + 2) * basis[l][9] +
				                    src.read(x * 4 + 2, y * 4 + 2) * basis[l][10] + src.read(x * 4 + 3, y * 4 + 2) * basis[l][11] +

				                    src.read(x * 4 + 0, y * 4 + 3) * basis[l][12] + src.read(x * 4 + 1, y * 4 + 3) * basis[l][13] +
				                    src.read(x * 4 + 2, y * 4 + 3) * basis[l][14] + src.read(x * 4 + 3, y * 4 + 3) * basis[l][15]) /
				                   16;
			            })
			        .finish();
		} else {
			layers[l] = Surface::build_from<int16_t>()
			                .generate(width, height, [](unsigned x, unsigned y) -> int16_t { return 0; })
			                .finish();
		}
	}

#endif
}

} // namespace lctm
