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

#pragma once

#include "uBaseDecoder.h"

namespace vnova {
namespace utility {

class BaseDecoderAV1 : public BaseDecoder {
public:
	BaseDecoderAV1();

	bool ParseNalUnit(const uint8_t *nal, uint32_t nalLength) override;
	BaseDecPictType::Enum GetBasePictureType() const override;
	BaseDecNalUnitType::Enum GetBaseNalUnitType() const override;
	int32_t GetQP() const override;
	uint32_t GetNalType() const override;
	int64_t GetPictureOrderCount() const override;
	uint32_t GetPictureWidth() const override;
	uint32_t GetPictureHeight() const override;
	bool GetDPBCanRefresh() const override;
	uint8_t GetMaxNumberOfReorderFrames() const override;
	uint32_t GetFrameRate() const override;
	uint32_t GetBitDepthLuma() const override;
	uint32_t GetBitDepthChroma() const override;
	uint32_t GetChromaFormatIDC() const override;
	uint32_t GetTemporalId() const override;
	NALDelimitier Delimiter() const override;
	int64_t GetPictureOrderCountIncrement() const override;

	// AV1 OBU types, as per AV1 spec section 5.1.1
	enum OBUType {
		OBU_SEQUENCE_HEADER = 1,
		OBU_TEMPORAL_DELIMITER,
		OBU_FRAME_HEADER,
		OBU_TILE_GROUP,
		OBU_METADATA,
		OBU_FRAME,
		OBU_REDUNDANT_FRAME_HEADER,
		OBU_TILE_LIST,
		OBU_PADDING = 15,
	};

	// Metadata OBU types
	enum MetadataType {
		METADATA_TYPE_ITUT_T35 = 1,
	};

	// Frame types, as per AV1 spec section 6.8.2
	enum FrameType { FRAME_KEY = 0, FRAME_INTER, FRAME_INTRA_ONLY, FRAME_SWITCH };

private:
	void ParseSequenceHeader(const uint8_t *payload, uint32_t size);
	void ParseFrameHeader(const uint8_t *payload, uint32_t size);

	uint32_t m_nal_type;
	BaseDecNalUnitType::Enum m_nal_type_base;
	BaseDecPictType::Enum m_picture_type;

	int64_t m_current_picture_order_count;
	int64_t m_picture_order_count_increment;

	uint32_t m_width;
	uint32_t m_height;
	uint32_t m_bit_depth;
	uint32_t m_chroma_format_idc;
};

} // namespace utility
} // namespace vnova
