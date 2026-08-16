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

#include "uBaseDecoderAV1.h"

#include "Diagnostics.hpp"

namespace vnova {
namespace utility {

// Minimal AV1 OBU stream parser, used to determine access unit boundaries, picture types and
// picture dimensions of an AV1 base stream (low overhead OBU format, as produced by SVT-AV1
// with --obu output).

namespace {

// Bit-level reader over a byte buffer
class BitReader {
public:
	BitReader(const uint8_t *data, uint32_t size) : data_(data), size_(size) {}

	uint32_t read(unsigned num_bits) {
		uint32_t value = 0;
		for (unsigned i = 0; i < num_bits; ++i)
			value = (value << 1) | read_bit();
		return value;
	}

	bool read_flag() { return read_bit() != 0; }

	// Number of bits remaining
	uint32_t remaining() const { return size_ * 8 - bit_offset_; }

private:
	uint8_t read_bit() {
		CHECK(bit_offset_ < size_ * 8);
		const uint8_t byte = data_[bit_offset_ >> 3];
		const uint8_t bit = (byte >> (7 - (bit_offset_ & 7))) & 0x01;
		++bit_offset_;
		return bit;
	}

	const uint8_t *data_;
	uint32_t size_;
	uint32_t bit_offset_ = 0;
};

// Read a leb128 value from the payload - returns the number of bytes consumed, or 0 on error
uint32_t uReadLeb128(const uint8_t *data, uint32_t size, uint64_t &value) {
	uint64_t v = 0;
	uint32_t i = 0;
	do {
		if (i >= size)
			return 0;
		v |= (uint64_t)(data[i] & 0x7f) << (7 * i);
	} while (data[i++] & 0x80 && i < 8);

	value = v;
	return i;
}

} // namespace

BaseDecoderAV1::BaseDecoderAV1()
    : m_nal_type(0), m_nal_type_base(BaseDecNalUnitType::Unknown), m_picture_type(BaseDecPictType::Unknown),
      m_current_picture_order_count(0), m_picture_order_count_increment(1), m_width(0), m_height(0), m_bit_depth(0),
      m_chroma_format_idc(1) {}

// Parse an OBU - 'nal' points at the first OBU header byte (no start code, no length prefix)
//
bool BaseDecoderAV1::ParseNalUnit(const uint8_t *nal, uint32_t nalLength) {
	if (nalLength < 1)
		return false;

	// OBU header
	const uint8_t header = nal[0];
	m_nal_type = (header >> 3) & 0x0f;
	m_nal_type_base = BaseDecNalUnitType::Unknown;

	const bool has_extension = (header >> 2) & 0x01;
	const bool has_size = (header >> 1) & 0x01;
	const bool reserved = header & 0x01;
	if (reserved)
		return false;

	uint32_t offset = 1;
	if (has_extension)
		offset += 1;

	uint32_t payload_size = nalLength - offset;
	if (has_size) {
		uint64_t size = 0;
		const uint32_t consumed = uReadLeb128(nal + offset, nalLength - offset, size);
		if (consumed == 0)
			return false;
		offset += consumed;
		payload_size = nalLength - offset;
		if (size > payload_size)
			return false;
	}

	const uint8_t *payload = nal + offset;

	switch (m_nal_type) {
	case OBU_SEQUENCE_HEADER:
		ParseSequenceHeader(payload, payload_size);
		break;

	case OBU_FRAME_HEADER:
		ParseFrameHeader(payload, payload_size);
		m_nal_type_base = BaseDecNalUnitType::Slice;
		break;

	case OBU_FRAME:
		// OBU_FRAME contains a frame header followed by tile data - key frame check via the
		// leading bits of the uncompressed header (frame_type)
		ParseFrameHeader(payload, payload_size);
		m_nal_type_base = BaseDecNalUnitType::Slice;
		break;

	case OBU_TILE_GROUP:
		m_nal_type_base = BaseDecNalUnitType::Slice;
		break;

	default:
		break;
	}

	return true;
}

// Parse a sequence header - as per the AV1 spec section 6.4.2. Only the fields up to and
// including the colour configuration are parsed.
//
void BaseDecoderAV1::ParseSequenceHeader(const uint8_t *payload, uint32_t size) {
	if (size < 4)
		return;

	BitReader r(payload, size);

	// seq_profile: 3 bits
	const unsigned seq_profile = r.read(3);
	// still_picture: 1 bit
	r.read_flag();
	// reduced_still_picture_header: 1 bit
	const bool reduced_still_picture_header = r.read_flag();

	if (reduced_still_picture_header) {
		// seq_level_idx[0]: 2 + major 3 bits + minor 2 bits (5 bits)
		// seq_tier[0]: 1 bit (only if major level > 3)
		const unsigned major_level = 2 + r.read(3);
		r.read(2);
		if (major_level > 3)
			r.read(1);
	} else {
		// timing_info_present_flag: 1 bit
		const bool timing_info_present = r.read_flag();
		bool decoder_model_present = false;
		unsigned buffer_delay_length = 0;
		if (timing_info_present) {
			// num_units_in_display_tick: 32 bits
			// time_scale: 32 bits
			r.read(64);
			// equal_picture_interval: 1 bit
			if (r.read_flag()) {
				// num_ticks_per_picture_minus_1: uvlc - a single flag bit followed by up to 7 bits
				if (r.read(1))
					r.read(7);
			}
			// decoder_model_info_present_flag: 1 bit
			decoder_model_present = r.read_flag();
			if (decoder_model_present) {
				// buffer_delay_length_minus_1: 5 bits
				buffer_delay_length = r.read(5) + 1;
				// num_units_in_decoding_tick: 32 bits
				// buffer_removal_time_length_minus_1: 5 bits
				// frame_presentation_time_length_minus_1: 5 bits
				r.read(42);
			}
		}
		// initial_display_delay_present_flag: 1 bit
		const bool initial_display_delay_present = r.read_flag();
		// operating_points_cnt_minus_1: 5 bits
		const unsigned operating_points = r.read(5) + 1;
		for (unsigned i = 0; i < operating_points; ++i) {
			// operating_point_idc[i]: 12 bits
			r.read(12);
			// seq_level_idx[i]: 2 + major 3 bits, minor 2 bits
			const unsigned major_level = 2 + r.read(3);
			// minor_level: 2 bits
			r.read(2);
			if (major_level > 3)
				// seq_tier[i]: 1 bit
				r.read(1);
			if (decoder_model_present) {
				// decoder_model_present_for_this_op[i]: 1 bit
				if (r.read_flag()) {
					// decoder_buffer_delay[i]: buffer_delay_length
					// encoder_buffer_delay[i]: buffer_delay_length
					// low_delay_mode_flag[i]: 1 bit
					r.read(2 * buffer_delay_length + 1);
				}
			}
			if (initial_display_delay_present) {
				// initial_display_delay_present_for_this_op[i]: 1 bit
				if (r.read_flag())
					// initial_display_delay_minus_1[i]: 4 bits
					r.read(4);
			}
		}
	}

	// frame_width_bits_minus_1: 4 bits
	// frame_height_bits_minus_1: 4 bits
	const unsigned width_n_bits = r.read(4) + 1;
	const unsigned height_n_bits = r.read(4) + 1;

	// max_frame_width_minus_1: width_n_bits
	// max_frame_height_minus_1: height_n_bits
	const unsigned width = r.read(width_n_bits) + 1;
	const unsigned height = r.read(height_n_bits) + 1;

	m_width = width;
	m_height = height;

	// Skip the remainder of the sequence header up to the colour configuration
	if (reduced_still_picture_header) {
		// frame_size_bits_minus_1 derived, no further fields before colour config
	} else {
		// frame_id_numbers_present_flag: 1 bit
		const bool frame_id_numbers_present = r.read_flag();
		if (frame_id_numbers_present) {
			// delta_frame_id_length_minus_1: 4 bits
			// additional_frame_id_length_minus_1: 3 bits
			r.read(7);
		}
		// use_128x128_superblock: 1 bit
		// enable_filter_intra: 1 bit
		// enable_intra_edge_filter: 1 bit
		r.read(3);
		// enable_interintra_compound: 1 bit
		// enable_masked_compound: 1 bit
		// enable_warped_motion: 1 bit
		// enable_dual_filter: 1 bit
		r.read(4);
		// enable_order_hint: 1 bit
		const bool enable_order_hint = r.read_flag();
		if (enable_order_hint) {
			// enable_jnt_comp: 1 bit
			// enable_ref_frame_mvs: 1 bit
			r.read(2);
		}
		// seq_choose_screen_content_tools: 1 bit
		const bool choose_screen_content_tools = r.read_flag();
		bool force_screen_content_tools = false;
		if (!choose_screen_content_tools)
			// seq_force_screen_content_tools: 1 bit
			force_screen_content_tools = r.read_flag();
		if (choose_screen_content_tools || force_screen_content_tools) {
			// seq_choose_integer_mv: 1 bit
			if (!r.read_flag())
				// seq_force_integer_mv: 1 bit
				r.read(1);
		}
		if (enable_order_hint)
			// order_hint_bits_minus_1: 3 bits
			r.read(3);
		// enable_superres: 1 bit
		// enable_cdef: 1 bit
		// enable_restoration: 1 bit
		r.read(3);
	}

	// Colour configuration
	// high_bitdepth: 1 bit
	const bool high_bitdepth = r.read_flag();
	// (seq_profile == 2 && high_bitdepth) ? twelve_bit: 1 bit
	const bool twelve_bit = (seq_profile == 2 && high_bitdepth) ? r.read_flag() : false;
	// (seq_profile != 1) ? monochrome: 1 bit
	const bool monochrome = (seq_profile != 1) ? r.read_flag() : false;

	// Bit depth, as per the AV1 spec section 6.4.2
	if (seq_profile == 2 && high_bitdepth && twelve_bit)
		m_bit_depth = 12;
	else if (high_bitdepth)
		m_bit_depth = 10;
	else
		m_bit_depth = 8;

	// Chroma format
	switch (seq_profile) {
	case 0:
		m_chroma_format_idc = 1; // 4:2:0
		break;
	case 1:
		m_chroma_format_idc = monochrome ? 0 : 3; // 4:4:4
		break;
	case 2:
		m_chroma_format_idc = monochrome ? 0 : 1; // 4:2:0 (or 4:4:4 if ss_hor == 0)
		break;
	default:
		m_chroma_format_idc = 1;
		break;
	}
}

// Parse the leading bits of an uncompressed frame header - as per the AV1 spec section 6.8.2
//
void BaseDecoderAV1::ParseFrameHeader(const uint8_t *payload, uint32_t size) {
	if (size < 1)
		return;

	// show_existing_frame: 1 bit
	const bool show_existing_frame = (payload[0] >> 7) & 0x01;

	// frame_type: 2 bits (only when show_existing_frame == 0)
	unsigned frame_type = FRAME_INTER;
	if (!show_existing_frame)
		frame_type = (payload[0] >> 5) & 0x03;

	switch (frame_type) {
	case FRAME_KEY:
		m_picture_type = BaseDecPictType::IDR;
		break;
	case FRAME_INTER:
	case FRAME_SWITCH:
		m_picture_type = BaseDecPictType::P;
		break;
	case FRAME_INTRA_ONLY:
		m_picture_type = BaseDecPictType::I;
		break;
	default:
		m_picture_type = BaseDecPictType::Unknown;
		break;
	}

	++m_current_picture_order_count;
}

BaseDecPictType::Enum BaseDecoderAV1::GetBasePictureType() const { return m_picture_type; }

BaseDecNalUnitType::Enum BaseDecoderAV1::GetBaseNalUnitType() const { return m_nal_type_base; }

int32_t BaseDecoderAV1::GetQP() const { return 0; }

uint32_t BaseDecoderAV1::GetNalType() const { return m_nal_type; }

int64_t BaseDecoderAV1::GetPictureOrderCount() const { return m_current_picture_order_count - 1; }

uint32_t BaseDecoderAV1::GetPictureWidth() const { return m_width; }

uint32_t BaseDecoderAV1::GetPictureHeight() const { return m_height; }

bool BaseDecoderAV1::GetDPBCanRefresh() const { return m_picture_type == BaseDecPictType::IDR; }

uint8_t BaseDecoderAV1::GetMaxNumberOfReorderFrames() const { return 0; }

uint32_t BaseDecoderAV1::GetFrameRate() const { return 0; }

uint32_t BaseDecoderAV1::GetBitDepthLuma() const { return m_bit_depth; }

uint32_t BaseDecoderAV1::GetBitDepthChroma() const { return m_bit_depth; }

uint32_t BaseDecoderAV1::GetChromaFormatIDC() const { return m_chroma_format_idc; }

uint32_t BaseDecoderAV1::GetTemporalId() const { return 0; }

NALDelimitier BaseDecoderAV1::Delimiter() const { return NALDelimiterOBU; }

int64_t BaseDecoderAV1::GetPictureOrderCountIncrement() const { return 1; }

std::unique_ptr<BaseDecoder> CreateBaseDecoderAV1() { return std::unique_ptr<BaseDecoder>{new BaseDecoderAV1{}}; }

} // namespace utility
} // namespace vnova
