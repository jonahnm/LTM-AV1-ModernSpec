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

#include <cstdint>
#include "ScanEnhancement.hpp"

#include <cstring>
#include <stdexcept>

#include "Diagnostics.hpp"

using namespace std;

namespace lctm {

struct RegisteredSEI {
	static void extract_enhancement_sei(const uint8_t *data, size_t size, bool emulation_prevention, uint64_t pts, bool is_base_idr,
	                                    std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback);
};

struct UnregisteredSEI {
	static void extract_enhancement_sei(const uint8_t *data, size_t size, bool emulation_prevention, uint64_t pts, bool is_base_idr,
	                                    std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback);
};

//// Extractor for raw byte sequence payloads - remove start code emulation prevention bytes
//
class rbsp_decoder {
public:
	rbsp_decoder(const uint8_t *d, size_t l, bool e) : data(d), len(l), emulatiom_prevention(e), emitted(0xFFFFFFFF), offs(0){};
	uint8_t get_byte();
	uint8_t *copy_sei(uint8_t *dst, size_t len);
	void copy(PacketBuilder *dst);

protected:
	const uint8_t *data;
	size_t len;
	bool emulatiom_prevention;
	uint32_t emitted;
	size_t offs;
};

uint8_t rbsp_decoder::get_byte() {
	if (offs >= len)
		throw out_of_range("No more RBSP");

	emitted <<= 8;
	emitted |= data[offs++];
	if (emulatiom_prevention && (emitted & 0xFFFFFF) == 0x000003) {
		emitted <<= 8;
		if (offs >= len)
			throw out_of_range("No more RBSP");

		emitted |= data[offs++];
	}
	return emitted & 0xFF;
}

uint8_t *rbsp_decoder::copy_sei(uint8_t *d, size_t l) {
	for (size_t i = 0; i < l; i++)
		d[i] = get_byte();

	return d;
}

void rbsp_decoder::copy(PacketBuilder *dst) {
	size_t i = 0;
	dst->reserve((uint32_t)len - (uint32_t)offs);
	while (true) {
		try {
			dst->data()[i] = get_byte();
		} catch (const std::out_of_range &) {
			dst->resize((uint32_t)i -
			            1); // 0x80 is added during encapsulation process but does not belong to the actual data - stop bit
			return;
		}
		i++;
	}
}

// Recognize the NALU marker (used by LCEVC,AVC,HEVC,VVC, but not EVC)
//
static inline bool is_nal_marker(const uint8_t *data) { return data[0] == 0 && data[1] == 0 && data[2] == 1; }

// Check for valid LCEVC NAL uint type
//
static inline bool is_lcevc_nal_unit_type(const uint8_t b) { return (b >= LCEVC_NON_IDR) && (b <= LCEVC_RSV); }

// Extract enhancement data and pass to callback
//
static void extract_enhancement_nal(const uint8_t *data, size_t size, bool emulation_prevention, uint64_t pts, const bool is_idr,
                                    std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {

	auto pss = Packet::build();
	pss.timestamp(pts);

	if (emulation_prevention) {
		rbsp_decoder rbsp(data, size, true);
		rbsp.copy(&pss);
	} else {
		pss.contents(data, (unsigned)size);
	}
	callback(pss.finish(), is_idr);
}

// Find any enhancement data in native LCEVC NAL Unit and pass to callback
//
static size_t scan_enhancement_nal(uint8_t *data, size_t data_size, uint64_t pts,
                                   std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	// Scan for Enhancement NALU
	for (unsigned i = 0; i < data_size - 4; ++i) {
		if (is_nal_marker(data + i) && (data[i + 3] & 0xc0) == 0x40 && data[i + 4] == 0xff) {
			const unsigned nal_unit_type = (data[i + 3] & 0x3e) >> 1;

			if (is_lcevc_nal_unit_type(nal_unit_type)) {
				const bool is_idr = (nal_unit_type == NalUnitType::LCEVC_IDR ? true : false);

				// Figure size of NALU
				unsigned e = (unsigned)data_size;
				for (unsigned j = i + 4; j < data_size - 3; ++j) {
					if (is_nal_marker(data + j)) {
						// Handle [0, 0, 0, 1] case
						if (data[j - 1] == 0x00)
							j--;
						e = j;
						break;
					}
				}
				extract_enhancement_nal(data + (i + 5), e - (i + 5), true, pts, is_idr, callback);

				// Remove enhancement data from bitstream leaving base codec bitstream
				memmove(data + i, data + e, data_size - e);
				data_size = data_size + i - e;
			}
		}
	}

	return data_size;
}

// Find any enhancement data in native EVC NAL Unit (u32 length-prefix) and pass to callback
//
static size_t scan_enhancement_nal_evc(uint8_t *data, size_t data_size, uint64_t pts,
                                       std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	unsigned offset = 0;

	while (offset < data_size - 6) {
		uint32_t nal_length = 0;
		memcpy(&nal_length, data + offset, 4);
		uint32_t total_length = nal_length + 4;

		if ((data[offset + 4] & 0xfe) == 0x3c && (data[offset + 5] & 0x3f) == 0x00) {
			const unsigned nal_unit_type = (data[offset + 4] & 0xf8) >> 3;
			const bool is_idr = (nal_unit_type == NalUnitType::LCEVC_IDR ? true : false);

			extract_enhancement_nal(data + offset + 6, nal_length - 2, false, pts, is_idr, callback);

			memmove(data + offset, data + offset + total_length, data_size - (offset + total_length));
			data_size -= total_length;
		} else {
			offset += total_length;
		}
	}

	return data_size;
}

// Find any enhancement data in native LCEVC NAL Unit and pass to callback (without memmove!)
// Used to extract LCEVC NAL Unit from registered SEI -> required for SDK decoding workflow
//
static void scan_enhancement_nal_prod(const uint8_t *data, size_t data_size, uint64_t pts,
                                      std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	// Scan for Enhancement NALU
	for (unsigned i = 0; i < data_size - 4; ++i) {
		if (is_nal_marker(data + i) && (data[i + 3] & 0xc0) == 0x40 && data[i + 4] == 0xff) {
			const unsigned nal_unit_type = (data[i + 3] & 0x3e) >> 1;

			if (is_lcevc_nal_unit_type(nal_unit_type)) {
				const bool is_idr = (nal_unit_type == NalUnitType::LCEVC_IDR ? true : false);

				// Figure size of NALU
				unsigned e = (unsigned)data_size;
				for (unsigned j = i + 4; j < data_size - 3; ++j) {
					if (is_nal_marker(data + j)) {
						// Handle [0, 0, 0, 1] case
						if (data[j - 1] == 0x00)
							j--;
						e = j;
						break;
					}
				}
				extract_enhancement_nal(data + (i + 5), e - (i + 5), true, pts, is_idr, callback);
			}
		}
	}
}

// Find any enhancement data in registered SEI NALU and pass to callback
// Used as a template parameter to scan_enhancement_sei_XXX()
//
void RegisteredSEI::extract_enhancement_sei(const uint8_t *data, size_t size, bool emulation_prevention, uint64_t pts,
                                            bool is_base_idr,
                                            std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	const unsigned USER_DATA_REGISTERED = 4;
	static const uint8_t sei_code[4] = {0xb4, 0x00, 0x50, 0x00};

	rbsp_decoder rbsp(data, size, emulation_prevention);

	size_t sei_type = 0;
	uint8_t byte;

	do {
		byte = rbsp.get_byte();
		sei_type += byte;
	} while (byte == 0xFF);

	if (sei_type == USER_DATA_REGISTERED) {
		size_t sei_length = 0;
		do {
			byte = rbsp.get_byte();
			sei_length += byte;
		} while (byte == 0xFF);

		if (sei_length > size) {
			WARN("SEI length overflow");
		} else {
			uint8_t tmp[sizeof(sei_code)];
			rbsp.copy_sei(tmp, sizeof(tmp));

			if (memcmp(sei_code, tmp, sizeof(sei_code)) == 0) {
				int64_t enhancement_len = sei_length - sizeof(sei_code);

				vector<uint8_t> buf(enhancement_len);
				rbsp.copy_sei(buf.data(), buf.size());

				// Additional NAL Encapsulation (LCEVC NALU Type)
				scan_enhancement_nal_prod(buf.data(), buf.size(), pts, callback);
			}
		}
	}
}

// Find any enhancement data in unregistered SEI NALU and pass to callback
// Used as a template parameter to scan_enhancement_sei_XXX()
//
void UnregisteredSEI::extract_enhancement_sei(const uint8_t *data, size_t size, bool emulation_prevention, uint64_t pts,
                                              bool is_base_idr,
                                              std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	const unsigned USER_DATA_UNREGISTERED = 5;
	static const uint8_t uuid[16] = {0xa7, 0xc4, 0x6d, 0xed, 0x49, 0xd8, 0x38, 0xeb,
	                                 0x9a, 0xad, 0x6d, 0xa6, 0x84, 0x97, 0xa7, 0x54};

	rbsp_decoder rbsp(data, size, emulation_prevention);

	size_t sei_type = 0;
	uint8_t byte;

	do {
		byte = rbsp.get_byte();
		sei_type += byte;
	} while (byte == 0xFF);

	if (sei_type == USER_DATA_UNREGISTERED) {
		size_t sei_length = 0;
		do {
			byte = rbsp.get_byte();
			sei_length += byte;
		} while (byte == 0xFF);

		if (sei_length > size) {
			WARN("SEI length overflow");
		} else {
			uint8_t tmp[sizeof(uuid)];
			rbsp.copy_sei(tmp, sizeof(tmp));

			if (memcmp(uuid, tmp, sizeof(uuid)) == 0) {
				int64_t enhancement_len = sei_length - sizeof(uuid);

				auto pss = Packet::build();
				pss.reserve((unsigned)enhancement_len);
				pss.timestamp(pts);
				rbsp.copy_sei(pss.data(), enhancement_len);
				callback(pss.finish(), is_base_idr);
			}
		}
	}
}

// AVC NALUs headers have one byte for type
template <typename SEI>
static size_t scan_enhancement_sei_avc(uint8_t *data, size_t data_size, uint64_t pts, bool is_base_idr,
                                       std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	// Scan for SEI NALU
	for (unsigned i = 0; i < data_size - 4; ++i) {
		if (is_nal_marker(data + i) && data[i + 3] == 0x6) {
			// Figure size of NALU
			unsigned e = (unsigned)data_size;
			for (unsigned j = i + 4; j < data_size - 3; ++j) {
				if (is_nal_marker(data + j)) {
					e = j;
					break;
				}
			}
			SEI::extract_enhancement_sei(data + (i + 4), e - (i + 4), true, pts, is_base_idr, callback);
		}
	}

	// SEI can survive base decoder - don't remove
	return data_size;
}

// HEVC NALUs headers have two bytes for type
template <typename SEI>
static size_t scan_enhancement_sei_hevc(uint8_t *data, size_t data_size, uint64_t pts, bool is_base_idr,
                                        std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	// Scan for SEI NALU
	for (unsigned i = 0; i < data_size - 5; ++i) {
		if (is_nal_marker(data + i) && data[i + 3] == 0x4e && data[i + 4] == 0x01) {
			// Figure size of NALU
			unsigned e = (unsigned)data_size;
			for (unsigned j = i + 4; j < data_size - 3; ++j) {
				if (is_nal_marker(data + j)) {
					e = j;
					break;
				}
			}
			SEI::extract_enhancement_sei(data + (i + 5), e - (i + 5), true, pts, is_base_idr, callback);
		}
	}

	// SEI can survive base decoder - don't remove
	return data_size;
}

// VVC NALUs headers have two bytes for type
template <typename SEI>
static size_t scan_enhancement_sei_vvc(uint8_t *data, size_t data_size, uint64_t pts, bool is_base_idr,
                                       std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	// Scan for SEI NALU
	for (unsigned i = 0; i < data_size - 5; ++i) {
		if (is_nal_marker(data + i) && data[i + 3] == 0x00 && (data[i + 4] & 0xf8) == 0xb8) {
			// Figure size of NALU
			unsigned e = (unsigned)data_size;
			for (unsigned j = i + 4; j < data_size - 3; ++j) {
				if (is_nal_marker(data + j)) {
					e = j;
					break;
				}
			}
			SEI::extract_enhancement_sei(data + (i + 5), e - (i + 5), true, pts, is_base_idr, callback);
		}
	}

	// SEI can survive base decoder - don't remove
	return data_size;
}

// EVC NALUs are u32 size prefixed, not marker delimited
template <typename SEI>
static size_t scan_enhancement_sei_evc(uint8_t *data, size_t data_size, uint64_t pts, bool is_base_idr,
                                       std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {

	unsigned offset = 0;

	while (offset < data_size - 6) {
		uint32_t nal_length = 0;
		memcpy(&nal_length, data + offset, 4);
		uint32_t total_length = nal_length + 4;

		if ((data[offset + 4] & 0xfe) == 0x38 && (data[offset + 5] & 0x3f) == 0x00) {
			SEI::extract_enhancement_sei(data + 6, nal_length - 2, false, pts, is_base_idr, callback);
		}
		offset += total_length;
	}

	// SEI can survive base decoder - don't remove
	return data_size;
}

// Find any enhancement data in AV1 metadata OBUs (ITU-T T.35, V-Nova provider code) and pass to
// callback; the metadata OBUs are removed from the buffer, leaving the base AV1 bitstream.
//
static size_t scan_enhancement_av1(uint8_t *data, size_t data_size, uint64_t pts,
                                   std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	const uint8_t kMetadataTypeItutT35 = 4;
	static const uint8_t kVNovaItu[4] = {0xb4, 0x00, 0x50, 0x00};

	size_t offset = 0;
	while (offset + 1 < data_size) {
		const uint8_t header = data[offset];
		const uint8_t obu_type = (header >> 3) & 0x0f;
		const bool has_extension = (header >> 2) & 0x01;
		const bool has_size = (header >> 1) & 0x01;

		size_t obu_start = offset;
		size_t pos = offset + 1;
		if (has_extension)
			pos += 1;

		uint64_t payload_size = 0;
		if (has_size) {
			unsigned i = 0;
			while (pos < data_size) {
				const uint8_t s = data[pos++];
				payload_size |= (uint64_t)(s & 0x7f) << (7 * i);
				++i;
				if (!(s & 0x80) || i >= 8)
					break;
			}
		} else {
			// OBUs without a size field cannot be framed - give up
			return data_size;
		}

		if (pos + payload_size > data_size)
			break;

		// OBU_METADATA (type 5) carrying ITU-T T.35 (metadata_type 1)
		if (obu_type == 5 && payload_size > 5 && data[pos] == kMetadataTypeItutT35) {
			const uint8_t *t35 = data + pos + 1;
			const uint32_t t35_size = (uint32_t)(payload_size - 1);

			if (t35_size > 4 && memcmp(t35, kVNovaItu, sizeof(kVNovaItu)) == 0) {
				const uint8_t *lcevc = t35 + sizeof(kVNovaItu);
				// The LCEVC NAL unit occupies the full T.35 payload (its own RBSP stop-bit
				// acts as the metadata OBU's trailing bits)
				const uint32_t lcevc_size = t35_size - (uint32_t)sizeof(kVNovaItu);

				// The LCEVC data is a complete NAL unit: marker + header + rbsp
				if (lcevc_size > 5 && is_nal_marker(lcevc) && (lcevc[3] & 0xc0) == 0x40 && lcevc[4] == 0xff) {
					const unsigned nal_unit_type = (lcevc[3] & 0x3e) >> 1;

					if (is_lcevc_nal_unit_type(nal_unit_type)) {
						const bool is_idr = (nal_unit_type == NalUnitType::LCEVC_IDR ? true : false);
						extract_enhancement_nal(lcevc + 5, lcevc_size - 5, true, pts, is_idr, callback);

						// Remove the metadata OBU from the buffer
						const size_t obu_end = pos + (size_t)payload_size;
						memmove(data + obu_start, data + obu_end, data_size - obu_end);
						data_size -= (obu_end - obu_start);
						continue;
					}
				}
			}
		}

		offset = pos + (size_t)payload_size;
	}

	return data_size;
}

// Scan for enhancement data in buffer - may remove data and returns new size
//
size_t scan_enhancement(uint8_t *data, size_t data_size, Encapsulation encapsulation, BaseCoding base_coding, uint64_t pts,
                        bool is_base_idr, std::function<void(const Packet &pkt, const bool is_lcevc_idr)> callback) {
	switch (encapsulation) {
	case Encapsulation_SEI_Registered:
		switch (base_coding) {
		case BaseCoding_AVC:
			return scan_enhancement_sei_avc<RegisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_HEVC:
			return scan_enhancement_sei_hevc<RegisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_VVC:
			return scan_enhancement_sei_vvc<RegisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_EVC:
			return scan_enhancement_sei_evc<RegisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_AV1:
			return scan_enhancement_av1(data, data_size, pts, callback);
		default:
			CHECK(0);
		}
		break;
	case Encapsulation_SEI_Unregistered:
		switch (base_coding) {
		case BaseCoding_AVC:
			return scan_enhancement_sei_avc<UnregisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_HEVC:
			return scan_enhancement_sei_hevc<UnregisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_VVC:
			return scan_enhancement_sei_vvc<UnregisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_EVC:
			return scan_enhancement_sei_evc<UnregisteredSEI>(data, data_size, pts, is_base_idr, callback);
		case BaseCoding_AV1:
			return scan_enhancement_av1(data, data_size, pts, callback);
		default:
			CHECK(0);
		}
		break;
	case Encapsulation_NAL:
		switch (base_coding) {
		case BaseCoding_AVC:
			return scan_enhancement_nal(data, data_size, pts, callback);
		case BaseCoding_HEVC:
			return scan_enhancement_nal(data, data_size, pts, callback);
		case BaseCoding_VVC:
			return scan_enhancement_nal(data, data_size, pts, callback);
		case BaseCoding_EVC:
			return scan_enhancement_nal_evc(data, data_size, pts, callback);
		case BaseCoding_AV1:
			return scan_enhancement_av1(data, data_size, pts, callback);
		case BaseCoding_YUV:
			return scan_enhancement_nal(data, data_size, pts, callback);
		default:
			CHECK(0);
		}
		break;
	case Encapsulation_None:
		return scan_enhancement_nal(data, data_size, pts, callback);

	default:
		CHECK(0);
	}

	return 0;
}
} // namespace lctm
