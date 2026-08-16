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

#include <cstdint>
#include <cstring>
#include "uESFile.h"
#include "Diagnostics.hpp"

namespace vnova {
namespace utility {

static const uint8_t kNalUnitMarkerSize = 3;
static const uint8_t kNalUnitMarker[kNalUnitMarkerSize] = {0x0, 0x0, 0x1};

	ESFile::ESFile() : m_file(nullptr), m_type(BaseDecoder::None), m_decoder(nullptr),
					   m_poc_highest(0), m_poc_offset(0) {
	mpucBuffer = (unsigned char *)malloc(BITSTREAM_BUFFER_SIZE);
}

ESFile::~ESFile() {
	free(mpucBuffer);
	Close();
}

bool ESFile::Open(const std::string &path, BaseDecoder::Codec type) {
	if (IsOpen())
		Close();

	if ((m_file = fopen(path.c_str(), "rb")) == nullptr)
		return false;

	m_type = type;

	return Reset();
}

bool ESFile::Reset() {
	m_decoder = CreateBaseDecoder(m_type);
	CHECK(!!m_decoder);

	// Seeking fails on non-seekable streams (pipes): this is fine, we only reset right
	// after opening, when the stream is already at position 0
	return fseek(m_file, 0, SEEK_SET) == 0 || errno == ESPIPE;
}

void ESFile::Close() {
	if (m_file != nullptr) {
		fclose(m_file);
		m_file = nullptr;
	}

	m_type = BaseDecoder::None;
}

bool ESFile::IsOpen() const { return m_file != nullptr; }

bool ESFile::IsEof() const { return feof(m_file) == 0; }

BaseDecoder::Codec ESFile::GetType() const { return m_type; }

uint32_t ESFile::GetPictureWidth() const {
	assert(m_decoder);

	return m_decoder->GetPictureWidth();
}

uint32_t ESFile::GetPictureHeight() const {
	assert(m_decoder);

	return m_decoder->GetPictureHeight();
}

uint32_t ESFile::GetBitDepth() const {
	assert(m_decoder);

	CHECK(m_decoder->GetBitDepthLuma() == m_decoder->GetBitDepthLuma());

	return m_decoder->GetBitDepthLuma();
}

uint32_t ESFile::GetChromaFormatIDC() const {
	assert(m_decoder);

	return m_decoder->GetChromaFormatIDC();
}

ESFile::Result ESFile::NextAccessUnit(AccessUnit &out) {
	Result result;

	result = ReadAccessUnit(out);

	if (result != Success)
		return result;

	return Success;
}

ESFile::Result ESFile::ReadAccessUnit(AccessUnit &out) {
	switch (m_decoder->Delimiter()) {
	case NALDelimiterMarker:
		return ReadAccessUnitMarker(out);

	case NALDelimiterU32Length:
		return ReadAccessUnitU32Length(out);

	case NALDelimiterOBU:
		return ReadAccessUnitOBU(out);

	default:
		CHECK(0);
		return NoFile;
	}
}

ESFile::Result ESFile::ReadAccessUnitMarker(AccessUnit &out) {
	if (m_file == nullptr)
		return NoFile;

	utility::DataBuffer buffer;
	std::vector<NalUnit> nalUnits;
	int32_t size = 0;
	uint32_t temporalId = 0;

	while (true) {
		const int c = fgetc(m_file);

		if (c != EOF)
			buffer.push_back(c);

		bool foundNalStart =
		    buffer.size() > kNalUnitMarkerSize && std::equal(buffer.end() - kNalUnitMarkerSize, buffer.end(), kNalUnitMarker);

		if (foundNalStart || feof(m_file)) {

			uint64_t nalLength = buffer.size();
			/* uint64_t */ int markerSize = 0;

			if (foundNalStart) {
				markerSize = kNalUnitMarkerSize;

				// Handle [0, 0, 0, 1] case
				if (buffer.size() > kNalUnitMarkerSize && buffer[nalLength - markerSize - 1] == 0x0)
					++markerSize;
			}

			if (nalLength > markerSize) {
				nalLength -= markerSize;

				unsigned result = m_decoder->ParseNalUnit(buffer.data(), static_cast<uint32_t>(nalLength));

				if (result == 1) {
					NalUnit unit;

					auto nalEnd = buffer.end() - markerSize;
					unit.m_type = m_decoder->GetNalType();
					temporalId = std::max(temporalId, m_decoder->GetTemporalId());

					// Move the next nal unit's marker into unit's buffer so we can just swap the two
					unit.m_data.insert(unit.m_data.end(), nalEnd, buffer.end());
					buffer.erase(nalEnd, buffer.end());
					std::swap(unit.m_data, buffer);

					nalUnits.push_back(std::move(unit));
					size += (int32_t)(nalLength);

					if (m_decoder->GetBaseNalUnitType() == BaseDecNalUnitType::Slice) {

						out.m_pictureType = m_decoder->GetBasePictureType();
						out.m_poc = GenerateIncreasingPOC();
						out.m_qp = m_decoder->GetQP();
						out.m_temporalId = temporalId;
						out.m_nalUnits = std::move(nalUnits);

						out.m_size = size;

						fseek(m_file, -static_cast<long>(markerSize), SEEK_CUR);

						return Success;
					}
				} else if (result == 2) {
					NalUnit unit;

					auto nalEnd = buffer.end() - markerSize;

					// Move the next nal unit's marker into unit's buffer so we can just swap the two
					unit.m_data.insert(unit.m_data.end(), nalEnd, buffer.end());
					buffer.erase(nalEnd, buffer.end());
					std::swap(unit.m_data, buffer);
					nalUnits.push_back(std::move(unit));
				} else
					return NalParsingError;
			}
		}

		if (c == EOF)
			break;
	}

	return EndOfFile;
}

ESFile::Result ESFile::ReadAccessUnitU32Length(AccessUnit &out) {
	if (m_file == nullptr)
		return NoFile;

	utility::DataBuffer buffer;
	std::vector<NalUnit> nalUnits;
	int32_t size = 0;

	while (true) {
		// Read length prefix
		uint32_t nal_length = 0;
		{
			const size_t result = fread(&nal_length, 1, sizeof(nal_length), m_file);
			if (result == 0)
				return EndOfFile;

			if (result != sizeof(nal_length))
				return NalParsingError;
		}

		// Read body of NAL unit
		buffer.resize(nal_length);
		if (fread(buffer.data(), 1, nal_length, m_file) != nal_length)
			return NalParsingError;

		if (m_decoder->ParseNalUnit(buffer.data(), nal_length)) {
			NalUnit unit;
			unit.m_type = m_decoder->GetNalType();
			unit.m_data.insert(unit.m_data.end(), (uint8_t *)&nal_length, (uint8_t *)&nal_length + sizeof(nal_length));
			unit.m_data.insert(unit.m_data.end(), buffer.begin(), buffer.end());
			nalUnits.push_back(std::move(unit));
			size += (int32_t)(nal_length + 4); // length + length prefix

			if (m_decoder->GetBaseNalUnitType() == BaseDecNalUnitType::Slice) {
				out.m_pictureType = m_decoder->GetBasePictureType();
				out.m_poc = GenerateIncreasingPOC();
				out.m_qp = m_decoder->GetQP();
				out.m_nalUnits = std::move(nalUnits);

				out.m_size = size;

				return Success;
			}

		} else {
			return NalParsingError;
		}
	}
}

// Read an access unit from an AV1 stream. Two framings are supported:
//  - low overhead OBU stream: each OBU is framed as
//      obu_header, [obu_extension_header], leb128(obu_size), obu_payload
//    and an access unit is terminated by the next temporal delimiter OBU (which itself
//    belongs to the following access unit)
//  - IVF container (the output of SvtAv1EncApp versions without --obu, and its default):
//    a 32-byte "DKIF" stream header followed by per-frame [12-byte header][OBU payload]
//    records, each payload being one complete access unit
//
ESFile::Result ESFile::ReadAccessUnitOBU(AccessUnit &out) {
	if (m_file == nullptr)
		return NoFile;

	// Detect the IVF container on the first access unit: read its 4-byte magic into the
	// lookahead buffer and replay it through the byte source (a pipe cannot be rewound, and
	// ungetc does not reliably push back more than one byte)
	if (!m_ivf_checked) {
		m_ivf_checked = true;

		unsigned char magic[4];
		const size_t got = fread(magic, 1, sizeof(magic), m_file);
		m_lookahead.assign(magic, magic + got);
		m_ivf = (got == sizeof(magic) && memcmp(magic, "DKIF", 4) == 0);
	}

	if (m_ivf)
		return ReadAccessUnitIVF(out);

	return ParseAccessUnitOBUs(out, nullptr, 0);
}

// Read an access unit from an IVF container: skip the 32-byte stream header once, then read
// one [4-byte LE size][8-byte timestamp] frame header and the frame payload, which contains
// the whole access unit as OBUs.
//
ESFile::Result ESFile::ReadAccessUnitIVF(AccessUnit &out) {
	if (!m_ivf_stream_header_skipped) {
		m_ivf_stream_header_skipped = true;

		// 32-byte IVF stream header; the 4 magic bytes are already in the lookahead buffer
		unsigned char header[32];
		memcpy(header, m_lookahead.data(), m_lookahead.size());
		const size_t got = fread(header + m_lookahead.size(), 1, sizeof(header) - m_lookahead.size(), m_file);
		if (got + m_lookahead.size() != sizeof(header))
			return EndOfFile;
		m_lookahead.clear();
	}

	// Read the 12-byte IVF frame header (4-byte little-endian size + 8-byte timestamp)
	unsigned char frame_header[12];
	size_t got = fread(frame_header, 1, sizeof(frame_header), m_file);
	if (got == 0)
		return EndOfFile;
	if (got != sizeof(frame_header))
		return NalParsingError;

	const uint32_t frame_size = (uint32_t)frame_header[0] | ((uint32_t)frame_header[1] << 8) |
	                            ((uint32_t)frame_header[2] << 16) | ((uint32_t)frame_header[3] << 24);

	utility::DataBuffer payload(frame_size);
	if (payload.size() != frame_size)
		return NalParsingError;
	if (fread(payload.data(), 1, frame_size, m_file) != frame_size)
		return NalParsingError;

	return ParseAccessUnitOBUs(out, payload.data(), payload.size());
}

// Parse access units from an OBU byte source. With 'buf' set, the OBUs are read from that
// memory buffer (one IVF frame payload = one access unit); otherwise they are read from the
// file, replaying any detection lookahead first. An access unit ends at the next temporal
// delimiter, or at the end of the source.
//
ESFile::Result ESFile::ParseAccessUnitOBUs(AccessUnit &out, const uint8_t *buf, size_t len) {
	utility::DataBuffer buffer;
	std::vector<NalUnit> nalUnits;
	int32_t size = 0;
	bool seen_frame = false;
	size_t pos = 0;

	// Return the next byte of the source (EOF at the end)
	auto next_byte = [&]() -> int {
		if (buf) {
			if (pos < len)
				return buf[pos++];
			return EOF;
		}
		if (!m_lookahead.empty()) {
			const int c = m_lookahead.front();
			m_lookahead.erase(m_lookahead.begin());
			return c;
		}
		return fgetc(m_file);
	};

	// A temporal delimiter read at the end of the previous access unit starts the next one
	// (pipes/FIFOs cannot be rewound, so the delimiter is buffered rather than seeked back)
	if (!m_pendingFirstNal.empty()) {
		buffer = std::move(m_pendingFirstNal);
		m_pendingFirstNal.clear();
		if (!m_decoder->ParseNalUnit(buffer.data(), (uint32_t)buffer.size()))
			return NalParsingError;
		NalUnit unit;
		unit.m_type = (buffer.empty()) ? 0 : (buffer[0] >> 3) & 0x0f;
		unit.m_data = std::move(buffer);
		size += (int32_t)unit.m_data.size();
		nalUnits.push_back(std::move(unit));
	}

	while (true) {
		// Read OBU header byte
		const int c = next_byte();
		if (c == EOF) {
			// End of file - flush any remaining AU
			if (!nalUnits.empty()) {
				out.m_pictureType = m_decoder->GetBasePictureType();
				out.m_poc = GenerateIncreasingPOC();
				out.m_qp = m_decoder->GetQP();
				out.m_nalUnits = std::move(nalUnits);
				out.m_size = size;
				return Success;
			}
			return EndOfFile;
		}

		const uint8_t header = (uint8_t)c;
		if (header & 0x01)
			return NalParsingError; // reserved bit set

		buffer.push_back(header);

		// OBU extension header
		if ((header >> 2) & 0x01) {
			const int ext = next_byte();
			if (ext == EOF)
				return NalParsingError;
			buffer.push_back((uint8_t)ext);
		}

		// OBU size field (leb128)
		uint64_t payload_size = 0;
		if ((header >> 1) & 0x01) {
			unsigned i = 0;
			while (true) {
				const int s = next_byte();
				if (s == EOF)
					return NalParsingError;
				buffer.push_back((uint8_t)s);
				payload_size |= (uint64_t)(s & 0x7f) << (7 * i);
				++i;
				if (!(s & 0x80) || i >= 8)
					break;
			}
		} else {
			// OBU without a size field cannot be framed - unsupported
			return NalParsingError;
		}

		// Read OBU payload
		for (uint64_t n = 0; n < payload_size; ++n) {
			const int p = next_byte();
			if (p == EOF)
				return NalParsingError;
			buffer.push_back((uint8_t)p);
		}

		const uint8_t obu_type = (header >> 3) & 0x0f;

		// Parse the OBU to track frame boundaries and picture state
		if (!m_decoder->ParseNalUnit(buffer.data(), (uint32_t)buffer.size()))
			return NalParsingError;

		// A temporal delimiter ends the current AU (and belongs to the next one)
		if (obu_type == 2 && seen_frame) {
			m_pendingFirstNal = std::move(buffer);

			out.m_pictureType = m_decoder->GetBasePictureType();
			out.m_poc = GenerateIncreasingPOC();
			out.m_qp = m_decoder->GetQP();
			out.m_temporalId = m_decoder->GetTemporalId();
			out.m_nalUnits = std::move(nalUnits);
			out.m_size = size;
			return Success;
		}

		// Frame OBUs: frame header, tile group, or whole frame
		if (obu_type == 3 || obu_type == 4 || obu_type == 6)
			seen_frame = true;

		// Store the OBU (as read from the file) in the AU
		NalUnit unit;
		unit.m_type = obu_type;
		unit.m_data = std::move(buffer);
		buffer.clear();
		size += (int32_t)(unit.m_data.size());
		nalUnits.push_back(std::move(unit));
	}
}

// Create a POC that always increases across IDR
//
uint64_t ESFile::GenerateIncreasingPOC() {
	uint64_t decoded_poc = m_decoder->GetPictureOrderCount();

	// If we see the start of an IDR and the POC goes backwards, then
	// offset the generated POC by the highest POC seen so far
	//
	if(m_decoder->GetBasePictureType() == BaseDecPictType::IDR && decoded_poc < m_poc_highest) {
		m_poc_offset = m_poc_highest;
	}

	const uint64_t poc = decoded_poc + m_poc_offset;

	if(poc > m_poc_highest) {
		m_poc_highest = poc + m_decoder->GetPictureOrderCountIncrement();
	}

	return poc;
}

} // namespace utility
} // namespace vnova
