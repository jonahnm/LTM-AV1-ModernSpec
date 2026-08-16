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
// Contributors: Sam Littlewood (sam.littlewood@v-nova.com)
//
// YUVReader.cpp
//
// Read raw YUV video files
//
#include <cstdint>
#include "YUVReader.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>

#if !defined(_WIN32)
#include <sys/stat.h>
#if !defined(_WIN32)
#include <unistd.h>
#endif
#endif

#include "Diagnostics.hpp"
#include "Image.hpp"
#include "Misc.hpp"
#include "Platform.hpp"

namespace lctm {

using namespace std;

// YUV4MPEG2 details - "YUV4MPEG2" magic, "FRAME" frame markers, and a sane header cap
static const unsigned kYUV4MPEG2_MAGIC_LEN = 9;
static const unsigned kYUV4MPEG2_FRAME_MARKER_LEN = 6;
static const unsigned kYUV4MPEG2_MAX_HEADER = 4096;

YUVReader::YUVReader(const std::string &name, float rate, FILE *file, uintmax_t fileSize)
    : name_(name), length_(0), rate_(rate), file_(file), fileSize_(fileSize) {}

YUVReader::YUVReader(const std::string &name, const ImageDescription &image_description, unsigned length, float rate, FILE *file,
                     uintmax_t fileSize)
    : name_(name), image_description_(image_description), length_(length), rate_(rate), file_(file), fileSize_(fileSize) {}

YUVReader::YUVReader(const std::string &name, const ImageDescription &image_description, unsigned length, float rate,
                     FILE *stream)
    : name_(name), fileSize_(static_cast<uintmax_t>(-1)), image_description_(image_description), length_(length), rate_(rate),
      sequential_(true), position_(0), stream_(stream) {

	CHECK(stream_ != nullptr);
}

FILE *YUVReader::input_stream() const { return stream_ ? stream_ : file_.get(); }

void YUVReader::update_data(const ImageDescription &image_description) {
	if (stream_)
		ERR("Cannot update data of a stream reader");

	if (fileSize_ == static_cast<uintmax_t>(-1))
		ERR("Cannot open YUV file");

	// Figure length
	unsigned length = static_cast<unsigned>(fileSize_ / image_description.byte_size());

	if (length == 0)
		ERR("YUV file is too small");

	image_description_ = image_description;
	length_ = length;

	return;
}

void YUVReader::set_position(unsigned position) const {
	CHECK(position < length_);

	if (sequential_) {
		// Streams can only be read sequentially
		CHECK(position == position_);
		position_ = position + 1;
	} else {
		uint64_t offset;
		if (y4m_)
			// Skip the header, each prior frame's payload and its "FRAME" marker, and the marker of this frame
			offset = y4m_header_size_ + (uint64_t)position * (uint64_t)(image_description_.byte_size() + kYUV4MPEG2_FRAME_MARKER_LEN) + kYUV4MPEG2_FRAME_MARKER_LEN;
		else
			offset = (uint64_t)position * (uint64_t)image_description_.byte_size();

		if (position != position_)
			CHECK(fseeko(file_.get(), offset, SEEK_SET) == 0);
		position_ = position;
	}
}

// True when a sequential input is exhausted - peeks a byte and pushes it back, so it must only
// be called between frames (it is not valid in the middle of a frame)
//
bool YUVReader::eof() const {
	if (!sequential_)
		return false;

	FILE *f = input_stream();
	const int c = fgetc(f);
	if (c == EOF)
		return true;

	CHECK(ungetc(c, f) == c);
	return false;
}

// Read bytes from the input, replaying any bytes consumed during format sniffing first (this
// only ever applies to the start of frame 0 on a non-seekable pipe)
//
void YUVReader::read_bytes(void *buf, size_t len) const {
	FILE *f = input_stream();
	size_t off = 0;

	if (!y4m_pending_.empty()) {
		const size_t c = std::min(len, y4m_pending_.size());
		memcpy(buf, y4m_pending_.data(), c);
		y4m_pending_.erase(0, c);
		off = c;
	}

	if (off < len && fread((char *)buf + off, len - off, 1, f) != 1)
		ERR("Cannot read YUV file");
}

// Get image from frame position
Image YUVReader::read(unsigned position, uint64_t timestamp) const {
	set_position(position);

	FILE *f = input_stream();
	if (y4m_ && sequential_) {
		// Consume the "FRAME" marker (short line, terminated by \n)
		char c = 0;
		while (c != '\n') {
			if (fread(&c, 1, 1, f) != 1)
				ERR("Cannot read YUV4MPEG2 stream");
		}
	}

	std::vector<Surface> surfaces;

	for (unsigned p = 0; p < image_description_.num_planes(); ++p) {
		auto b = Surface::build_from<int8_t>();
		b.reserve_bpp(image_description_.width(p), image_description_.height(p), image_description_.byte_depth(),
		              image_description_.row_stride(p));
		if (image_description_.rows_are_contiguous(p)) {
			read_bytes(b.data(), image_description_.plane_size(p));
		} else {
			for (unsigned y = 0; y < image_description_.height(p); ++y) {
				read_bytes(b.data(0, y), image_description_.row_size(p));
			}
		}
		surfaces.push_back(b.finish());
	}

	return Image(format("%s:%d", name_.c_str(), position_), image_description_, timestamp, surfaces);
}

// Parse the picture details from the filename
//
// Uses roughly the same conventions as Vooya and YUVDeluxe
//
// Returns a description - if format == PF_NONE, then no format was recognised
// If a rate is recognized, and pRate != NULL, then the frame rate will be stored through that pointer
//

// Map of format names to internal version
static const struct {
	const char *name;
	int bits;
	ImageFormat format;
} KnownFormats[] = {
    {"420", -1, IMAGE_FORMAT_YUV420P8},  {"420p", -1, IMAGE_FORMAT_YUV420P8},  {"p420", -1, IMAGE_FORMAT_YUV420P8},
    {"yuv", -1, IMAGE_FORMAT_YUV420P8},

    {"420", 8, IMAGE_FORMAT_YUV420P8},   {"420p", 8, IMAGE_FORMAT_YUV420P8},   {"p420", 8, IMAGE_FORMAT_YUV420P8},
    {"yuv", 8, IMAGE_FORMAT_YUV420P8},

    {"420", 10, IMAGE_FORMAT_YUV420P10}, {"420p", 10, IMAGE_FORMAT_YUV420P10}, {"p420", 10, IMAGE_FORMAT_YUV420P10},
    {"yuv", 10, IMAGE_FORMAT_YUV420P10},

    {"y", 8, IMAGE_FORMAT_Y8},           {"y", 10, IMAGE_FORMAT_Y10},          {"y", 16, IMAGE_FORMAT_Y16},

};

static ImageDescription ParseYUVFilename(const string &name, float *pRate) {

	ImageFormat image_format = IMAGE_FORMAT_NONE;
	unsigned width = 0;
	unsigned height = 0;

	// Parse filename for picture description
	//
	vector<string> parts;
	split(parts, name, ("-_."));

	static const regex dimensions_re("([0-9]+)x([0-9]+)");                     // Size
	static const regex fps_re("([0-9]+)(fps|hz)");                             // Hz
	static const regex bits_re("([0-9]+)(bits?|bpp)");                         // Bit depth
	static const regex format_re("(420|420p|p420|422|p422|422p|yuv|yuyv|y|)"); // Format

	string format;
	int bits = -1;

	for (auto const p : parts) {
		cmatch m;
		const string lp = lowercase(p);

		if (regex_match(lp.c_str(), m, dimensions_re)) {
			width = stoi(m[1]);
			height = stoi(m[2]);
		}

		if (regex_match(p.c_str(), m, fps_re)) {
			if (pRate)
				*pRate = stof(m[1].str());
		}

		if (regex_match(p.c_str(), m, bits_re)) {
			bits = stoi(m[1].str());
		}

		if (regex_match(p.c_str(), m, format_re)) {
			if (format.empty())
				format = lowercase(m[1].str());
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(KnownFormats); ++i)
		if (format == KnownFormats[i].name && bits == KnownFormats[i].bits)
			image_format = KnownFormats[i].format;

	return ImageDescription(image_format, width, height);
}

// Open file for reading, given explicit format
//
static bool ParseYUV4MPEG2HeaderLine(const std::string &line, unsigned *width, unsigned *height, ImageFormat *format,
                                     unsigned *fps_num, unsigned *fps_den);
static ImageFormat YUV4MPEG2ChromaFormat(const std::string &value);

std::unique_ptr<YUVReader> CreateYUVReader(const std::string &name, const ImageDescription &description, unsigned rate) {
	// Open the file (for a FIFO this blocks until a writer connects). "-" means read from
	// stdin - use a duplicate so the reader can own the stream (and seek it when stdin is a
	// regular file) without ever closing the real stdin
	//
	UniquePtrFile yuvFile;
	if (name == "-") {
#if defined(_WIN32)
		yuvFile.reset(fdopen(_dup(_fileno(stdin)), "rb"));
#else
		yuvFile.reset(fdopen(dup(fileno(stdin)), "rb"));
#endif
	} else {
		yuvFile.reset(std::fopen(name.c_str(), "rb"));
	}
	if (!yuvFile)
		ERR("Cannot open YUV file");

	// Can we seek? Only regular files - pipes and FIFOs are sequential
	//
	bool regular = true;
	uintmax_t fileSize = 0;
#if !defined(_WIN32)
	struct stat st = {};
	if (fstat(fileno(yuvFile.get()), &st) == 0) {
		regular = S_ISREG(st.st_mode);
		fileSize = (uintmax_t)st.st_size;
	}
#else
	fileSize = file_size(name);
#endif

	// Sniff for a YUV4MPEG2 header - consume a few bytes up front and replay them as part of
	// the first frame if the input turns out not to be y4m (needed for non-seekable inputs)
	//
	bool y4m = false;
	unsigned header_width = 0, header_height = 0;
	unsigned header_fps_num = 0, header_fps_den = 0;
	ImageFormat header_format = IMAGE_FORMAT_NONE;
	uint64_t header_size = 0;
	std::string pending;

	{
		char magic[kYUV4MPEG2_MAGIC_LEN] = {};
		const size_t got = fread(magic, 1, sizeof(magic), yuvFile.get());
		pending.assign(magic, got);

		if (got == kYUV4MPEG2_MAGIC_LEN && memcmp(magic, "YUV4MPEG2", kYUV4MPEG2_MAGIC_LEN) == 0) {
			// Read the rest of the header line
			char c = 0;
			while (pending.size() < kYUV4MPEG2_MAX_HEADER && pending.back() != '\n') {
				if (fread(&c, 1, 1, yuvFile.get()) != 1)
					break;
				pending += c;
			}
			if (pending.back() == '\n' &&
			    ParseYUV4MPEG2HeaderLine(pending, &header_width, &header_height, &header_format, &header_fps_num, &header_fps_den)) {
				y4m = true;
				header_size = pending.size();
				// The header is not part of the frame data - discard it (frame markers are
				// consumed by read()); only raw-stream sniff bytes are replayed
				pending.clear();
			}
		}
	}

	unsigned length = 0;
	float frame_rate = (float)rate;

	if (y4m) {
		const uintmax_t frame_stride = ImageDescription(header_format, header_width, header_height).byte_size() +
		                               kYUV4MPEG2_FRAME_MARKER_LEN;
		if (regular && fileSize > header_size)
			length = (unsigned)((fileSize - header_size) / frame_stride);
		else
			length = (unsigned)std::numeric_limits<unsigned>::max();

		if (length == 0)
			ERR("YUV file is too small");

		frame_rate = header_fps_den ? (float)header_fps_num / (float)header_fps_den : frame_rate;
	} else if (regular) {
		// Rewind - nothing was consumed
		CHECK(fseeko(yuvFile.get(), 0, SEEK_SET) == 0);
		pending.clear();

		length = (unsigned)(fileSize / description.byte_size());
		if (length == 0)
			ERR("YUV file is too small");
	} else {
		// Non-seekable raw stream (pipe/FIFO): length unknown, play back the sniffed bytes
		length = (unsigned)std::numeric_limits<unsigned>::max();
	}

	unique_ptr<YUVReader> reader(
	    new YUVReader(name, description, length, frame_rate, yuvFile.release(), regular ? fileSize : (uintmax_t)-1));

	reader->sequential_ = !regular;
	if (!regular)
		reader->position_ = 0;
	reader->y4m_ = y4m;
	reader->y4m_header_size_ = header_size;
	reader->y4m_pending_ = pending;
	if (y4m)
		reader->image_description_ = ImageDescription(header_format, header_width, header_height);

	return reader;
}

// Probe a regular file for a YUV4MPEG2 header and report its geometry, format and frame rate
//
bool ProbeYUV4MPEG2File(const std::string &name, unsigned *width, unsigned *height, ImageFormat *format, float *rate) {
#if !defined(_WIN32)
	// Only seekable regular files can be probed safely
	struct stat st = {};
	if (stat(name.c_str(), &st) != 0 || !S_ISREG(st.st_mode))
		return false;
#endif
	UniquePtrFile f(std::fopen(name.c_str(), "rb"));
	if (!f)
		return false;

	char magic[kYUV4MPEG2_MAGIC_LEN] = {};
	if (fread(magic, 1, sizeof(magic), f.get()) != kYUV4MPEG2_MAGIC_LEN ||
	    memcmp(magic, "YUV4MPEG2", kYUV4MPEG2_MAGIC_LEN) != 0)
		return false;

	std::string line(magic, kYUV4MPEG2_MAGIC_LEN);
	char c = 0;
	while (line.size() < kYUV4MPEG2_MAX_HEADER && line.back() != '\n') {
		if (fread(&c, 1, 1, f.get()) != 1)
			return false;
		line += c;
	}
	if (line.back() != '\n')
		return false;

	unsigned fps_num = 0, fps_den = 0;
	if (!ParseYUV4MPEG2HeaderLine(line, width, height, format, &fps_num, &fps_den))
		return false;

	if (rate)
		*rate = fps_den ? (float)fps_num / (float)fps_den : 0.0f;

	return true;
}

// Parse a YUV4MPEG2 header line - e.g. "YUV4MPEG2 W3840 H2160 F25:1 Ip A1:1 C420mpeg2\n"
//
// The interlace flag, aspect ratio and extensions are ignored - the model expects progressive
// frames in the signalled colour space.
//
bool ParseYUV4MPEG2HeaderLine(const std::string &line, unsigned *width, unsigned *height, ImageFormat *format,
                              unsigned *fps_num, unsigned *fps_den) {
	unsigned w = 0, h = 0, fn = 25, fd = 1;
	ImageFormat fmt = IMAGE_FORMAT_NONE;

	std::istringstream iss(line);
	std::string token;
	while (iss >> token) {
		if (token.size() < 2)
			continue;
		switch (token[0]) {
		case 'W':
			w = (unsigned)strtoul(token.c_str() + 1, nullptr, 10);
			break;
		case 'H':
			h = (unsigned)strtoul(token.c_str() + 1, nullptr, 10);
			break;
		case 'F': {
			const unsigned n = (unsigned)strtoul(token.c_str() + 1, nullptr, 10);
			const char *sep = strchr(token.c_str() + 1, ':');
			const unsigned d = sep ? (unsigned)strtoul(sep + 1, nullptr, 10) : 1;
			if (n && d) {
				fn = n;
				fd = d;
			}
			break;
		}
		case 'C':
			fmt = YUV4MPEG2ChromaFormat(token.substr(1));
			break;
		default:
			// I (interlace), A (aspect), X (extensions) - ignored
			break;
		}
	}

	if (!w || !h)
		return false;

	if (fmt == IMAGE_FORMAT_NONE)
		fmt = IMAGE_FORMAT_YUV420P8;

	if (width)
		*width = w;
	if (height)
		*height = h;
	if (format)
		*format = fmt;
	if (fps_num)
		*fps_num = fn;
	if (fps_den)
		*fps_den = fd;
	return true;
}

// Map a YUV4MPEG2 chroma parameter (e.g. "420mpeg2", "420p10", "422p12", "444p14", "mono10")
// to an ImageFormat
//
ImageFormat YUV4MPEG2ChromaFormat(const std::string &value) {
	const std::string s = lowercase(value);

	unsigned depth = 8;
	if (s.find("p16") != std::string::npos)
		depth = 16;
	else if (s.find("p14") != std::string::npos)
		depth = 14;
	else if (s.find("p12") != std::string::npos)
		depth = 12;
	else if (s.find("p10") != std::string::npos)
		depth = 10;

	unsigned subsampling = 0; // 0 = 420, 1 = 422, 2 = 444, 3 = mono
	if (s.compare(0, 4, "mono") == 0)
		subsampling = 3;
	else if (s.size() >= 3 && s[0] == '4' && s[1] == '4')
		subsampling = 2;
	else if (s.size() >= 3 && s[0] == '4' && s[2] == '2')
		subsampling = 1;

	switch (subsampling) {
	case 1:
		return depth == 10 ? IMAGE_FORMAT_YUV422P10 : depth == 12 ? IMAGE_FORMAT_YUV422P12 : depth == 14 ? IMAGE_FORMAT_YUV422P14
		                                                                      : depth == 16 ? IMAGE_FORMAT_YUV422P16
		                                                                                    : IMAGE_FORMAT_YUV422P8;
	case 2:
		return depth == 10 ? IMAGE_FORMAT_YUV444P10 : depth == 12 ? IMAGE_FORMAT_YUV444P12 : depth == 14 ? IMAGE_FORMAT_YUV444P14
		                                                                      : depth == 16 ? IMAGE_FORMAT_YUV444P16
		                                                                                    : IMAGE_FORMAT_YUV444P8;
	case 3:
		return depth == 10 ? IMAGE_FORMAT_Y10 : depth == 12 ? IMAGE_FORMAT_Y12 : depth == 14 ? IMAGE_FORMAT_Y14
		                                                       : depth == 16 ? IMAGE_FORMAT_Y16
		                                                                     : IMAGE_FORMAT_Y8;
	default:
		return depth == 10 ? IMAGE_FORMAT_YUV420P10 : depth == 12 ? IMAGE_FORMAT_YUV420P12 : depth == 14 ? IMAGE_FORMAT_YUV420P14
		                                                                      : depth == 16 ? IMAGE_FORMAT_YUV420P16
		                                                                                    : IMAGE_FORMAT_YUV420P8;
	}
}

// Open file for reading, format is inferred from filename
//
unique_ptr<YUVReader> CreateYUVReader(const string &name) {
	// Does it have a sensible name?
	float rate = 25.0f;
	ImageDescription description = ParseYUVFilename(name, &rate);
	if (description.format() == IMAGE_FORMAT_NONE)
		ERR("Cannot parse YUV filename");

	return CreateYUVReader(name, description, (unsigned)rate);
}

// Create Dummy Reader, will be filled later
//
unique_ptr<YUVReader> CreateYUVReader(const string &name, unsigned rate) {
	// Is file there?
	const uintmax_t fileSize = file_size(name);
	if (fileSize == static_cast<uintmax_t>(-1))
		return 0;

	UniquePtrFile yuvFile(std::fopen(name.c_str(), "rb"));

	if (yuvFile) {
		return unique_ptr<YUVReader>(new YUVReader(name, (float)rate, yuvFile.release(), fileSize));
	}

	ERR("Cannot open YUV file");
	return 0;
};

// Read from an existing stream (e.g. a pipe) - the stream is not owned and must be closed by
// the caller. The stream is read sequentially; 'length' is the number of frames available.
//
unique_ptr<YUVReader> CreateYUVReader(FILE *stream, const ImageDescription &description, unsigned length, unsigned rate) {
	return unique_ptr<YUVReader>(new YUVReader("<stream>", description, length, (float)rate, stream));
}

} // namespace lctm
