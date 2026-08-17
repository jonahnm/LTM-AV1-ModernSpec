#!/usr/bin/env python3
"""In-place fixer for LCEVC-in-AV1 .lvc files produced by buggy encoder builds.

Two defects are repaired:

1. Missing OBU trailing byte: builds between cf1dd08 and a0e8d98 dropped the
   trailing 0x80 byte of the AV1 metadata OBU. libdav1d strips the final payload
   byte (the AV1 trailing one-bit byte) before handing the T.35 message to
   ffmpeg, so without it the embedded LCEVC NAL loses its RBSP stop bit and the
   decoder fails with "Invalid value at rbsp_stop_one_bit: bitstream ended".

2. Flag-only EncodedData blocks: when every residual surface quantises to zero
   the block contains only the per-layer entropy flags with no data after them.
   No parser family accepts such a block: ffmpeg's CBS requires at least one
   data byte after the flags, and the V-Nova SDK's config parser requires the
   block to be consumed exactly. The block is removed and the Picture config is
   rewritten to the "no enhancement" form, matching the encoder's output for
   empty frames.

Both repairs rebuild the metadata OBU's payload from the validated, RBSP-unescaped
process-block stream, then re-escape it and rewrite the OBU size field. Already
correct files are left untouched (validated before any modification) and the tool
is idempotent.

Usage:
    python3 scripts/fix_lvc_trailing.py file.lvc [file2.lvc ...]
    python3 scripts/fix_lvc_trailing.py --check file.lvc    # dry run, report only

Options:
    --check       Report what would change without writing.
    --backup      Write file.lvc.bak before modifying.
"""

import os
import shutil
import sys


class Obu:
    __slots__ = ("start", "header_byte", "size_bytes", "payload_offs", "payload_size", "fixed")

    def __init__(self, start, header_byte, size_bytes, payload_offs, payload_size):
        self.start = start
        self.header_byte = header_byte
        self.size_bytes = size_bytes
        self.payload_offs = payload_offs
        self.payload_size = payload_size
        self.fixed = 0  # number of trailing bytes stripped (0 if untouched)

    @property
    def end(self):
        return self.payload_offs + self.payload_size


K_VNOVA = b"\xb4\x00\x50\x00"


def read_leb128(data, pos):
    value = 0
    shift = 0
    nbytes = 0
    while pos < len(data):
        b = data[pos]
        pos += 1
        nbytes += 1
        value |= (b & 0x7F) << shift
        shift += 7
        if not (b & 0x80):
            break
    return value, pos, nbytes


def write_leb128(value):
    out = bytearray()
    while True:
        b = value & 0x7F
        value >>= 7
        if value:
            b |= 0x80
        out.append(b)
        if not value:
            break
    return bytes(out)


def rbsp_unescape(data):
    """Remove RBSP emulation-prevention bytes (0x03 inserted after 00 00 00/01/02/03)."""
    out = bytearray()
    zeros = 0
    for b in data:
        if zeros == 2 and b == 0x03:
            zeros = 0
            continue
        if b == 0:
            zeros += 1
        else:
            zeros = 0
        out.append(b)
    return bytes(out)


def rbsp_escape(data):
    """Insert emulation-prevention bytes exactly as the encoder's rbsp_encapsulate does."""
    out = bytearray()
    zeros = 0
    for b in data:
        if zeros == 2 and (b & ~3) == 0:
            out.append(0x03)
            zeros = 0
        if b == 0:
            zeros += 1
        else:
            zeros = 0
        out.append(b)
    return bytes(out)


def bit_of(data, i):
    return (data[i >> 3] >> (7 - (i & 7))) & 1


def read_mb(blocks, pos):
    """Read a big-endian multibyte size; returns (value, next_pos) or (None, pos)."""
    value = 0
    while pos < len(blocks):
        b = blocks[pos]
        pos += 1
        value = (value << 7) | (b & 0x7F)
        if not (b & 0x80):
            return value, pos
    return None, pos


def parse_blocks(blocks):
    """Parse the byte-aligned LCEVC process blocks.

    Yields (content_off, ptype, content). content_off is the offset of the block
    payload in the unescaped stream. Parsing stops silently at the first error.
    """
    pos = 0
    n = len(blocks)
    while True:
        if pos >= n:
            return
        b0 = blocks[pos]
        size_type = b0 >> 5
        ptype = b0 & 0x1F
        if size_type == 6 or ptype > 6:
            return
        pos += 1
        if size_type == 7:
            size, pos = read_mb(blocks, pos)
            if size is None:
                return
        else:
            size = size_type
        if pos + size > n:
            return
        yield pos, ptype, blocks[pos : pos + size]
        pos += size


def block_walk_ok(raw):
    """True iff the RBSP-escaped block stream parses exactly to its end after unescaping."""
    blocks = rbsp_unescape(raw)
    pos = 0
    n = len(blocks)
    while pos < n:
        b0 = blocks[pos]
        size_type = b0 >> 5
        ptype = b0 & 0x1F
        if size_type == 6 or ptype > 6:
            return False
        pos += 1
        if size_type == 7:
            size, pos = read_mb(blocks, pos)
            if size is None:
                return False
        else:
            size = size_type
        if pos + size > n:
            return False
        pos += size
    return True


def global_flag_shape(content):
    """From a Global config block: (nplanes, nlayers, temporal_enabled)."""
    if len(content) * 8 < 18:
        return None
    pp_type = bit_of(content, 0)
    transform_type = bit_of(content, 7)
    temporal_enabled = bit_of(content, 17)
    nplanes = 3 if pp_type else 1
    nlayers = 16 if transform_type else 4
    return (nplanes, nlayers, temporal_enabled)


def picture_flag_shape(content, temporal_enabled):
    """From a Picture config block: (enhancement_enabled, temporal_signalling_present)."""
    if len(content) * 8 < 7:
        return None
    no_enhancement = bit_of(content, 0)
    temporal_refresh = bit_of(content, 6)
    temporal_signalling = temporal_enabled and not temporal_refresh
    return (no_enhancement == 0), temporal_signalling


def flag_bytes(nplanes, nlayers, enhancement_enabled, temporal_signalling):
    """Size in bytes of the entropy-flag section of an EncodedData block."""
    bits = 0
    for _ in range(nplanes):
        if enhancement_enabled:
            bits += 2 * 2 * nlayers
        if temporal_signalling:
            bits += 2
    return (bits + 7) // 8


def write_mb(value):
    """Big-endian multibyte size, same encoding as the serializer: first byte is the
    most significant group, 'more' bit set on all but the final byte."""
    parts = []
    while True:
        parts.append(value & 0x7F)
        value >>= 7
        if not value:
            break
    out = bytearray()
    for i, p in enumerate(reversed(parts)):
        out.append((0x80 if i < len(parts) - 1 else 0x00) | p)
    return bytes(out)


def rewrite_picture_config(content):
    """Rewrite an enhancement-enabled Picture config block (24-bit header + misc)
    into the no-enhancement form (single byte: no_enhancement=1, reserved, picture
    type, temporal refresh, temporal signalling cleared). Called when the frame's
    EncodedData block is removed, matching the encoder's output for empty frames."""
    if len(content) < 3:
        return None
    no_enhancement = bit_of(content, 0)
    picture_type = bit_of(content, 5)
    temporal_refresh = bit_of(content, 6)
    if no_enhancement:
        return None  # already no-enhancement
    return bytes([(1 << 7) | (picture_type << 2) | (temporal_refresh << 1)])


def reemit_blocks(blocks, remove_at):
    """Re-serialise the unescaped process-block stream, dropping any block whose
    content offset+length is in remove_at (flag-only EncodedData blocks, which no
    parser family accepts) and rewriting the Picture config to the no-enhancement
    form. Block size fields are recomputed accordingly."""
    out = bytearray()
    for content_off, ptype, content in parse_blocks(blocks):
        if content_off + len(content) in remove_at:
            continue
        if ptype == 2:
            new_pic = rewrite_picture_config(content)
            if new_pic is not None:
                content = new_pic
        size = len(content)
        size_type = size if size <= 5 else 7
        out.append((size_type << 5) | ptype)
        if size_type == 7:
            out += write_mb(size)
        out += content
    return bytes(out)


def analyze_obu(payload, state):
    """Analyse one V-Nova T.35 metadata OBU payload.

    Returns (has_trailing, removes, blocks, new_state):
      has_trailing - True if the OBU already carries the final 0x80 byte. The AV1
                     metadata OBU must terminate with trailing bits: libdav1d
                     strips the last payload byte (the trailing one-bit byte)
                     before handing the T.35 message to ffmpeg, so without it
                     the embedded LCEVC NAL loses its RBSP stop bit and fails
                     with "rbsp_stop_one_bit: bitstream ended".
      removes      - set of unescaped block-stream offsets of flag-only
                     EncodedData blocks (flags with no data), which ffmpeg's
                     CBS rejects ("no data after flags") and the V-Nova SDK
                     rejects via its exact block-size check. They are removed
                     and the Picture config rewritten to "no enhancement".
      blocks       - unescaped block stream (for rebuilding), or None if untouched
      new_state    - updated global-config flag shape ((nplanes, nlayers, te) or None)
    """
    n = len(payload)
    if n < 10:
        return False, set(), None, state
    if payload[0] != 0x04 or payload[1 : 5] != K_VNOVA:
        return False, set(), None, state
    if payload[5:8] != b"\x00\x00\x01":
        return False, set(), None, state
    if (payload[8] & 0x3E) >> 1 not in (28, 29):
        return False, set(), None, state

    # 1) Find the block region. Two layouts exist in the wild:
    #    - spec-conformant: [blocks][NAL RBSP stop 0x80][OBU trailing 0x80]
    #    - broken (cf1dd08/a0e8d98-era encoder): [blocks][NAL RBSP stop 0x80]
    has_trailing = False
    blocks_raw = None
    if n >= 11 and payload[-1] == 0x80 and payload[-2] == 0x80 and block_walk_ok(payload[10:-2]):
        blocks_raw = payload[10:-2]
        has_trailing = True
    elif payload[-1] == 0x80 and block_walk_ok(payload[10:-1]):
        blocks_raw = payload[10:-1]
    if blocks_raw is None:
        return False, set(), None, state

    # 2) Walk the blocks: track configs and locate flag-only EncodedData blocks.
    blocks = rbsp_unescape(blocks_raw)
    shape = state
    pic_shape = None
    removes = set()  # content offsets of flag-only EncodedData blocks
    for content_off, ptype, content in parse_blocks(blocks):
        if ptype == 1:
            s = global_flag_shape(content)
            if s:
                shape = s
        elif ptype == 2 and shape:
            s = picture_flag_shape(content, shape[2])
            if s:
                pic_shape = s
        elif ptype in (3, 4) and shape and pic_shape:
            fb = flag_bytes(shape[0], shape[1], pic_shape[0], pic_shape[1])
            # Flag-only blocks appear either as exactly the flag section or (from
            # the a0e8d98-era encoder) with a single 0x01 padding byte appended.
            if len(content) == fb or (len(content) == fb + 1 and content[-1] == 0x01):
                removes.add(content_off + len(content))

    return has_trailing, removes, blocks, shape


def needs_fix(data, obu, state):
    pl = data[obu.payload_offs : obu.payload_offs + obu.payload_size]
    return analyze_obu(pl, state)


def parse_obus(data):
    """Walk the AV1 OBU stream, returning (list_of_Obu, error)."""
    obus = []
    pos = 0
    while pos < len(data):
        header_byte = data[pos]
        has_extension = (header_byte >> 2) & 0x01
        has_size = (header_byte >> 1) & 0x01
        if not has_size:
            return obus, True
        p = pos + 1
        if has_extension:
            p += 1
            if p >= len(data):
                return obus, True
        size, p, _ = read_leb128(data, p)
        if p + size > len(data):
            return obus, True
        obus.append(Obu(pos, header_byte, p, p, int(size)))
        pos = p + size
    return obus, False


def process(path, do_write, backup):
    with open(path, "rb") as f:
        data = f.read()

    obus, err = parse_obus(data)
    if err:
        print(f"{path}: could not parse OBU stream (not a raw AV1 .lvc?)", file=sys.stderr)
        return 1

    # Per-OBU edits: (has_trailing, removes, blocks)
    edits = {}
    state = None
    n_meta = 0
    for obu in obus:
        if (obu.header_byte >> 3) & 0x0F != 5:
            continue
        n_meta += 1
        pl = data[obu.payload_offs : obu.payload_offs + obu.payload_size]
        has_trailing, removes, blocks, state = analyze_obu(pl, state)
        if not has_trailing or removes:
            edits[id(obu)] = (has_trailing, removes, blocks)
            obu.fixed = len(removes) + (0 if has_trailing else 1)

    total = sum(len(rm) + (0 if has else 1) for has, rm, _ in edits.values())
    action = "check" if not do_write else "fix"
    print(f"{path}: {action}: {len(edits)}/{n_meta} metadata OBUs, {total} byte(s) of repairs")
    if not do_write:
        return 0
    if not edits:
        print(f"{path}: nothing to fix")
        return 0

    # Rebuild the file: verbatim except for edited metadata OBUs, whose payload is
    # rebuilt from the unescaped block stream (flag-only blocks dropped, Picture
    # config rewritten), re-escaped, with the NAL RBSP stop bit and the OBU
    # trailing byte (if missing) re-appended.
    out = bytearray()
    for obu in obus:
        if id(obu) not in edits:
            out += data[obu.start : obu.end]
            continue
        has_trailing, removes, blocks = edits[id(obu)]
        blocks = reemit_blocks(blocks, removes)
        escaped = rbsp_escape(blocks)
        new_payload = bytearray(data[obu.payload_offs : obu.payload_offs + 10])
        new_payload += escaped + b"\x80"
        if has_trailing:
            new_payload += b"\x80"
        out.append(obu.header_byte)
        if obu.header_byte & (1 << 2):
            out.append(data[obu.size_bytes - 1])
        out += write_leb128(len(new_payload))
        out += new_payload

    if backup:
        shutil.copyfile(path, path + ".bak")

    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(bytes(out))
    os.replace(tmp, path)
    print(f"{path}: wrote {len(out)} bytes (was {len(data)})")
    return 0


def main(argv):
    if not argv:
        print(__doc__, file=sys.stderr)
        return 1
    do_write = True
    backup = False
    files = []
    for a in argv:
        if a == "--check":
            do_write = False
        elif a == "--backup":
            backup = True
        else:
            files.append(a)
    rc = 0
    for f in files:
        rc |= process(f, do_write, backup)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
