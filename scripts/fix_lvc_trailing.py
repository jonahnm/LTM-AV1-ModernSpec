#!/usr/bin/env python3
"""In-place fixer for LCEVC-in-AV1 .lvc files produced by the buggy encoder.

Older builds of the model wrote an extra trailing byte (0x80) at the end of the
AV1 metadata OBU that carries the V-Nova T.35 LCEVC payload. Conforming decoders
(ffmpeg's CBS/libliblcevc_dec) include that byte in the T.35 payload, making the
LCEVC NAL one byte too long so it is misread as an extra process block.

This tool walks the AV1 OBU stream and, for each ITU-T T.35 / V-Nova metadata OBU,
strips the trailing byte(s) so the embedded LCEVC NAL ends at its own RBSP stop bit,
and rewrites the OBU size field accordingly. It is safe on already-correct files
(it validates the LCEVC process-block framing before modifying anything).

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

    def new_size(self):
        return self.payload_size - self.fixed

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
    """Remove emulation-prevention bytes: a 0x03 inserted after 00 00 00/01/02/03
    is dropped (RBSP rule used by rbsp_encapsulate in the encoder and by FFmpeg's
    CBS / the in-tree rbsp_decoder)."""
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


def block_walk_ok(blocks, log=print):
    """Validate the LCEVC process-block framing: a byte-aligned sequence of
    [size_type(3)|payload_type(5)] headers, optional multibyte size, payload.
    The input is the RBSP-escaped block bytes as stored in the NAL; it is
    unescaped first. Returns True iff the blocks consume the buffer exactly."""
    blocks = rbsp_unescape(blocks)
    pos = 0
    n = len(blocks)
    k = 0
    while True:
        if pos >= n:
            # More blocks expected but buffer exhausted.
            return False
        b0 = blocks[pos]
        pos += 1
        size_type = b0 >> 5
        payload_type = b0 & 0x1F
        if payload_type > 6:
            return False
        if size_type == 6:
            return False
        if size_type == 7:
            size = 0
            while True:
                if pos >= n:
                    return False
                b = blocks[pos]
                pos += 1
                size = (size << 7) | (b & 0x7F)
                if not (b & 0x80):
                    break
        else:
            size = size_type
        if pos + size > n:
            return False
        pos += size
        k += 1
        if pos == n:
            return True
        if pos > n:
            return False


def needs_fix(data, obu, log):
    """Determine how many trailing bytes to strip for this metadata OBU.
    Returns the number of bytes to strip (>=1) or 0 if the OBU is already correct."""
    pl = obu.payload_offs
    payload = data[pl : pl + obu.payload_size]
    if obu.payload_size < 1:
        return 0
    # Must look like a V-Nova T.35 metadata OBU.
    if payload[0] != 0x04:
        return 0
    if payload[1 : 1 + len(K_VNOVA)] != K_VNOVA:
        return 0
    # NAL must start with a start code + valid LCEVC NAL header.
    if obu.payload_size < 10:
        return 0
    if payload[5:8] != b"\x00\x00\x01":
        return 0
    hdr = payload[8]
    if (hdr & 0x3E) >> 1 not in (28, 29):
        return 0

    # The embedded NAL is [start(3)][nal header(2)][RBSP-escaped block stream][0x80 stop].
    # Stripping K trailing bytes leaves the stop bit at payload[len-K-1]; the block
    # stream before it must parse to the exact end. The correct cut is the smallest
    # K>=0 that validates; K==0 means the OBU is already conformant.
    n = len(payload)
    best = None
    for K in range(0, 6):
        stop_idx = n - K - 1
        if stop_idx < 10:
            break
        if payload[stop_idx] != 0x80:
            continue
        blocks = payload[10:stop_idx]
        if block_walk_ok(blocks):
            best = K
            break
    if best is None or best == 0:
        return 0
    return best


def parse_obus(data):
    """Walk the AV1 OBU stream, returning (list_of_Obu, error)."""
    obus = []
    pos = 0
    while pos < len(data):
        header_byte = data[pos]
        obu_type = (header_byte >> 3) & 0x0F
        has_extension = (header_byte >> 2) & 0x01
        has_size = (header_byte >> 1) & 0x01
        if not has_size:
            return obus, True
        p = pos + 1
        if has_extension:
            p += 1
            if p >= len(data):
                return obus, True
        size, p, nbytes = read_leb128(data, p)
        if p + size > len(data):
            return obus, True
        obus.append(Obu(pos, header_byte, p - nbytes, p, int(size)))
        pos = p + size
    return obus, False


def process(path, do_write, backup):
    with open(path, "rb") as f:
        data = f.read()

    obus, err = parse_obus(data)
    if err:
        print(f"{path}: could not parse OBU stream (not a raw AV1 .lvc?)", file=sys.stderr)
        return 1

    n_meta = 0
    n_fixed_obus = 0
    total_bytes = 0
    changed = False
    for obu in obus:
        if obu.header_byte >> 3 & 0x0F == 5:
            n_meta += 1
            strip = needs_fix(data, obu, print)
            if strip:
                obu.fixed = strip
                n_fixed_obus += 1
                total_bytes += strip
                changed = True

    action = "check" if not do_write else "fix"
    print(f"{path}: {action}: {n_fixed_obus}/{n_meta} metadata OBUs, "
          f"{total_bytes} byte(s) of trailing data to remove")
    if not do_write:
        return 0 if changed else 0

    if not changed:
        print(f"{path}: nothing to fix")
        return 0

    # Rebuild the file: keep all bytes verbatim, but for fixed OBUs rewrite the
    # LEB128 size field and drop the stripped trailing payload bytes.
    out = bytearray()
    for obu in obus:
        if obu.fixed:
            out.append(obu.header_byte)
            if obu.header_byte & (1 << 2):  # has_extension
                out.append(data[obu.size_bytes - 1])
            out += write_leb128(obu.new_size())
            out += data[obu.payload_offs : obu.payload_offs + obu.new_size()]
        else:
            out += data[obu.start : obu.end]

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