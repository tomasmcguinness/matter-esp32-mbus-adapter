#!/usr/bin/env python3
"""Align a corrupted M-Bus telegram against what the simulator actually sent.

The master's hexdump alone cannot tell you whether a short frame means the slave
stopped transmitting or the master stopped recognising where characters begin.
This does: it rebuilds the transmitted bit stream and finds, for each received
byte, where in that stream it was sampled from.

The bus only ever fails one way - a space (current sinking, a 0 bit) sags back
to mark, never the reverse - so a received byte is feasible at bit offset p only
if every sent 1 bit in the window came back as a 1. Sent 0s may read as either.
That constraint is tight enough that the alignment is essentially unique.

Usage:
    ./mbus_align.py --fill 11 < dump.txt
    pbpaste | ./mbus_align.py --fill 55

where dump.txt is pasted straight from the console, log prefixes and all:

    I (2421809) MBus: RX frame, 39 of 63 byte(s):
    I (2421809) MBus: 0x4081ae7c   68 39 39 68 ...  |h99h..rxV4.-,4..|

or from the simulator's own console, where the HAT loopback is reported the
same way (--group 1 picks the echo line, since 'tx:' comes first):

    tx:   68 39 39 68 08 05 ...
    echo: 68 39 39 68 08 05 ...

Only fill-byte telegrams can be reconstructed ('p' or 'fHH' on the simulator
console). With real meter values the master cannot know what was sent.
"""

import argparse
import re
import sys

# Record table from tools/kamstrup-403-sim/kamstrup-403-sim.ino, in the order
# encode_records() emits them: (DIF, VIF, payload bytes).
RECORDS = [(0x04, 0x06, 4), (0x04, 0x14, 4), (0x04, 0x22, 4), (0x04, 0x2D, 4),
           (0x04, 0x3B, 4), (0x02, 0x59, 2), (0x02, 0x5D, 2), (0x02, 0x61, 2)]

HEADER_FIXED = [0x08, 0x05, 0x72,              # C A CI
                0x78, 0x56, 0x34, 0x12,        # id, BCD LSB first
                0x2D, 0x2C,                    # manufacturer
                0x34, 0x04]                    # version, medium
ACCESS_INDEX = 15                              # access number's index in the frame

DUMP_LINE = re.compile(r'0x([0-9a-fA-F]{8})\s+((?:[0-9a-fA-F]{2}\s+)+)')
HEX_BYTE = re.compile(r'^[0-9a-fA-F]{2}$')
MIN_RUN = 4          # shortest run of hex pairs taken as data rather than prose


def _hex_run(line):
    """Longest run of whitespace-separated hex pairs on a line, as bytes."""
    best, run = [], []
    for tok in line.split():
        if HEX_BYTE.match(tok):
            run.append(int(tok, 16))
            if len(run) > len(best):
                best = list(run)
        else:
            run = []
    return best if len(best) >= MIN_RUN else []


def parse_dump(text, group):
    """Pull hex bytes out of a console capture.

    Two shapes turn up. ESP_LOG_BUFFER_HEXDUMP wraps a frame over several lines
    with an address on each, so those are grouped by address running
    contiguously - one capture usually holds the raw frame and then the user
    block. The simulator instead prints a whole frame per line ('tx:', 'echo:'),
    so in that shape each line is its own group.
    """
    groups, addr_end = [], None
    for line in text.splitlines():
        body = line.split('|')[0]
        m = DUMP_LINE.search(body)
        if m:
            addr = int(m.group(1), 16)
            data = [int(b, 16) for b in m.group(2).split()]
            if addr_end is None or addr != addr_end:
                groups.append([])
            groups[-1] += data
            addr_end = addr + len(data)
            continue
        addr_end = None
        data = _hex_run(body)
        if data:
            groups.append(data)
    if not groups:
        sys.exit("no hex bytes found on stdin")
    if group >= len(groups):
        sys.exit(f"only {len(groups)} dump(s) found, asked for #{group}")
    return groups[group]


def build_expected(l_field, fill, access):
    """Rebuild the telegram the simulator built for this L and fill byte."""
    records_len = l_field - (3 + 12)
    if records_len < 0:
        sys.exit(f"L=0x{l_field:02x} is too short to hold a 12-byte header")
    body, used = [], 0
    for dif, vif, n in RECORDS:
        if used == records_len:
            break
        body += [dif, vif] + [fill] * n
        used += 2 + n
    if used != records_len:
        sys.exit(f"L=0x{l_field:02x} implies {records_len} record bytes, "
                 f"which no whole number of records produces")
    body = HEADER_FIXED + [access] + [fill] * 3 + body
    return [0x68, l_field, l_field, 0x68] + body + [sum(body) & 0xFF, 0x16]


def char_bits(b):
    """One 8E1 character in transmission order: start, d0..d7, even parity, stop."""
    return [0] + [(b >> i) & 1 for i in range(8)] + [bin(b).count('1') % 2, 1]


def align(sent, recv):
    stream = []
    for b in sent:
        stream += char_bits(b)
    stream += [1] * 40                       # trailing idle mark

    def feasible(p, r):
        if p + 10 >= len(stream) or stream[p] != 0:
            return False                     # no start bit to trigger on
        return all(not (stream[p + 1 + i] == 1 and not ((r >> i) & 1))
                   for i in range(8))

    out, cur = [], 0
    for r in recv:
        p = cur
        while p < len(stream) and not feasible(p, r):
            p += 1
        if p >= len(stream):
            out.append((r, None, None, None))
            continue
        flips = [i for i in range(8)
                 if ((r >> i) & 1) and stream[p + 1 + i] == 0]
        out.append((r, p // 11, p % 11, flips))
        cur = p + 11
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--fill', required=True,
                    help="fill byte the simulator was set to, e.g. 11 or 0x11")
    ap.add_argument('--access', default=None,
                    help="access number, if the received one at index 15 is suspect")
    ap.add_argument('--group', type=int, default=0,
                    help="which hexdump in the capture to use (default: the first)")
    args = ap.parse_args()

    fill = int(args.fill, 16)
    recv = parse_dump(sys.stdin.read(), args.group)
    if len(recv) < ACCESS_INDEX + 1:
        sys.exit(f"only {len(recv)} bytes received - too few to reconstruct")
    if recv[0] != 0x68:
        sys.exit(f"dump does not start with 0x68 (got 0x{recv[0]:02x})")

    access = int(args.access, 16) if args.access else recv[ACCESS_INDEX]
    sent = build_expected(recv[1], fill, access)
    print(f"fill 0x{fill:02x}, access 0x{access:02x}: "
          f"{len(sent)} bytes sent, {len(recv)} received\n")

    rows = align(sent, recv)
    print(f"{'rx#':>4} {'byte':>4}  {'from sent idx':>13} {'phase':>6}  note")
    prev, swallowed, slips, unexplained = -1, 0, 0, 0
    for k, (r, src, phase, flips) in enumerate(rows):
        if src is None:
            unexplained += 1
            print(f"{k:4d}    {r:02x}  {'unexplained':>13}")
            continue
        notes = []
        gap = src - prev - 1
        if gap > 0:
            swallowed += gap
            notes.append(f"{gap} character(s) swallowed")
        if phase:
            slips += 1
            notes.append(f"PHASE SLIP +{phase} bits")
        elif flips:
            notes.append("bits " + ",".join(f"d{i}" for i in flips) + " sagged 0->1")
        print(f"{k:4d}    {r:02x}  {src:13d} {phase:6d}  {'; '.join(notes)}")
        prev = src

    tail = rows[-1][1]
    print(f"\n{unexplained} byte(s) not explainable as space-to-mark sag"
          f"{' - the model does not hold here' if unexplained else ''}")
    print(f"{swallowed + max(0, len(sent) - 1 - (tail if tail is not None else -1))}"
          f" sent character(s) produced no received character")
    print(f"{slips} byte(s) sampled out of character alignment")
    if tail is not None and tail < len(sent) - 1:
        print(f"\nThe last byte received came from sent index {tail}, not {len(sent)-1}. "
              f"The slave transmitted\nthe remaining {len(sent)-1-tail} character(s); "
              f"the master stopped seeing character boundaries.")


if __name__ == '__main__':
    main()
