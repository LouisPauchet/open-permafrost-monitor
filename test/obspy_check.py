"""Cross-checks the firmware's Steim2 MiniSEED encoder with ObsPy.

Run after `pio test -e native` (which writes the fixture files into the
project directory):

    pip install obspy
    python test/obspy_check.py
"""
import sys

from obspy import read

st = read("steim2_fixture.mseed")
expected = [int(line) for line in open("steim2_fixture.txt")]

st.merge()  # contiguous records must merge into one gap-free trace
assert len(st) == 1, f"expected 1 merged trace, got {len(st)}: {st}"
tr = st[0]
assert tr.stats.sampling_rate == 200.0, tr.stats.sampling_rate
assert str(tr.stats.starttime) == "2026-10-27T12:00:00.000000Z", tr.stats.starttime
assert tr.stats.mseed.encoding == "STEIM2", tr.stats.mseed.encoding
assert tr.stats.npts == len(expected), (tr.stats.npts, len(expected))
mismatch = [i for i, (a, b) in enumerate(zip(tr.data, expected)) if int(a) != b]
assert not mismatch, f"{len(mismatch)} samples differ, first at {mismatch[0]}"
print(f"ObsPy OK: {tr.id} {tr.stats.npts} samples, {tr.stats.sampling_rate} Hz, lossless")
sys.exit(0)
