#!/usr/bin/env python3
"""Checks src/horus/horus_v3.cpp against the ASN.1 definition, bit for bit.

  c++ -O1 -o /tmp/horus_v3_cli tools/horus_v3_cli.cpp src/horus/horus_v3.cpp
  pip install asn1tools
  python3 tools/test_horus_v3.py /tmp/horus_v3_cli path/to/horusdemodlib/horusbinaryv3/HorusBinaryV3.asn1

Each random packet is encoded by the C++ code and by asn1tools (UPER); the frames must be
identical (CRC16 first, UPER bytes, zero padding to the frame size), and the C++ output must decode
back to the same values with constraint checking on. Packets without a note use a 32-byte frame
(longer ones must be refused); packets with a note use the smallest of 32, 48 or 64 bytes that fits.
"""
import binascii, random, subprocess, sys
import asn1tools

cli, asn = sys.argv[1], sys.argv[2]
codec = asn1tools.compile_files(asn, "uper")
ALPHABET = "-/0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
rnd = random.Random(1234)

NOTE_CHARS = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 _+/=-."

def frame_from_asn(d, sizes):
    """(frame, size) for the smallest size in `sizes` that fits, or (None, last size)."""
    uper = codec.encode("Telemetry", d)
    for size in sizes:
        if len(uper) <= size - 2:
            body = uper + bytes(size - 2 - len(uper))       # pad after the 2-byte CRC
            return binascii.crc_hqx(body, 0xFFFF).to_bytes(2, "little") + body, size
    return None, sizes[-1]

def case():
    d = dict(payloadCallsign="".join(rnd.choice(ALPHABET) for _ in range(rnd.randint(1, 15))),
             sequenceNumber=rnd.randint(0, 65535), timeOfDaySeconds=rnd.randint(-1, 86400),
             latitude=rnd.randint(-9000000, 9000000), longitude=rnd.randint(-18000000, 18000000),
             altitudeMeters=rnd.randint(-1000, 50000))
    if rnd.random() < .6: d["velocityHorizontalKilometersPerHour"] = rnd.randint(0, 512)
    if rnd.random() < .6: d["gnssSatellitesVisible"] = rnd.randint(0, 31)
    if rnd.random() < .5: d["ascentRateCentimetersPerSecond"] = rnd.randint(-32767, 32767)
    if rnd.random() < .5: d["temperatureCelsius-x10"] = {"internal": rnd.randint(-1023, 1023)}
    if rnd.random() < .5: d["milliVolts"] = {"battery": rnd.randint(0, 16383)}
    if rnd.random() < .4:
        n = rnd.randint(1, 36)
        d["extraSensors"] = [{"name": "note", "values": ("horusStr", "".join(rnd.choice(NOTE_CHARS) for _ in range(n)))}]
    return d

def args(d, size):
    g = lambda k, f=str: "-" if k not in d else f(d[k])
    return [d["payloadCallsign"], str(d["sequenceNumber"]), str(d["timeOfDaySeconds"]),
            str(d["latitude"]), str(d["longitude"]), str(d["altitudeMeters"]),
            g("velocityHorizontalKilometersPerHour"), g("gnssSatellitesVisible"),
            g("ascentRateCentimetersPerSecond"),
            "-" if "temperatureCelsius-x10" not in d else str(d["temperatureCelsius-x10"]["internal"]),
            "-" if "milliVolts" not in d else str(d["milliVolts"]["battery"])] + (
            [d["extraSensors"][0]["values"][1], str(size)] if "extraSensors" in d else [])

bad = 0; toolong = 0; N = int(sys.argv[3]) if len(sys.argv) > 3 else 2000
for i in range(N):
    d = case()
    sizes = [32, 48, 64] if "extraSensors" in d else [32]
    try: want, size = frame_from_asn(d, sizes)
    except Exception as e: print("asn1 could not encode", d, e); bad += 1; continue
    out = subprocess.run([cli] + args(d, size), capture_output=True, text=True).stdout.strip()
    if want is None:                                  # too long: the encoder must refuse
        toolong += 1
        if not out.startswith("ERR"): print("should have refused", d); bad += 1
        continue
    got = bytes.fromhex(out)
    if got != want:
        bad += 1
        if bad <= 5: print("MISMATCH", d, "\n  c++ ", got.hex().upper(), "\n  asn1", want.hex().upper())
        continue
    back = codec.decode("Telemetry", got[2:], check_constraints=True)
    for k, v in d.items():
        if k == "extraSensors":
            if back.get(k) != [{"name": "note", "values": ("horusStr", v[0]["values"][1])}]: print("ROUNDTRIP note", v, back.get(k)); bad += 1; break
        elif back.get(k) != v: print("ROUNDTRIP", k, v, back.get(k)); bad += 1; break
print("%d packets (%d too long for their frame and correctly refused), %d failures" % (N, toolong, bad)); sys.exit(1 if bad else 0)
