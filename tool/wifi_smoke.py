#!/usr/bin/env python3
"""TCP smoke test for the QMC5883P logger.

Join the SoftAP ProtoMag (passphrase protomag1) before you run this script.
The script does not change the host Wi-Fi network.

The default run checks 200 Hz, gain 8 G, chip id 80, and fft_n 128.
The config command also sends range_ga 8.
Use --rate 1500 --range 30 for the fast check. The device snaps the rate to
200 or 1500, and the range to 2, 8, 12, or 30.
"""

import socket
import struct
import sys

HOST = "192.168.4.1"
PORT = 3333
BINS = 64


def put_type(major, value):
    prefix = major << 5
    if value < 24:
        return bytes([prefix | value])
    if value <= 0xFF:
        return bytes([prefix | 24, value])
    if value <= 0xFFFF:
        return bytes([prefix | 25]) + struct.pack(">H", value)
    return bytes([prefix | 26]) + struct.pack(">I", value)


def put_text(text):
    raw = text.encode("utf-8")
    return put_type(3, len(raw)) + raw


def put_uint(value):
    return put_type(0, value)


def encode_config(rate_hz, range_ga):
    # Map of 5 pairs: cmd, mode, sensors, rate_hz, range_ga.
    body = b"".join([
        put_text("cmd"),
        put_text("config"),
        put_text("mode"),
        put_text("single"),
        put_text("sensors"),
        put_type(4, 1) + put_text("mag"),
        put_text("rate_hz"),
        put_uint(rate_hz),
        put_text("range_ga"),
        put_uint(range_ga),
    ])
    return put_type(5, 5) + body


def encode_verb(verb):
    body = put_text("cmd") + put_text(verb)
    return put_type(5, 1) + body


class Reader:
    def __init__(self, data):
        self.data = data
        self.i = 0

    def pull(self):
        if self.i >= len(self.data):
            raise ValueError("truncated cbor")
        value = self.data[self.i]
        self.i += 1
        return value

    def arg(self, ai):
        if ai < 24:
            return ai
        if ai == 24:
            return self.pull()
        if ai == 25:
            return struct.unpack(">H", self.take(2))[0]
        if ai == 26:
            return struct.unpack(">I", self.take(4))[0]
        if ai == 27:
            return struct.unpack(">Q", self.take(8))[0]
        raise ValueError("unsupported cbor arg")

    def take(self, n):
        end = self.i + n
        if end > len(self.data):
            raise ValueError("truncated payload")
        chunk = self.data[self.i:end]
        self.i = end
        return chunk

    def item(self):
        type_byte = self.pull()
        major = type_byte >> 5
        ai = type_byte & 0x1F
        if major == 0:
            return self.arg(ai)
        if major == 1:
            return -1 - self.arg(ai)
        if major == 3:
            return self.take(self.arg(ai)).decode("utf-8")
        if major == 4:
            count = self.arg(ai)
            return [self.item() for _ in range(count)]
        if major == 5:
            count = self.arg(ai)
            out = {}
            for _ in range(count):
                key = self.item()
                out[key] = self.item()
            return out
        if major == 7 and ai == 20:
            return False
        if major == 7 and ai == 21:
            return True
        if major == 7 and ai == 26:
            return struct.unpack(">f", self.take(4))[0]
        if major == 7 and ai == 27:
            return struct.unpack(">d", self.take(8))[0]
        raise ValueError("unsupported cbor major %s" % major)


def frame(payload):
    return struct.pack(">I", len(payload)) + payload


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return buf


def recv_map(sock):
    hdr = recv_exact(sock, 4)
    length = struct.unpack(">I", hdr)[0]
    if length == 0 or length > 65536:
        raise ValueError("bad frame length %s" % length)
    payload = recv_exact(sock, length)
    value = Reader(payload).item()
    if not isinstance(value, dict):
        raise ValueError("frame is not a map")
    return value


def fail(message):
    print("FAIL:", message)
    return 1


def as_float(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("not a number: %r" % (value,))
    return float(value)


def snap_rate(rate):
    if abs(rate - 1500) < abs(rate - 200):
        return 1500
    return 200


def snap_range(ga):
    choices = (2, 8, 12, 30)
    best = choices[0]
    best_delta = abs(ga - best)
    for item in choices[1:]:
        delta = abs(ga - item)
        if delta < best_delta:
            best = item
            best_delta = delta
    return best


def parse_args(argv):
    host = HOST
    port = PORT
    rate = 200
    gain = 8
    positionals = []
    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg == "--rate":
            i += 1
            if i >= len(argv):
                raise ValueError("--rate needs a number")
            rate = int(argv[i])
        elif arg == "--range":
            i += 1
            if i >= len(argv):
                raise ValueError("--range needs a number")
            gain = int(argv[i])
        elif arg in ("-h", "--help"):
            print("usage: wifi_smoke.py [host] [port] [--rate 200|1500] [--range G]")
            return None
        else:
            positionals.append(arg)
        i += 1
    if len(positionals) >= 1:
        host = positionals[0]
    if len(positionals) >= 2:
        port = int(positionals[1])
    return host, port, rate, gain


def main():
    parsed = parse_args(sys.argv)
    if parsed is None:
        return 0
    host, port, rate, gain = parsed
    expect_rate = snap_rate(rate)
    expect_gain = snap_range(gain)
    sock = socket.create_connection((host, port), timeout=5)
    sock.settimeout(15)
    try:
        sock.sendall(frame(encode_config(rate, gain)))
        reply = recv_map(sock)
        if reply.get("rsp") != "config":
            return fail("rsp is %r" % reply.get("rsp"))
        if reply.get("ok") is not True:
            return fail("config ok is false: %r" % reply)
        if reply.get("chip_id") != "80":
            return fail("chip_id is %r" % reply.get("chip_id"))
        if as_float(reply.get("odr_hz")) != float(expect_rate):
            return fail("odr_hz is %r" % reply.get("odr_hz"))
        if as_float(reply.get("gain_ga")) != float(expect_gain):
            return fail("gain_ga is %r" % reply.get("gain_ga"))
        if int(reply.get("fft_n")) != 128:
            return fail("fft_n is %r" % reply.get("fft_n"))
        if expect_rate == 200 and expect_gain == 8:
            print("config ok odr_hz=200 gain_ga=8 fft_n=128 chip_id=80")
        else:
            print("config ok odr_hz=%s gain_ga=%s fft_n=128 chip_id=80" % (
                expect_rate, expect_gain))

        sock.sendall(frame(encode_verb("start")))
        saw_mag = False
        saw_fft = False
        while not (saw_mag and saw_fft):
            msg = recv_map(sock)
            sensor = msg.get("sensor")
            if sensor == "mag":
                for key in ("x", "y", "z"):
                    as_float(msg[key])
                if msg.get("unit") != "uT":
                    return fail("mag unit is %r" % msg.get("unit"))
                saw_mag = True
                print("mag x=%s y=%s z=%s" % (msg["x"], msg["y"], msg["z"]))
            elif sensor == "mag_fft":
                for key in ("mx", "my", "mz", "mabs"):
                    bins = msg.get(key)
                    if not isinstance(bins, list) or len(bins) != BINS:
                        return fail("%s len %s" % (key, None if bins is None else len(bins)))
                df_hz = as_float(msg.get("df_hz"))
                expect_df = float(expect_rate) / 128.0
                if abs(df_hz - expect_df) > 0.001:
                    return fail("df_hz is %r" % msg.get("df_hz"))
                saw_fft = True
                print("mag_fft bins=%d df_hz=%s" % (BINS, msg.get("df_hz")))
        sock.sendall(frame(encode_verb("stop")))
        print("stop sent")
    finally:
        sock.close()
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
