"""Send LispBM REPL commands to a VESC-Express board over USB and print its Lisp output.
Usage: python tools/vesc_repl.py COM34 "(master-charger-status)" [--listen SECONDS]
VESC Tool must not hold the port. DTR/RTS stay low so the ESP32 is not reset.
"""
import argparse, struct, sys, time
import serial

COMM_LISP_PRINT = 135
COMM_LISP_REPL_CMD = 138


def crc16(d):
    c = 0
    for x in d:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def frame(payload):
    head = bytes([2, len(payload)]) if len(payload) < 256 else bytes([3]) + struct.pack(">H", len(payload))
    return head + payload + struct.pack(">H", crc16(payload)) + b"\x03"


def packets(buf):
    """Yield (payload, rest) for every complete packet at the start of buf."""
    while True:
        i = next((k for k, b in enumerate(buf) if b in (2, 3)), None)
        if i is None:
            return b""
        buf = buf[i:]
        hl = 2 if buf[0] == 2 else 3
        if len(buf) < hl:
            return buf
        n = buf[1] if hl == 2 else struct.unpack(">H", buf[1:3])[0]
        if len(buf) < hl + n + 3:
            return buf
        p = buf[hl:hl + n]
        if buf[hl + n + 2] == 3 and struct.unpack(">H", buf[hl + n:hl + n + 2])[0] == crc16(p):
            yield p
            buf = buf[hl + n + 3:]
        else:
            buf = buf[1:]


def open_port(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, 115200, 0.05, False, False
    s.open()
    return s


def run(s, cmd, listen):
    s.write(frame(bytes([COMM_LISP_REPL_CMD]) + cmd.encode() + b"\0"))
    buf, end, out = b"", time.time() + listen, []
    while time.time() < end:
        buf += s.read(4096)
        rest = b""
        gen = packets(buf)
        try:
            while True:
                p = next(gen)
                if p and p[0] == COMM_LISP_PRINT:
                    line = p[1:].decode("utf8", "replace").rstrip("\0")
                    out.append(line)
                    print(line, flush=True)
        except StopIteration as e:
            rest = e.value or b""
        buf = rest
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("cmd", nargs="+")
    ap.add_argument("--listen", type=float, default=1.5)
    a = ap.parse_args()
    s = open_port(a.port)
    for c in a.cmd:
        run(s, c, a.listen)
        time.sleep(0.6)  # firmware ignores REPL commands closer than 0.5 s apart
