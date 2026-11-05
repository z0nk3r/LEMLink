#!/usr/bin/env python3
"""Every verb in docs/PROTOCOL.md against a running sim, both ways it can answer.

    python3 test_wire.py [host]

It moves a few switches and knobs and puts them back (a knob to its nearest whole
number), so run it on a sim nobody is flying. It works with a patched or a stock
LEM.dll, or with no LM loaded, and checks what each should answer.
"""
import socket
import sys
import time

PROTOCOL = 2
TOGGLE = "LeftXPointerSwitch"
SPRING = "ORDEALSlewSwitch"
SLEW2 = "RadarSlewSwitch2"
KNOBS = ("FloodRotary", "LtgAnunNumKnob", "LtgFloodOhdFwdKnob", "LtgIntegralKnob",
         "ORDEALAltSetRotary", "Panel12AntPitchKnob", "Panel12AntYawKnob")

# Request -> how many fields a success has.
PATCH_FIELDS = {"F 0": 10, "F 1": 10, "X 0": 6, "X 1": 6, "Y": 9, "D": 9, "B": 4, "O": 4,
                "Q": 1, "L": 2, "J": 10, "I": 14, "W 0": 1, "W 4": 1, "T 0": 1, "T 1": 1,
                "N EPSDCVoltMeter": 1, "A 3676": 1, "A 3676 2": 2, "A 3676 16": 16}

REFUSED = ("P extra", "Y extra", "Z extra", "C extra", "G", f"G {TOGGLE} extra", "S x", "K x",
           "N", "W", "T", "F", "X", "A", "S x 1 1 1", "K x 1 1", "A 1 1 1",
           f"S {TOGGLE} abc", f"S {TOGGLE} -1", f"S {TOGGLE} 1.5", f"S {TOGGLE} 1 hold",
           f"S {TOGGLE} 1 2", f"K {KNOBS[0]} abc", f"K {KNOBS[0]} 1e3", f"K {KNOBS[0]} nan",
           f"K {KNOBS[0]} inf", f"K {KNOBS[0]} 0x10", f"K {KNOBS[0]} 1.2.3", f"K {KNOBS[0]} -",
           "W x", "W 5", "W -1", "T x", "T 2", "F x", "F 2", "X x", "X 2",
           "A xyz", "A 9", "A 4000", "A -1", "A 3676 0", "A 3676 17", "A 3676 x")

failures = []
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.25)
addr = (sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1", 8060)


def check(name, got, want):
    if got != want:
        failures.append(name)
        print(f"FAIL {name}: got {got!r}, want {want!r}")


def ask(*requests):
    """All of them in one datagram, so one frame: {request: (mark, fields)}."""
    sock.sendto(("\n".join(requests) + "\n").encode(), addr)
    got, end = {}, time.time() + 8
    while len(got) < len(set(requests)) and time.time() < end:
        try:
            lines = sock.recv(4096).decode(errors="replace").splitlines()
        except socket.timeout:
            continue
        for line in lines:
            for mark in (" = ", " ! "):
                request, found, rest = line.partition(mark)
                if found:
                    got[request] = (mark.strip(), rest.split())
                    break
    return got


def one(request):
    return ask(request).get(request)


def state(item):
    answer = one(f"G {item}")
    return int(answer[1][0]) if answer and answer[0] == "=" else None


def refused(request, code):
    check(request, one(request), ("!", [code]))


def grammar():
    """What needs no LM and no patch to say."""
    for request in ("Ping", "Garbage", "q", "? hello", "Sx", "SS x 1", "z"):
        refused(request, "bad-verb")
    sock.settimeout(8)      # a sim at one frame a second is still a sim
    long = "G " + "x" * 200
    sock.sendto((long + "\n").encode(), addr)
    check("too long a request", sock.recv(4096).decode(), long[:100] + " ! bad-args\n")
    # A blank line has no reply, and spaces at the ends are not repeated.
    sock.sendto(b"\n\n  \n   P  \n\n", addr)
    check("one reply for blank lines around P", sock.recv(4096).decode().startswith("P = "), True)
    sock.settimeout(2)
    try:
        check("nothing more", sock.recv(4096), None)
    except socket.timeout:
        pass
    sock.settimeout(0.25)


def no_lm():
    for request in ("S x 1", "K x 1", "G x", "N x", "W 0", "T 0", "F 0", "X 0", "A 1",
                    "Y", "D", "B", "O", "Q", "I", "L", "J", "C", "Z"):
        refused(request, "no-lm")


def panel():
    """Runs with TOGGLE and SPRING restored by the caller."""
    for request in REFUSED:
        refused(request, "bad-args")
    for request in ("S NoSuchItem 1", "S NoSuchItem 1 1", "G NoSuchItem", "K NoSuchItem 1.5"):
        refused(request, "no-item")

    # The refusals above moved nothing, and -1 did not stick the switch.
    for value in (1, 0, 1):
        got = ask(f"S {TOGGLE} {value}", f"G {TOGGLE}")
        check(f"S {TOGGLE} {value} reads back", got.get(f"S {TOGGLE} {value}"), ("=", [str(value)]))
        check(f"G {TOGGLE} after {value}", got.get(f"G {TOGGLE}"), ("=", [str(value)]))
    refused(f"S {TOGGLE} -1", "bad-args")
    check("-1 refused and the switch held", state(TOGGLE), 1)

    # The reply is what the control holds, not what was asked for.
    rest = state(SPRING)
    check("a spring without hold is back already", one(f"S {SPRING} 2"), ("=", [str(rest)]))
    check("a spring held stays", one(f"S {SPRING} 2 1"), ("=", ["2"]))
    check("still held a frame later", state(SPRING), 2)
    check("released", one(f"S {SPRING} {rest}"), ("=", [str(rest)]))
    top = one(f"S {KNOBS[3]} 99")
    check("a whole number on a knob clamps", top, ("=", ["8"]))


def knobs(patched):
    before = {k: state(k) for k in KNOBS}
    check("all seven knobs are items", None in before.values(), False)
    try:
        for knob in KNOBS:
            top = float(one(f"K {knob} 9999.0")[1][0])
            bottom = float(one(f"K {knob} -9999.0")[1][0])
            check(f"K {knob} clamps at both ends", -9999 < bottom < top < 9999, True)
            middle = (bottom + top) // 2 + 0.5
            want = middle if patched else middle + 0.5      # rounded, through the panel
            mark, fields = one(f"K {knob} {middle}")
            check(f"K {knob} {middle}", (mark, abs(float(fields[0]) - want) < 0.01), ("=", True))
            check(f"K {knob} takes a whole number", one(f"K {knob} {int(middle)}")[0], "=")
    finally:
        for knob, value in before.items():
            ask(f"S {knob} {value}")
    if patched:
        refused(f"K {TOGGLE} 1.5", "bad-args")


def displays(patched):
    got = ask(*PATCH_FIELDS)
    for request, count in PATCH_FIELDS.items():
        answer = got.get(request)
        if patched:
            check(f"{request} fields", answer and (answer[0], len(answer[1])), ("=", count))
        else:
            check(request, answer, ("!", ["no-display"]))
    refused("N NoSuchMeter", "no-item" if patched else "no-display")

    if patched:
        check("second slew contact rests at centre", state(SLEW2), 1)
        check("and stays without a hold", one(f"S {SLEW2} 4"), ("=", ["4"]))
        check("still there a frame later", state(SLEW2), 4)
        check("released", one(f"S {SLEW2} 1"), ("=", ["1"]))
    else:
        refused(f"S {SLEW2} 4", "no-item")

    z = one("Z")
    check("Z fields", z and (z[0], len(z[1])), ("=", 27))
    c = one("C")
    check("C answers or has no CSM", c and (len(c[1]) == 7 if c[0] == "=" else c[1] == ["no-csm"]), True)

    # One datagram is one frame: both ends of a batch read one clock.
    batch = [f"G {TOGGLE}"] * 1 + ["Z", "P"] + [f"N Meter{i}" for i in range(30)]
    got = ask(*batch)
    check("one datagram, one frame", got["Z"][1][0], got["P"][1][2])
    check("every line of a batch answered", len(got), len(batch))


def main():
    grammar()
    hello = one("P")
    if not hello:
        print("no answer on port 8060: is Orbiter running, with LEMLink ticked?")
        return 1
    check("P fields", (hello[0], len(hello[1])), ("=", 4))
    check("protocol", int(hello[1][0]), PROTOCOL)
    float(hello[1][2])
    dll, vessel = hello[1][1], hello[1][3]
    print(f"LEM.dll: {dll}, vessel: {vessel}")
    check("no vessel exactly when no DLL state", dll == "-", vessel == "-")

    if vessel == "-":
        no_lm()
    else:
        toggle, spring, knob = state(TOGGLE), state(SPRING), state(KNOBS[3])
        try:
            panel()
        finally:
            ask(f"S {TOGGLE} {toggle}", f"S {SPRING} {spring}", f"S {KNOBS[3]} {knob}")
        check("toggle restored", state(TOGGLE), toggle)
        check("spring restored", state(SPRING), spring)
        knobs(dll == "patched")
        displays(dll == "patched")

    print(f"{len(failures)} FAILED" if failures else "all passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
