# LEMLink

The Orbiter side of the OpenLEM pit. LEMLink is an Orbiter 2024 plugin that
reads and writes the Project Apollo - NASSP Lunar Module's panel by name over
UDP, so a physical switch moves the matching switch in the LM.

**Sister repos:**
* [LEMBridge](https://github.com/z0nk3r/LEMBridge): the bridge and mapper that talk to this plugin.
* [OpenLEM](https://github.com/z0nk3r/OpenLEM): the hardware.

All three sit side by side in one workspace folder, with `cache/`, `runs/` and
`refs/`.

## What is in here

```
LEMLink
├── LEMLink.cpp, LEMLink.rc, resource.h   the plugin (one file, plus its launchpad strings)
├── build.bat                             builds build\LEMLink.dll with MSVC on the Windows box
├── run_nassp.sh                          launches Orbiter + NASSP under Wine, set up correctly
├── test_wire.py                          every verb against a running sim, both ways it can answer
├── docs/PROTOCOL.md                      every verb and what it answers
├── docs/WINE.md                          the Wine recipe and its traps
├── vendor/                               NASSP headers + connector.cpp at the pinned revision;
│   └── REVISION                          which revision, and why it is load-bearing
├── nassp_patch/                          the local LEM.dll patch: a read-only display connector
│   ├── apply_patch.py                    applies it to a NASSP tree (never edit by hand)
│   └── patch.md                          what it changes and why
├── LICENSE.md                            GPL v2 (NASSP-derived)
└── README.md
```

## The protocol, in short

UDP 8060 on the machine running Orbiter:

```
S <item> <state> [1]   set (1 = hold, for a spring-loaded switch)
K <item> <value>       set a continuous knob
G <item>               read
P                      protocol, LEM.dll state, sim time and vessel
```

Every reply starts with its request: `G Foo = 1`, or `G Foo ! no-item` when it
fails. `docs/PROTOCOL.md` has every verb.

LEMLink listens on this machine only (127.0.0.1), because whoever reaches the
socket can move every switch in the LM. For a bridge on another machine, set
`LEMLINK_ADDRESS` in Orbiter's environment to the address to listen on
(`0.0.0.0` for all of them). The on-screen note shows that address, or
`BIND FAILED` if the port could not be opened.

The patched `LEM.dll` adds read-only display verbs: C&W, lamps, meters,
FDAIs, X-pointers, tapes, DSKY, DEDA, timers and lighting. On a stock DLL they
answer `! no-display`.

## Building

Build on Windows with MSVC 14.42 or 14.44 (v143), **not mingw**. The vtable
layout has to match the compiler that built NASSP's `LEM.dll`.

```
build.bat C:\Orbiter2024
```

The build box only needs `Orbitersdk\include` and `Orbitersdk\lib`; it never
runs Orbiter. Three traps, none of which says what is wrong:
- **Use `/MD`, not `/MT`.**
- **Link `Orbiter.lib` and `Orbitersdk.lib`.**
- **Include `<winsock2.h>` before `Orbitersdk.h`.**

Copy `build\LEMLink.dll` to `<Orbiter>\Modules\Plugin\` and tick **LEMLink** in
the launchpad's Modules tab. That adds `LEMLink` under `ACTIVE_MODULES` in
`Orbiter.cfg`.

**Re-vendor `vendor/` whenever NASSP is updated.** A mismatched ABI compiles
cleanly and corrupts memory at run time. `vendor/REVISION` names the revision.

## Running NASSP under Wine

```bash
./run_nassp.sh -l                  # scenarios with an LM
./run_nassp.sh -w "<scenario>"     # launch, wait for the panels, leave it running
```

It refuses to start while Orbiter's own `msvcp140.dll` is in place, because
that DLL crashes NASSP's AGC thread under Wine. The full recipe and its
traps are in `docs/WINE.md`.

## The NASSP patch

`nassp_patch/apply_patch.py <nassp-source-copy>` applies the patch. Then build
`LEM.dll` from that tree. Keep the stock `LEM.dll.ci-x64-47` beside it:
swapping it back is how you tell a NASSP bug from one of ours.
`nassp_patch/patch.md` explains each edit.
