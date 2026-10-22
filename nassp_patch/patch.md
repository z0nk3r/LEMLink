# Local NASSP patch: a connector for LM display data

`apply_patch.py` applies it to a NASSP source tree at the revision
`../vendor/REVISION` pins: **CaptainSwag101/NASSP @ 96b6380943c3**, release
NASSP-V9.0-Orbiter2024-47.

**This is a local patch, not an upstream contribution.** It means building
`LEM.dll` from source rather than using the released binary, which also means
the ABI our plugin matches becomes one we control. The cost is that a NASSP
update is a rebase of this patch rather than a download.

## Why

`MFD_PANEL_INTERFACE` reaches anything registered with `PanelSwitches`: every
switch, breaker and talkback. It does not reach a display that a subsystem
draws for itself, and most of the LM's displays are in that position:

- the 51 caution and warning lights, drawn by `CWEA.RedrawLeft/Right()`
  straight out of `LEM_CWEA::LightStatus[5][8]`
- the mission and event timers, `MissionTimer` objects passed *into* their
  control switches rather than registered themselves
- meters, whose needle is a `double` behind `MeterSwitch::GetDisplayValue()`
  that the `int GetState()` on `PanelSwitchItem` cannot carry
- the FDAIs, X-pointers and range/rate tapes, which are computed inside the
  panel's redraw code or in subsystem members
- the DSKY, DEDA and AOT counter, held as digit arrays in their own classes

This adds one read-only connector that answers all of them and nothing else.

## The files

| File | What it is |
|---|---|
| `lmdd_types.h` | The wire between the two DLLs: message numbers and reply structs. Compiled into **both** `LEM.dll` and `LEMLink.dll`, so there is one copy of the ABI. Plain C, no NASSP includes. |
| `lemdisplaydata.h/.cpp` | `LEMDisplayDataConnector`, registered on `VIRTUAL_CONNECTOR_PORT` with type `LM_DISPLAY_DATA`. |
| `apply_patch.py` | Copies those three into `src_lm/` and makes the edits below. |

## Applying it

```bash
python3 apply_patch.py <copy-of-NASSP>/Orbitersdk/samples/ProjectApollo
```

Every edit is anchored on exact text that must appear exactly once, or the run
stops before writing anything. Files are read and written as latin-1 (NASSP's
sources are ISO-8859), and a CRLF tree is refused rather than half-matched.

| File | Edit |
|---|---|
| `src_sys/connector.h` | `LM_DISPLAY_DATA` appended after `VHF_RNG` - at the end, so no ordinal moves |
| `src_sys/toggleswitch.h/.cpp` | `PanelSwitches::GetMeterByName()`, the panel-wide walk `SetState()` does inline, with `dynamic_cast` so a switch's name answers 0; `GetContinuousByName()`, the same for a continuous knob |
| `src_sys/toggleswitch.h` | `ContinuousSwitch::SetDisplayValue(double)`, inline and non-virtual: `SwitchTo(DisplayToAngle(v))`, what `SetState(int)` does without the rounding |
| `src_sys/FDAI.h` | `GetBallAttitude()`, returning the drawn (slewed) ball attitude |
| `src_lm/LEM.h` | the include, the `displayDataConnector` member **as the last member of `LEM`**, `GetFDAIRatesErrors()`, and friendship in `LEM` and `LEM_RadarTape` |
| `src_lm/lm_ags.h` | friendship in `LEM_DEDA`, whose digit arrays are protected |
| `src_sys/dsky.h` | friendship in `DSKY`, for the protected `IsStatusPowered()`, `IsSegmentPowered()` and `ELOff` the panel draws on |
| `src_lm/LEM.cpp` | construct and register the connector |
| `src_lm/lempanel.cpp` | the body of `LEM::GetFDAIRatesErrors()` |
| `src_lm/lempanel.cpp`, `lemvc.cpp` | the four inline FDAI needle blocks replaced by calls to it |
| `src_lm/lempanel.cpp`, `LEM.h` | `LEM::GetPanelLampBits()`, generated: each lamp's condition is extracted from its redraw `case`, so a NASSP change to one stops the patch rather than drifting |
| `Project/LEM/CMakeLists.txt` | `lemdisplaydata.cpp` added after `lemconnector.cpp` |
| `src_lm/lemswitches.h/.cpp` | NASSP bug fix: `LMAbortButton::SwitchTo` overrides instead of hiding, so ABORT through the connector reaches the LGC |
| `src_lm/lemsystems.cpp` | NASSP bug fix (build 47): `CrossPointer::Timestep` re-reads its relays each step (Init copied them once) and takes the RR branch on a de-energised rate/error relay, so RATE/ERR MON and MODE SEL route the X-pointers again |
| `src_lm/lemsystems.cpp` | the altitude-rate tape's direction: upstream `6d7d2dfb67` reversed it; the patch restores it, so a descent reads negative (black is positive, per the LM-11 handbook vol 2, 4.8.2.2) |
| `src_lm/lemswitches.h` | friendship in `EngineStartButton`, so `LMDD_GET_PANEL_LAMPS` reports its lamp (bit 25) through its own `LightLogic()` |
| `src_lm/lemvc.cpp`, `LEM.h` | `LEM::GetVCLegendBits()`, generated and appended to `lemvc.cpp` (where the `VC_MAT_` ids are in scope): the legend block of `clbkVCRedrawEvent` copied whole, run against a local `SetVCLighting` that records a bit per material, with `XP_LIT_ON`/`XP_LIT_OFF` redefined to 1 and 0. The script refuses the block if it holds any statement but a legend call, or lights a set of materials other than `LEGENDS`, since the copy executes inside the connector. Answers `LMDD_GET_VC_LEGENDS` (LEMLink `L`) |
| `src_lm/lemswitches.cpp`, `lm_eps.cpp` | talkbacks read as drawn: `LEMRCSQuadTalkback` and `RecorderTalkback::GetState()` return `failOpen ? 1 : 0` with their breaker dead, as `DrawSwitch` draws, and DES BATS' state is gated on `EPS_DISP_CB` where `lm_eps.cpp` sets it. Without it, `G` on the 8 quads, RECORDER and DES BATS answers the logic state while NASSP draws the unpowered face. Drawing is unchanged |
| `src_lm/lemvc.cpp` (checked, not edited) | `LMDD_GET_LIGHTING` restates how the VC's `AID_LMVC_LIGHTING` case scales each lighting channel, because that case lights meshes and cannot be run; `check_lighting_sources()` stops the patch if any expression it mirrors (`LIGHTING_SOURCES`) is gone from that case |

**The FDAI edit is the one that changes NASSP code.** The 2D panel and the VC
each computed the needle inputs inline, left and right: four copies. They now
call one helper, so the connector serves exactly what is drawn. Before removing
each copy the script checks it IS the helper's body, whitespace aside, with
only the two switch names differing; if NASSP ever edits one of the four, the
patch refuses rather than silently changing what the panel draws.

## Building

On a Windows build machine, the way the fork's CI builds it, with
`<orbiter>` an Orbiter 2024 install (or a copy of its `Orbitersdk/`) and
`<work>` any scratch folder:

```
cmake -DORBITER_INSTALL_DIR:STRING=<orbiter> ^
      -DFINAL_INSTALL_DIR:STRING=<work>/patched_out ^
      -B <work>/build_patched -G "Visual Studio 17 2022" -A x64 ^
      <work>/patched/NASSP-<rev>/Orbitersdk/samples/ProjectApollo
cmake --build <work>/build_patched --config RelWithDebInfo --target LEM -j 4
```

Three things the configure needs that are not obvious:

- `<orbiter>` needs `Orbitersdk/` **including XRSound's headers and lib**,
  and `lua.dll` at its root: CMake takes `lua.dll`'s presence to mean an
  OpenOrbiter install and selects the dynamic CRT that matches the shipped
  DLLs.
- The VS Build Tools need the **ATL** component
  (`Microsoft.VisualStudio.Component.VC.ATL`); without it the link fails on
  `atls.lib`.
- A stock build from the same tree (`build_stock`) is how build parity was
  proved: it loads and runs A11-13 identically to the CI binary.

Install only `LEM.dll`, keeping the CI binary beside it:

    Modules/ProjectApollo/LEM.dll.ci-x64-47     the release's own (sha ae17b790...)
    Modules/ProjectApollo/LEM.dll               the patched build (sha d36bdb04...)

Swapping back is how you tell a NASSP bug from one of ours.

## The messages

All are `val1.pValue` = a caller-owned struct from `lmdd_types.h` whose first
field `size` the caller sets; a size the connector does not expect is refused
(`val2.bValue = false`), so a plugin built against an older layout gets "no
display" instead of a buffer overrun.

| Message | Reply | Source in NASSP |
|---|---|---|
| `LMDD_GET_CW_LIGHT(_ROW)` | one light / a row packed 2 bits each | `CWEA.GetCWLightStatus` |
| `LMDD_GET_MISSION/EVENT_TIMER` | seconds | `MissionTimerDisplay`, `EventTimerDisplay` |
| `LMDD_GET_METER` | the needle in dial units | `MeterSwitch::GetDisplayValue()` by name |
| `LMDD_GET_FDAI` | rates, errors -1..+1 as drawn, ball degrees, powered | `GetFDAIRatesErrors()`, `fdai*.GetBallAttitude()` |
| `LMDD_GET_XPOINTER` | needles -1..+1, raw velocity, scale switch | `crossPointer*.GetVelocities()` / 20 |
| `LMDD_GET_TAPE` | tape positions (px), commanded m and m/s, three fail flags | `RadarTape` |
| `LMDD_GET_DSKY` | PROG/VERB/NOUN/R1-3 strings, lamp bits, power | `dsky` getters |
| `LMDD_GET_DEDA` | address, data, OPR ERR, power | `deda.Adr/Data` (friend) |
| `LMDD_GET_PANEL_LAMPS` | 21 bits, `LMDD_LAMP_*` | `LEM::GetPanelLampBits()`, generated by `apply_patch.py` from each lamp's own `case` in `clbkPanelRedrawEvent` |
| `LMDD_GET_ATTITUDE` | IMU gimbals, GASTA and AGS attitude, ORDEAL pitch offsets | `imu`, `gasta`, `aea`, `ordeal` - what `lemsystems.cpp` hands each FDAI |
| `LMDD_GET_AOT` | reticle counter 0..360, shaft detent, dimmer | `optics` |
| `LMDD_SET_KNOB`, `LMDD_GET_KNOB` | a continuous knob's value in display units, as a double (`val1.pValue` name, `val3.dValue` value in; `val1.dValue` out) | `ContinuousSwitch` by name; the one write, since a knob is a control |

## Why it is shaped this way

- **Never change the layout of a class another DLL uses.** `mcc.dll`,
  `ApolloRTCCMFD`, `ProjectApolloMFD` and `LC37` are the released, unpatched
  builds and reach `LEM` members by offset. A member added mid-class moves
  every later one, and the Apollo 5 uplink (`MCC::LM_uplink`, a `memcpy` into
  `lm->PCM.mcc_data`) then writes 2 KB at the wrong offset and crashes in
  `vcruntime140`. So: the one data member goes last in `LEM` (only `LEM.dll` ever
  constructs one), every other edit is a method, a friend or an appended enum
  value, and the ABORT fix overrides an existing virtual rather than adding
  one (the original `SwitchTo(int)` was not virtual).

- **Read-only, but for knobs.** A display is an output of the simulated
  systems; a write would be overwritten on the next timestep. A continuous
  knob is a control, and the panel connector can only set it in whole display
  units, so `LMDD_SET_KNOB` is the one write.
- **Needles as drawn, not the quantity behind them.** An FDAI error needle can
  be showing the RR, the LGC or the AGS depending on two switches, and the pit
  wants the needle.
- **Append-only enums.** `ConnectorType` and `LMDDMessageType` ordinals cross
  the DLL boundary; one that moved would read a DSKY as an FDAI.
- **Reading a meter has a side effect, and it is harmless.**
  `GetDisplayValue()` advances the meter's own time-based smoothing on every
  call, whoever calls it, so the connector reads exactly what the panel would
  draw. The first call after a load returns the saved needle without querying
  (`lastDrawTime == -1`); LEMBridge's `tools/display_dump.py` discards one read of every meter
  first for that reason.
- **Friendship, not exposure.** `CWEA`, the timers, `MainPanel`, the radar
  tape's members and the DEDA's digits are protected; `LEM_EDS` and others
  already reach them as friends.
- The CSM has the same gap. This is deliberately LM-only.
