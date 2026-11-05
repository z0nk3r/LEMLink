# LEMLink's wire protocol

How a program talks to the LM through this plugin: UDP port 8060 on the machine
running Orbiter, in plain text, so any of it can be tried from a terminal.

```bash
echo 'G ORDEALModeSwitch' | nc -u -w1 127.0.0.1 8060
```

## The rule

A request is a line: one uppercase letter, then its arguments, separated by spaces.
A reply starts with the request it answers.

```
<request> = <fields>      it worked
<request> ! <code>        it did not, and why
```

```
G ORDEALModeSwitch = 1
G NoSuchItem ! no-item
F 0 = 1 0.0000 -0.0000 0.0068 0.0000 -0.0732 0.0000 0.035 35.210 0.012
W 7 ! bad-args
```

- Every line that is not blank gets exactly one reply line, in a datagram of its own.
- The request is repeated exactly as it came, without spaces at either end, so a
  reply can be matched by looking it up. A request is 100 characters at most.
- A verb's reply always has the same number of fields, split on spaces.
- Bit fields are hex and AGC words are octal. Everything else is decimal.

| Code | Meaning |
|---|---|
| `no-lm` | no LM in this scenario, or it has not finished loading |
| `no-display` | a stock `LEM.dll`: this verb needs the patch in `nassp_patch/` |
| `display-mismatch` | the plugin and `LEM.dll` were built from different `lmdd_types.h` |
| `no-item` | NASSP has no control or meter by that name |
| `no-csm` | no CSM in this scenario |
| `bad-args` | wrong number of arguments, or one that is not a valid value |
| `bad-verb` | not one of the verbs below |

## Batches

The plugin answers from `clbkPreStep`, so one round trip costs one sim frame. A
datagram can carry many requests, newline separated, and the plugin takes up to 32
datagrams of up to 1023 bytes a frame. Every reading in a batch comes from the same
timestep, which is what lets a selector and its needle be read together.

Nothing is answered while Orbiter is paused. Requests wait for the first frame after it.

## Controls

These work on any LM, patched or not. Names are NASSP's registered ones, as in
LEMBridge's `docs/nassp_states.csv`.

| Sent | Fields | Meaning |
|---|---|---|
| `P` | `<protocol> <dll> <simt> <vessel>` | who is there; see below |
| `S <item> <state> [hold]` | `<state>` | move a control, then read it back |
| `K <item> <value>` | `<value>` | set a continuous knob, then read it back |
| `G <item>` | `<state>` | read a control |

**`P`** works with no LM loaded. `protocol` is 2, and changes whenever a verb or a
reply's fields do. `dll` is `patched`, `stock`, `mismatch`, or `-` with no LM. `simt`
is the sim's clock in seconds, and `vessel` is the LM's name, or `-` with no LM.

**`S`** takes a whole number, 0 or more, and `hold` is `0` or `1`. Use 1 for a
spring-loaded switch, or NASSP snaps it back before the next timestep sees the throw.
On a pit the physical switch is the spring: the hand lets go, the node reports the
rest position, and the bridge commands it. The reply is the control's state in the same
frame, so a clamp or a spring shows. NASSP does not check a toggle or breaker's range:
it stores and reports whatever it is given.

**`RadarSlewSwitch2`** is the second contact of a diagonal on RR SLEW. It exists only on
a patched `LEM.dll`, and a stock one answers `no-item`. It takes `RadarSlewSwitch`'s
position numbers, 0 down, 1 centre, 2 up, 3 left, 4 right, and the radar moves for both
items at full rate. It rests at 1 until told otherwise, so it needs no `hold`. It makes no
sound, draws nothing and is not saved in a scenario.

**`K`** takes a decimal number, a minus sign and a point allowed, no exponent. It goes
through the same path as a mouse drag, clamped at the knob's ends. An item that exists
but is not a continuous knob is `bad-args`. On a stock `LEM.dll` the knob takes whole
numbers only, through the panel: the value is rounded, and the reply is what it then holds.

## Displays

These need the patch and answer `no-display` on a stock `LEM.dll`, except `C` and `Z`.

| Sent | Fields | Meaning |
|---|---|---|
| `N <meter>` | `<value>` | a meter's displayed value, by registered name, in dial units |
| `W <0-4>` | `<packed>` | a row of eight C&W lights, two bits each, column 0 in the low bits |
| `T <0\|1>` | `<seconds>` | mission timer (0) or event timer (1) |
| `F <0\|1>` | `<powered> <rate x3> <error x3> <ball x3>` | FDAI, left (0) or right (1): roll, pitch, yaw needles -1..+1 as drawn; ball in degrees |
| `X <0\|1>` | `<powered> <fine> <needle x2> <raw x2>` | X-pointer: forward then lateral, -1..+1; raw is the unclamped velocity, 20 = full scale |
| `Y` | `<powered> <monitor> <pfail> <sfail> <tfail> <range_pos> <rate_pos> <range_m> <rate_mps>` | range/alt and rate tapes: positions in NASSP's tape pixels (0..8486, 0..5599), commanded values in m and m/s |
| `D` | `<segpwr> <statpwr> <lamps> <prog> <verb> <noun> <r1> <r2> <r3>` | DSKY; lamp bits are `LMDD_DSKY_*` in `nassp_patch/lmdd_types.h` |
| `B` | `<powered> <opr_err> <adr> <data>` | DEDA |
| `O` | `<shaft> <dimmer> <counter_deg> <reticle_rad>` | AOT: reticle counter 0..360, shaft detent 0..5 |
| `Q` | `<lit>` | the lamps the panel draws from subsystem logic |
| `L` | `<lit> <numeric>` | the legends only NASSP's virtual cockpit lights |
| `J` | `<integral> <numeric> <anun> <dsky> <flood_cdr> <flood_lmp> <flood_side> <ordeal> <side_8_11> <side_12_14_16>` | every lighting channel |
| `I` | `<imu_pwr> <caged> <gasta_pwr> <imu x y z> <gasta r p y> <ags r p y> <ordeal1> <ordeal2>` | the chain behind the FDAI ball, degrees: IMU gimbal angles, the GASTA's attitude (PGNS), the AGS attitude, and the ORDEAL pitch offset added to FDAI 1 and 2 |
| `A <octal> [1-16]` | `<word> ...` | LGC erasable memory from that address, octal, read-only |
| `C` | `<name> <pos x3> <vel x3>` | truth for the rendezvous radar: the nearest ProjectApollo Saturn-class vessel minus the LM, global frame, m and m/s |
| `Z` | `<simt> <alt_ground> <alt_meanrad> <vspeed> <gs_local x3> <angvel x3> <pitch> <bank> <yaw> <thrust_N> <mass_kg> <rot x9> <pos x3>` | truth for the LM, from Orbiter and not NASSP |

**Text fields.** A blank digit travels as `_` and an empty field as `-`, so the fields
always split on spaces. A vessel name is sent the same way, with `_` for a space or any
other unprintable character.

**`Q`** has one bit each, named `LMDD_LAMP_*` in `nassp_patch/lmdd_types.h`: both LUNAR
CONTACT lights, the seven component cautions, both stage-sequence relay lights and ten
power-failure lamps, then MASTER ALARM (bit 21) and the annunciator and C/W PWR lighting
supplies the C&W grid is drawn through (22, 23). Bit 25 is ENGINE START's lamp as
`EngineStartButton::LightLogic()` draws it: relay K15, set by START and cleared by STOP,
not the momentary button. Bits 24 and 26 are always set and mean nothing.

**`L`** has one bit each, named `LMDD_LEGEND_*`: per X-pointer ELEV RT, AZ RT, LAT VEL,
FWD VEL, X1, X10 (CDR bits 0-5, LMP 6-11), then the tape's ALT / ALT RATE (12) and RANGE
/ RANGE RATE (13) and the RCS PRESS meter's He x10 (14). A bit is the virtual cockpit's
condition, from `LEM::GetVCLegendBits()`, which `nassp_patch/apply_patch.py` generates
from `lemvc.cpp`'s own block. `numeric` is the NUM dimmer channel's volts / 115: a legend
is drawn at max(numeric, 0.15) lit and 0.15 unlit, so it is visibly lit only above 0.15.

**`J`** has every channel as NASSP's virtual cockpit scales it: integral and numerics are
LCA volts / 115 (INTEGRAL knob 15-75 V, numerics 20-110 V, LTG O'RIDE passes the whole AC
bus), annunciator volts / 6, DSKY EL volts / 250, the floods bus volts / 28 (so above 1 on
a 33.8 V battery bus), ORDEAL `LightingPower()` (1, 0.5, 0), then the two switches that
gate the integral on panels 8/11 and 12/14/16.

**`C`** needs no patch, and answers `no-csm` with none in the scenario. `lm_rr.cpp`
measures exactly its |pos| and vel along pos while the radar is locked.

**`Z`** needs no patch, and is how an instrument is judged against the vehicle rather
than against itself. `alt_ground` is the vessel origin's height, about 3.6 m above the
footpads on the surface. `gs_local` and `angvel` are in Orbiter's left-handed vessel frame
(x right, y up, z forward). `rot` is GetRotationMatrix (vessel to global) row by row, and
`pos` the position from the gravity reference in the global frame.
