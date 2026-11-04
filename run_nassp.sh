#!/usr/bin/env bash
#
# Launch Orbiter 2024 + NASSP under Wine, set up the way docs/WINE.md
# describes.
#
#   ./run_nassp.sh                  straight into the default LM scenario (Apollo 5)
#   ./run_nassp.sh -p               the launchpad: settings, or pick a scenario
#   ./run_nassp.sh -l               list the scenarios that have an LM
#   ./run_nassp.sh "<scenario>"     straight into one, by its path under
#                                   Scenarios/ with no .scn extension
#   ./run_nassp.sh -w "<scenario>"  the same, but wait until the panels are up
#                                   and then leave it running in the background
#
# Override either path if your copy lives somewhere else:
#   ORBITER=~/Orbiter2024  WINEPREFIX=~/.wine-orbiter  ./run_nassp.sh
#
set -u

ORBITER="${ORBITER:-$HOME/Orbiter2024}"
export WINEPREFIX="${WINEPREFIX:-$HOME/.wine-orbiter}"
export WINEDEBUG="${WINEDEBUG:--all}"
export DISPLAY="${DISPLAY:-:0}"

# The default is the unmanned LM test flight: the only NASSP scenario with an
# LM and no CSM, so it is the fastest way to a live LM cockpit.
DEFAULT_SCN="Project Apollo - NASSP/Apollo - Mission Scenarios/Apollo 5/Apollo 5 - Before Launch T-60s"

die() { echo "run_nassp: $*" >&2; exit 1; }

[ -x "$ORBITER/Orbiter.exe" ] || die "no Orbiter.exe under $ORBITER (set ORBITER=)"
[ -d "$WINEPREFIX" ] || die "no wine prefix at $WINEPREFIX (see docs/WINE.md)"

# Orbiter ships its own msvcp140 next to the exe, and the exe's directory wins
# the DLL search order. Those copies null-deref in std::mutex under Wine and
# kill NASSP's AGC thread the moment a mission loads. Refuse to start rather
# than let someone spend an afternoon on the crash dump.
if [ -f "$ORBITER/msvcp140.dll" ]; then
    die "$ORBITER/msvcp140.dll is in place; NASSP will crash on mission load.
     mkdir -p '$ORBITER/_crt_stash' && mv '$ORBITER'/msvcp140*.dll '$ORBITER/_crt_stash/'"
fi

if [ "${1:-}" = "-l" ]; then
    # By what is IN the scenario, not its name: an LM that LEMLink can fly
    # is a vessel of class ProjectApollo/LEM in the ship list. A filename test
    # missed "PDI T-15m", "Before TPI" and "Before MCC-2", and a scenario
    # before transposition carries the LM inside the S-IVB, not as a vessel.
    # grep -a: NASSP's files are ISO-8859, and plain grep skips them silently.
    find "$ORBITER/Scenarios/Project Apollo - NASSP" -iname "*.scn" -print0 2>/dev/null |
        xargs -0 grep -a -l -i -E ':ProjectApollo[/\\]LEM[[:space:]]*$' 2>/dev/null |
        sed "s|$ORBITER/Scenarios/||; s|\.scn$||" | sort
    exit 0
fi

# One sim at a time: a second one cannot get LEMLink's port, and it would blank
# the running one's Orbiter.log. By process name, because pgrep -f also matches
# any shell command that merely mentions Orbiter.exe.
running=$(pgrep -x Orbiter.exe) && die "Orbiter is already running (pid $running)"

if [ "${1:-}" = "-p" ]; then
    # Settings and scenario picking. Launching a second scenario from the
    # launchpad after quitting one is the crash noted below, so quit Orbiter
    # rather than return here between scenarios.
    cd "$ORBITER" || die "cannot enter $ORBITER"
    exec wine Orbiter.exe
fi

WAIT=0
if [ "${1:-}" = "-w" ]; then WAIT=1; shift; fi
SCN="${1:-$DEFAULT_SCN}"

[ -f "$ORBITER/Scenarios/$SCN.scn" ] || die "no such scenario: $SCN.scn
     try:  $0 -l"

cd "$ORBITER" || die "cannot enter $ORBITER"
: > Orbiter.log

# -s runs one scenario and stops there. Deliberately one scenario per process:
# a second "Creating simulation session" in the same run faults inside
# d3d9client, so returning to the launchpad and picking again is what crashes.
echo "run_nassp: $SCN"
if [ "$WAIT" = "0" ]; then
    exec wine Orbiter.exe -s "$SCN"
fi

wine Orbiter.exe -s "$SCN" >/dev/null 2>&1 &
pid=$!
for _ in $(seq 1 40); do
    grep -aq "Finished initialising panels" Orbiter.log 2>/dev/null && break
    kill -0 "$pid" 2>/dev/null || die "Orbiter exited while loading; see $ORBITER/Orbiter.log"
    sleep 2
done
grep -aq "Finished initialising panels" Orbiter.log 2>/dev/null \
    || die "panels never came up, and Orbiter is still running (pid $pid); see $ORBITER/Orbiter.log"

grep -a "LEMLink:" Orbiter.log | sed 's/^/  /'
echo "run_nassp: up, pid $pid"
