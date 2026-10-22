# Running NASSP under Wine

Orbiter 2024 and NASSP V9.0 (x64) run on Linux through Wine and DXVK. The
recipe is short, but none of it is guessable, and three of the four steps
look like they are working when they are not.

```bash
WINEPREFIX=~/.wine-orbiter wineboot --init     # win64: Wine 10 is wow64-only
WINEPREFIX=~/.wine-orbiter winetricks -q \
    d3dx9 d3dcompiler_43 d3dcompiler_47 dxvk vcrun2022
mv <orbiter>/msvcp140*.dll <orbiter>/_crt_stash/
cd <orbiter> && WINEPREFIX=~/.wine-orbiter wine Orbiter.exe -s "<scenario>"
```

`run_nassp.sh` does all of this and refuses to start while the stashed DLLs
are back in place. `Orbiter.exe -s <scn>` launches a scenario directly, and
`-S` launches one and exits when it closes. Without either, the launchpad
opens. Orbiter 2024 and its NASSP are x86-64, which matters because Wine 10 is
wow64-only and cannot run the 32-bit 2016 line.

**Orbiter ships its own CRT, and that CRT is what crashes.** `msvcp140.dll`
and its friends sit in the Orbiter root, and the exe's own directory wins the
DLL search order, so whatever winetricks puts in `system32` is never loaded.
Those app-local builds null-deref inside `std::mutex::lock()` under Wine. They
kill NASSP's AGC worker thread the moment a mission loads. Moving them aside
is the whole fix. You can recognise this crash because the faulting address
does not move when you swap the `system32` copy, since that copy was never in
play.

**Wine's HLSL compiler cannot build D3D9Client's shaders.** Without native
D3DX9 it fails on `NewPlanet.hlsl` with `E5017: ... not yet implemented
feature`. `d3dcompiler_43` matters as much as `d3dx9` does, because native
`d3dx9_43` forwards compilation to it.

**`vcrun2015` is the wrong redistributable.** NASSP is built with MSVC 14.42,
so install `vcrun2022`.

**The 2D-panel FDAIs render black, and no Wine setting fixes it.** They are
drawn with legacy OpenGL into a DIB, which Wine implements through OSMesa, and
distribution Wine packages are often built without OSMesa. The virtual
cockpit (F8) is unaffected. Attitude, rates and errors are computed either
way.

**Load one scenario per process.** A second session in the same Orbiter run
faults inside `d3d9client`. Quit Orbiter between scenarios rather than
returning to the launchpad.

**NASSP's sources are ISO-8859, so plain `grep` silently skips them.** Use
`grep -a`.
