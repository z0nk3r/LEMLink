@echo off
rem Builds LEMLink.dll. Run this on the Windows build box, not the pit PC.
rem
rem   build.bat [path-to-Orbiter]
rem
rem Defaults to C:\Orbiter2024. Only the Orbitersdk subfolder is read, so a
rem headers-and-libs copy is enough - the build box never needs to run Orbiter
rem and never needs a GPU.
rem
rem /MD is not a choice: Orbitersdk.lib is itself built against the dynamic
rem CRT, and linking it from an /MT object fails with LNK2038 and then buries
rem that one line under thirty unresolved externals. Match it and they all go.
rem
rem The DLL therefore wants vcruntime140/ucrtbase at run time. That is fine
rem under Wine: those resolve from the prefix's system32, which is where the
rem working copies live once Orbiter's own crashing msvcp140 is stashed - see
rem docs\WINE.md.

setlocal

set ORBITER=%~1
if "%ORBITER%"=="" set ORBITER=C:\Orbiter2024
set SDK=%ORBITER%\Orbitersdk

if not exist "%SDK%\include\Orbitersdk.h" (
  echo ERROR: no Orbiter SDK at "%SDK%"
  echo Pass the Orbiter folder as the first argument.
  exit /b 1
)

rem -- find the compiler -----------------------------------------------------
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
if not exist "%VSWHERE%" (
  echo ERROR: vswhere not found; Visual Studio Build Tools are not installed.
  exit /b 1
)
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * ^
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 ^
    -property installationPath`) do set VSPATH=%%i
if "%VSPATH%"=="" (
  echo ERROR: no MSVC x64 toolset found.
  exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1

rem Both libs, not just Orbitersdk.lib. The oapi* functions are exported by
rem orbiter.exe, so Orbiter.lib is their import library; Orbitersdk.lib only
rem carries the SDK's own helper objects. Linking one of the two fails with
rem thirty LNK2019s naming __imp_ symbols, which does not hint at the cause.

rem -- build -----------------------------------------------------------------
if not exist build mkdir build

rem The launchpad's Modules tab reads its description and category out of the
rem DLL's string table, ids 1000 and 1001 - there is no API for it. Without the
rem .res the module still loads and still works, it is just nameless in the UI.
rc /nologo /fo build\LEMLink.res LEMLink.rc
if errorlevel 1 (
  echo.
  echo RESOURCE COMPILE FAILED
  exit /b 1
)

cl /nologo /LD /EHsc /O2 /MD /W3 ^
   /DWIN32 /D_WINDOWS /D_CRT_SECURE_NO_WARNINGS ^
   /I "%SDK%\include" /I vendor /I . ^
   /Fobuild\ /Fdbuild\ ^
   LEMLink.cpp vendor\connector.cpp ^
   /link /OUT:build\LEMLink.dll ^
   build\LEMLink.res ^
   "%SDK%\lib\Orbiter.lib" "%SDK%\lib\Orbitersdk.lib" ^
   ws2_32.lib user32.lib gdi32.lib

if errorlevel 1 (
  echo.
  echo BUILD FAILED
  exit /b 1
)

echo.
echo built build\LEMLink.dll
echo copy it to ^<Orbiter^>\Modules\Plugin\ and tick it in the launchpad Modules tab
endlocal
