@echo off
setlocal
set "WORKBENCH=%~dp0..\.local\workbench-live"
if not exist "%WORKBENCH%\runtime\walking-runtime\launch_voice_workbench.py" goto not_ready
"%WORKBENCH%\.venv\Scripts\python.exe" -B -X utf8 "%WORKBENCH%\runtime\walking-runtime\launch_voice_workbench.py"
if not errorlevel 1 exit /b 0
pause
exit /b 1

:not_ready
echo The local live workbench setup is missing. See the workbench guide.
pause
exit /b 1
