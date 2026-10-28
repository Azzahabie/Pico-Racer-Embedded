@echo off
rem Build tools\dist\TelemetryViewer.exe from telemetry_viewer.py (one file,
rem page bundled in).  Double-click, or run from any folder:
rem
rem     tools\build_viewer.bat
rem
rem Needs: py -m pip install pyinstaller paho-mqtt pyserial amqtt
rem The exe holds no network settings; they are chosen in its Settings dialog
rem and kept in %APPDATA%\TelemetryViewer\settings.json.

setlocal
set "TOOLS=%~dp0"
set "WORK=%TEMP%\telemetry_viewer_build"

py -m PyInstaller --noconfirm --clean --onefile --console ^
    --name TelemetryViewer ^
    --distpath "%TOOLS%dist" ^
    --workpath "%WORK%" --specpath "%WORK%" ^
    --add-data "%TOOLS%telemetry_viewer.html;." ^
    --paths "%TOOLS%." ^
    --hidden-import telemetry_decode ^
    --collect-submodules amqtt ^
    --collect-data amqtt ^
    --copy-metadata amqtt ^
    --collect-submodules serial ^
    "%TOOLS%telemetry_viewer.py"
if errorlevel 1 exit /b %errorlevel%

echo Built %TOOLS%dist\TelemetryViewer.exe
