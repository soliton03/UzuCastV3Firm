@echo off
setlocal EnableExtensions

set "PYTHONIOENCODING=utf-8"
set "PYTHONUTF8=1"

set "BAT_DIR=%~dp0"
set "PROJECT_DIR=%BAT_DIR%uzuCastV3_Sub"
set "PIO_PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
set "PIO_ENV=esp32dev_s460800"
set "UPLOAD_PORT="
set "UPLOAD_SPEED=460800"

:parse_args
if "%~1"=="" goto args_done
if /i "%~1"=="/h" goto show_help_ok
if /i "%~1"=="/?" goto show_help_ok
if /i "%~1"=="-h" goto show_help_ok
if /i "%~1"=="-?" goto show_help_ok
if /i "%~1"=="/s" (
  if "%~2"=="" (
    echo [ERROR] /s requires a baud rate.
    goto show_help_error
  )
  set "UPLOAD_SPEED=%~2"
  shift
  shift
  goto parse_args
)
echo %~1| findstr /r "^[/\-]" >nul
if not errorlevel 1 (
  echo [ERROR] Unknown option: %~1
  goto show_help_error
)
set "UPLOAD_PORT=%~1"
shift
goto parse_args

:args_done
if "%UPLOAD_PORT%"=="" set "UPLOAD_PORT=COM4"

if "%UPLOAD_SPEED%"=="460800" (
  set "PIO_ENV=esp32dev_s460800"
) else if "%UPLOAD_SPEED%"=="921600" (
  set "PIO_ENV=esp32dev"
) else if "%UPLOAD_SPEED%"=="115200" (
  set "PIO_ENV=esp32dev_s115200"
) else if "%UPLOAD_SPEED%"=="230400" (
  set "PIO_ENV=esp32dev_s230400"
) else (
  echo [ERROR] Invalid upload speed: %UPLOAD_SPEED%
  goto show_help_error
)

echo %UPLOAD_PORT%| findstr /i /r "^COM[0-9][0-9]*$" >nul
if errorlevel 1 (
  if "%UPLOAD_PORT%"=="115200" goto speed_without_s
  if "%UPLOAD_PORT%"=="230400" goto speed_without_s
  if "%UPLOAD_PORT%"=="460800" goto speed_without_s
  if "%UPLOAD_PORT%"=="921600" goto speed_without_s
  echo [ERROR] Invalid COM port: %UPLOAD_PORT%
  goto show_help_error
)

if not exist "%PIO_PY%" (
  echo [ERROR] PlatformIO Python not found:
  echo   %PIO_PY%
  exit /b 1
)

if not exist "%PROJECT_DIR%\platformio.ini" (
  echo [ERROR] Project not found:
  echo   %PROJECT_DIR%
  exit /b 1
)

echo.
echo [Sub] UzuCast V3 Sub - ESP32 board required on %UPLOAD_PORT% (NOT ESP32-S3)
echo [Sub] Project: uzuCastV3_Sub, env=%PIO_ENV%
echo [Sub] Platform: pioarduino (shared with Main)
echo [Sub] Framework: Arduino ESP32 3.3.9
echo [Sub] Upload baud: %UPLOAD_SPEED% bps
echo.

call :do_upload
if errorlevel 1 goto upload_failed
exit /b 0

:speed_without_s
echo [ERROR] Baud rate requires /s: writeslave.bat COM5 /s %UPLOAD_PORT%
goto show_help_error

:upload_failed
echo.
echo [HINT] Chip mismatch on %UPLOAD_PORT%?
echo   Main board ESP32-S3  : run writemaster.bat [COMx] [/s baud]
echo   Sub board ESP32      : run writeslave.bat [COMx] [/s baud]
echo   Close Serial Monitor before upload.
echo.
echo Build failed. Running clean and retrying...
"%PIO_PY%" -m platformio run -d "%PROJECT_DIR%" -e %PIO_ENV% -t clean
if errorlevel 1 exit /b 1
call :do_upload
exit /b %ERRORLEVEL%

:do_upload
"%PIO_PY%" -m platformio run -d "%PROJECT_DIR%" -e %PIO_ENV% -t upload --upload-port %UPLOAD_PORT%
exit /b %ERRORLEVEL%

:show_help_ok
call :show_help
exit /b 0

:show_help_error
echo.
call :show_help
exit /b 1

:show_help
echo.
echo UzuCast V3 Sub - writeslave.bat
echo   Build and upload firmware to the ESP32 Sub board (NOT ESP32-S3).
echo.
echo Usage:
echo   writeslave.bat [COMx] [/s baud]
echo.
echo Options:
echo   COMx          Serial port (default: COM4)
echo   /s baud       Upload baud rate (default: 460800)
echo                 Allowed: 115200, 230400, 460800, 921600
<nul set /p "=  /h, /?  -h    Show this help"
echo.
echo.
echo Examples:
echo   writeslave.bat
echo   writeslave.bat COM7
echo   writeslave.bat /s 115200
echo   writeslave.bat COM7 /s 460800
echo.
exit /b 0
