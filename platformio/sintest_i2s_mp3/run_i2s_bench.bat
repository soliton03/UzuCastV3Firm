@echo off
setlocal EnableExtensions
set "PYTHONIOENCODING=utf-8"
set "PYTHONUTF8=1"
echo Close Serial Monitor on COM11 and COM7 before running.
python "%~dp0tools\run_i2s_bench.py" --master-port COM11 --slave-port COM7 --mode MP3
exit /b %ERRORLEVEL%
