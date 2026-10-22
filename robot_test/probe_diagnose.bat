@echo off
setlocal
cd /d "%~dp0"
py -3.11 rho4_probe.py validate
py -3.11 rho4_probe.py diagnose
pause
