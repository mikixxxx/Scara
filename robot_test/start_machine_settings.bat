@echo off
setlocal
cd /d "%~dp0"
py -3.11 rho4_machine_settings.py
if errorlevel 1 pause
