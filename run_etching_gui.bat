@echo off
cd /d "%~dp0"
python etching_gui.py
if errorlevel 1 pause
