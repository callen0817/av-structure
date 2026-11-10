@echo off
cd /d "%~dp0"
echo Launching AV Structure...
python gui.py
if errorlevel 1 (
    .venv_310\Scripts\python.exe gui.py
)
