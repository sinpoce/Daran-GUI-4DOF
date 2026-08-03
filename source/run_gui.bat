@echo off
rem DrEmpower Arm Console - quick launcher
rem Double-click to start the GUI. Installs deps automatically on first run.
setlocal
set PYTHONIOENCODING=utf-8
cd /d "%~dp0"

rem ---- dependency check (auto-install if missing) ----
python -c "import PySide6, serial" 1>nul 2>nul
if errorlevel 1 (
    echo [First run] Installing dependencies ...
    python -m pip install -r requirements.txt --disable-pip-version-check
    python -c "import PySide6, serial" 1>nul 2>nul
    if errorlevel 1 (
        echo.
        echo Dependency install FAILED. Please run manually:
        echo     python -m pip install -r requirements.txt
        pause
        exit /b 1
    )
)

rem ---- launch: prefer pythonw (no console window), fallback to python ----
where pythonw >nul 2>nul
if errorlevel 1 (
    python main.py %*
    if errorlevel 1 pause
) else (
    start "ArmGUI" pythonw main.py %*
)
exit /b 0
