@echo off
REM ============================================================
REM  DrEmpower Arm GUI - build standalone exe
REM  Run from the source directory (double click, or in a terminal)
REM ============================================================
setlocal
cd /d "%~dp0"

set "PYTHON_EXE=python"
if exist ".venv\Scripts\python.exe" set "PYTHON_EXE=.venv\Scripts\python.exe"

echo.
echo [1/6] installing pinned runtime and build dependencies
"%PYTHON_EXE%" -m pip install -r requirements.txt -r requirements-build.txt --disable-pip-version-check
if errorlevel 1 goto :fail

echo.
echo [2/6] checking opencv-contrib (aruco must survive)
"%PYTHON_EXE%" -c "import cv2,sys; ok=hasattr(cv2,'aruco'); print('  cv2',cv2.__version__,'aruco',ok); sys.exit(0 if ok else 1)"
if errorlevel 1 (
  echo.
  echo   ERROR: cv2.aruco missing - plain opencv-python is installed.
  echo   Fix with:
  echo       pip uninstall -y opencv-python opencv-python-headless
  echo       pip install --force-reinstall opencv-contrib-python==4.10.0.84
  goto :fail
)

echo.
echo [3/6] checking the pinned PyInstaller environment
"%PYTHON_EXE%" -c "import PyInstaller; print('  PyInstaller', PyInstaller.__version__)"
if errorlevel 1 goto :fail

echo.
echo [4/6] cleaning previous build
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

echo.
echo [5/6] building (this takes a few minutes)
"%PYTHON_EXE%" -m PyInstaller --clean --noconfirm ArmGUI.spec
if errorlevel 1 goto :fail

echo.
echo [6/6] checking single-file output
if not exist dist\ArmGUI.exe goto :fail

echo.
echo ============================================================
echo  BUILD OK
echo    executable    : dist\ArmGUI.exe
echo.
echo  To distribute: copy only dist\ArmGUI.exe.
echo  The target PC does NOT need Python installed.
echo ============================================================
goto :end

:fail
echo.
echo  BUILD FAILED - see messages above.
exit /b 1

:end
endlocal
pause
