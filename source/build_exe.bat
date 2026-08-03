@echo off
REM ============================================================
REM  DrEmpower Arm GUI - build standalone exe
REM  Run from the arm_gui directory (double click, or in PyCharm terminal)
REM ============================================================
setlocal
cd /d "%~dp0"

echo.
echo [1/5] checking opencv-contrib (aruco must survive)
python -c "import cv2,sys; ok=hasattr(cv2,'aruco'); print('  cv2',cv2.__version__,'aruco',ok); sys.exit(0 if ok else 1)"
if errorlevel 1 (
  echo.
  echo   ERROR: cv2.aruco missing - plain opencv-python is installed.
  echo   Fix with:
  echo       pip uninstall -y opencv-python opencv-python-headless
  echo       pip install --force-reinstall opencv-contrib-python==4.10.0.84
  goto :fail
)

echo.
echo [2/5] installing PyInstaller if needed
python -c "import PyInstaller" 2>nul || pip install pyinstaller

echo.
echo [3/5] cleaning previous build
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

echo.
echo [4/5] building (this takes a few minutes)
pyinstaller ArmGUI.spec --noconfirm
if errorlevel 1 goto :fail

echo.
echo [5/5] checking single-file output
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
