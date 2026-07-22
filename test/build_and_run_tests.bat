@echo off
REM Build and run unit tests for STM32H7 Autonomy Demo
REM Requires: MinGW-w64 or any GCC with C++11 support

setlocal enabledelayedexpansion

echo ========================================
echo  Building Unit Tests...
echo ========================================
echo.

set "TEST_DIR=%~dp0"
set "SRC_DIR=%TEST_DIR%..\src"

set "SOURCES=%TEST_DIR%test_main.cpp"
set "SOURCES=%SOURCES% %SRC_DIR%\components\pid_controller.cpp"
set "SOURCES=%SOURCES% %SRC_DIR%\components\flight_controller.cpp"
set "SOURCES=%SOURCES% %SRC_DIR%\components\command_receiver.cpp"
set "SOURCES=%SOURCES% %SRC_DIR%\components\error_manager.cpp"
set "SOURCES=%SOURCES% %SRC_DIR%\components\autonomy_controller.cpp"
set "SOURCES=%SOURCES% %SRC_DIR%\components\camera_stream.cpp"
set "SOURCES=%SOURCES% %SRC_DIR%\drivers\motor_mixer.cpp"

set "INCLUDES=-I%TEST_DIR%..\include -I%TEST_DIR%"

REM Try different compilers
where g++ >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    set "COMPILER=g++"
    goto :compile
)

where clang++ >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    set "COMPILER=clang++"
    goto :compile
)

echo ERROR: No C++ compiler found!
echo Please install MinGW-w64 from: https://www.mingw-w64.org/
exit /b 1

:compile
echo Compiler: %COMPILER%
echo Sources: %SOURCES%
echo.

%COMPILER% -std=c++11 -Wall -Wno-unused-variable -Wno-unused-parameter %INCLUDES% %SOURCES% -o %TEST_DIR%test_drone.exe

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [FAILED] Compilation failed with error code %ERRORLEVEL%
    exit /b %ERRORLEVEL%
)

echo [OK] Compilation successful
echo.
echo ========================================
echo  Running Unit Tests...
echo ========================================
echo.

%TEST_DIR%test_drone.exe

if %ERRORLEVEL% EQU 0 (
    echo.
    echo [PASSED] All tests passed!
) else (
    echo.
    echo [FAILED] Some tests failed!
)

exit /b %ERRORLEVEL%
