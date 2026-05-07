@echo off
REM ============================================================================
REM JFBMS Build Script - Builds both Master and Slave firmware
REM ============================================================================
REM Usage: Run this script from ESP-IDF Command Prompt
REM Output:
REM   build_slave/  - JFBMS Slave firmware
REM   build_master/ - JFBMS Master firmware
REM ============================================================================

echo.
echo ============================================
echo    JFBMS Firmware Build Script
echo ============================================
echo.

REM Build JFBMS Slave
echo [1/2] Building JFBMS Slave...
echo ----------------------------------------
idf.py -B build_slave -DIDF_TARGET=esp32c3 -DHW_NAME=JFBMS_SLAVE -DSDKCONFIG=build_slave/sdkconfig build
if %errorlevel% neq 0 (
    echo.
    echo ERROR: Slave build failed!
    exit /b 1
)
echo.
echo Slave build complete: build_slave/
echo.

REM Build JFBMS Master
echo [2/2] Building JFBMS Master...
echo ----------------------------------------
idf.py -B build_master -DIDF_TARGET=esp32c3 -DHW_NAME=JFBMS_MASTER -DSDKCONFIG=build_master/sdkconfig build
if %errorlevel% neq 0 (
    echo.
    echo ERROR: Master build failed!
    exit /b 1
)
echo.
echo Master build complete: build_master/
echo.

echo ============================================
echo    Build Complete!
echo ============================================
echo.
echo Firmware locations:
echo   Slave:  build_slave\vesc_express.bin
echo   Master: build_master\vesc_express.bin
echo.
echo To flash:
echo   Slave:  idf.py -B build_slave flash
echo   Master: idf.py -B build_master flash
echo.
