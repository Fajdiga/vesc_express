@echo off
REM Build JFBMS Slave firmware
echo Building JFBMS Slave...
idf.py -B build_slave -DIDF_TARGET=esp32c3 -DHW_NAME=JFBMS_SLAVE -DSDKCONFIG=build_slave/sdkconfig build
if %errorlevel% equ 0 (
    echo.
    echo Build complete: build_slave\
    echo To flash: idf.py -B build_slave flash
)
