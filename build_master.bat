@echo off
REM Build JFBMS Master firmware
echo Building JFBMS Master...
idf.py -B build_master -DIDF_TARGET=esp32c3 -DHW_NAME=JFBMS_MASTER -DSDKCONFIG=build_master/sdkconfig build
if %errorlevel% equ 0 (
    echo.
    echo Build complete: build_master\
    echo To flash: idf.py -B build_master flash
)
