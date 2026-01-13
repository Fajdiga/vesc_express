@echo off
REM Build JFBMS Master firmware
echo Building JFBMS Master...
set HW_SRC=hwconf/jetfleet/jfbms_master/hw_jfbms_master.c
set HW_HEADER=hwconf/jetfleet/jfbms_master/hw_jfbms_master.h
idf.py -B build_master build
if %errorlevel% equ 0 (
    echo.
    echo Build complete: build_master\vesc_express.bin
    echo To flash: idf.py -B build_master flash
)
