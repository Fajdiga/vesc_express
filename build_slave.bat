@echo off
REM Build JFBMS Slave firmware
echo Building JFBMS Slave...
set HW_SRC=hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.c
set HW_HEADER=hwconf/jetfleet/jfbms_slave/hw_jfbms_slave.h
idf.py -B build_slave build
if %errorlevel% equ 0 (
    echo.
    echo Build complete: build_slave\vesc_express.bin
    echo To flash: idf.py -B build_slave flash
)
