set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

# Detect if building for slave hardware (no WiFi/BLE needed)
set(IS_SLAVE_BUILD FALSE)
if(DEFINED HW_FILE_NAME)
    string(FIND "${HW_FILE_NAME}" "jfbms_slave" SLAVE_POS)
    if(NOT SLAVE_POS EQUAL -1)
        set(IS_SLAVE_BUILD TRUE)
        message(STATUS "Detected slave build - excluding WiFi/BLE modules")
    endif()
endif()

set(COMPONENT_SRCS
"main.c"
"buffer.c"
"comm_uart.c"
"comm_usb.c"
"comm_can.c"
"packet.c"
"crc.c"
"commands.c"
"terminal.c"
"mempools.c"
"log.c"
"adc.c"
"ublox.c"
"nmea.c"
"utils.c"
"flash_helper.c"
"rb.c"
"bms.c"
"log_comm.c"
"digital_filter.c"

"config/confsrc.c"
"hwconf/hw.c"

"lispif.c"
"lispif_events.c"
"lbm_vesc_utils.c"
"lispif_vesc_extensions.c"
"lbm_color_extensions.c"
"lispBM/src/env.c"
"lispBM/src/fundamental.c"
"lispBM/src/heap.c"
"lispBM/src/lbm_memory.c"
"lispBM/src/print.c"
"lispBM/src/stack.c"
"lispBM/src/symrepr.c"
"lispBM/src/tokpar.c"
"lispBM/src/extensions.c"
"lispBM/src/lispbm.c"
"lispBM/src/eval_cps.c"
"lispBM/platform/freertos/src/platform_mutex.c"
"lispBM/platform/freertos/src/platform_timestamp.c"
"lispBM/src/lbm_channel.c"
"lispBM/src/lbm_c_interop.c"
"lispBM/src/lbm_custom_type.c"
"lispBM/src/lbm_flat_value.c"
"lispBM/src/lbm_prof.c"
"lispBM/src/lbm_defrag_mem.c"
"lispBM/src/lbm_image.c"
"lispBM/src/extensions/array_extensions.c"
"lispBM/src/extensions/math_extensions.c"
"lispBM/src/extensions/string_extensions.c"
"lispBM/src/extensions/display_extensions.c"
"lispBM/src/extensions/tjpgd.c"
"lispBM/src/extensions/mutex_extensions.c"
"lispBM/src/extensions/lbm_dyn_lib.c"
"lispBM/src/extensions/ttf_extensions.c"
"lispBM/src/extensions/schrift.c"
"lispBM/src/extensions/ttf_backend.c"
)

# Slave has no WiFi/BLE hardware - use stubs. All other builds (master, etc.) get real modules.
if(IS_SLAVE_BUILD)
    list(APPEND COMPONENT_SRCS "comm_wifi_ble_stub.c")
else()
    list(APPEND COMPONENT_SRCS
        "comm_wifi.c"
        "wifi/lispif_wifi_extensions.c"
        "ble/lispif_ble_extensions.c"
        "rgbled/lispif_rgbled_extensions.c"
    )

    # Select BLE host stack implementation files at build time.
    # NimBLE is used on ESP32-C6 (smaller flash footprint); Bluedroid on
    # everything else (C3, S3, …) so existing hardware is unaffected.
    if(CONFIG_BT_NIMBLE_ENABLED)
        list(APPEND COMPONENT_SRCS
            "comm_ble_nimble.c"
            "ble/custom_ble_nimble.c"
        )
    else()
        list(APPEND COMPONENT_SRCS
            "comm_ble_bluedroid.c"
            "ble/custom_ble_bluedroid.c"
        )
    endif()
endif()

list(APPEND COMPONENT_SRCS

"touch/lispif_touch_extensions.c"

"display/lispif_disp_extensions.c"
"display/disp_sh8501b.c"
"display/disp_ili9341.c"
"display/disp_ssd1306.c"
"display/disp_st7789.c"
"display/disp_ili9488.c"
"display/disp_st7735.c"
"display/disp_ssd1351.c"
"display/disp_icna3306.c"
"display/disp_sh8601.c"

"drivers/bme280/bme280.c"
"drivers/bme280/bme280_if.c"
"drivers/hwspi.c"
"drivers/spi_bb.c"
"drivers/enc_as504x.c"
"drivers/imu/imu.c"
"drivers/imu/lsm6ds3.c"
"drivers/imu/ahrs.c"

"lowzip/lowzip.c"
)

set(COMPONENT_ADD_INCLUDEDIRS
"."
"config"
"hwconf"
"hwconf/trampa"
"hwconf/trampa/bms_rb"
"hwconf/vesc"
"hwconf/vesc/vbms32"
"hwconf/jetfleet"
"hwconf/jetfleet/bms32"
"hwconf/jetfleet/jfbms_slave"
"hwconf/jetfleet/jfbms_master"
"hwconf/jetfleet/jfbms_master_c6"
"hwconf/vesc/vdisp_dual"
"hwconf/vesc/scope"
"hwconf/vesc/duet"
"hwconf/vesc/link"
"hwconf/vesc/vdisp_900"
"hwconf/vesc/nanolog"
"hwconf/vesc/vbms16"
"hwconf/vesc/rmcore"
"hwconf/vesc/dash16"
"hwconf/vesc/wcore"
"hwconf/rescue"
"hwconf/avaspark"

"lispBM"
"lispBM/include"
"lispBM/platform/freertos/include"
"display"
"wifi"
"ble"
"rgbled"
"touch"
"drivers"
"drivers/bme280"
"drivers/imu"
"lowzip"
)

register_component()

add_compile_definitions(
    LBM_USE_DYN_FUNS
    LBM_USE_DYN_MACROS
    LBM_USE_DYN_LOOPS
    LBM_USE_DYN_ARRAYS
    LBM_USE_TIME_QUOTA
    LBM_USE_ERROR_LINENO
    LBM_USE_MACRO_REST_ARGS
)
