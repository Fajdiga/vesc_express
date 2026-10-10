# VESC Express

This is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3, ESP32C6 and ESP32S3 but other ESP32 devices can be added.

## Toolchain

Instructions for how to set up the toolchain can be found here:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html)

### Get Release 6.0.1

The instructions linked above install the latest ESP-IDF branch. To install the supported stable release you can navigate to the installation directory and use the following commands:

```bash
git clone -b v6.0.1 --recursive https://github.com/espressif/esp-idf.git esp-idf-v6.0.1
cd esp-idf-v6.0.1/
./install.sh esp32c3 esp32c6 esp32s3
```

At the moment development is done using the stable 6.0.1 release. Note that different IDF versions are very likely to cause compatibility issues, so it is strongly recommended to use version 6.0.1.

## Building

Set the target chip/architecture with
```bash
idf.py set-target <target> 
```

where target is esp32c3, esp32c6 or esp32s3. You will need to run a fullclean or remove the build directory when changing targets.

Each normal build target uses its own shared 4 MB base file `sdkconfig.defaults.<target>`.

Boards that need non-default flash or PSRAM settings should instead provide their own full `sdkconfig.defaults.<hw_file>` file next to the shared target configs in the repository root.

Once the toolchain is set up in the current path, the project can be built with

```bash
idf.py build
```

That will create vesc_express.bin in the build directory, which can be used with the bootloader in VESC Tool. If the ESP32C3 does not come with firmware preinstalled, the USB-port can be used for flashing firmware using the built-in bootloader. That also requires bootloader.bin and partition-table.bin which also can be found in the build directory. This can be done from VESC Tool or using idf.py.

All targets can be built with

```bash
python build_all.py
```

That will create all required firmware files under the build_output directory, with hardware names as child directories. All target switching is handled automatically with the build_all command.

JetFleet BMS targets bundle their default Lisp applications for automatic
installation and startup on erased devices. Existing uploaded applications are
preserved. JF Link starts its native controller automatically. Pack settings
and slave IDs must match the installed hardware.

JetFleet BMS charge gates do not isolate pack discharge. The ESC must enforce
cell-voltage and temperature discharge limits using BMS CAN data, and reduce or
stop discharge when that data becomes stale or disappears. The JFBMS32 shunt
measures the charge port; ESC input current is added for pack-current reporting.
Slaves use TS1 only on each BQ, with the configured NTC resistance and beta
matching the fitted sensor (10 kOhm by default). Removing their 5 V supply
asserts hardware shutdown. Installers must set unique slave IDs, cell counts
and the master's slave count; unexpected IDs and repeated frames produce
warnings without changing that responsibility.

The standard VESC CAN protocol is preserved. Fresh complete master snapshots
include unsafe measured voltages and temperatures; stale/incomplete snapshots
stop transmitting. [Stock VESC](https://github.com/vedderb/bldc/blob/master/bms.c)
removes BMS-derived limits after two seconds without status data. It therefore
does not provide the missing-data discharge cutoff required above by itself.

Protection updates require both the firmware and the matching board
`*_main.lisp` application. Updating firmware preserves an existing uploaded
Lisp application, so replace it through VESC Tool as part of the update.
The master requires `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y`; its hardware
defaults enable this for new builds, and compilation rejects an older
sdkconfig with it disabled. On JFBMS32 and slaves, `(bms-watchdog-stack)` reports
the watchdog's minimum free stack in bytes for hardware testing.

### Custom Hardware Targets

If you wish to build the project with custom hardware config files you should add the hardware config files to the "**main/hwconf**" directory and use the HW_NAME build flag
```bash
idf.py build -DHW_NAME="VESC Express T"
```

**Note:** If you ever change the environment variables, or if when you first start using them, you need to first run `idf.py reconfigure` before building (with the environment variables still set of course!), as the build system unfortunately can't automatically detect this change. Running `idf.py fullclean` has the same effect as this forces cmake to rebuild the build configurations.
