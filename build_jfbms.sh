#!/usr/bin/env sh
set -e

TIMESTAMP=$(date -u +"%Y%m%d_%H%MZ")
OUT_DIR="output/${TIMESTAMP}"
mkdir -p "${OUT_DIR}"

build_one() {
    hw_name="$1"
    build_dir="$2"
    bin_prefix="$3"

    idf.py -B "${build_dir}" \
        -DIDF_TARGET=esp32c3 \
        -DHW_NAME="${hw_name}" \
        -DSDKCONFIG="${build_dir}/sdkconfig" \
        build

    cp "${build_dir}/github-code.bin" "${OUT_DIR}/${bin_prefix}.bin"
}

build_one "JFBMS32v1" "build_jfbms32_v1" "jfbms32_v1"
build_one "JFBMS32v2" "build_jfbms32_v2" "jfbms32_v2"

cp "build_jfbms32_v2/partition_table/partition-table.bin" "${OUT_DIR}/partition-table.bin"
cp "build_jfbms32_v2/bootloader/bootloader.bin" "${OUT_DIR}/bootloader.bin"

zip -r -j "output/${TIMESTAMP}.zip" "${OUT_DIR}"
