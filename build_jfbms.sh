export TIMESTAMP=$(date -u +"%Y%m%d_%H%MZ")
mkdir "output/${TIMESTAMP}"

idf.py fullclean

export HW_HEADER=hw_jf_bms32_v1_shutdown.h && 
export HW_SRC=hw_jf_bms32.c && 
export BIN_PREFIX=jf_bms32_v1_shutdown && 
idf.py build

idf.py clean

export HW_HEADER=hw_jf_bms32_v1_noshutdown.h && 
export HW_SRC=hw_jf_bms32.c && 
export BIN_PREFIX=jf_bms32_v1_noshutdown && 
idf.py build

idf.py clean

export HW_HEADER=hw_jf_bms32_v2.h && 
export HW_SRC=hw_jf_bms32.c && 
export BIN_PREFIX=jf_bms32_v2 && 
idf.py build

cp "build/partition_table/partition-table.bin" "output/${TIMESTAMP}/partition-table.bin"
cp "build/bootloader/bootloader.bin" "output/${TIMESTAMP}/bootloader.bin"

zip -r -j "output/${TIMESTAMP}.zip" "output/${TIMESTAMP}"