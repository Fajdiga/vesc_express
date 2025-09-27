# post_build.py
import os
import shutil
import sys
from datetime import datetime, timezone
import time

build_dir = sys.argv[1]
project_name = sys.argv[2]

# UTC timestamp with Z suffix (Zulu time)
timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%MZ")

# Set your desired filename prefix here
custom_prefix = "jf_bms32_ntc_"
custom_name = f"{custom_prefix}{timestamp}.bin"

# Paths
src_bin = os.path.join(build_dir, f"{project_name}.bin")
dst_bin = os.path.join(build_dir, custom_name)

# Wait for the .bin file to appear (up to 5 seconds)
for i in range(10):
    if os.path.exists(src_bin):
        break
    time.sleep(0.5)

if not os.path.exists(src_bin):
    print(f"[post_build] ERROR: Source binary does not exist after waiting: {src_bin}")
    sys.exit(1)

shutil.copyfile(src_bin, dst_bin)
print(f"[post_build] Copied {src_bin} -> {dst_bin}")