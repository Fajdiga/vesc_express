# post_build.py
import os
import shutil
import sys
import time
from datetime import datetime

build_dir = sys.argv[1]
project_name = sys.argv[2]

# Set your desired filename prefix here (use env var or default to project name)
custom_prefix = os.environ.get('BIN_PREFIX', project_name)
timestamp = os.environ.get('TIMESTAMP', datetime.now().strftime('%Y%m%d_%H%M%S'))
custom_name = f"{custom_prefix}_{timestamp}.bin"

# Paths
src_bin = os.path.join(build_dir, f"{project_name}.bin")
destination_dir = os.path.join(f"{build_dir}/../output" , timestamp)
print(destination_dir)
# Create the destination directory if it doesn't exist
os.makedirs(destination_dir, exist_ok=True)
dst_bin = os.path.join(destination_dir, custom_name)

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