#!/usr/bin/env python3
import os
import sys
import subprocess
import glob
import re
import shutil
import threading
import time

# Color setup
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Persistent status bar
# Uses a terminal scroll region so idf output scrolls only in rows 1..N-1
# and the last row is permanently reserved — no cursor save/restore bleed.

_status_text = ""
_spinner_frames = "|/-\\"
_spinner_idx = 0
_spinner_stop = threading.Event()
_spinner_thread = None

def _rows():
    return shutil.get_terminal_size().lines

def _cols():
    return shutil.get_terminal_size().columns

def _spinner_loop():
    global _spinner_idx
    while not _spinner_stop.is_set():
        _spinner_idx = (_spinner_idx + 1) % len(_spinner_frames)
        _draw_status()
        time.sleep(0.1)

def init_status():
    """Reserve the bottom row and start the spinner thread."""
    global _spinner_thread
    if not sys.stdout.isatty():
        return
    rows = _rows()
    sys.stdout.write(
        f"\033[1;{rows - 1}r"  # scroll region = all rows except last
        f"\033[{rows - 1};1H"  # place cursor at last line of scroll region
    )
    sys.stdout.flush()
    _spinner_stop.clear()
    _spinner_thread = threading.Thread(target=_spinner_loop, daemon=True)
    _spinner_thread.start()

def set_status(text):
    """Update the status bar text (spinner redraws automatically)."""
    global _status_text
    _status_text = text

def _draw_status():
    """Render the status bar in the reserved bottom row."""
    if not sys.stdout.isatty():
        return
    rows, cols = _rows(), _cols()
    spin = _spinner_frames[_spinner_idx]
    bar_text = f"{spin} {_status_text}"
    pad = max(0, cols - len(bar_text))
    bar = f"{Colors.BOLD}{Colors.OKBLUE}{bar_text}{' ' * pad}{Colors.ENDC}"
    sys.stdout.write(
        f"\033[s"              # save cursor
        f"\033[{rows};1H"     # jump to reserved last row
        f"\033[2K"            # clear it
        f"{bar}"
        f"\033[u"             # restore cursor (stays in scroll region)
    )
    sys.stdout.flush()

def clear_status():
    """Stop the spinner, restore normal scroll region, and clear the status bar."""
    _spinner_stop.set()
    if not sys.stdout.isatty():
        return
    rows, cols = _rows(), _cols()
    sys.stdout.write(
        f"\033[r"             # reset scroll region to full terminal
        f"\033[{rows};1H"    # go to last row
        f"\033[2K"           # clear status bar
        "\n"                  # ensure we're past it
    )
    sys.stdout.flush()

def print_status(msg, color=Colors.OKBLUE):
    print(f"{color}{msg}{Colors.ENDC}")
    _draw_status()

def run_streamed(cmd, **kwargs):
    """Run a command, streaming stdout+stderr, redrawing the status bar each line."""
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        **kwargs
    )
    for line in proc.stdout:
        sys.stdout.write(line)
        _draw_status()
    proc.wait()
    return proc

# Hardware config discovery
def get_hw_configs():
    configs = []
    files = glob.glob("main/hwconf/**/hw_*.h", recursive=True)

    for f in files:
        hw_name = None
        hw_target = None
        try:
            with open(f, 'r') as header:
                content = header.read()
                name_match = re.search(r'#define\s+HW_NAME\s+"(.*?)"', content)
                if name_match:
                    hw_name = name_match.group(1)
                target_match = re.search(r'#define\s+HW_TARGET\s+"(.*?)"', content)
                if target_match:
                    hw_target = target_match.group(1)
            if hw_name and hw_target:
                configs.append({
                    'name': hw_name,
                    'target': hw_target,
                    'file': f
                })
        except Exception as e:
            print(f"Error parsing {f}: {e}")
    # Sort by target (SoC) first, then by name — groups same-chip builds together
    configs.sort(key=lambda x: (x['target'], x['name']))
    return configs

def safe_name(name):
    safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', name).strip('_')
    return safe if safe else "hw"

def idf_command():
    idf_py = os.environ.get("IDF_PY")
    if idf_py:
        return [idf_py]

    idf_path = os.environ.get("IDF_PATH")
    if idf_path:
        idf_py = os.path.join(idf_path, "tools", "idf.py")
        if os.path.exists(idf_py):
            return [sys.executable, idf_py]

    return ["idf.py"]

def get_idf_version(idf_path):
    version_header = os.path.join(idf_path, "components", "esp_common", "include", "esp_idf_version.h")
    try:
        with open(version_header, "r", encoding="utf-8") as f:
            content = f.read()
        major = re.search(r'#define\s+ESP_IDF_VERSION_MAJOR\s+(\d+)', content).group(1)
        minor = re.search(r'#define\s+ESP_IDF_VERSION_MINOR\s+(\d+)', content).group(1)
        patch = re.search(r'#define\s+ESP_IDF_VERSION_PATCH\s+(\d+)', content).group(1)
        return f"{major}.{minor}.{patch}"
    except Exception:
        return None

def idf_environment():
    env = os.environ.copy()

    idf_path = env.get("IDF_PATH")
    if idf_path:
        version = get_idf_version(idf_path)
        if version:
            env.setdefault("ESP_IDF_VERSION", version)

    python_env_path = os.path.dirname(os.path.dirname(sys.executable))
    if os.path.exists(os.path.join(python_env_path, "pyvenv.cfg")):
        env.setdefault("IDF_PYTHON_ENV_PATH", python_env_path)

        tools_path = os.path.abspath(os.path.join(python_env_path, "..", "..", ".."))
        if os.path.exists(tools_path):
            env.setdefault("IDF_TOOLS_PATH", tools_path)

            path_dirs = []
            for pattern in [
                os.path.join(tools_path, "cmake", "*", "bin"),
                os.path.join(tools_path, "ninja", "*"),
                os.path.join(tools_path, "riscv32-esp-elf", "*", "riscv32-esp-elf", "bin"),
                os.path.join(tools_path, "xtensa-esp-elf", "*", "xtensa-esp-elf", "bin"),
            ]:
                path_dirs.extend(sorted(glob.glob(pattern), reverse=True))

            if path_dirs:
                env["PATH"] = os.pathsep.join(path_dirs + [env.get("PATH", "")])

    return env

def remove_build_dir(build_dir, build_root):
    abs_build_dir = os.path.abspath(build_dir)
    abs_build_root = os.path.abspath(build_root)
    common = os.path.commonpath([abs_build_dir, abs_build_root])

    if common != abs_build_root or abs_build_dir == abs_build_root:
        raise RuntimeError(f"Refusing to remove unexpected build dir: {abs_build_dir}")

    if os.path.exists(abs_build_dir):
        shutil.rmtree(abs_build_dir)

def build_target(config, output_dir, fresh=False, idx=0, total=0):
    build_root = os.path.join(output_dir, "_build_idf6")
    build_dir = os.path.join(build_root, config['target'], safe_name(config['name']))
    sdkconfig = os.path.abspath(os.path.join(build_dir, "sdkconfig"))
    if fresh:
        remove_build_dir(build_dir, build_root)
    os.makedirs(build_dir, exist_ok=True)

    print_status(f"\n========================================")
    print_status(f"Building: {config['name']} ({config['target']})")
    print_status(f"Config: {config['file']}")
    print_status(f"Dir: {build_dir}")
    print_status(f"========================================")

    cmd_base = idf_command() + [
        "-B", build_dir,
        f"-DIDF_TARGET={config['target']}",
        f"-DHW_NAME={config['name']}",
        f"-DSDKCONFIG={sdkconfig}",
    ]

    set_status(f"{idx}/{total} | {config['name']} ({config['target']}) | Building")
    print_status("--> Building...")
    res = run_streamed(cmd_base + ["build"], shell=False, env=idf_environment())

    if res.returncode == 0:
        print_status(f"SUCCESS: {config['name']}", Colors.OKGREEN)

        set_status(f"{idx}/{total} | {config['name']} ({config['target']}) | Copying artifacts")
        try:
            target_output_dir = os.path.join(output_dir, config['target'], config['name'])
            os.makedirs(target_output_dir, exist_ok=True)

            firmware_bins = [
                p for p in glob.glob(os.path.join(build_dir, "*.bin"))
                if os.path.isfile(p)
            ]
            if not firmware_bins:
                raise FileNotFoundError(f"Missing firmware artifact in {build_dir}")

            src_bin = max(firmware_bins, key=os.path.getmtime)
            src_boot = os.path.join(build_dir, "bootloader", "bootloader.bin")
            src_pt = os.path.join(build_dir, "partition_table", "partition-table.bin")

            shutil.copy2(src_bin, os.path.join(target_output_dir, "vesc_express.bin"))
            shutil.copy2(src_bin, os.path.join(target_output_dir, os.path.basename(src_bin)))
            print_status(f"--> Copied firmware to {target_output_dir}")

            if not os.path.exists(src_boot):
                raise FileNotFoundError(f"Missing artifact: {src_boot}")
            shutil.copy2(src_boot, os.path.join(target_output_dir, "bootloader.bin"))
            print_status(f"--> Copied bootloader to {target_output_dir}")

            if not os.path.exists(src_pt):
                raise FileNotFoundError(f"Missing artifact: {src_pt}")
            shutil.copy2(src_pt, os.path.join(target_output_dir, "partition-table.bin"))
            print_status(f"--> Copied partition table to {target_output_dir}")

        except Exception as e:
            print_status(f"FAILED to copy artifacts for {config['name']}: {e}", Colors.FAIL)
            return False

        return True
    else:
        print_status(f"FAILED: {config['name']}", Colors.FAIL)
        return False

def main():
    if not os.path.exists("main/hwconf"):
        print_status("Error: main/hwconf directory not found", Colors.FAIL)
        sys.exit(1)

    # This is the firmware stub string
    res_firmwares_string = '        <file>TARGET_DESTINATION_DIRECTORY/TARGET_DESTINATION_FILENAME</file>\n'

    # This is the XML stub string
    resource_xml_stub_string = '''
<RCC>
   <qresource prefix="/res/firmwares_esp/">
REPLACEABLE_STRING
   </qresource>
</RCC>
'''

    # Declare an empty string
    res_string = ""

    configs = get_hw_configs()
    args = sys.argv[1:]
    if "--list" in args:
        for config in configs:
            print(f"{config['target']}\t{config['name']}\t{config['file']}")
        sys.exit(0)

    fresh = "--fresh" in args or "--clean" in args
    args = [arg for arg in args if arg not in ("--fresh", "--clean")]

    # Prepare output directory
    output_dir = "build_output"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print_status(f"Created output directory: {output_dir}")

    filters = [arg.lower() for arg in args]
    if filters:
        configs = [
            config for config in configs
            if any(
                filt == config['target'].lower()
                or filt in config['name'].lower()
                or filt in config['file'].lower()
                for filt in filters
            )
        ]

    total = len(configs)
    print_status(f"Found {total} hardware configurations.")

    success_count = 0
    failed_configs = []
    init_status()

    try:
        for idx, config in enumerate(configs, start=1):
            set_status(f"{idx}/{total} | {config['name']} ({config['target']}) | Starting")
            if build_target(config, output_dir, fresh, idx, total):
                success_count += 1

                target_res_string = res_firmwares_string.replace("TARGET_DESTINATION_DIRECTORY",
                    config['target']).replace("TARGET_DESTINATION_FILENAME", config['name'] + "/bootloader.bin")
                res_string = res_string + target_res_string
                target_res_string = res_firmwares_string.replace("TARGET_DESTINATION_DIRECTORY",
                    config['target']).replace("TARGET_DESTINATION_FILENAME", config['name'] + "/partition-table.bin")
                res_string = res_string + target_res_string
                target_res_string = res_firmwares_string.replace("TARGET_DESTINATION_DIRECTORY",
                    config['target']).replace("TARGET_DESTINATION_FILENAME", config['name'] + "/vesc_express.bin")
                res_string = res_string + target_res_string
            else:
                failed_configs.append(config['name'])
    except KeyboardInterrupt:
        clear_status()
        print_status("\nBuild interrupted by user.", Colors.WARNING)
        sys.exit(1)

    clear_status()

    print("\n" + "="*40)
    print(f"Build Summary: {success_count}/{total} Succeeded")
    print(f"Artifacts: {os.path.abspath(output_dir)}")

    with open(os.path.join(output_dir, 'res_fw.qrc'), 'w') as f:
        print(resource_xml_stub_string.replace("REPLACEABLE_STRING", res_string[:-1]), file=f)

    if failed_configs:
        print_status(f"Failed: {', '.join(failed_configs)}", Colors.FAIL)
        sys.exit(1)
    else:
        print_status("All builds successful!", Colors.OKGREEN)
        sys.exit(0)

if __name__ == "__main__":
    main()
