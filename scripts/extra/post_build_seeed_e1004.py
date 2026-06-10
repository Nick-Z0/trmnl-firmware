Import("env")
import subprocess
from pathlib import Path

# Post-build merge for the Seeed reTerminal E1004 (ESP32-S3 N32R8).
#
# This board has 32 MB of QIO flash (not the 4 MB assumed by
# post_build_seeed.py), so it needs its own merge parameters:
#   - --flash_size 32MB
#   - QIO flash mode @ 80 MHz (matches boards/esp32s3_n32r8.json)
#
# Note: the merged image contains the bootloader, partition table and the
# application only.  SPIFFS (the cached-image filesystem) is flashed
# separately via the normal `pio run -t uploadfs` flow.

def post_build(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    output = build_dir / "merged_firmware.bin"

    subprocess.run([
        "pio", "pkg", "exec", "-p", "tool-esptoolpy", "esptool.py", "--",
        "--chip", "ESP32S3",
        "merge_bin",
        "-o", str(output),
        "--flash_mode", "qio",
        "--flash_freq", "80m",
        "--flash_size", "32MB",
        "0x0000", str(build_dir / "bootloader.bin"),
        "0x8000", str(build_dir / "partitions.bin"),
        "0x10000", str(build_dir / "firmware.bin"),
    ], check=True)

    print(f"Merged firmware (E1004, 32MB): {output}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build)
