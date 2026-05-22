"""
PlatformIO pre-upload script — 1200bps CDC touch for ESP32-S3 bootloader entry.

ESP32-S3 with USB CDC requires a 1200bps serial open/close (CDC "touch") to
trigger the USB bootloader. Without this, esptool cannot connect when the
device is running normal firmware.
"""
Import("env")
import serial, time

def before_upload(source, target, env):
    port = env.GetProjectOption("upload_port", None)
    if not port:
        print("[pre_upload] No upload_port configured, skipping 1200bps touch")
        return
    print(f"[pre_upload] 1200bps CDC touch on {port} ...")
    try:
        s = serial.Serial(port, 1200, timeout=1)
        time.sleep(0.1)
        s.close()
        time.sleep(2.5)
        print("[pre_upload] Done — bootloader ready")
    except Exception as e:
        print(f"[pre_upload] Warning: {e}")

env.AddPreAction("upload", before_upload)
