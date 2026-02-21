"""
PlatformIO pre-build script — injects version from git tags.

This script runs automatically before every build (local and CI).
It reads the version from the most recent git tag using:
  git describe --tags --dirty --always

Examples:
  v1.4.2              → FIRMWARE_VERSION="1.4.2"
  v1.4.2-3-gabcdef    → FIRMWARE_VERSION="1.4.2-3-gabcdef"  (dev build)
  v1.4.2-3-gabcdef-d  → FIRMWARE_VERSION="1.4.2-3-gabcdef-d" (dirty)
  gabcdef             → FIRMWARE_VERSION="0.0.0-gabcdef"     (no tags)

The version is injected as a C++ build flag:
  -DFIRMWARE_VERSION="1.4.2"

The #ifndef guard in main.cpp ensures the hardcoded fallback is only
used when this script is not active (e.g., direct IDE builds).
"""

import subprocess
import datetime

Import("env")  # noqa: F821 — PlatformIO environment


def run(cmd):
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, cwd=env.get("PROJECT_DIR")
        )
        return result.stdout.strip()
    except Exception:
        return ""


def get_version():
    raw = run("git describe --tags --dirty=-d --always --match='v[0-9]*'")
    if not raw:
        return "0.0.0-unknown"
    # Strip leading 'v'
    if raw.startswith("v"):
        raw = raw[1:]
    # If it's just a hash (no tags at all), prefix with 0.0.0-
    import re
    if not re.match(r"^\d+\.\d+", raw):
        return f"0.0.0-{raw}"
    return raw


def get_git_hash():
    return run("git rev-parse --short HEAD") or "unknown"


version = get_version()
git_hash = get_git_hash()
build_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

env.Append(CPPDEFINES=[
    ("FIRMWARE_VERSION", env.StringifyMacro(version)),
    ("GIT_HASH",         env.StringifyMacro(git_hash)),
    ("BUILD_TIME",       env.StringifyMacro(build_time)),
])

print(f"  ┌─ Version: {version}")
print(f"  ├─ Git:     {git_hash}")
print(f"  └─ Built:   {build_time}")
