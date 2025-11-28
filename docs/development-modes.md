# Development Modes

This workspace supports **two development modes**: container-based and local (host) development. While the required configuration changes are minimal, supporting both modes significantly improves the developer experience, especially for embedded workflows that require physical USB device access.

## Quick Reference

| Mode | Best For | USB Device Access |
|------|----------|-------------------|
| **Container** | Isolated, reproducible builds; no local toolchain setup | Limited (requires Docker USB passthrough) |
| **Local** | Flashing/debugging physical ESP32 devices | Full native access |

---

## Container Development

Use the development container for an isolated, pre-configured environment.

### How to Open

1. Open VS Code
2. Open the repository folder or `paku_core.code-workspace`
3. When prompted, click **"Reopen in Container"**
   - Or use Command Palette: `Dev Containers: Reopen in Container`

### Features

- PlatformIO pre-installed
- All required VS Code extensions installed
- Consistent environment across machines

### USB Device Access (Container)

USB passthrough in containers is **platform-dependent** and may not work on all systems:

- **Linux**: Usually works with `--privileged` mode (already configured)
- **macOS**: Docker Desktop has limited USB passthrough support; most ESP32 devices will **not** be accessible without third-party tools
- **Windows**: USB passthrough is generally **not supported** in Docker Desktop; WSL2 may offer limited options with additional configuration

**Security Note**: The container configuration mounts the entire USB bus (`/dev/bus/usb`) which exposes all USB devices. This is necessary for ESP32 development but grants broad device access to the container.

**Recommendation**: For reliable USB flashing, use [Local Development](#local-development) instead.

---

## Local Development

Use local development when you need direct access to USB devices for flashing or debugging the ESP32.

### Prerequisites

- [VS Code](https://code.visualstudio.com/)
- [PlatformIO extension](https://platformio.org/install/ide?install=vscode) (or [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html))
- USB cable for ESP32 connection

### How to Open

1. Open VS Code
2. Open the repository folder or `paku_core.code-workspace`
3. **Do NOT** reopen in container—stay on the host machine

### USB Device Access

Your host machine has direct access to USB ports. Simply connect the ESP32 via USB and use PlatformIO to upload:

```bash
cd paku_core
pio run -t upload
```

---

## Choosing the Right Mode

### Use Container When:
- Setting up a new development environment quickly
- Building and testing code without physical devices
- Ensuring consistent builds across team members
- CI/CD pipelines

### Use Local When:
- Flashing firmware to the physical ESP32
- Debugging with hardware-in-the-loop
- Using serial monitor with physical devices
- USB passthrough is unreliable on your platform

---

## Working with paku-iot

Both `paku-core` (EDGE firmware) and `paku-iot` (host-side services) can be developed within the same workspace structure. The development mode you choose affects only how you interact with `paku-core`:

- **paku-iot**: Runs on the host or in containers—no USB requirements
- **paku-core**: Requires USB for flashing; prefer local development for this

You can:
1. Develop `paku-iot` in a container
2. Switch to local mode (or a separate VS Code window) for `paku-core` USB operations

---

## Technical Notes

The workspace configuration includes:

- **`.devcontainer/devcontainer.json`**: Container configuration with PlatformIO and USB passthrough attempts
- **`paku_core.code-workspace`**: Multi-root workspace supporting both modes
- **Privileged mode**: Enabled in the container for USB access on Linux

### Why Minimal Changes Matter

> While the actual changes to support both modes are minor (a devcontainer.json and workspace settings), the improvement to developer experience is significant. Embedded developers no longer need to choose between containerized toolchains and hardware access—they can use both seamlessly.
