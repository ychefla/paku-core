# Paku Tools

Utility scripts for development, testing, and simulation.

## Available Tools

### ruuvi-simulator.py

Simulates Ruuvi tag BLE data via MQTT for testing the paku-core → paku-iot pipeline without actual hardware.

#### Requirements

```bash
pip install paho-mqtt
```

#### Usage

```bash
# Basic usage (connects to localhost:1883)
python3 ruuvi-simulator.py

# Specify broker and options
python3 ruuvi-simulator.py --broker 192.168.1.100 --port 1883 --location cabin --interval 5

# Verbose mode (shows all published messages)
python3 ruuvi-simulator.py -v
```

#### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--broker`, `-b` | localhost | MQTT broker hostname |
| `--port`, `-p` | 1883 | MQTT broker port |
| `--interval`, `-i` | 5.0 | Publishing interval in seconds |
| `--location`, `-l` | cabin | Location name for topics |
| `--mac` | E3:F5:6A:1B:2C:3D | Simulated Ruuvi MAC address |
| `--verbose`, `-v` | false | Print all published messages |

#### Published Topics

The simulator publishes to paku-style topics:

- `paku/temperature/moko/{location}` - Temperature in °C
- `paku/humidity/moko/{location}` - Humidity in %
- `paku/pressure/moko/{location}` - Pressure in hPa
- `paku/voltage/{location}` - Battery voltage in V

And raw Ruuvi data for debugging:

- `ruuvi/{mac}/raw` - Complete sensor reading as JSON

#### Example Output

```
Ruuvi Simulator
===============
Broker: localhost:1883
MAC: E3:F5:6A:1B:2C:3D
Location: cabin
Interval: 5.0s

Publishing simulated Ruuvi data (Ctrl+C to stop)...

[14:32:15] Published: temp=22.5°C, humidity=52.3%, pressure=1015.2hPa
[14:32:20] Published: temp=21.8°C, humidity=53.1%, pressure=1014.8hPa
[14:32:25] Published: temp=23.1°C, humidity=51.9%, pressure=1015.5hPa
```

## Notes

- These tools are for development and testing purposes
- The simulator produces realistic but synthetic data
- For production testing, use actual Ruuvi hardware
