#!/usr/bin/env python3
"""
ruuvi-simulator.py - Simulates Ruuvi tag BLE advertisements via MQTT

This script can be used to test the paku-core → paku-iot pipeline without
actual Ruuvi hardware. It generates realistic sensor data and publishes
it to MQTT topics that mimic what a real integration would produce.

Usage:
    python3 ruuvi-simulator.py --broker localhost --port 1883

Requirements:
    pip install paho-mqtt

Note: This simulates the OUTPUT (parsed Ruuvi data to MQTT), not the BLE
advertisement itself. For BLE simulation, you would need additional hardware
or the Bleak library on a BLE-capable computer.
"""

import argparse
import json
import random
import time
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Error: paho-mqtt not installed. Run: pip install paho-mqtt")
    exit(1)


class RuuviSimulator:
    """Simulates a Ruuvi tag with realistic sensor data."""

    def __init__(self, mac_address: str, name: str = "Simulated Ruuvi"):
        self.mac_address = mac_address
        self.name = name
        # Base values with small variations
        self.base_temperature = 22.0
        self.base_humidity = 50.0
        self.base_pressure = 1013.25
        self.battery_voltage = 3.0

    def get_reading(self) -> dict:
        """Generate a simulated Ruuvi sensor reading."""
        timestamp = datetime.now().strftime("%H:%M:%S")

        return {
            "mac": self.mac_address,
            "name": self.name,
            "temperature": round(self.base_temperature + random.uniform(-2, 2), 2),
            "humidity": round(self.base_humidity + random.uniform(-5, 5), 1),
            "pressure": round(self.base_pressure + random.uniform(-5, 5), 2),
            "acceleration_x": round(random.uniform(-100, 100), 0),
            "acceleration_y": round(random.uniform(-100, 100), 0),
            "acceleration_z": round(random.uniform(900, 1100), 0),  # ~1g
            "battery_voltage": round(self.battery_voltage + random.uniform(-0.1, 0.1), 2),
            "tx_power": 4,
            "movement_counter": random.randint(0, 255),
            "measurement_sequence": random.randint(0, 65535),
            "timestamp": timestamp,
            "rssi": random.randint(-80, -40),
        }


def create_paku_payload(value: float, timestamp: str) -> str:
    """Create a paku-core compatible JSON payload."""
    return json.dumps({"value": value, "timestamp": timestamp})


def on_connect(client, userdata, flags, rc, properties=None):
    """Callback when connected to MQTT broker."""
    if rc == 0:
        print(f"Connected to MQTT broker")
    else:
        print(f"Failed to connect, return code {rc}")


def main():
    parser = argparse.ArgumentParser(
        description="Simulate Ruuvi tag data via MQTT"
    )
    parser.add_argument(
        "--broker", "-b",
        default="localhost",
        help="MQTT broker hostname (default: localhost)"
    )
    parser.add_argument(
        "--port", "-p",
        type=int,
        default=1883,
        help="MQTT broker port (default: 1883)"
    )
    parser.add_argument(
        "--interval", "-i",
        type=float,
        default=5.0,
        help="Publishing interval in seconds (default: 5.0)"
    )
    parser.add_argument(
        "--location", "-l",
        default="cabin",
        help="Location name for topics (default: cabin)"
    )
    parser.add_argument(
        "--mac",
        default="E3:F5:6A:1B:2C:3D",
        help="Simulated Ruuvi MAC address"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print published messages"
    )

    args = parser.parse_args()

    # Create MQTT client
    client = mqtt.Client(
        client_id="ruuvi-simulator",
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2
    )
    client.on_connect = on_connect

    # Create simulated Ruuvi tag
    ruuvi = RuuviSimulator(args.mac, f"Ruuvi {args.location.capitalize()}")

    print(f"Ruuvi Simulator")
    print(f"===============")
    print(f"Broker: {args.broker}:{args.port}")
    print(f"MAC: {args.mac}")
    print(f"Location: {args.location}")
    print(f"Interval: {args.interval}s")
    print()

    try:
        client.connect(args.broker, args.port, 60)
        client.loop_start()

        print("Publishing simulated Ruuvi data (Ctrl+C to stop)...")
        print()

        while True:
            reading = ruuvi.get_reading()
            timestamp = reading["timestamp"]

            # Publish to paku-style topics
            topics_and_values = [
                (f"paku/temperature/moko/{args.location}", reading["temperature"]),
                (f"paku/humidity/moko/{args.location}", reading["humidity"]),
                (f"paku/pressure/moko/{args.location}", reading["pressure"]),
                (f"paku/voltage/{args.location}", reading["battery_voltage"]),
            ]

            for topic, value in topics_and_values:
                payload = create_paku_payload(value, timestamp)
                client.publish(topic, payload)

                if args.verbose:
                    print(f"  {topic}: {payload}")

            # Also publish raw Ruuvi data for debugging
            raw_topic = f"ruuvi/{args.mac.replace(':', '').lower()}/raw"
            client.publish(raw_topic, json.dumps(reading))

            if args.verbose:
                print(f"  {raw_topic}: {json.dumps(reading)}")
                print()
            else:
                print(f"[{timestamp}] Published: temp={reading['temperature']}°C, "
                      f"humidity={reading['humidity']}%, "
                      f"pressure={reading['pressure']}hPa")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nStopping simulator...")
    except ConnectionRefusedError:
        print(f"Error: Could not connect to MQTT broker at {args.broker}:{args.port}")
        print("Make sure the broker is running and accessible.")
        return 1
    finally:
        client.loop_stop()
        client.disconnect()

    print("Simulator stopped.")
    return 0


if __name__ == "__main__":
    exit(main())
