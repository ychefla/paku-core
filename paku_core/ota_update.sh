#!/bin/bash
#
# OTA Update Script for ESP8266 Device
# Usage: ./ota_update.sh [device_id] [firmware_version]
#
# This script:
# 1. Starts a simple HTTP server to host the firmware
# 2. Sends an MQTT command to trigger OTA update on the device
# 3. Monitors MQTT topics for OTA progress and status
#

set -e

# Configuration
DEVICE_ID="${1:-esp8266_wired}"
FIRMWARE_VERSION="${2:-$(date +%Y%m%d-%H%M%S)}"
FIRMWARE_FILE=".pio/build/esp8266-wired-sensors/firmware.bin"
HTTP_PORT=8888
MQTT_BROKER="localhost"
MQTT_PORT=1883

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}ESP8266 OTA Update Script${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""

# Check if firmware file exists
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo -e "${RED}Error: Firmware file not found: $FIRMWARE_FILE${NC}"
    echo "Please build the firmware first:"
    echo "  platformio run -e esp8266-wired-sensors"
    exit 1
fi

# Get firmware info
FIRMWARE_SIZE=$(wc -c < "$FIRMWARE_FILE" | tr -d ' ')
FIRMWARE_SHA256=$(shasum -a 256 "$FIRMWARE_FILE" | awk '{print $1}')

echo -e "${GREEN}✓ Firmware file found${NC}"
echo "  File: $FIRMWARE_FILE"
echo "  Size: $FIRMWARE_SIZE bytes ($(echo "scale=2; $FIRMWARE_SIZE/1024" | bc) KB)"
echo "  SHA256: $FIRMWARE_SHA256"
echo ""

# Check if mosquitto_pub is available
if ! command -v mosquitto_pub &> /dev/null; then
    echo -e "${RED}Error: mosquitto_pub not found${NC}"
    echo "Please install mosquitto-clients:"
    echo "  macOS: brew install mosquitto"
    echo "  Ubuntu: sudo apt-get install mosquitto-clients"
    exit 1
fi

# Get local IP address
LOCAL_IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | head -1)
if [ -z "$LOCAL_IP" ]; then
    LOCAL_IP="localhost"
fi

echo -e "${YELLOW}Starting HTTP server...${NC}"
echo "  Host: $LOCAL_IP:$HTTP_PORT"
echo ""

# Start Python HTTP server in background
cd $(dirname "$FIRMWARE_FILE")
python3 -m http.server $HTTP_PORT > /dev/null 2>&1 &
HTTP_SERVER_PID=$!
cd - > /dev/null

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping HTTP server...${NC}"
    kill $HTTP_SERVER_PID 2>/dev/null || true
    exit
}
trap cleanup EXIT INT TERM

# Wait for HTTP server to start
sleep 2

# Test if firmware is accessible
FIRMWARE_URL="http://${LOCAL_IP}:${HTTP_PORT}/firmware.bin"
if curl -s -f -o /dev/null "$FIRMWARE_URL"; then
    echo -e "${GREEN}✓ Firmware accessible at: $FIRMWARE_URL${NC}"
else
    echo -e "${RED}Error: Cannot access firmware at $FIRMWARE_URL${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Sending OTA command to device: $DEVICE_ID${NC}"
echo ""

# Create OTA command JSON
OTA_COMMAND=$(cat <<EOF
{
  "url": "$FIRMWARE_URL",
  "checksum": "$FIRMWARE_SHA256",
  "version": "$FIRMWARE_VERSION"
}
EOF
)

echo "OTA Command:"
echo "$OTA_COMMAND" | jq '.' 2>/dev/null || echo "$OTA_COMMAND"
echo ""

# Send OTA command via MQTT
MQTT_TOPIC="paku/edge/${DEVICE_ID}/cmd/ota"

echo -e "${YELLOW}Publishing to MQTT topic: $MQTT_TOPIC${NC}"
echo "$OTA_COMMAND" | mosquitto_pub -h "$MQTT_BROKER" -p "$MQTT_PORT" -t "$MQTT_TOPIC" -l

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ OTA command sent successfully${NC}"
else
    echo -e "${RED}✗ Failed to send OTA command${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Monitoring OTA progress...${NC}"
echo "Press Ctrl+C to stop monitoring (update will continue on device)"
echo ""
echo "Topics being monitored:"
echo "  - paku/edge/${DEVICE_ID}/ota/status"
echo "  - paku/edge/${DEVICE_ID}/ota/progress"
echo "  - paku/edge/${DEVICE_ID}/ota/result"
echo ""

# Subscribe to OTA topics and monitor progress
mosquitto_sub -h "$MQTT_BROKER" -p "$MQTT_PORT" -v \
    -t "paku/edge/${DEVICE_ID}/ota/status" \
    -t "paku/edge/${DEVICE_ID}/ota/progress" \
    -t "paku/edge/${DEVICE_ID}/ota/result" \
    | while read -r topic message; do
    
    # Parse and format the message
    if echo "$message" | jq . > /dev/null 2>&1; then
        FORMATTED=$(echo "$message" | jq -r '. | "\(.timestamp // "unknown") - \(.status // .state // "unknown") - \(.percent // 0)%"')
        
        case "$topic" in
            */ota/status)
                echo -e "${BLUE}[STATUS]${NC} $FORMATTED"
                ;;
            */ota/progress)
                echo -e "${YELLOW}[PROGRESS]${NC} $FORMATTED"
                ;;
            */ota/result)
                RESULT=$(echo "$message" | jq -r '.status // .result // "unknown"')
                if [ "$RESULT" = "success" ]; then
                    echo -e "${GREEN}[RESULT]${NC} $FORMATTED"
                    echo ""
                    echo -e "${GREEN}========================================${NC}"
                    echo -e "${GREEN}OTA Update Successful!${NC}"
                    echo -e "${GREEN}========================================${NC}"
                    echo "Device should restart with new firmware now."
                    exit 0
                else
                    echo -e "${RED}[RESULT]${NC} $FORMATTED"
                    echo ""
                    echo -e "${RED}========================================${NC}"
                    echo -e "${RED}OTA Update Failed!${NC}"
                    echo -e "${RED}========================================${NC}"
                    exit 1
                fi
                ;;
        esac
    else
        echo "$topic: $message"
    fi
done

# Keep HTTP server running
wait $HTTP_SERVER_PID
