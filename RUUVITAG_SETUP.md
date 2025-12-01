# RuuviTag Location Setup Guide

## Problem

Your RuuviTags are appearing with auto-generated location names like `tag_0`, `tag_1`, `tag_2` instead of friendly names like "cabin", "kitchen", etc.

## Root Cause

RuuviTags are being auto-discovered rather than pre-registered with friendly names. This happens when:
1. `RUUVI_TAG_COUNT` is not defined in `secrets.h`
2. MAC addresses in `secrets.h` don't match the actual RuuviTag MAC addresses
3. MAC address format is incorrect (must be uppercase with colons)

## Solution

### Step 1: Find Your RuuviTag MAC Addresses

Use the serial monitor output when paku-core starts. You'll see messages like:
```
RuuviTag discovered: AA:BB:CC:DD:EE:FF
```

Or check the MQTT messages to see which MAC addresses are broadcasting.

### Step 2: Update secrets.h

Edit `paku_core/include/secrets.h` and add:

```cpp
// RuuviTag Configuration
#define RUUVI_TAG_COUNT 2  // Set to actual number of tags

static const char* RUUVI_TAG_MACS[] = {
    "AA:BB:CC:DD:EE:01",  // Replace with actual MAC (UPPERCASE with colons!)
    "AA:BB:CC:DD:EE:02"
};

static const char* RUUVI_TAG_LOCATIONS[] = {
    "cabin",     // Friendly location name
    "kitchen"    // Another friendly location name
};
```

**Important:**
- MAC addresses must be in UPPERCASE with colons separating bytes
- `RUUVI_TAG_COUNT` must exactly match the number of entries in both arrays
- Location names should be short, lowercase, no spaces (will be used in MQTT topics)

### Step 3: Rebuild and Upload

```bash
cd paku_core
pio run -t upload
```

### Step 4: Verify

After restart, check serial output:
```
Registering known RuuviTags...
  Registered: AA:BB:CC:DD:EE:01 -> cabin
  Registered: AA:BB:CC:DD:EE:02 -> kitchen
```

And check MQTT topics - they should now be:
- `paku/sensors/cabin/data`
- `paku/sensors/kitchen/data`

Instead of:
- `paku/sensors/tag_0/data`
- `paku/sensors/tag_1/data`

## MQTT Topic Structure

After proper registration, RuuviTag data will be published to:
```
paku/sensors/{location}/data
```

Where `{location}` is the friendly name you set in `RUUVI_TAG_LOCATIONS`.

## Troubleshooting

### Tags Still Show as tag_X

1. **Check MAC address format**: Must be uppercase, e.g., `"AA:BB:CC:DD:EE:FF"` not `"aa:bb:cc:dd:ee:ff"`
2. **Verify RUUVI_TAG_COUNT**: Must match array lengths exactly
3. **Check serial output**: Look for "Registered:" messages on startup
4. **Verify compilation**: Make sure secrets.h changes were saved and code was rebuilt

### How to Find MAC Addresses

1. **From Serial Monitor**: Watch for auto-discovery messages
2. **From MQTT Explorer**: Subscribe to `paku/sensors/+/data` and note the MAC addresses in the payload
3. **From RuuviTag Sticker**: Some tags have MAC printed on them
4. **From Ruuvi Station App**: Shows MAC addresses of nearby tags

### Common Mistakes

❌ **Wrong:** `#define RUUVI_TAG_COUNT 2` but only 1 entry in arrays
✅ **Correct:** Count matches array lengths

❌ **Wrong:** `"aa:bb:cc:dd:ee:ff"` (lowercase)
✅ **Correct:** `"AA:BB:CC:DD:EE:FF"` (uppercase)

❌ **Wrong:** Location = "Living Room" (spaces)
✅ **Correct:** Location = "living_room" or "lounge"

## Database Impact

Once you fix the location names and restart, new data will use the proper location names. However:
- Old data in the database will still have the old `tag_X` device_ids
- Grafana queries will need to use the new device_id values
- You may want to update Grafana dashboards to use the new location names
