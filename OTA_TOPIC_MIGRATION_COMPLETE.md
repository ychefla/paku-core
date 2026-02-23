# OTA Topic Migration - COMPLETE

## Summary

All OTA MQTT topics have been successfully migrated from `paku/devices` to `paku/edge` structure.

## Final Topic Structure

### Command Topic (Subscribe)
- **Topic:** `paku/edge/{device_id}/cmd/ota`
- **Payload:** `{"url": "...", "checksum": "...", "version": "..."}`
- **Used by:** GitHub Actions workflow, ota_update.sh script

### Response Topics (Publish)
- **Status:** `paku/edge/{device_id}/ota/status` - Acknowledgment when OTA command received
- **Progress:** `paku/edge/{device_id}/ota/progress` - Download/install progress updates
- **Result:** `paku/edge/{device_id}/ota/result` - Final success/failure result

## Firmware Version History

| Version | Changes | Status |
|---------|---------|--------|
| 1.3.0 | Legacy `paku/devices` topics only | Deprecated |
| 1.4.0 | Dual topic support (both old and new) | Superseded |
| 1.4.1 | New `paku/edge` command topic only | Superseded |
| **1.4.2** | **All response topics migrated to `paku/edge`** | **Current** |

## Files Modified

### Firmware (paku-core)
- ✅ `paku_core/src/main.cpp`
  - Line 31: Version → 1.4.2
  - Line 2564: Result topic → `paku/edge/{id}/ota/result`
  - Line 3042: Status topic → `paku/edge/{id}/ota/status`
  - Line 3068: Progress topic → `paku/edge/{id}/ota/progress`

- ✅ `paku_core/ota_update.sh`
  - Command topic → `paku/edge/{id}/cmd/ota`
  - Monitoring topics → `paku/edge/{id}/ota/*`

### Documentation
- ✅ `OTA_TOPIC_MIGRATION.md` - Updated with Phase 4 completion
- ⚠️ `docs/edge/ota-integration.md` - Needs update (still shows old topics)
- ⚠️ `docs/edge/ota-testing-checklist.md` - Needs update (examples use old topics)
- ⚠️ `docs/edge/ota-implementation-summary.md` - Needs update
- ⚠️ `paku_core/OTA_QUICK_GUIDE.md` - Needs update
- ⚠️ `paku_core/PERFORM_OTA_NOW.md` - Needs update

## Version Information Visibility

The firmware version is **already visible** in MQTT messages:

### Device Status (paku/edge/{id}/status)
```json
{
  "device_id": "ESP8266-9608CF16",
  "firmware_version": "1.4.2",
  "device_model": "esp8266-wired-sensors",
  "online": true,
  "last_seen": "2025-12-26T...",
  ...
}
```

### OTA Status Response (paku/edge/{id}/ota/status)
```json
{
  "timestamp": "2025-12-26T...",
  "status": "accepted",
  "current_version": "1.4.2",
  "target_version": "1.5.0"
}
```

### OTA Result Response (paku/edge/{id}/ota/result)
```json
{
  "timestamp": "2025-12-26T...",
  "current_version": "1.4.2",
  "target_version": "1.5.0",
  "success": true,
  "result_code": 0,
  "message": "Success"
}
```

## Testing

### Monitor All OTA Topics
```bash
# All response topics
mosquitto_sub -h 37.27.192.107 -p 1883 -v -t 'paku/edge/+/ota/#'

# Just command topic
mosquitto_sub -h 37.27.192.107 -p 1883 -v -t 'paku/edge/+/cmd/ota'

# Specific device
mosquitto_sub -h 37.27.192.107 -p 1883 -v -t 'paku/edge/ESP8266-9608CF16/ota/#'
```

### Send Test Command
```bash
DEVICE_ID="ESP8266-9608CF16"
mosquitto_pub -h 37.27.192.107 -p 1883 \
  -t "paku/edge/${DEVICE_ID}/cmd/ota" \
  -m '{"url":"http://example.com/test.bin","version":"test","checksum":"abc123"}'
```

## Deployment

### Build and Flash v1.4.2
```bash
cd paku_core
pio run -e esp8266-wired-sensors  # or lilygo-t-display-s3, esp32-ch340c-30pin
pio run -e esp8266-wired-sensors -t upload
```

### Deploy via GitHub Actions
Use the OTA workflow in paku-iot repository:
- Target version: v1.4.2
- Rollout strategy: Full (all devices)
- Devices will automatically update to new firmware

## Next Steps

1. **Deploy v1.4.2** to all devices via OTA workflow
2. **Update documentation** - Replace all references to old topics
3. **Update backend services** - Ensure collector subscribes to new response topics
4. **Monitor migration** - Check device status for version updates

## Rollback

If issues arise:
- Previous firmware (v1.4.1) can still receive OTA commands on `paku/edge/{id}/cmd/ota`
- Response topics will be on old structure but workflow can be adjusted
- No database changes needed - migration is non-breaking

---

**Migration Completed:** 2025-12-26  
**Firmware Version:** 1.4.2  
**Status:** ✅ Ready for deployment
