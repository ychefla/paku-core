#pragma once
/**
 * @file site_config.h
 * @brief Installation (site) identifier used as the MQTT topic root.
 *
 * All MQTT topics follow {site_id}/{system}/{device_id}/{topic_type}
 * (see paku-iot/docs/mqtt_schema.md). SITE_ID selects the first segment.
 *
 * Set it per PlatformIO environment with a build flag, e.g.
 *     -DSITE_ID=\"honkaranta\"
 * Build flags apply to both src/ and lib/, so every topic stays consistent.
 * Do NOT define it in secrets.h — libraries cannot see that file.
 *
 * Usage: string-literal concatenation, e.g. SITE_ID "/edge/" → "paku/edge/".
 */

#ifndef SITE_ID
#define SITE_ID "paku"
#endif
