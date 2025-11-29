/**
 * @file payload.h
 * @brief Payload handling for MQTT messages
 * 
 * This module provides functions for creating and managing payloads
 * for MQTT messages in the Paku IoT system.
 */
#pragma once

#include <Arduino.h>

/**
 * @brief Maximum number of payloads that can be queued
 */
#define MAX_PAYLOADS 30

/**
 * @brief Structure to hold MQTT payload data
 */
struct Payload {
    String topic;
    String data;
};

/**
 * @brief Creates a JSON payload string from value and timestamp
 * 
 * @param value The numeric value to include in the payload
 * @param timestamp The timestamp string
 * @return String The JSON formatted payload string
 */
String createPayloadJson(float value, const String& timestamp);

/**
 * @brief Stores a payload in the payload buffer
 * 
 * @param payloads Array of Payload structures
 * @param payloadIndex Current index in the payload array (will be incremented)
 * @param maxPayloads Maximum number of payloads allowed
 * @param topic The MQTT topic
 * @param value The numeric value
 * @param timestamp The timestamp string
 * @return true if payload was stored successfully
 * @return false if payload buffer is full
 */
bool storePayload(Payload* payloads, int& payloadIndex, int maxPayloads,
                  const String& topic, float value, const String& timestamp);
