/**
 * @file payload.cpp
 * @brief Payload handling implementation for MQTT messages
 */
#include "payload.h"

String createPayloadJson(float value, const String& timestamp) {
    return "{\"value\": " + String(value) + ", \"timestamp\": \"" + timestamp + "\"}";
}

bool storePayload(Payload* payloads, int& payloadIndex, int maxPayloads,
                  const String& topic, float value, const String& timestamp) {
    if (payloadIndex >= maxPayloads) {
        return false;
    }
    
    payloads[payloadIndex].topic = topic;
    payloads[payloadIndex].data = createPayloadJson(value, timestamp);
    payloadIndex++;
    return true;
}
