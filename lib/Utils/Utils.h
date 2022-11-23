#ifndef _UTILS_h
#define _UTILS_h

#include <Arduino.h>

class UtilsClass
{

public:
    void setDeviceName(char *deviceName)
    {
#ifdef ESP32
        uint32_t chipId = 0;
        for (int i = 0; i < 17; i = i + 8)
        {
            chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
        }

        sprintf(deviceName, "%s-%d", MQTT_DEVICE_NAME, chipId);
#endif

#ifdef ESP8266
        sprintf(deviceName, "%02x", ESP.getChipId());
#endif
    }

public:
    char *getFullTopic(char *baseTopic, const char *topic)
    {
        snprintf(_fullTopic, sizeof(_fullTopic), "%s/%s", baseTopic, topic);
        return _fullTopic;
    }

private:
    char _fullTopic[100];

    // void setFullTopic(char *fullTopic, const char *entityType, const char *entityId, const char *topic)
    // {
    //     sprintf(fullTopic, "homeassistant/%s/%s/%s", entityType, entityId, topic);
    // }
};

UtilsClass Utils;
extern UtilsClass Utils;

#endif