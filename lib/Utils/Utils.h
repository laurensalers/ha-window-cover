#ifndef _UTILS_h
#define _UTILS_h

#include <Arduino.h>

class UtilsClass
{

public:
    void setDeviceName(char *deviceName, const char *name)
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
        sprintf(deviceName, "%s-%02x", name, ESP.getChipId());
#endif
    }

    void setFullTopic(char *fullTopic, const char *baseTopic, const char *topic)
    {
        sprintf(fullTopic, "%s/%s", baseTopic, topic);
    }
};

UtilsClass Utils;
extern UtilsClass Utils;

#endif