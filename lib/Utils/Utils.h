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

        sprintf(deviceName, "cover-%d", chipId);
#endif

#ifdef ESP8266
        sprintf(deviceName, "cover-%02x", ESP.getChipId());
#endif
    }
};

UtilsClass Utils;
extern UtilsClass Utils;

#endif