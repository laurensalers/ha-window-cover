#ifndef _UTILS_h
#define _UTILS_h

#include <Arduino.h>
#include <SoftwareSerial.h>

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

#if DEBUG
#ifdef STEPPER_TMC2209
SoftwareSerial debugSerial(SOFTWARE_SERIAL_RX, SOFTWARE_SERIAL_TX);
#endif
#ifdef STEPPER_A4988
HardwareSerial debugSerial = Serial;
#endif
#endif

#ifdef STEPPER_TMC2209
HardwareSerial &stepperSerial = Serial;
#endif

#endif