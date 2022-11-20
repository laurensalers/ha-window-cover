#ifndef _MQTTTYPES_h
#define _MQTTTYPES_h

#include <Arduino.h>

typedef struct
{
    String topic;
    String payload;
} MqttReceivedMessage;

#endif