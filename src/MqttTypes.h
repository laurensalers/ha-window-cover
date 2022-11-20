#ifndef _MQTTTYPES_h
#define _MQTTTYPES_h

#include <Arduino.h>

#define MAX_MESSAGE_LENGTH 20

typedef struct
{
    String topic;
    String payload;
} MqttReceivedMessage;

#endif