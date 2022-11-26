#ifndef _TYPES_h
#define _TYPES_h

#include <Arduino.h>

typedef enum
{
    UNKNOWN = 0,
    CALIBRATE = 1,
    READY = 2,
    ERROR = -1
} SystemState;

typedef enum
{
    COVER_STOPPED,
    COVER_OPENING,
    COVER_CLOSING,
} CoverState;

typedef struct
{
    String topic;
    String payload;
} MqttReceivedMessage;

#endif