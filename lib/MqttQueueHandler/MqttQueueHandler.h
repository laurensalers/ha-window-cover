#ifndef _MQTTWRAPPER_h
#define _MQTTWRAPPER_h

#include <MQTTClient.h>
#include <Utils.h>

#define MAX_MESSAGE_LENGTH 20

typedef struct
{
    char message[MAX_MESSAGE_LENGTH];
    const char *topic;
    bool retain;
} MqttMessage;

class MqttQueueHandler
{
public:
    MqttQueueHandler(MQTTClient *client, const char *baseTopic)
    {
        _client = client;
        _baseTopic = baseTopic;
    }

    void loop()
    {
        if (_mqttMessageCount == 0 || !_client->connected())
        {
            return;
        }

        for (byte i = 0; i < 2 && _mqttMessageCount > 0; i++)
        {
            MqttMessage current = _mqttMessageQueue[_mqttCurrentMessage];

            char fullTopic[50];
            Utils.setFullTopic(fullTopic, _baseTopic, current.topic);
#if DEBUG
            Serial.println(fullTopic);
            Serial.println(current.message);
#endif
            _client->publish(fullTopic, current.message, current.retain, 0);

            _mqttCurrentMessage++;
            _mqttMessageCount--;

            if (_mqttCurrentMessage >= 10)
            {
                _mqttCurrentMessage = 0;
            }
        }
    }

    void queueMessage(const char *topic, const char *value, bool retain = false)
    {
        queueMessage(topic, (char *)value, retain);
    }

    void queueMessage(const char *topic, int value, bool retain = false)
    {
        char intValue[5];
        sprintf(intValue, "%d", value);
        queueMessage(topic, intValue, retain);
    }

private:
    const char *_baseTopic;

    MQTTClient *_client;
    MqttMessage _mqttMessageQueue[MAX_MESSAGE_LENGTH];
    byte _mqttMessageCount = 0;
    byte _mqttMessageQueuePosition = 0;
    byte _mqttCurrentMessage = 0;

    void queueMessage(const char *topic, char *message, bool retain)
    {
        MqttMessage mqttMessage = _mqttMessageQueue[_mqttMessageQueuePosition];

        strcpy(mqttMessage.message, message);
        mqttMessage.topic = topic;
        mqttMessage.retain = retain;

        _mqttMessageQueue[_mqttMessageQueuePosition] = mqttMessage;

        _mqttMessageQueuePosition++;
        _mqttMessageCount++;

        if (_mqttMessageQueuePosition >= 10)
        {
            _mqttMessageQueuePosition = 0;
        }
    }
};

#endif