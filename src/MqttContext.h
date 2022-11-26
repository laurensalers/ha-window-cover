#ifndef _MQTTCONTEXT_h
#define _MQTTCONTEXT_h

#include <Arduino.h>
#include <MQTT.h>

class MqttContext
{
public:
    MqttContext(
        MQTTClient *client,
        const char *entityType,
        char *deviceName,
        const char *subDeviceName = "")
    {
        _client = client;
        _deviceName = deviceName;
        _subDeviceName = subDeviceName;
        _entityType = entityType;
    }

    char *getBaseTopic()
    {
        sprintf(_baseTopic,
                "homeassistant/%s/%s%s",
                _entityType,
                _deviceName,
                _subDeviceName);

        return _baseTopic;
    }

    void publish(const char *topic, char *value, bool retain = false)
    {
        if (!_client->connected())
        {
            return;
        }

        char *fullTopic = getFullTopic(topic);

#if DEBUG
        // debugSerial.printf("[%s] %s\n", fullTopic, value);
#endif
        _client->publish(fullTopic, value, retain, 0);
    }

    void publish(const char *topic, int value, bool retain = false)
    {
        char val[10];
        sprintf(val, "%d", value);
        publish(topic, val, retain);
    }

    void publish(const char *topic, long value, bool retain = false)
    {
        char val[10];
        sprintf(val, "%ld", value);
        publish(topic, val, retain);
    }

    void subscribe(const char *topic)
    {
        char *fullTopic = getFullTopic(topic);

#if DEBUG
        debugSerial.printf("sub: [%s]\n", fullTopic);
#endif

        _client->subscribe(fullTopic);
    }

    void unsubscribe(const char *topic)
    {
        char *fullTopic = getFullTopic(topic);

#if DEBUG
        debugSerial.printf("unsub: [%s]\n", fullTopic);
#endif

        _client->unsubscribe(fullTopic);
    }

private:
    char _baseTopic[100];
    char _fullTopic[100];
    char *_deviceName;
    const char *_subDeviceName;
    const char *_entityType;
    MQTTClient *_client;

    char *getFullTopic(const char *topic)
    {
        sprintf(_fullTopic,
                "%s/%s",
                getBaseTopic(),
                topic);

        return _fullTopic;
    }
};

#endif