#include <Arduino.h>
#include <WiFiManager.h>
#include <MQTT.h>

#include <Config.h>
#include <MqttQueueHandler.h>
#include <Utils.h>
#include <ObservableValue.h>

// Topics
const char *topicStepperPositionGet = "stepperPosition/get";
const char *topicStepperPositionMaxGet = "stepperPositionMax/get";

char deviceName[50];

WiFiClient net;
WiFiManager wifiManager;
MQTTClient client;
MqttQueueHandler mqttQueueHandler(&client, deviceName);

// Observables
ObservableValue stepperPosition(0);
ObservableValue stepperPositionMax(0);

bool connectMqtt()
{
  if (!WiFi.isConnected() || client.connected())
  {
    return false;
  }

  return client.connect(deviceName, "esp32", "cynu4c9r");
}

void setup()
{
  Utils.setDeviceName(deviceName);

  // WiFi config
  WiFi.mode(WIFI_STA);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.autoConnect(deviceName, WM_ACCESSPOINT_PASSWORD);
  wifiManager.startWebPortal();

  // Bind mqtt to observables
  stepperPosition.addObserver([](int value)
                              { mqttQueueHandler.queueMessage(topicStepperPositionGet, value); });
  stepperPositionMax.addObserver([](int value)
                                 { mqttQueueHandler.queueMessage(topicStepperPositionMaxGet, value); });

  // MQTT Config
  client.begin("homeassistant.lan", 1883, net);
}

void triggerObservers()
{
  stepperPosition.trigger();
  stepperPositionMax.trigger();
}

void loop()
{
  bool connectionStateChanged = connectMqtt();
  if (connectionStateChanged)
  {
    triggerObservers();
  }

  wifiManager.process();
  client.loop();
  mqttQueueHandler.loop();
}