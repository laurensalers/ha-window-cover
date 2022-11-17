#include <Arduino.h>
#include <WiFiManager.h>
#include <AccelStepper.h>
#include <MQTT.h>

#include <Config.h>
#include <MqttQueueHandler.h>
#include <Utils.h>
#include <ObservableValue.h>

// Topics
const char *topicStepperPositionSavedGet = "stepperPositionSaved/get";
const char *topicStepperPositionGet = "stepperPosition/get";
const char *topicStepperPositionSet = "stepperPosition/set";
const char *topicStepperPositionMaxGet = "stepperPositionMax/get";
const char *topicStepperPositionMaxSet = "stepperPositionMax/set";
const char *topicPositionGet = "position/get";
const char *topicPositionSet = "position/set";

char deviceName[50];
bool restoreSavedStepperPosition = false;

WiFiClient net;
WiFiManager wifiManager;
MQTTClient client;
MqttQueueHandler mqttQueueHandler(&client, deviceName);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Observables
ObservableValue<long> stepperPosition(0);
ObservableValue<long> stepperPositionMax(0);
ObservableValue<byte> position(0); // In percents

bool connectMqtt()
{
  if (!WiFi.isConnected() || client.connected())
  {
    return false;
  }

  return client.connect(deviceName, "esp32", "cynu4c9r");
}

void handleMqttMessage(String &topic, String &payload)
{
  if (restoreSavedStepperPosition && topic.endsWith(topicStepperPositionSavedGet))
  {
    stepperPosition.setValue(payload.toInt());
    stepper.setCurrentPosition(stepperPosition.value());
    restoreSavedStepperPosition = false;
    return;
  }

  if (topic.endsWith(topicStepperPositionMaxSet))
  {
    stepperPositionMax.setValue(payload.toInt());
    return;
  }

  if (topic.endsWith(topicPositionSet))
  {
    int targetPosition = payload.toInt();
    int sanitizedTargetPosition = max(min(targetPosition, 100), 0);

    float stepperPosition = (sanitizedTargetPosition / 100) * stepperPositionMax.value();

    stepper.moveTo(stepperPosition);
    return;
  }
}

void configureStepper()
{
  // TODO:
  // stepper.setEnablePin(ENABLE_PIN);
  // stepper.setAcceleration();
  // stepper.setMaxSpeed();

  stepper.disableOutputs();
}

void calculatePosition()
{
  if (stepperPositionMax.value() == 0)
  {
    position.setValue(0);
    return;
  }

  float pos = (stepperPositionMax.value() / stepperPosition.value()) * 100;
  position.setValue(pos);
}

void bindObservers()
{
  stepperPosition.addObserver([](long value)
                              { mqttQueueHandler.queueMessage(topicStepperPositionGet, value); 
                                mqttQueueHandler.queueMessage(topicStepperPositionSavedGet, value, true); });

  stepperPositionMax.addObserver([](long value)
                                 { mqttQueueHandler.queueMessage(topicStepperPositionMaxGet, value); });

  position.addObserver([](byte value)
                       { mqttQueueHandler.queueMessage(topicPositionGet, value); });
}

void setup()
{
  Utils.setDeviceName(deviceName);

  // WiFi config
  WiFi.mode(WIFI_STA);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.autoConnect(deviceName, WM_ACCESSPOINT_PASSWORD);
  wifiManager.startWebPortal();

  // Bind mqtt to observabless
  bindObservers();

  // MQTT Config
  client.begin("homeassistant.lan", 1883, net);
  client.onMessage(handleMqttMessage);

  client.subscribe(topicStepperPositionSet);
  client.subscribe(topicStepperPositionMaxSet);

  configureStepper();
}

void triggerObservers()
{
  stepperPosition.trigger();
  stepperPositionMax.trigger();
  position.trigger();
}

void loop()
{
  bool connectionStateChanged = connectMqtt();
  if (connectionStateChanged)
  {
    triggerObservers();
  }

  // Other
  wifiManager.process();
  client.loop();
  mqttQueueHandler.loop();

  // Stepper
  stepper.run();
  stepperPosition.setValue(stepper.currentPosition());
}