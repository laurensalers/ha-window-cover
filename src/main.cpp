#include <Arduino.h>
#include <WiFiManager.h>
#include <AccelStepper.h>
#include <MQTT.h>

#include <Config.h>
#include <MqttQueueHandler.h>
#include <Utils.h>
#include <ObservableValue.h>

// TODO
// * Store max value in eeprom
// * MQTT discover for HA

// Topics
const char *topicStepperPositionSavedGet = "stepperPositionSaved/get";
const char *topicStepperPositionGet = "stepperPosition/get";
const char *topicStepperPositionSet = "stepperPosition/set";
const char *topicStepperPositionMaxGet = "stepperPositionMax/get";
const char *topicStepperPositionMaxSet = "stepperPositionMax/set";
const char *topicPositionGet = "position/get";
const char *topicPositionSet = "position/set";
const char *topicSystemStateGet = "systemstate/get";
const char *topicSystemStateSet = "systemstate/set";
const char *topicMove = "move/set";

char deviceName[50];

typedef enum
{
  UNKNOWN = 0,
  CALIBRATE = 1,
  READY = 2
} SystemState;

WiFiClient net;
WiFiManager wifiManager;
MQTTClient client;
MqttQueueHandler mqttQueueHandler(&client, deviceName);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Observables
ObservableValue<long> stepperPosition(0);
ObservableValue<long> stepperPositionMax(0);
ObservableValue<byte> position(0); // In percent
ObservableValue systemState(SystemState::UNKNOWN);

bool connectMqtt()
{
  if (!WiFi.isConnected() || client.connected())
  {
    return false;
  }

  return client.connect(deviceName, "esp32", "cynu4c9r");
}

void updateSystemState(SystemState requestedState)
{
  Serial.printf("Current state: %d, requested state: %d\n", systemState.value(), requestedState);

  // Move into calibration state
  if ((systemState.value() == SystemState::UNKNOWN || systemState.value() == SystemState::READY) && requestedState == SystemState::CALIBRATE)
  {
    systemState.setValue(requestedState);
    return;
  }

  // Move from calibration state to ready state
  if (systemState.value() == SystemState::CALIBRATE && stepperPositionMax.value() > 0 && requestedState == SystemState::READY)
  {
    systemState.setValue(requestedState);
    return;
  }

  Serial.println("Systemstate not changed");
}

void handleMqttMessage(String &topic, String &payload)
{
  Serial.printf("[%s]: %s\n", topic, payload);

  if (systemState.value() == SystemState::UNKNOWN && topic.endsWith(topicStepperPositionSavedGet) && !payload.isEmpty())
  {
    stepperPosition.setValue(payload.toInt());
    stepper.setCurrentPosition(stepperPosition.value());
    updateSystemState(SystemState::READY);
    return;
  }

  if (topic.endsWith(topicSystemStateSet))
  {
    if (payload.equals("calibrate"))
    {
      updateSystemState(SystemState::CALIBRATE);
      return;
    }
    return;
  }

  if (topic.endsWith(topicStepperPositionMaxSet))
  {
    // Save current position as max position
    if (payload == "save")
    {
      stepperPositionMax.setValue(stepperPosition.value());
    }
    else
    {
      stepperPositionMax.setValue(payload.toInt());
    }

    updateSystemState(SystemState::READY);
    return;
  }

  // Allow when ready or calibrating
  if (systemState.value() != SystemState::UNKNOWN && topic.endsWith(topicMove))
  {
    if (payload.equals("down"))
    {
      stepper.moveTo(0);
      return;
    }

    if (payload.equals("up"))
    {
      long santizedValue = systemState.value() == SystemState::CALIBRATE
                               ? LONG_MAX
                               : stepperPositionMax.value();

      stepper.moveTo(santizedValue);
      return;
    }

    stepper.stop();
    return;
  }

  // Only allow ready state handlers below this
  if (systemState.value() != SystemState::READY)
  {
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
// A4988
#ifdef STEPPER_A4988
  stepper.setEnablePin(ENABLE_PIN);
  stepper.setAcceleration(STEPPER_ACCELERATION);
  stepper.setMaxSpeed(STEPPER_MAXSPEED);
  stepper.disableOutputs();
#endif

  // TMC2209
  // TODO...
}

void updatePosition()
{
  if (stepperPositionMax.value() == 0)
  {
    position.setValue(0);
    return;
  }

  float pos = (stepperPositionMax.value() / stepperPosition.value()) * 100;
  position.setValue(pos);
}

void handleSystemStateChange(SystemState state)
{
  if (state == SystemState::READY || state == SystemState::CALIBRATE)
  {
    stepper.enableOutputs();
  }
  else
  {
    stepper.disableOutputs();
  }

  mqttQueueHandler.queueMessage(topicSystemStateGet, state);
}

void bindObservers()
{
  stepperPosition.addObserver([](long value)
                              { mqttQueueHandler.queueMessage(topicStepperPositionGet, value); 
                                mqttQueueHandler.queueMessage(topicStepperPositionSavedGet, value, true);
                                updatePosition(); });

  stepperPositionMax.addObserver([](long value)
                                 { mqttQueueHandler.queueMessage(topicStepperPositionMaxGet, value); });

  position.addObserver([](byte value)
                       { mqttQueueHandler.queueMessage(topicPositionGet, value); });

  systemState.addObserver(handleSystemStateChange);
}

void mqttSubscribe(const char *topic)
{
  client.subscribe(Utils.getFullTopic(deviceName, topic));
}

void setup()
{
  Serial.begin(115200);

  Utils.setDeviceName(deviceName, "curtain");

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

  mqttSubscribe(topicStepperPositionSet);
  mqttSubscribe(topicStepperPositionMaxSet);
  mqttSubscribe(topicPositionSet);
  mqttSubscribe(topicMove);
  mqttSubscribe(topicSystemStateSet);

  configureStepper();
}

void triggerObservers()
{
  stepperPosition.trigger();
  stepperPositionMax.trigger();
  position.trigger();
  systemState.trigger();
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