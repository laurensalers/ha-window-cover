#include <Arduino.h>
#include <WiFiManager.h>
#include <AccelStepper.h>
#include <MQTT.h>

#include <Config.h>
#include <Utils.h>
#include <QueueHandler.h>
#include <MqttTypes.h>
#include <ObservableValue.h>

// TODO
// * Store max value in eeprom
// * MQTT discover for HA
// * Implement message queue, we may not sub/pub from the handle message cb
// * Change system state based on observers not in the mqtt handler

// Topics
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
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

QueueHandler<MqttReceivedMessage> mqttReceivedMessageQueue;

// Observables
ObservableValue<long> stepperPosition(0);
ObservableValue<long> stepperPositionMax(0);
ObservableValue<byte> position(0); // In percent
ObservableValue systemState(SystemState::UNKNOWN);

void mqttSubscribe(const char *topic)
{
  char fullTopic[50];

  Utils.setFullTopic(fullTopic, deviceName, topic);
  Serial.printf("sub: [%s]\n", fullTopic);

  client.subscribe(fullTopic);
}

void mqttUnSubscribe(const char *topic)
{
  char fullTopic[50];

  Utils.setFullTopic(fullTopic, deviceName, topic);
  Serial.printf("unsub: [%s]\n", fullTopic);

  client.unsubscribe(fullTopic);
}

bool connectMqtt()
{
  if (client.connected() || !WiFi.isConnected())
  {
    return false;
  }

  bool connected = client.connect(deviceName, "esp32", "cynu4c9r");

  if (connected)
  {
    mqttSubscribe(topicStepperPositionSet);
    mqttSubscribe(topicStepperPositionMaxSet);
    mqttSubscribe(topicPositionSet);
    mqttSubscribe(topicMove);
    mqttSubscribe(topicSystemStateSet);
    mqttSubscribe(topicStepperPositionGet);
  }

  Serial.printf("WiFi connected: %i, Mqtt connected: %i\n", WiFi.isConnected(), client.connected());

  return connected;
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

  // mqttQueueHandler.sendMessage(topicSystemStateGet, state);
}

void bindObservers()
{
  // stepperPosition.addObserver([](long value)
  //                             { mqttQueueHandler.sendMessage(topicStepperPositionGet, value, true);
  //                               updatePosition(); });

  // stepperPositionMax.addObserver([](long value)
  //                                { mqttQueueHandler.sendMessage(topicStepperPositionMaxGet, value); });

  // position.addObserver([](byte value)
  //                      { mqttQueueHandler.sendMessage(topicPositionGet, value); });

  // systemState.addObserver(handleSystemStateChange);
}

void handleMqttMessage(MqttReceivedMessage message)
{
  Serial.print("[");
  Serial.print(message.topic);
  Serial.print("] ");
  Serial.println(message.payload);

  if (systemState.value() == SystemState::UNKNOWN && message.topic.endsWith(topicStepperPositionGet) && !message.payload.isEmpty())
  {
    mqttUnSubscribe(topicStepperPositionGet);
    stepperPosition.setValue(message.payload.toInt());
    stepper.setCurrentPosition(stepperPosition.value());
    return;
  }

  if (message.topic.endsWith(topicSystemStateSet))
  {
    if (message.payload.equals("calibrate"))
    {
      updateSystemState(SystemState::CALIBRATE);
      return;
    }
    return;
  }

  if (message.topic.endsWith(topicStepperPositionMaxSet))
  {
    // Save current position as max position
    if (message.payload == "save")
    {
      stepperPositionMax.setValue(stepperPosition.value());
    }
    else
    {
      stepperPositionMax.setValue(message.payload.toInt());
    }
    return;
  }

  // Allow when ready or calibrating
  if (systemState.value() != SystemState::UNKNOWN && message.topic.endsWith(topicMove))
  {
    if (message.payload.equals("down"))
    {
      stepper.moveTo(0);
      return;
    }

    if (message.payload.equals("up"))
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

  if (message.topic.endsWith(topicPositionSet))
  {
    int targetPosition = message.payload.toInt();
    int sanitizedTargetPosition = max(min(targetPosition, 100), 0);

    float stepperPosition = (sanitizedTargetPosition / 100) * stepperPositionMax.value();

    stepper.moveTo(stepperPosition);
    return;
  }
}

void handleMqttMessageReceive(String &topic, String &payload)
{
  MqttReceivedMessage *message = mqttReceivedMessageQueue.getQueueEntry();

  message->topic = topic;
  message->payload = payload;
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
  mqttReceivedMessageQueue.setHandler(handleMqttMessage);
  client.begin("homeassistant.lan", 1883, net);
  client.onMessage(handleMqttMessageReceive);

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
  mqttReceivedMessageQueue.deQueue();

  // Stepper
  stepper.run();
  stepperPosition.setValue(stepper.currentPosition());
}