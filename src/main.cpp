#include <Arduino.h>
#include <WiFiManager.h>
#include <AccelStepper.h>
#include <MQTT.h>
#include <ESP_EEPROM.h>

#include <Config.h>
#include <Utils.h>
#include <QueueHandler.h>
#include <MqttHelper.h>
#include <ObservableManager.h>
#include <ObservableValue.h>

// TODO
// * [ ] Implement TMC2209

// Topics
const char *topicDiscovery = "config";
const char *topicStepperPositionGet = "stepperPosition/get";
const char *topicStepperPositionMaxGet = "stepperPositionMax/get";
const char *topicStepperPositionMaxSet = "stepperPositionMax/set";
const char *topicPositionGet = "position/get";
const char *topicPositionSet = "position/set";
const char *topicPositionStateGet = "positionState/get";
const char *topicSystemStateGet = "systemState/get";
const char *topicSystemStateSet = "systemState/set";
const char *topicMoveSet = "move/set";

char deviceName[50];
char coverBaseTopic[100];

typedef enum
{
  UNKNOWN = 0,
  CALIBRATE = 1,
  READY = 2
} SystemState;

typedef enum
{
  COVER_STOPPED,
  COVER_OPENING,
  COVER_CLOSING,
} CoverState;

WiFiClient net;
WiFiManager wifiManager;
MQTTClient client(1000);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

QueueHandler<MqttReceivedMessage> mqttReceivedMessageQueue;

// Observables
ObservableValue<long> stepperPosition(-1);
ObservableValue<long> stepperPositionMax(0);
ObservableValue<byte> position(0); // In percent
ObservableValue systemState(SystemState::UNKNOWN);
ObservableValue<CoverState> positionState(CoverState::COVER_STOPPED);

ObservableValueBase *observables[] = {
    &positionState,
    &stepperPosition,
    &stepperPositionMax,
    &position,
    &systemState,
};

ObservableManagerClass observableManager(observables);

void mqttPublish(const char *fullTopic, char *value, bool retain = false)
{
  if (!client.connected())
  {
    return;
  }

#if DEBUG
  // Serial.printf("[%s] %s\n", fullTopic, value);
#endif
  client.publish(fullTopic, value, retain, 0);
}

void mqttPublish(const char *fullTopic, int value, bool retain = false)
{
  char val[10];
  sprintf(val, "%d", value);
  mqttPublish(fullTopic, val, retain);
}

void mqttPublish(const char *fullTopic, long value, bool retain = false)
{
  char val[10];
  sprintf(val, "%ld", value);
  mqttPublish(fullTopic, val, retain);
}

void mqttSubscribe(const char *fullTopic)
{
#if DEBUG
  Serial.printf("sub: [%s]\n", fullTopic);
#endif

  client.subscribe(fullTopic);
}

void mqttUnSubscribe(const char *fullTopic)
{
#if DEBUG
  Serial.printf("unsub: [%s]\n", fullTopic);
#endif

  client.unsubscribe(fullTopic);
}

void mqttPublishDeviceDiscovery()
{
  char payload[1000];
  char localIp[20];
  char mac[50];

  WiFi.localIP().toString().toCharArray(localIp, sizeof(localIp));
  WiFi.macAddress().toCharArray(mac, sizeof(mac));

  // Send cover discovery
  sprintf(payload,
          "{"
          "\"name\": \"Cover %s\","
          "\"unique_id\": \"%s\","
          "\"device\": {"
          "  \"name\": \"Cover %s\","
          "  \"connections\": [[\"mac\", \"%s\"]],"
          "  \"identifiers\": [\"%s\"],"
          "  \"configuration_url\": \"http://%s/\""
          "},"
          "\"availability\": {"
          "  \"topic\": \"%s%s\""
          "},"
          "\"command_topic\": \"%s%s\","
          "\"payload_open\": \"open\","
          "\"payload_close\": \"close\","
          "\"payload_stop\": \"stop\","
          "\"state_topic\": \"%s%s\","
          "\"position_topic\": \"%s%s\","
          "\"set_position_topic\": \"%s%s\","
          "\"qos\": 0,"
          "\"retain\": false,"
          "\"optimistic\": false,"
          "\"device_class\": \"shade\""
          "}",
          deviceName,
          deviceName,
          deviceName,
          mac,
          deviceName,
          localIp,
          Utils.getFullTopic(coverBaseTopic, ""), topicSystemStateGet,
          Utils.getFullTopic(coverBaseTopic, ""), topicMoveSet,
          Utils.getFullTopic(coverBaseTopic, ""), topicPositionStateGet,
          Utils.getFullTopic(coverBaseTopic, ""), topicPositionGet,
          Utils.getFullTopic(coverBaseTopic, ""), topicPositionSet);

  mqttPublish(Utils.getFullTopic(coverBaseTopic, "config"), payload);
}

bool connectMqtt()
{
  if (client.connected() || !WiFi.isConnected())
  {
    return false;
  }

  bool connected = client.connect(deviceName, "esp32", "cynu4c9r");

#if DEBUG
  Serial.printf("WiFi connected: %i, Mqtt connected: %i\n", WiFi.isConnected(), client.connected());
#endif

  if (connected)
  {
    mqttSubscribe(Utils.getFullTopic(coverBaseTopic, topicStepperPositionGet));
    mqttSubscribe(Utils.getFullTopic(coverBaseTopic, topicStepperPositionMaxSet));
    mqttSubscribe(Utils.getFullTopic(coverBaseTopic, topicPositionSet));
    mqttSubscribe(Utils.getFullTopic(coverBaseTopic, topicMoveSet));
    mqttSubscribe(Utils.getFullTopic(coverBaseTopic, topicSystemStateSet));

    mqttPublishDeviceDiscovery();
  }

  return connected;
}

void updateSystemState(SystemState requestedState)
{
#if DEBUG
  Serial.printf("Current state: %d, requested state: %d\n", systemState.value(), requestedState);
#endif

  // Move into calibration state
  if ((systemState.value() == SystemState::UNKNOWN || systemState.value() == SystemState::READY) && requestedState == SystemState::CALIBRATE)
  {
    systemState.setValue(requestedState);
    return;
  }

  // Move from calibration state to ready state
  // Move from unknown state when max position is set and
  if ((systemState.value() == SystemState::CALIBRATE || systemState.value() == SystemState::UNKNOWN) && stepperPosition.value() > -1 && stepperPositionMax.value() > 0 && requestedState == SystemState::READY)
  {
    systemState.setValue(requestedState);
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
  stepper.setCurrentPosition(0);
  stepper.disableOutputs();
#endif

  // TMC2209
  // TODO...
}

void updatePosition()
{
  if (systemState.value() == SystemState::UNKNOWN || stepperPositionMax.value() == 0 || stepperPosition.value() == 0)
  {
    position.setValue(0);
    return;
  }

  float pos = ((float)stepperPosition.value() / stepperPositionMax.value()) * 100;
  position.setValue(pos);
}

void handleSystemStateChange(SystemState state)
{
  switch (state)
  {
  case SystemState::CALIBRATE:
  case SystemState::READY:
    stepper.enableOutputs();
    mqttPublish(Utils.getFullTopic(coverBaseTopic, topicSystemStateGet), (char *)"online");
    break;

  default:
    stepper.disableOutputs();
    mqttPublish(Utils.getFullTopic(coverBaseTopic, topicSystemStateGet), (char *)"offline");
    break;
  }
}

void bindObservers()
{
  stepperPosition.addObserver([](long value) { //
    if (systemState.value() != SystemState::UNKNOWN || value > 0)
    {
      mqttPublish(Utils.getFullTopic(coverBaseTopic, topicStepperPositionGet), value, true);
    }

    if (systemState.value() == SystemState::CALIBRATE && value > -1)
    {
      stepperPositionMax.setValue(value);
    }

    updatePosition();
  });

  stepperPositionMax.addObserver([](long value) { //
    mqttPublish(Utils.getFullTopic(coverBaseTopic, topicStepperPositionMaxGet), value);
    updatePosition();
  });

  position.addObserver([](byte value) { //
    mqttPublish(Utils.getFullTopic(coverBaseTopic, topicPositionGet), value);
  });

  positionState.addObserver([](CoverState value) { //
    switch (value)
    {
    case CoverState::COVER_OPENING:
      mqttPublish(Utils.getFullTopic(coverBaseTopic, topicPositionStateGet), (char *)"opening");
      break;
    case CoverState::COVER_CLOSING:
      mqttPublish(Utils.getFullTopic(coverBaseTopic, topicPositionStateGet), (char *)"closing");
      break;
    case CoverState::COVER_STOPPED:
      mqttPublish(Utils.getFullTopic(coverBaseTopic, topicPositionStateGet), (char *)"stopped");
      break;
    }
  });

  systemState.addObserver(handleSystemStateChange);
}

void saveState()
{
  EEPROM.put(EEPROM_ADDRESS, stepperPositionMax.value());
  EEPROM.commit();
}

void handleMqttMessage(MqttReceivedMessage message)
{
#if DEBUG
  Serial.print("[");
  Serial.print(message.topic);
  Serial.print("] ");
  Serial.println(message.payload);
#endif

  if (systemState.value() == SystemState::UNKNOWN && stepperPosition.value() == -1 && message.topic.endsWith(topicStepperPositionGet) && !message.payload.isEmpty())
  {
    mqttUnSubscribe(Utils.getFullTopic(coverBaseTopic, topicStepperPositionGet));
    stepper.setCurrentPosition(message.payload.toInt());
    stepperPosition.setValue(stepper.currentPosition());
    updateSystemState(SystemState::READY);
    return;
  }

  if (message.topic.endsWith(topicSystemStateSet))
  {
    if (message.payload.equals("calibrate"))
    {
      mqttUnSubscribe(Utils.getFullTopic(coverBaseTopic, topicStepperPositionGet));
      updateSystemState(SystemState::CALIBRATE);
      return;
    }
    if (message.payload.equals("save"))
    {
      saveState();
      updateSystemState(SystemState::READY);
    }
    return;
  }

  if (message.topic.endsWith(topicStepperPositionMaxSet))
  {
    stepperPositionMax.setValue(message.payload.toInt());
    updateSystemState(SystemState::READY);
    return;
  }

  // Allow when ready or calibrating
  if (systemState.value() != SystemState::UNKNOWN && message.topic.endsWith(topicMoveSet))
  {
    if (message.payload.equals("close"))
    {
      stepper.moveTo(0);
      return;
    }

    if (message.payload.equals("open"))
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

    float stepperPosition = ((float)sanitizedTargetPosition / 100) * stepperPositionMax.value();

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
#if DEBUG
  Serial.begin(115200);
#endif

  Utils.setDeviceName(deviceName);
  sprintf(coverBaseTopic, "homeassistant/cover/%s", deviceName);

  // WiFi config
  WiFi.mode(WIFI_STA);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.autoConnect(deviceName, WM_ACCESSPOINT_PASSWORD);
  wifiManager.startWebPortal();

  // Bind mqtt to observables
  bindObservers();

  // MQTT Config
  mqttReceivedMessageQueue.setHandler(handleMqttMessage);
  client.begin("homeassistant.lan", 1883, net);
  client.onMessage(handleMqttMessageReceive);

  // Restore stored data
  long stepperMaxPosition = 0;
  EEPROM.begin(sizeof(long));

  if (EEPROM.percentUsed() >= 0)
  {
    EEPROM.get(EEPROM_ADDRESS, stepperMaxPosition);
    stepperPositionMax.setValue(stepperMaxPosition);
  }

  configureStepper();
}

void loop()
{
  bool connectionStateChanged = connectMqtt();
  observableManager.trigger(connectionStateChanged);

  // Other
  wifiManager.process();
  client.loop();
  mqttReceivedMessageQueue.deQueue();

  // Stepper
  if (stepperPosition.value() > -1)
  {
    stepper.run();

    if (stepper.isRunning())
    {
      bool opening = stepper.currentPosition() < stepper.targetPosition();
      positionState.setValue(opening ? CoverState::COVER_OPENING : CoverState::COVER_CLOSING);
    }
    else
    {
      positionState.setValue(CoverState::COVER_STOPPED);
    }

    stepperPosition.setValue(stepper.currentPosition());
  }
}