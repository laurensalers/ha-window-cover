## Discovery

```js
((deviceId, friendlyName, deviceUrl) => {
  const baseTopic = `homeassistant/cover/${deviceId}`;
  const message = {
    name: friendlyName,
    unique_id: deviceId,
    availability: {
      topic: `${baseTopic}/systemstate/get`,
    },
    command_topic: `${baseTopic}/cover/move/set`,
    payload_open: "up",
    payload_close: "down",
    payload_stop: "stop",
    state_topic: `${baseTopic}/positionstate/get`,
    position_topic: `${baseTopic}/position/get`,
    set_position_topic: `${baseTopic}/position/set`,
    qos: 0,
    retain: false,
    optimistic: false,
    device_class: "shade",
  };

  return JSON.stringify(message);
})("104dca", "Vouwgordijn rechts", "192.168.1.174");
```
