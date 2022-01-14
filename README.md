# Light Stack MQTT Controller

ESP32 firmware meant to drive a simple industrial-style stack light with three lights, red, orange/amber, and green and expose those controls over MQTT.

## Patterns

The core concept for light control is through the use of a pattern, which is effectively a very simple timing-based program. The core of a pattern is one or more "frames". Each frame is just the on/off value to set each light to, and a delay time in milliseconds to hold that frame. Frames are processed in sequence, looping back to the first frame after the end time runs out. If an frame needs to run indefinitely, a value is passed to indicate that. Additionally, a pattern has a delay value and a preset name, to indicate how long to run the pattern until switching to the preset. Similar to frames, if a pattern needs to run indefinitely a value is passed to indicate that.

## Controls

All user control of the device is handled over MQTT. At first connection, the device announces that it is online by publishing to the topic `light-stack/<device-id>/identity` where the `<device-id>` is a unique value for that device. Currently the contents of the message don't matter, however the value "online" is currently passed. The controller will then subscribe to a series of topics in the format `<light-stack>/<all|device-id>/<control>`.

The controls are exposed in two ways: pattern definitions and pattern presets. A definition is sent as a JSON document that will be validated and assigned as the current pattern and run. A preset is a hardcoded, named pattern that ships with the firmware. The named presets can also be used as the next value for timed patterns.

The firmware, being designed for the ESP32 chipset, also exposes wifi hotspot setup and configuration for MQTT server, port and credentials.

## Pattern JSON

The JSON format for pattern definitions is as follows:

```json
{
  "frames": [
    { "red": true, "amber": false, "green": false, "delay": 200 },
    { "red": false, "amber": true, "green": false, "delay": 200 },
    { "red": false, "amber": false, "green": true, "delay": 200 }
  ],
  "delay": 1500,
  "next_preset": "off"
}
```

To specify a `delay` value is "indefinite", use the value `-1`. If a pattern is set as indefinite, the `next_preset` value is ignored and may be set to an empty string.

## License

This code is released under the MIT License.
