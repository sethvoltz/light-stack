# Light Stack MQTT Controller

ESP32 firmware meant to drive a simple industrial-style stack light with three lights, red, orange/amber, and green and expose those controls over MQTT.

## Patterns

The core concept for light control is through the use of a pattern, which is effectively a very simple timing-based program. The core of a pattern is one or more "frames". Each frame is just the on/off value to set each light to, and a delay time in milliseconds to hold that frame. Frames are processed in sequence, looping back to the first frame after the end time runs out. If an frame needs to run indefinitely, a value is passed to indicate that. Additionally, a pattern has a delay value and a preset name, to indicate how long to run the pattern until switching to the preset. Similar to frames, if a pattern needs to run indefinitely a value is passed to indicate that.

## Controls

All user control of the device is handled over MQTT. At first connection, the device announces that it is online by publishing to the topic `light-stack/<device-id>/identity` where the `<device-id>` is a unique value for that device and a value of "online". The device also registers an MQTT Last Will and Testament to publish the value "offline" when it disconnects. It also subscribes to the control topic `identify` which will trigger publishing to its identity topic. The controller will then subscribe to a series of topics in the format `<light-stack>/<all|device-id>/<control>`.

Two controls are exposed: pattern definitions as `definition` and pattern presets as `preset`. A definition is sent as a JSON document that will be validated and assigned as the current pattern and run. A preset is a hardcoded, named pattern that ships with the firmware. The named presets can also be used as the next value for timed patterns.

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

## Firmware

The light-stack support over the air firmware updates. Navigate to `http://<device-up>/update` to get to the page. It will prompt you for a username and password. The username is `update`, unless changed in code, and the password is configured when performing the initial wifi setup.

New firmware is located at `<project-root>/.pio/build/esp32/firmware.bin` and can be selected from the filesystem or dropped into the browser update page. A progress bar will show the upload and store to flash. After 100% the new firmware will be selected and the device will reboot, at which point you should see the initial wifi and MQTT connectivity status lights before it returns to the default user pattern.

_**Note:** This project uses a custom partition table to expand the available program storage and reduce the SPIFFS partition. This also means it assumes a 4MB cache ESP32 and other flash sizes will likely fail._

## Hardware

Below are the key pieces of hardware required to build this, as well as product links to the specific ones I used:

- 12v red-amber-green stack light [Amazon](https://www.amazon.com/gp/product/B086WSHZKV/)
- ESP32 dev board [Amazon](https://www.amazon.com/gp/product/B08PNWB81Z/)
- N-Channel MOSFET transistor (I used the IRF640)
- Buck Converter (to drop 12v in to 5v for the ESP)
- Wire sockets (TBD)
- Protoboard (TBD)
- 2x10 .1" female headers
- 2x10 .1" male headers
- Power barrel jack
- Wire
- Enclosure (3D Model TBD)

## License

This code is released under the MIT License.
