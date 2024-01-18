


# Thunderboard BTHome V2
Thunderboard Sense 2 sensors and buttons implementation with BTHome V2 protocol, compatible with Home Assistant.

This will enable advertising of BTHome V2 temperature, humidity, and battery for 0.5 seconds every 60 seconds, while maintain EM2 deep sleep state when there's no advertising, so optimized for long battery life. 

If you want a version that would advertise all the sensors on the board, at the cost of a bit less battery life, then check out [this branch: all_sensors](https://github.com/cheadrian/thunderboard_bthome_v2/tree/all_sensors)

## Importing
If you have a Thunderboard Sense 2 version BRD4166A you can [File] -> [Import] `thunderboard_sense2_bthome_v2.sls` from `SimplicityStudio` directory.

Make sure you have the latest firmware version installed on the board.

Otherwise, you should manually configure the Thunderboard Sense 2 Software Components in the Simplicity Studio by:

- Add the [Third-Party Hardware Drivers](https://docs.silabs.com/application-examples/1.3.0/ae-getting-started/how-do-you-use-it#adding-sdk-extensions-for-hardware-drivers) into the SDK
- Create a new project from example `Third Party Hardware Drivers - BT Home v2`
- Install `Relative Humidity and Temperature sensor` 
- Install `Power supply measurement`
- Configure `vcom` by disabling `Restrict the energy mode to allow the reception.`
- Add a new `Simple Button` instance named `btn1`

You can read more and see step-by-step pictures [in this article](https://medium.com/@che-adrian/thunderboard-sense-2-silicon-labs-to-home-assistant-with-bthome-074975f9243b).

Then you can modify your `app.c` in accordance with this git `src`.

## Usage
After you flash the software, you can simply enter into the Home Assistant and add the BTHome V2 instance in the devices.

Encryption is disabled in this example. You can enable it by editing `app.c`.

## Custom HA integration
BTHome V2 is supported natively in Home Assistant and doesn't require any custom integration, and it is stable.

If you want to experiment with more functions with the Thunderboard Sense 2, check out [this HA custom integration](https://github.com/cheadrian/thunderboard-ha).

## Related

- [Bluetooth - BTHome v2 - Humidity and Temperature Monitor (SHTC3)](https://github.com/SiliconLabs/bluetooth_applications/tree/master/bluetooth_bthome_v2_humidity_and_temperature_monitor#bluetooth---bthome-v2---humidity-and-temperature-monitor-shtc3)
- [Bluetooth - BTHome v2 - Switch](https://github.com/SiliconLabs/bluetooth_applications/tree/master/bluetooth_bthome_v2_switch)
- [TPHD - Example - BTHome v2](https://github.com/SiliconLabs/third_party_hw_drivers_extension/tree/master/app/example/bthome_v2)