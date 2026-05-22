# ESPHome-HPMA115S0-Sensor-Component
Integration for the HPMA115S0 Dust and Particle Sensor into ESPHome.

This repository started from the upstream project by tom1422 and adds support for the HPMA variant observed on this installation.

## Added in this fork
- Autosend frame support for devices emitting 0x42 0x4D frames on UART.
- Standard autosend mapping for the non-compact variant: Data1 -> PM2.5 and Data2 -> PM10.
- Public commands for start measurement, stop measurement, enable autosend, stop autosend, set adjustment coefficient, and read adjustment coefficient.
- Optional adjustment coefficient diagnostic sensor.

## How to install
1. Download this repo through main branch or the releases page.
2. Extract the "custom_components" folder and place it in your esphome directory (where your .yaml files are works for me)
3. Add the sensor configuration to your ESPHome Project:
```
  - platform: hpma115s0_esphome
    pm_2_5:
      name: "Particulate Matter 2.5"
    pm_10_0:
      name: "Particulate Matter 10.0"
    # These aqi lines are optional
    aqi_2_5:
      name: "Air Quality Index Particulate Matter 2.5"
    aqi_10_0:
      name: "Air Quality Index Particulate Matter 10.0"
    #Uncomment below if you have the compact sensor
    #pm_1_0:
    #  name: "Particulate Matter 1.0"
    #pm_4_0:
    #  name: "Particulate Matter 4.0"
    update_interval: 10s
```
4. Next add the uart configuration near the top of your ESPHome Project file:
```
uart:
  id: uart_bus
  tx_pin: 9 #Change tx_pin and rx_pin to the ones you're using
  rx_pin: 10
  baud_rate: 9600
```

## Use directly from GitHub with external_components
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Mamath2000/ESPHome-HPMA115S0-Sensor-Component.git
    components: [ hpma115s0_esphome ]
```

## Issues
Please report issues/improvements in the issues tab along with any debug info you find. Thank you.

## Releases
To see version history, [click here](https://github.com/tom1422/ESPHome-HPMA115S0-Sensor-Component/releases).
