# INTELLIGENT GREENHOUSE

## Overview

This is a System which allows you not to worry about future of your plants. It has autonomic water pump system which is based on humidity sensor, also light is referenced to illumination sensor. You will be informed if the watertank catches critical level.

## Description

The system is equipped light sensor, humidity sensor and autonomic waterpump system which collects measurements every second, to make the perfect conditions for your plants. The algorithm of actions is very simple. Is too dark? Inteliligent Greenhouse (next. IG) will turn the ligth on. The soil moisture is too low? IG will pump some water. Keep calm, IG gives the water in small portions and wait some time for next result from sensor to awoid making the ground too wet. The water level in canister is displayed by value between 0.000 and 1.000

#### Relevant software:

- STM32CubeMX 5.0.0
- STM Studio
- System Workbench for STM32


## Tools

#### STM32F407G-DISC1 - Discovery - STM32F4DISCOVERY:
- 32-bit ARM®Cortex®-M4 with FPU core
- 1-Mbyte Flash memory
- 192-Kbyte RAM in an LQFP100 package

#### Analog Soil Moisture Sensor:
- Power supply: 3.3v or 5v
- Output voltage signal: 0~4.2v
- Current: 35mA

#### Liquid Level Sensor:
- Detection depth: 48mm
- Power: 2.0V ~ 5.0V
- Mounting holes size: 2.0mm

#### Light Sensor, Ambient Light Detecting:
- Resolution: 16 bit
- Operating voltage: 3.3V ~ 5V
- Mounting holes size: 2.0mm

#### Songle Relay SRD-05VDC-SL-C:
- Coil Voltage: 5VDC
- Contact Rating (Current): 10A
- Switching Voltage: (250VAC , 110VDC) Max

#### 6V Mini Water Pump:
- Working Voltage: 4v-12V
- Working Current: 0.8A
- Motor Diameter: 27mm
- Water Pump lenght: 52mm
- Drain Hole: 4mm

## How to run

### Modules connections to STM32F407G:
#### Analog Soil Moisture Sensor:
- A0 - PA6
- 3,3V 
- GND

#### Liquid Level Sensor:
- AOUT - PA4
- 5V
- GND

#### Light Sensor, Ambient Light Detecting:
- SCL - PA8
- SDA - PC9
- 3,3V
- GND


## How to compile

Import project to System Workbench for STM32 and build program. If everything is ready, you can run program. Make sure that you have plugged STM32 to your PC. If program is imported to STM, it can be powered by simple phone charger.

## Future improvements

- Add better display to project
- Temperature sensor

## Attributions

- Light intensity digital sensor TSL2581FN documentation:  https://www.waveshare.com/w/upload/6/69/TSL2580-81_DS000417_1-00.pdf

## License

[MIT](https://choosealicense.com/licenses/mit/)



## Credits

- Maciej Anglart
- Arkadiusz Dokowicz
- Mi³osz Szkudlarek

The project was conducted during the Microprocessor Lab course held by the
Institute of Control and Information Engineering, Poznan University of Technology.

Supervisor: Adam Bondyra
