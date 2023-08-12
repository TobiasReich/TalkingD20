# Talking D20
This is an adaptation of the tutorial project from Adafruit.
If you want the ordinary thing, have a look here:

https://learn.adafruit.com/talking-d20-20-sided-gaming-die


## Overview

My problem was that the Adafruit Trinked did not work well for me. And since I wanted to invest some more time in other devices like the ESP32 and the XIAO nRF52840 I wanted to use the later one instead. This might make things even better since it includes already a 6 DOF IMU device.

The code however will be originating from here

https://learn.adafruit.com/talking-d20-20-sided-gaming-die/

with the 3d models coming from here:

https://www.thingiverse.com/thing:955433

I guess we will find out how well this works in the end - and if it gets better by using this type of device. What I was hoping for was a smaller device (since it is smaller than the trinket and does not require the IMU) as well as some better power management (as far as I can say it comes directly with a battery charging chip) so I wouldn't need the Adafruit Battery Backpack (https://www.adafruit.com/product/2124).

More info about the used nRF52840 can be found here:

https://wiki.seeedstudio.com/XIAO_BLE/


## Devices

nRF52840:
https://wiki.seeedstudio.com/XIAO_BLE/

Adafruit Audio FX Sound Board - 2MB version:
https://www.adafruit.com/product/2133

Amplifier:
https://www.adafruit.com/product/2130


# Pinouts (for now)

## Sound Board

- Ground
- Power - 3.3V
- TX - GPIO 6 (DEFAULT TX)
- RX - GPIO 7 (DEFAULT RX)
- RESET - GPIO 8
- UG - GROUND

## Amplifier

- Ground
- Power: 3.3V
- AUDIO IN +: Audio Board R channel
- AUDIO IN -: Audio Board Gnd channel
- SHUTDOWN: NOT CONNECTED (for now?)


# Known issues:

-


# Schematics

-
