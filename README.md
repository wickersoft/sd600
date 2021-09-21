# Custom Firmware for the Illuminate SD600 Photography Flash

[Some not quite correct schematics!](/schematics)

While repairing a number of SD600 flash lamps, I destroyed the MCU of one of them. Not accepting its loss, I proceeded to reverse engineer its circuitry and write some firmware that imitates most of the original's functions.

**Features:**
* Charging
* Charge-Ready Beep, can be disabled by switch
* Fault detection based on capacitor bank's charging curve
* Waste flash when intensity is decreased
* Dimming of incandescent lamp, can be disabled by switch
* Responds to Test switch and signal on 6.3mm jack
* Response time to trigger signal ~2µs
* Optical trigger
* Model/Full Switch

**To be added:**
* Possibly remove dependencies on Arduino core

The firmware was built in Arduino because the MCU is an ATmega48P. ATmega328P in QFP32 package is fully compatible with SD600 controller board. Controller board also contains a 6x1 ICSP header in 2mm pin pitch.
