# lorawan_eharvester_node
hardware and code artefacts of lorawan energy harvester node

<img src="https://raw.githubusercontent.com/ablexOnGithub/energyharvester/master/img/Energy__Harvester_3D.PNG" alt="3D view of harvester PCB">

## Technical features
* AVR ATmega328PB
* RFM95 LoRaWAN-Modul 868MHz
* TI BQ25570 Energy Harvester with MPPT, configured for solar power source
* 1F Supercap
* 2x Grove I2C
* 1x Grove Analog
* 1x Grove Digital
* 1x JST for battery connection
* 1x JST for solar panel connection
* 1x FTDI connector for serial console and programming
* 1x u.FL socket for LoRa antenna
* Power supply can be configured for different battery (LiPo, LiFePo or similar)
* Switchable Vcc for Grove connectors through CMOS switches
* ISP socket for AVR programming

## Sensor connections
In general all sensor sockets (Grove layout) have a switchable Vcc line. It is off by default and has to enabled by corresponding GPIO lines.

### I2C
There are 2 hardware I2C ports in the 328PB. Arduino currently supports one out of the box (running as a 328P Arduino Pro Mini clone). The second I2C port is prepared for future use.

#### Grove I2C 01
The SDA and SCL lines of the socket are connected to PC4 and PC5 accordingly.

__GPIO PD4 == High (1) activates Vcc__

When activating Vcc, additionally SDA and SCL are pulled high to Vcc via 10 kOhm resistors. 

#### Grove I2C 02
The SDA1 and SCL1 lines of the socket are connected to PE0 and PE1 accordingly.

__GPIO PD5 == High (1) activates Vcc__

When activating Vcc, additionally SDA1 and SCL1 are pulled high to Vcc via 10 kOhm resistors.

### Grove Analog
The analog signal connected to the socket is fed into the ADC1 input of the AVR.

__GPIO PD6 == High (1) activates Vcc__

### Grove Digital
Vcc on the digital Grove socket is permanently supplied.

## Energy harvesting power supply
The Energy Harvester integrates a harvesting power supply chip BQ25570 from Texas Instruments. It provides the 3.3V Vcc supply voltage for the AVR and attached sensors. It also provides the charging voltage for the batteries and the supercap. It also features MPPT (Maximum Power Point Tracking), to extract maximum energy from solar panels even in low light scenarios. Other power sources like thermal energy generators (TEG) can also be adapted as power source. 5.5V is the maximum allowed charge voltage level of the BQ25570. And the maximum input voltage of the power source is 5.1V. The allowed peak input power is 510mW, thus a solar cell for this energy harvester should be selected accordingly and cells > 0.5W seem not to be feasible. A better approach would be to use two 0.25W cells in parallel to adapt to the sun path by pointing into different directions.

Product page:
http://www.ti.com/product/BQ25570

### Configuration of battery type
The image shows a resistor configuration sheet (example of LiFePo configuration), provided as a template by Texas Instruments. The sheet references the predecessor BQ25505, which provides the same configuration system with multiple resistors. All resistors are 1% 1/10W SMD parts in size 0603.

<img src="https://raw.githubusercontent.com/ablexOnGithub/energyharvester/master/img/LiFePo_configuration.PNG" alt="Example of resistor configuration sheet">

#### Li-Po cells
To configure the harvester for LiPo battery cells the corresponding resistors have to have the following values. This adjusts the overvoltage level

Part 	| Value
ROK1 	| 4.99 MOhm
ROK2_1	| 3.3 MOhm
ROK2_2	| 4.02 MOhm
ROK3 	| 806 kOhm
ROV1	| 5.62 MOhm
ROV2_2  | 3.3 MOhm
ROC1a 	| 3.3 MOhm
ROC1b 	| 10 MOhm
ROC2a	| 1 MOhm
ROC2b 	| 5.6 MOhm

#### LiFePo cells
To configure the harvester for LiFePo battery cells the corresponding resistors have to have the following values.

Part 	| Value
ROK1 	| 4.99 MOhm
ROK2_1	| 4.99 MOhm
ROK2_2	| 1 MOhm
ROK3 	| 1 MOhm
ROV1	| 4.99 MOhm
ROV2_1  | 4.02 MOhm
ROV2_2  | 806 kOhm
ROC1a 	| 3.3 MOhm
ROC1b 	| 10 MOhm
ROC2a	| 1 MOhm
ROC2b 	| 5.6 MOhm

#### Supercap mode
Info for a Supercap only mode. I'm currently trying out a Vishay supercap with 90F capacity at 5.5V. Idea is to increase overvoltage level to 5.5V roughly, to have more headroom for the Buck converter to longer provide 3.3V to the sensor electronics. 

Untested resistor configuration:

Part 	| Value
ROK1 	| 4.22 MOhm
ROK2_1	| 4.02 MOhm
ROK2_2	| 3.3 MOhm
ROK3 	| 681 kOhm
ROV1	| 3.3 MOhm
ROV2_1  | 3.3 MOhm
ROV2_2  | 3.3 MOhm
ROC1a 	| 3.3 MOhm
ROC1b 	| 10 MOhm
ROC2a	| 1 MOhm
ROC2b 	| 5.6 MOhm

## Tracking of Vcc
Using a voltage divider Vcc is measured against the AVR internal reference voltage (1.1V, selected via Software). The corresponding voltage is fed into ADC0 of the AVR, to allow for a dynamic adjustment of the sleep interval, related to the actual Vcc voltage level. The lower Vcc the longer the sleep intervall to safe power.