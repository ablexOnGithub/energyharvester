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
* 1x JST for battery connect
* 1x JST for solar panel connection
* 1x FTDI connector for serial console and programming
* 1x u.FL socket for LoRa antenna
* Power supply can be configured for different battery (LiPo, LiFePo or similar)
* Switchable Vcc for Grove connectors through CMOS switches
* ISP socket for AVR programming

## Energy harvesting power supply

## Sensor connections

## Configuration of battery type
The image shows a resistor configuration sheet (example of LiFePo configuration), provided as a template by Texas Instruments, the manufacturer of the harvesting power supply chip BQ25570. The sheet references the predecessor BQ25505, which provides the same configuration system with multiple resistors.

<img src="https://raw.githubusercontent.com/abourgett/lorawan_eharvester_node/master/img/LiFePo_configuration.PNG" alt="Resistor configuration sheet">


### Li-Po cells

### LiFePo cells
