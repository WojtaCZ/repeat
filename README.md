# REPEAT - Smart IoT Gateway

REPEAT is a device based on the nRF5340, intended to serve as an IoT gateway in BLE, Zigbee, Thread or Matter networks.  It fully supports the previously mentioned protocols for wireless communication, for wired link, an 100Mbit ethernet interface, with PoE 802.3 af for power, is implemented. USB C is also used for additional communication and power. 

<p align="center">
<img src="./resources/pcb.png" width="400"/>
</p>

The device supports Zephyr RTOS. [Drivers](zephyr-w5100) (for the ethernet interface) and [board definition files](boards) are provided.

## Hardware

The board is equiped with an nRF5340 as the main SoC with 32Mbit flash memory chip (MX25R3235F) for data storage. nRF21540 is used as an RF frontend. W5100S serves as a 100Mbit ethernet MAC.SI3402 is used for PoE power managment. 

Pin mapping of the MCU can be found bellow. For more information, [the schematic](schematic.pdf) or the [the KiCad project](repeat-hw) is available.

<p align="center">
<img src="./resources/mcu.svg" width="800"/>
</p>

## Programming

For programming the device, a [SOICbite](https://github.com/SimonMerrett/SOICbite) connector is used, thus a SOIC8 clip is needed to establish the connection. The provided clip has a white mark, indicating what side should be at the TOP side of the PCB, as seen in figure bellow.

<p align="center">
<img src="./resources/clip.png" width="800"/>
</p>

The clip can be connecter to a standard ARM Debug connector on any nRF DK or programmer. **REPEAT needs to be powered externally while being programmed, the debug connector does not serve as a power source.**

### Board definition

[Board definition files](boards) for the REPEAT board are provided and need to be installed in Zephyr for the board to work. **The provided files are still a work in progress but should be fully functional.**

### Wiznet W5100S driver

The W5100S IC used as a ethernet MAC has no available drivers in Zephyr, thus a [custom driver](zephyr-w5100) needs to be installed in Zephyr, if the ethernet functionality is required.


## Software example

A simple [software example](example) is provided, to demonstrate a BLE to MQTT gateway functionality. The repeat board listens to all bluetooth advertisements and sends the MAC address, RSSI and raw data in JSON format to the MQTT broker on topic `/repeat/ble/XX:XX:XX:XX:XX:XX` where _XX:XX:XX:XX:XX:XX_ is MAC address of the sender.
