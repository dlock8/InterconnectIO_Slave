# First TestStation InterconnectIO SCPI Slave firmware

The complete FTS project is documented on a github website:  https://dlock8.github.io/FTS_Website/

On this deposit, we have the firmware used to create the Slave Pico load to control the interconnectIO board.

The hardware support for the firmware is on github location: https://github.com/dlock8/InterconnectIO_Board

The Slave Pico firmware enables the Raspberry Pi Pico to function as an I2C slave device, acting as a large Input/Output integrated circuit. 
The device receives a command byte, which specifies the action to be performed, followed by a data byte indicating where the action should be applied.

## License
This project is licensed under the BSD 3-Clause License. See the [LICENSE](./LICENSE) file for more details.


## Project setup

This project has been developped on raspberry pi 4 following installation instruction from this pdf [getting-started-with-pico_C.pdf](./documentation/getting-started-with-pico_C.pdf),  a copy of the pdf is located on the main folder.
Visual studio has been used for development and raspberry pi 4 for Pico debug.  

Based on https://github.com/vmilea/pico_i2c_slave. The pico_i2c_slave software has been added to enable the Raspberry Pi Pico to function as an I2C device.


The compilation is performed using Visual Studio and the important extension installed are:

* Cmake v0.0.17
* Cmake Tools v1.20.10
* Cortex-Debug v1.12.1
* debug-tracker-vscode v0.0.15
* Doxygen Documentation Generator v1.4.0
* Doxygen runner v1.8.0
* Github Pull Request v0.96.0
* Hex Editor v1.10.0
* MemoryView v0.0.25
* peripheral Viewer v1.4.6
* RTOS view v0.0.7
* C/C++ v1.21.6


I not sure if all extensions are required but is the one installed presently.


## Building

Build of this cmake project is performed with Visual Studio

## Development

* [`slave.c`](slave.c) is the main source file for the firmware.
* [`CMakeLists.txt`](CMakeLists.txt) contains build instructions for CMake, including how to build the SCPI library.
* [`pico_sdk_import.cmake`](pico_sdk_import.cmake) was (as usual) copied verbatim from the Pico SDK and allows CMake to interact with the SDK’s functionality.

## Installation

* The Files slave.uf2 contains the firmware to be loaded on the Pico RP2040 board using USB cable and boot button.
* When software loaded, the Pico board should be installed on the location marked SLAVE_1, SLAVE_2 and SLAVE_3 on interconnectIO Board.
* On board Pico Led will flash slowly (heartbeat) on power ON.


## I2C Command supported

Pico i2C Slave mode Protocol

I2C Command is 2 bytes long:  Command_byte (1 byte) + Data (1 byte)


| Command_Byte | Function   |  Description |
| --- | --- | --- |
| 00 |  Reserved  | used for special purpose |
| 01 | Get Major Version  | return major version of firmware |
| 02 | Get Minor Version  | return minor version of firmware |
| 10 | Clear GPx          | Write 0 on GPx                   |
| 11 | Set GPx            | Write 1 on GPx                   |
| 12 | Clear Bank x       | Open all relay from bank x       |
| 13 | Read Bank x        | Read Bank status (Bit0 = CH0, Bit1=CH1, Bit7=CH7) |
| 15 | Read GPx           | Read GPx state    | 
| 20 | Set Dir GPx Out    | Set direction Out for Gpx  |                                   
| 21 | Set Dir GPx In     | Set direction In for Gpx  |                                   
| 25 | Get Dir GPx        | Read GPX direction, 0 = In , 1 = Out |
| 30 | Set GPx strength = 2mA  | Set GPx output max current |
| 31 | Set GPx strength = 4mA  | Set GPx output max current |
| 32 | Set GPx strength = 8mA  | Set GPx output max current |
| 33 | Set GPx strength = 12mA | Set GPx output max current |
| 35 | Get Gpx drive strength  | Read strength:  0: 2mA, 1: 4mA, 2: 8mA, 3: 12mA |
| 41 | Set pull_up GPx         | Add pull-up to Gpx  |
| 45 | Get pull-up GPx         | Read pull-up state (1: pull-up active) |  
| 50 | Disable pulls           | Remove  pull-up and pull-down  |
| 51 | Set pull_down GPx       | Add pull-down to Gpx    |  
| 55 | Get pull-down GPx       | Read pull-down state (1: pull-down active) |  
| 60 | Set Pads State value    | Set Pads State register to use with command 61 |
| 61 | Set GPx to Pads State   | Write contains of command 60 on GPx |   
| 65 | Get Pads state Gpx      | Read PAD register for Gpx |



## 8 Bit I/O port I2C Command

| Command_Byte | Function   |  Description |
| --- | --- | --- |
|80  | Set IO Mask Port 0    | 8 bit mask direction   0 = In , 1 = Out |
|81  | Set IO Output Port 0  | Set Output Line   0= Low  1=High       |
|85  | Read IO Input Port 0  | Get Input Line    0= Low  1=High      |
|90  | Set IO Mask Port 1    | 8 bit mask direction   0 = In , 1 = Out |
|91  | Set IO Output Port 1  | Set Output Line   0= Low  1=High     |
|95  | Read IO Input Port 1  | Get Input Line    0= Low  1=High        |
|100 | Device Status         | Bit Status  (8 bits)             |  
|    | Bit 0                 | Config Completed   0: true |
|    | Bit 1                 | Command accepted   0: true |
|    | Bit 2                 | Error  1= true|
|    | Bit 3                 | watchdog trigged 1= true|


## I2C Communication Example

On these example, we use the i2c loopback mode 

    send_master(11, 22);     Command: 11, Data:22.   Set the GPIO 22 to 1 
    send_master(15, 0x02);   Command: 15, Data:0x02. Read the state of GPIO 02 
    send_master(85, 0xC0);   Command: 85, Data:0xC0. Read the Input pin 7 and 6 of Port 0 
    send_master(100, 0x00);  Command: 100, Data:0x00. Read the Device Status, Data 0x00 is mandatory but not used 
