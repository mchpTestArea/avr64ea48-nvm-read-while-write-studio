<!-- Please do not change this logo with link -->

[![MCHP](images/microchip.png)](https://www.microchip.com)

# Update the title for avr64ea48-nvm-read-while-write-studio here

<!-- This is where the introduction to the example goes, including mentioning the peripherals used -->

## Related Documentation

<!-- Any information about an application note or tech brief can be linked here. Use unbreakable links!
     In addition a link to the device family landing page and relevant peripheral pages as well:
     - [AN3381 - Brushless DC Fan Speed Control Using Temperature Input and Tachometer Feedback](https://microchip.com/00003381/)
     - [PIC18F-Q10 Family Product Page](https://www.microchip.com/design-centers/8-bit/pic-mcus/device-selection/pic18f-q10-product-family) -->

## Software Used

<!-- All software used in this example must be listed here. Use unbreakable links!
     - MPLAB® X IDE 5.30 or newer [(microchip.com/mplab/mplab-x-ide)](http://www.microchip.com/mplab/mplab-x-ide)
     - MPLAB® XC8 2.10 or a newer compiler [(microchip.com/mplab/compilers)](http://www.microchip.com/mplab/compilers)
     - MPLAB® Code Configurator (MCC) 3.95.0 or newer [(microchip.com/mplab/mplab-code-configurator)](https://www.microchip.com/mplab/mplab-code-configurator)
     - MPLAB® Code Configurator (MCC) Device Libraries PIC10 / PIC12 / PIC16 / PIC18 MCUs [(microchip.com/mplab/mplab-code-configurator)](https://www.microchip.com/mplab/mplab-code-configurator)
     - Microchip PIC18F-Q Series Device Support (1.4.109) or newer [(packs.download.microchip.com/)](https://packs.download.microchip.com/) -->

- Microchip Studio for AVR® and SAM Devices 7.0.132 or newer [(Microchip Studio for AVR® and SAM Devices 7.0.132)](https://www.microchip.com/en-us/development-tools-tools-and-software/microchip-studio-for-avr-and-sam-devices?utm_source=GitHub&utm_medium=TextLink&utm_campaign=MCU8_MMTCha_MPAE_Examples&utm_content=avr64ea48-nvm-read-while-write-studio-github)
- AVR® GCC 5.4.0 or newer compiler [(AVR® GCC 5.4)](https://www.microchip.com/en-us/development-tools-tools-and-software/gcc-compilers-avr-and-arm?utm_source=GitHub&utm_medium=TextLink&utm_campaign=MCU8_MMTCha_MPAE_Examples&utm_content=avr64ea48-nvm-read-while-write-studio-github)

## Hardware Used

<!-- All hardware used in this example must be listed here. Use unbreakable links!
     - PIC18F47Q10 Curiosity Nano [(DM182029)](https://www.microchip.com/Developmenttools/ProductDetails/DM182029)
     - Curiosity Nano Base for Click boards™ [(AC164162)](https://www.microchip.com/Developmenttools/ProductDetails/AC164162)
     - POT Click board™ [(MIKROE-3402)](https://www.mikroe.com/pot-click) -->

## Setup

<!-- Explain how to connect hardware and set up software. Depending on complexity, step-by-step instructions and/or tables and/or images can be used -->

## Operation

<!-- Explain how to operate the example. Depending on complexity, step-by-step instructions and/or tables and/or images can be used -->

## Summary

<!-- Summarize what the example has shown -->

Independent of page size, the device program memory is split into two physical sections:
* Non Read-While-Write (NRWW) and
* Read-While-Write (RWW)  

![Memory Map](images/AVR-EA%20Memory%20Map.jpg)

The sizes for NRWW and RWW are available in the Memory Overview table in the part datasheet.

![Memory Size Overview](images/AVR-EA%20Memory%20Size%20Overview.jpg)

The logical BOOT section size can overlap all or part of the NRWW section, as shown above. 

An application, configured such that the routines for reading from and writing to the program memory are placed in the NRWW section, allows for the CPU to not be blocked while such procedures are in progress. 

A use case is a Bootloader scenario, where CPU is servicing commands from communication peripheral (I2C/USART) to the Host programmer, and simultaneously programming the RWW section, containing App Code/Data. Another use case could be a data logging scenario in which an analog peripheral interrupts regularly with a stream of data, which simultaneously needs to be stored to program memory. In both scenarios it is important that the CPU is not blocked while the NVMCTRL peripheral performs the operation on program memory.

The current application will show the benefits of having the program memory split into NRWW/RWW. The user will be able to compare these benefits against the standard way AVR 8 bit microcontrollers operate, that do not have this section split. 

Upon building the project and programming the executable file to the AVR-EA device pymcuprog is used to read the contents of the program memory where data is going to be written to. 

> pymcuprog read -m flash -o 0x1f00 -b512

Expected result is as follows:

<pre>
Reading...
Memory type: flash
---------------------------------------------------------
0x801F00: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
0x801F10: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801F20: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801F30: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801F40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801F50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801F60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801F70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

0x801F80: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801F90: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801FA0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801FB0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801FC0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801FD0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801FE0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x801FF0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 
0x802000: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802010: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802020: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802030: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802040: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802050: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802060: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802070: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 
0x802080: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x802090: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x8020A0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x8020B0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x8020C0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x8020D0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x8020E0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0x8020F0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
---------------------------------------------------------

</pre>

The user is invited to press the button SW0. To confirm the press the LED0 will be lit as long as the button will be pressed. A button press will trigger the generation of data and commiting it to program memory. 

Once again the user is invited to read the prorgam memory with the same command as above.

<pre>
Reading...
Memory type: flash
---------------------------------------------------------
0x801F00: 00 00 00 00 01 01 01 01 02 02 02 02 03 03 03 03
0x801F10: 04 04 04 04 05 05 05 05 06 06 06 06 07 07 07 07
0x801F20: 08 08 08 08 09 09 09 09 0A 0A 0A 0A 0B 0B 0B 0B
0x801F30: 0C 0C 0C 0C 0D 0D 0D 0D 0E 0E 0E 0E 0F 0F 0F 0F
0x801F40: 10 10 10 10 11 11 11 11 12 12 12 12 13 13 13 13
0x801F50: 14 14 14 14 15 15 15 15 16 16 16 16 17 17 17 17
0x801F60: 18 18 18 18 19 19 19 19 1A 1A 1A 1A 1B 1B 1B 1B
0x801F70: 1C 1C 1C 1C 1D 1D 1D 1D 1E 1E 1E 1E 1F 1F 1F 1F

0x801F80: 20 20 20 20 21 21 21 21 22 22 22 22 23 23 23 23
0x801F90: 24 24 24 24 25 25 25 25 26 26 26 26 27 27 27 27
0x801FA0: 28 28 28 28 29 29 29 29 2A 2A 2A 2A 2B 2B 2B 2B
0x801FB0: 2C 2C 2C 2C 2D 2D 2D 2D 2E 2E 2E 2E 2F 2F 2F 2F
0x801FC0: 30 30 30 30 31 31 31 31 32 32 32 32 33 33 33 33
0x801FD0: 34 34 34 34 35 35 35 35 36 36 36 36 37 37 37 37
0x801FE0: 38 38 38 38 39 39 39 39 3A 3A 3A 3A 3B 3B 3B 3B
0x801FF0: 3C 3C 3C 3C 3D 3D 3D 3D 3E 3E 3E 3E 3F 3F 3F 3F

0x802000: 42 42 42 42 43 43 43 43 44 44 44 44 45 45 45 45
0x802010: 46 46 46 46 47 47 47 47 48 48 48 48 49 49 49 49
0x802020: 4A 4A 4A 4A 4B 4B 4B 4B 4C 4C 4C 4C 4D 4D 4D 4D
0x802030: 4E 4E 4E 4E 4F 4F 4F 4F 50 50 50 50 51 51 51 51
0x802040: 52 52 52 52 53 53 53 53 54 54 54 54 55 55 55 55
0x802050: 56 56 56 56 57 57 57 57 58 58 58 58 59 59 59 59
0x802060: 5A 5A 5A 5A 5B 5B 5B 5B 5C 5C 5C 5C 5D 5D 5D 5D
0x802070: 5E 5E 5E 5E 5F 5F 5F 5F 60 60 60 60 61 61 61 61

0x802080: 62 62 62 62 63 63 63 63 64 64 64 64 65 65 65 65
0x802090: 66 66 66 66 67 67 67 67 68 68 68 68 69 69 69 69
0x8020A0: 6A 6A 6A 6A 6B 6B 6B 6B 6C 6C 6C 6C 6D 6D 6D 6D
0x8020B0: 6E 6E 6E 6E 6F 6F 6F 6F 70 70 70 70 71 71 71 71
0x8020C0: 72 72 72 72 73 73 73 73 74 74 74 74 75 75 75 75
0x8020D0: 76 76 76 76 77 77 77 77 78 78 78 78 79 79 79 79
0x8020E0: 7A 7A 7A 7A 7B 7B 7B 7B 7C 7C 7C 7C 7D 7D 7D 7D
0x8020F0: 7E 7E 7E 7E 7F 7F 7F 7F 80 80 80 80 81 81 81 81
---------------------------------------------------------
</pre>

Data is available in the program memory. 

The user is invited to probe the follwing pins, in order to notice the benefits of NRWW/RWW operation:
* PA2 - line toggled every time the TCB0 interrupt is serviced 
* PA3 - indicates a fill buffer operation (prior to writing to program memory)
  * HIGH - buffer loading in progress
  * LOW - finished (or no) buffer loading
* PA4 - indicates a flash page erase operation
  * HIGH - page erase in progress
  * LOW - page erase finished (or no) erase operation
* PA5 - indicates a flash page write operation
  * HIGH - page write in progress
  * LOW - page write finished (or no) write operation

Upon the press of the button operation of the firmware is as follows
* initialise a write to NRWW program memory area
  * enable TCB0 - simulates data aquisition 
* erase the NRWW program memory area where data will be later saved
  * one can observe the 2 HIGH periods in the FLPER channel. 
  * during both these periods the PA2 is not toggling, which means that the TCB0 interrupt is not triggered, as the CPU is held, while the NVMCTRL(er) erases the program memory
  * 2 flash pages are erased to in the area: 0x1F00 - 0x1FFF
* write (aquired) data to the NRWW program memory area 
  * PA3 high once enough data to fill a page has been aquired; data copied into the buffer
  * PA5 high once as soon as the write operation starts.
    * during this periods the PA2 is not toggling, which means that the TCB0 interrupt is not triggered, as the CPU is held, while the NVMCTRL(er) writes the program memory page
    * 2 flash pages are written to in the area: 0x1F00 - 0x1FFF
* erase the RWW program memory area where data will be later saved
  * one can observe the 2 HIGH periods in the FLPER channel. 
  * during both these periods the PA2 is toggling, which means that the TCB0 interrupt is triggered, and the CPU is servicing the interrupt, while, in parallel, the NVMCTRL(er) erases the program memory
  * 2 flash pages are erased in the area: 0x2000 - 0x20FF
* write (aquired) data to the RWW program memory area 
  * PA3 high once enough data to fill a page has been aquired; data copied into the buffer
  * PA5 high once as soon as the write operation starts.
    * during this periods the PA2 is toggling, which means that the TCB0 interrupt is triggered, as the CPU is held, while the NVMCTRL(er) writes the program memory page
    * 2 flash pages are written to in the area: 0x2000 - 0x20FF
* FW resumes IDLE state, waiting for a button press that will repeat the operation above


![Scope Capture](images/AVR-EA%20Scope%20Capture.jpg)


Note: 
* Interrupt code located in the RWW section may halt the CPU if the associated interrupt is triggered while the RWW section is erased or written. In different words, for the CPU not block, while the write operation on RWW program memory is in progress, the code that the CPU reads, needs to be located in the NRWW area. 
Moving the interrupt vector table

### Moving the Interup Vector Table
Note that after reset, the default vector table location is after the BOOT section, as long as the BOOT fuse is different than 0. The peripheral interrupts can be used in the code running in the BOOT section by relocating the interrupt vector table at the beginning of this section. That is done by setting the IVSEL bit in the CPUINT.CTRLA register. Refer to the CPUINT section for details. If no interrupt source is used, then there is no need to change this value. 

Functions or interrupts placed in the NRWW can run while an operation on the RWW section is ongoing. However steps should be taken to ensure there are no interrupts or jumps to code located inside the RWW. This may lead to the software end up in a unknown state or the CPU blocking, waiting for the erase/write operation to complete. Hence the benefits of the NRWW/RWW split are forfeit, at best.
