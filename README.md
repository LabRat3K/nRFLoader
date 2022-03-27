NRF Bootloader for PIC16F1823/25
================================

This is a bare bones OTA bootloader written entirely in assembly. Designed for a microchip pic (16f1823), connected to a NRF24L01 wireless module. To communicate with the device I use an arduino with another NRF24L01 module, to take the serial commands from the bootloader, and transmit them to the device. As with the original bootloader by Matt Sarnoff, I have managed to keep the code footprint within 512-bytes.

To do: extend to PIC16F1825, update the Arduino code to be more robust, replace the Arduino with a WNRF (WIFI to NRF gateway), and spend some time on the timings surrounding role (tx/rx) change, so that this can be made more reliable. But it does work! 

This development was influenced by the workby Matt Sarnoff and his 512-byte USB bootloader for the 16F1454. The upload tool is cloned from the repo with minor updates to support serial communications, and a 16 byte write latch. 

## License

The contents of this repository, including the bootloader itself, the programming script, and the sample code, is released under a [3-clause BSD license](LICENSE). If you use the bootloader for a project, commercial or non-commercial, linking to this GitHub page is strongly recommended. 

## Credits

Written by Andrew Williams.
Another Useless Labrat Project.

Based on [PIC16F1-USB-Bootloader](https://github.com/74hc595/PIC16F1-USB-Bootloader)
