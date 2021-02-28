NRF Bootloader for PIC16F1823/25
================================

This is a bare bones bootloader written entirely in assembly. It is still a work in progress and as of yet does not work 100%. (Still struggling with the AutoAck feature)

This development was influenced by the workby Matt Sarnoff and his 512-byte USB bootloader for the 16F1454. The upload tool is cloned from the repo with minor updates to support serial communications, and a 16 byte write latch. 

## License

The contents of this repository, including the bootloader itself, the programming script, and the sample code, is released under a [3-clause BSD license](LICENSE). If you use the bootloader for a project, commercial or non-commercial, linking to this GitHub page is strongly recommended. 

## Credits

Written by Andrew Williams.
Another Useless Labrat Project.

Based on [PIC16F1-USB-Bootloader](https://github.com/74hc595/PIC16F1-USB-Bootloader)
