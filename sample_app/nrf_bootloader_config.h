/**
 * Based upon USB 512-Word CDC Bootloader Application Code
 * Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
 * v1.0, February 12, 2015
 * Released under a 3-clause BSD license: see the accompanying LICENSE file.
 *
 * Bootloader configuration convenience macro for applications.
 * The APP_CONFIG() macro must be invoked in global scope in exactly one
 * source file.
 */
#ifndef BOOTLOADER_NRF_CONFIG_H
#define BOOTLOADER_NRF_CONFIG_H

/**
 * Application Version identifier
 */
#define APP_CONFIG(major, minor)  __code unsigned char app_config = (major&0x0f<<4)|(minor&0x0f)

#endif
