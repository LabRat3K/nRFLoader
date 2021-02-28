; vim:noet:sw=8:ts=8:ai:syn=pic
;
; NRF24L01 Bootloader for PIC16(L)F1823/5
; Copyright (c) 2021, Andrew Williams (ratsnest.ca)
; Based on project by Matt Sarnoff
;
; Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
; v1.0, February 12, 2015
; Released under a 3-clause BSD license: see the accompanying LICENSE file.
;
; Constants used by the bootloader communications protocol.
; *All* messages are 32 bytes in length, first byte
NRF_BL_CMD              equ     0x00

; START command: <0x00> <ADRL> <ADRH> <ERASE?>
NRF_BL_CMD_START        equ     0x00
NRF_BL_START_ADRL       equ     1
NRF_BL_START_ADRH       equ     2
NRF_BL_START_ERASE      equ     3

; MIDDLE command:<0x01> <30 bytes (15 words)> (64 byte flash requires 2x this mesage)
NRF_BL_CMD_WRITE        equ     0x01
NRF_BL_WRITE_START      equ     1

; COMMIT command:<0x02><size><cksum>< size*2 bytes (size words)>
; incl size of the DATA payload
NRF_BL_CMD_COMMIT       equ     0x02
NRF_BL_COMMIT_SIZE      equ     1
NRF_BL_CKSUM            equ     2
NRF_BL_COMMIT_DATA      equ     3

; AUDIT command: <0x03><ADRL><ADRH><SIZEL><SIZEH><CKSUM_ODD><CKSUM_EVEN>
NRF_BL_CMD_AUDIT        equ     0x03
NRF_BL_AUDIT_ADRL       equ     1
NRF_BL_AUDIT_ADRH       equ     2
NRF_BL_AUDIT_SIZEL      equ     3
NRF_BL_AUDIT_SIZEH      equ     4
NRF_BL_AUDIT_ODDSUM     equ     5
NRF_BL_AUDIT_EVENSUM    equ     6

; RESET command
NRF_BL_CMD_RESET        equ     0x04
