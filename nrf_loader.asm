; -------------------------------------------------------------
; NRF Loader
; (c) 2019 Andrew Williams, Just Some Guy Productions.
;
; 16F1823
;
; CHANGELOG: Version 1 (underway)
; WHEN       WHO WHY
; -------------------------------------------------------------
; 2021-02-20 ADW Clean-up.. and more clean-up. Remove the unused code and minimize.
; 2021-02-18 ADW Space saving - replace inline with function calls
;                and remove uneeded "chip init" (leave that to the APP)
;   To Do: Routine to read DevId from EEPROM
;          Setup Radio for P2P from 1D<id:4> to 0xC0DEC1
;              (Need to tell radio 3 byte addr)
;          Attempt to send an "I am here with timeout/ACK"  - ACK determined enter BL proper, or APP
;
; 2019-09-02 ADW New file
;
; ---
; 16F1823 SPI to NRF module Connections
;
; PORT          PIN             CONFIGURATION
; ==================================
; VCC			1			VCC
; A5			2			SPI_CE
; A4			3			PWM3
; A3/MCLR		4			MCLR
; C5			5			SERIAL DEBUG
; C4			6			SERIAL_DEBUG
; C3			7			SPI_CS
; C2			8			SPI_SDO
; C1			9			SPI_SDI
; C0			10			SPI_SCK
; A2			11			SPI_IRQ
; PGC/A1		12			PGC/PWM2
; PGD/A0		13			PGD/PWM1
; VSS			14			GND
;
   ;list      p=16F1823     		; list directive to define processor
   #include <p16f1823.inc>			; processor specific variable definitions
   errorlevel  -302        		; suppress message 302 from list file


; Configuration settings
   __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_ON & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
   __CONFIG _CONFIG2, _WRT_BOOT & _PLLEN_ON & _STVREN_OFF & _BORV_LO & _LVP_ON

; NOTE: _WRT_BOOT - lock 0x000-0x1FFF from self-destruction (play it safe)
;

; ** NOTE - change MCLR_EN to MCLR_DIS after debugging completed.

; -------------------------
; User definable options
; -------------------------
#define USER_SPACE_START (0x0200)
#define USER_SPACE_SIZE  (0x07FF - USER_SPACE_START)
#define DISABLE_RESET
; ------------------------------------------------------------------------------
; ------------------------------------------------------------------------------
; System definitions - Users should not be messing below this line.
; (I know you will.. we always do)
;
#define CLOCKRATE 32000000
#define PAYLOAD_SIZE 32


; --------------------------
; Bank 0 Memory : 80 bytes
; --------------------------
 CBLOCK 0x20
		rxpayload:PAYLOAD_SIZE		; Values read from NRF Packet
		TXADDR:3
	; -- The follow block is a cached copy from the EEPROM
		RXADDR:3
		APP_CSUML
		APP_CSUMH
		end_bank0_vars:0
 ENDC
 if end_bank0_vars > 0x6F
	error "Bank 20 Variable Space overrun"
 endif

; --------------------------
; Bank 1 Memory : 32 bytes
; --------------------------
 CBLOCK 0xA0
		end_bank1_vars:0
 ENDC
 if end_bank1_vars > 0xBF
	error "Bank A0 Variable Space overrun"
 endif

; --------------------------
; Global Bank Memory : 16 bytes (all banks)
; --------------------------
 CBLOCK 0x70
		; Debugger uses 0x70 - put a dummy value here for now.
		dummyData
  		; Common memory page
		nrfTempByte		; local variable to hold value being written to the NRF register
		nrfByteCount	; count bytes in the tx or rx transaction
		nrfStatus		; local copy of the nrf STATUS register

		BL_TEMP
		BL_CMD
		BL_CALC_SUML
		BL_CALC_SUMH
		BL_SIZE_H
		BL_SIZE_L
		BL_WAIT_COUNT
        end_global_vars:0
 ENDC
 if end_global_vars > 0x7F
	error "Global Variable Space overrun"
 endif


; ------------------------------
; Linear Memory Map
; ------------------------------
 CBLOCK 0x2000
		; This is a linear mapping of all 112 bytes (80 + 32)
 ENDC
 if end_global_vars > 0x29AF
	error "Linear Map Variable Space overrun"
 endif

; ---------------------------------------------
; These defines will be BOARD specific
;
#define NRF_SPI_CS		LATC,3
#define NRF_CE 			LATA,5	;	LATC,4

; NRF Register Definitions
#include "nrf_header.inc"

; NRF Read/Write MACROS
#include "nrf_macros.inc"

; ==================
; -- Main Program --
; ==================
 	ORG 0x000
        clrf PCLATH
        goto BL_INIT

 	ORG 0x004
; -------Interrupt Routine --------------------
BL_IRQ_HANDLER
		goto	APP_IRQ_HANDLER	; Note: NO IRQ handler in the BootLoader

; ---------- Main Program Starts Here ----------------------------
	; BootLoader - Main Loop - Published address
	; Count down BL_WAIT_COUNT iterations - and then do an AUDIT check
	;    During the count down - poll the NRF network for incoming payloads
	; If Audit check passes  - Jump to application
	; If Audit fails - reset the board
BL_MAIN_LOOP
	BANKSEL PIR1 ; BANK 0
		; Count # times through the loop, then do an audit
		btfss	PIR1,TMR1IF
		goto	_check_nrf_network
		bcf		PIR1,TMR1IF
		decfsz	BL_WAIT_COUNT,F
		goto	_check_nrf_network
	; Timeout
		call	BL_DO_AUDIT
		btfss	WREG,0
		goto	BL_CMD_RESET 		; CSUM failure... reset and try again
		goto	APP_EVENT_HANDLER 	;; That's it..hand off the APPLICATION

_check_nrf_network
		nrfReadReg NRF_STATUS	; BSR=2
	; WARNING - this method is *NOT* the proper way. Race condition can result
	; in this bit not being set. Instead need to check if the pipe# (se arduino RF24.cpp)
		movwf	nrfStatus	; POLL  NRF for the Rx of payload
		btfss	nrfStatus,6
		goto	BL_MAIN_LOOP    ; Repeat Ad Absurdum.
	; NRF Packet has been received in the queue
_nrf_payload_pending
		nrfReadPayload rxpayload, PAYLOAD_SIZE	; Read the 32 byte payload
		nrfWriteRegL NRF_STATUS, 0x40	; Clear the RX interrupt flags
		; Was this...
		lsrf	nrfStatus,w
		andlw	0x07 			; w contains the pipe index...
		; short cut - only receive on Pipe 1 - ignore others
		decfsz	WREG,W
		goto	BL_MAIN_LOOP	; Re-enable RX mode

	; Appears to be a valid BL packet - reset the timer counter
		clrf	BL_WAIT_COUNT

	BANKSEL rxpayload
        ;; Based on the CMD byte .. take action (is the upper nibble 0x8n?
		movfw	rxpayload
		andlw	0xF0
		xorlw	0x80
		bnz		BL_MAIN_LOOP

		movfw	rxpayload
		clrf	PCLATH ; For the jump table below
		andlw	0x07
		addwf	PCL,F
jumptable
		goto	BL_CMD_START 	; 0x80 CMD_START
		goto	BL_CMD_WRITE 	; 0x81 CMD_WRITE
		goto	BL_CMD_COMMIT	; 0x82 CMD_COMMIT
		goto 	BL_CMD_AUDIT	; 0x83 CMD_AUDIT
		goto	BL_MAIN_LOOP	; 0x84 Re-enable RX mode
		goto	BL_MAIN_LOOP	; 0x85 Re-enable RX mode
		goto	BL_CMD_RESET	; 0x86 CMD_BOOT
		goto	BL_CMD_BIND		; 0x87 CMD_BCAST

	; --- END OF THE MAIN LOOP

; -----------------------------------------------------------------------
; -----------------------------------------------------------------------
;;-----
BL_CMD_BIND 	; Reply from server contains PIPE index to write to
		movfw	rxpayload+1
		movwf	TXADDR
		movfw	rxpayload+2
		movwf	TXADDR+1
		movfw	rxpayload+3
		movwf	TXADDR+2
		nrfWriteRegEx NRF_RX_ADDR_P0, TXADDR, 3
		nrfWriteRegEx NRF_TX_ADDR,    TXADDR, 3
		movlw   0x02
		movwf	nrfTempByte ; enable AA
		nrfWriteReg NRF_EN_AA, nrfTempByte  ; BSR=2
		goto	BL_MAIN_LOOP

;;-----------
BL_REPLY_ACK
		movlw	0x01
		goto	_BL_REPLY
BL_REPLY_NACK
		clrf	WREG
_BL_REPLY	; Send the ACK packet back to the server - byte[1] contains
			; 0x00 - success  not 0x00 FAIL CODE
			; Note: WREG holds error code
	BANKSEL rxpayload
		movwf	rxpayload+1
		movlw   0x03
		movwf	nrfTempByte ; enable AA
		nrfWriteReg NRF_EN_AA, nrfTempByte  ; BSR=2
		goto	sendPayload

;;----------
BL_CMD_AUDIT
		call	BL_DO_AUDIT
		goto	_BL_REPLY		; Return AUDIT results to the caller.

BL_DO_AUDIT
	BANKSEL BL_CALC_SUML
		movfw	APP_CSUML		; Seed the CSUM calculation
		movwf	BL_CALC_SUML
		movfw	APP_CSUMH
		movwf	BL_CALC_SUMH

	BANKSEL EEADRL
		movlw	LOW USER_SPACE_START
		movwf	EEADRL
		movlw	HIGH USER_SPACE_START
		movwf	EEADRH
		movlw	HIGH USER_SPACE_SIZE
		movwf	BL_SIZE_H
		movlw	LOW	USER_SPACE_SIZE
		movwf	BL_SIZE_L

_calcsum		; THE CSUM LOOP
		bcf		EECON1, CFGS
		bsf		EECON1, EEPGD
		bsf		EECON1, RD
		NOP
		NOP
		movfw	EEDATL
		addwf	BL_CALC_SUML,F
		btfsc	STATUS,C
		incf	BL_CALC_SUMH,F
		movfw	EEDATH
		addwf	BL_CALC_SUMH,F

		incf	EEADRL,F
		btfsc	STATUS,Z
		incf	EEADRH,F

		decfsz	BL_SIZE_L,F
		goto	_calcsum
		decfsz	BL_SIZE_H,F
		goto	_calcsum

		; Finished looping through CODE space
		movfw	BL_CALC_SUMH
;		andlw	0x3F
		btfss	STATUS,Z
		retlw	0x00
		movfw	BL_CALC_SUML
		btfss	STATUS,Z
		retlw	0x00
		retlw	0x01


;; End of BL_CMD_AUDIT
;; ----------

	; Resets the device if the received byte matches the reset character.
BL_CMD_RESET
#ifdef DISABLE_RESET
	goto	BL_MAIN_LOOP
#else
		reset
#endif

;; End of BL_CMD_RESET
;; ----------

	; Sets the write address, expected checksum of the next 32 words,
	; and erases the row at that address if the last byte of the command matches
	; the "erase" character.
BL_CMD_START
	clrf	BL_CALC_SUML	; Clear the calculated CHECKSUM
	movfw	rxpayload+NRF_BL_START_ERASE
	movwf	BL_CMD	; Store for analysis after the BANKSEL
	movfw	rxpayload+NRF_BL_START_ADRL
	movwf	BL_TEMP ; Store for use after the BANKSEL
	movfw	rxpayload+NRF_BL_START_ADRH

	banksel	EEADRL  ; BSR=3
	movwf	EEADRH
	movfw	BL_TEMP
	movwf	EEADRL
; do we need to erase?
	btfss	BL_CMD,0 	; If the lower bit is SET, erase first
	goto	BL_REPLY_ACK

; Erases the row of flash in PMADRH:PMADRL.
; BSR=3
_bootloader_erase		; From the Microchip 16F1823 data sheet
	bsf 	EECON1,EEPGD 	; Point to program memory
	bcf 	EECON1,CFGS 	; Not configuration space
	bsf 	EECON1,FREE 	; Specify an erase operation
	bsf 	EECON1,WREN 	; Enable writes
	call	UNLOCK_FLASH	; stalls until erase finishes
	bcf		EECON1,WREN		; clear write enable flag
	goto	BL_REPLY_ACK

;; End of BL_CMD_START
;; ----------


; Copies the incoming data into the EEADR latch , keeping a running CSUM for
; validation during the COMMIT command. (Note COMMIT will call into the
; internal _wloop entry point, for 1 of the remaining 2 words, then verify
; the CSUM for the last word. If it passes, it will commit the last word
; and the overall FLASH write)
; EECON
; BSR=0
BL_CMD_WRITE
; If the value in the temp register is 0 after all 32/64 bytes have been copied
; to the write latches, proceed with the write.
	movlw   LOW rxpayload+NRF_BL_WRITE_START ; Load FSR0 to point to the start of the payload
	movwf   FSR0L
	movlw   HIGH rxpayload
	movwf   FSR0H

	movlw	(1<<EEPGD) | (1<<LWLO)|(1<<WREN)	; write to latches only
	banksel	EECON1
		movwf	EECON1
; simultaneously compute the checksum of the 16/32 words and copy them to the
; write latches
		movlw	15				; (1823=16, 1825=32) number of words to write
		movwf	BL_TEMP			; used for loop count
		call	_wloop
		goto	BL_REPLY_ACK

_wloop_with_incr  ; comman outside of the apparent function. only increment if we are going back into the loop
		incf	EEADRL,f		; increment write address - note we might exit pointing to last byte written
_wloop
		moviw	FSR0++		; load lower byte
		addwf	BL_CALC_SUML,f	; add lower byte to checksum
		movwf	EEDATL			; copy to write latch
		moviw	FSR0++			; load upper byte
		addwf	BL_CALC_SUML,f		; add upper byte to checksum
		movwf	EEDATH			; copy to write latch
		call	UNLOCK_FLASH	; execute unlock sequence

								; may still have more words to go
		decfsz	BL_TEMP,f		; decrement loop count
								; if  0, we're done writing for this payload
		goto	_wloop_with_incr

		return

;; End of BL_CMD_WRITE
;; ----------

BL_CMD_COMMIT
; Write the final words (with CSUM validation), triggering the FLASH write.
; Note COMMIT will call into the internal _wloop entry point, for 3 of the remaining 4 words, then verify
; the CSUM for the last word. If it passes, it will commit the last word
; and the overall FLASH write)
		movlw   LOW rxpayload+NRF_BL_COMMIT_DATA ; Load FSR0 to point to the start of the payload
		movwf   FSR0L
		movlw   HIGH rxpayload
		movwf   FSR0H
		movfw   rxpayload+NRF_BL_CKSUM ; Load FSR0 to point to the start of the payload
		addwf	BL_CALC_SUML,f

	banksel	EECON1
		;	Write the LAST WORDs
		movlw	1			; Will be 2 for the 16f1825
		movwf	BL_TEMP
		;incf	EEADRL,f	; Advance by 1 byte because _wloop return with EEADRL pointing to last byte written
		call	_wloop_with_incr

		tstf	BL_CALC_SUML	; verify the checksum (should sum to 0)
		skpz
		goto	_commit_fail	; if there's a mismatch, abort the write

							; checksum is valid, write the data
		bcf		EECON1,LWLO
		call	UNLOCK_FLASH	; stalls until write finishes
		movlw	0x01
_commit_exit
		bcf		EECON1,WREN		; Disable Writes
		clrf	EECON1
		goto	_BL_REPLY
_commit_fail	; if jumping to here WREG !=0
		clrw	; set WREG to 0x00
		goto	_commit_exit


; verify the write: compare each byte in the buffer to its counterpart that
; was just written to flash.
; we do this backwards so we don't waste instructions resetting the pointers.
; (note: PMADRH:PMADRL is already pointing at the last written word, but FSR0
; is pointing to one byte past the end of the buffer)
#ifdef SUPPORT_VERIFY_NOT_DONE
	bsf	FSR1H,5			; set loop count to 32 (just need to set one bit because it's already 0)
_vloop	bsf	EECON1,RD		; read word from flash
	nop				; 2 required nops
	nop
	moviw	--FSR0			; get high byte of expected word
	subwf	PMDATH,w		; compare with high byte written to flash
	skpz
	retlw	BSTAT_VERIFY_FAILED
	moviw	--FSR0			; get low byte of expected word
	subwf	PMDATL,w		; compare with low byte written to flash
	skpz
	retlw	BSTAT_VERIFY_FAILED
	decf	PMADRL,f		; decrement read address
	decfsz	FSR1H,f			; decrement loop count
	goto	_vloop
#endif
	goto	BL_REPLY_ACK

;; End of BL_CMD_COMMIT
;; ----------

;;; Executes the flash unlock sequence, performing an erase or write.
;;; arguments:	EECON1 bits CFGS, LWLO, FREE and WREN set appropriately
;;;		BSR=3
;;; returns:	none
;;; clobbers:	W
UNLOCK_FLASH	; From the Microship 16F1823 data sheet
	movlw 	0x55 	  ; Start of required sequence to initiate erase
	movwf 	EECON2 	  ; Write 55h
	movlw 	0xAA
	movwf 	EECON2 	  ; Write AAh
	bsf		EECON1,WR ; Set WR bit to begin erase
	nop 		      ; Any instructions here are ignored as processor
			  		  ; halts to begin erase sequence
	nop  		  ; Processor will stop here and wait for erase complete.

	return


; --- BOOT LOADER START
;    Init the Chip (incl SPI)
;    Init the NRF Registers (via SPI)
;    Start Listening <-- should be attempting to determine if the NRF upgrade server is out there

BL_INIT
	; --- Initialize pins, oscillator, etc -----

	; BANK 0 Register INIT
	BANKSEL PORTA
		movlw	0x7D
		movwf	T1CON

	; BANK 2 Register INIT
	BANKSEL LATC
		clrf	LATA				; All LATA Pins off
		clrf	LATC
		bsf		NRF_SPI_CS			; Default to HIGH (active LOW)
		bcf		NRF_CE				; NRF CE - standby mode

	; BANK 1 Register INIT
	BANKSEL WDTCON
	;	bcf		WDTCON,0			; turn off WDT ; We are going to want a Watchdog here, to ensure recovery of failing devices
		movlw	0xF0				; setup internal oscillator
		movwf	OSCCON				; 32Mhz (8Mhz internal oscillator, 4xPLL )
		movfw	OSCSTAT
		btfss	OSCSTAT,PLLR			; Wait for OSCILLATOR to stabilize
		goto	$-2

		clrf 	TRISA				; outputs:PWM 0,1,4
		bsf 	TRISA,2				; SPI_IRQ - INPUT
		movlw 	0x02				; outputs:CS, SDO, NRF_CE
		movwf	TRISC				; inputs: SDI

		movlw	0x08				; Enable internal pull-ups, Interrupt on Falling edge
		movwf	OPTION_REG			; No Prescalar for the TMR0 clock

	; BANK 3 Register INIT
	BANKSEL ANSELA	; BANK3
		clrf	ANSELA				; Turn off A/D Converters
		clrf	ANSELC

	; BANK 4 Register INIT (SPI)
  	BANKSEL SSP1STAT
		bcf	SSP1STAT,SMP			; Sample in the middle
		bsf	SSP1STAT,CKE			; from online example with RFM12B

		movlw	0x01				; Go for divide by 8 = 4Mhz clock
		movwf	SSP1ADD

       	movlw   0x2A				; Mode 0,0 = CKE=1 CKP=0
       	movwf   SSP1CON1			; Select SPI clock and enable SSP

	; --- RAM Init section ------
		; copy EEPROM data to RAM Cache
		call 	BL_READ_ID 			; Call to read EEPROM data into page 0 variables

	; --- Global Parameters
		clrf	nrfStatus
		clrf	nrfByteCount
		clrf 	nrfTempByte

	; --- NRF Radio Register Init section ------
		; Set DYN_PD, 0x00 ; Note RESET sets to 0x00 .. so do nothing (save code space)
		;nrfWriteReg DYNPD, nrfTempByte

		; Set AUTO_ACK, 0x00 - no P2P setup yet
		nrfWriteReg NRF_EN_AA, nrfTempByte

		; Set Data Rate - 2MBPS & HIGH Power
		nrfReadReg NRF_RF_SETUP	; Results left in W
		iorlw	0x06
		movwf	nrfTempByte
		nrfWriteReg NRF_RF_SETUP, nrfTempByte

		; Set STATUS, RX_DR | TX_DS | MAX_RT
		movlw	0x70
		movwf	nrfTempByte
		nrfWriteReg NRF_STATUS, nrfTempByte

		; Set RF_CH, 82 (avoid WIFI interference (we hope))
		movlw	82
		movwf	nrfTempByte
		nrfWriteReg NRF_RF_CH, nrfTempByte

		nrfFlush NRF_FLUSH_TX ; Flush Tx

		; Set CRC Length -
		movlw	0x7E	; CRC0 |  EN_CRC | PWR_UP
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte

		; Setup for 3 byte address length
		movlw	0x01
		movwf	nrfTempByte
		nrfWriteReg NRF_SETUP_AW, nrfTempByte

		; Write the payload size for Pipe 0 & 1
		movlw PAYLOAD_SIZE
		movwf nrfTempByte
		nrfWriteReg NRF_RX_PW_P0, nrfTempByte
		nrfWriteReg NRF_RX_PW_P1, nrfTempByte

	BANKSEL RXADDR ; Address for default Server is C0DEC1
		movlw	0xC0
		movwf	TXADDR+2	;TxAddr = C0DEC1
		movlw	0xDE
		movwf	TXADDR+1
		movlw	0xC1
		movwf	TXADDR+0
		nrfWriteRegEx NRF_TX_ADDR,    TXADDR, 3
		nrfWriteRegEx NRF_RX_ADDR_P0, TXADDR, 3
		nrfWriteRegEx NRF_RX_ADDR_P1, RXADDR, 3

		; Setup max number of retries
		movlw	0x0F
		movwf	nrfTempByte
		nrfWriteReg	NRF_SETUP_RETR, nrfTempByte

	;--- End of NRF Register INIT ------

	;--- Broadcast an "I am Here" message to the Server ----
		; Announce to Server 0x87, <RXADD:3>
	BANKSEL rxpayload
		movlw	NRF_BL_CMD
		movwf	rxpayload
		movfw	RXADDR
		movwf	rxpayload+1
		movfw	RXADDR+1
		movwf	rxpayload+2
		movfw	RXADDR+2
		movwf	rxpayload+3
		; Enqueue the payload in the Tx Register
sendPayload
	BANKSEL LATA
		bcf		NRF_CE

	; Turn on Radio in Tx Mode
		nrfReadReg NRF_CONFIG	; Results left in W
		bcf		WREG,0			; SET PRIM_TX
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte
		call 	delay_130us
		nrfFlush NRF_FLUSH_RX ; Flush Rx

	; Enable RXADDR for Pipe0
		movlw 0x03
		movwf nrfTempByte
		nrfWriteReg NRF_EN_RXADDR, nrfTempByte

		nrfWritePayload 0xA0, rxpayload,PAYLOAD_SIZE

_triggerXmit
	;Clear any pending flags
		movlw	0x70
		movwf	nrfTempByte
		nrfWriteReg NRF_STATUS, nrfTempByte ; BSR=2

	; NRF requires CE for at least 10us
		bsf		NRF_CE	; Enable Transmitter

		movlw	27		;ToDo  get a scope on this...but seems to be working
		nop				; Need the extra Cycle Count
		decfsz	WREG,f	; 8 instructions to make a 10us spin (@ 32Mhz)
		goto 	$-2

	; Need a better way to tell that the payload was sent.. this sucks
		bcf		NRF_CE
	; w4ack - poll the NRF_STATUS unti the TX_DS flag is set
	; Note: for Broadcast this is set when the transmit completes, for P2P this is set
	; when we get an ACK from the far end. (If no far end this will never get set)

w4ack
		nrfReadReg NRF_STATUS
		btfsc	WREG,5  ; TX_DS flag
		goto	_ack_done
		btfss	WREG,4  ; MAX_RT flag
		goto	w4ack

_handle_msx_rt
		bsf		WREG,4 ; Write a 1 to clear the bit
		movwf	nrfTempByte
		nrfWriteReg NRF_STATUS, nrfTempByte
	; Toggle the CE to try and send again..
		goto	_triggerXmit


_ack_done
		movlw 0x20	; Clear the TX_DS bit
		movwf nrfTempByte
		nrfWriteReg NRF_STATUS, nrfTempByte

		; Disable RXADDR for Pipe0 - AutoAck reqt ?
		movlw 0x02
		movwf nrfTempByte
		nrfWriteReg NRF_EN_RXADDR, nrfTempByte

		; Move Radio into LISTEN mode
		nrfReadReg NRF_CONFIG	; Results left in W
		bsf	WREG,0
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte ; BSR=2
		nrfFlush NRF_FLUSH_RX ; Flush Rx

		;Start receiving
		bsf		NRF_CE 			; From standby into Listening Mode
		goto	BL_MAIN_LOOP

delay_130us
		movlw 	200	; 130us delay
		nop
		nop
		decfsz	WREG,F
		goto 	$-2
		return
; ===============================================
; End of Main BootLoader Loop
; ===============================================


; ------------------------------------------------------------
; NRF/SPI read/write routines
; ------------------------------------------------------------
_nrfReadReg ; W contains register to read
		call	_nrf_cmd_setup
	BANKSEL SSP1BUF
		clrf	SSP1BUF			; Write 0x00 in order to clock out another byte
		btfss 	SSP1STAT,BF		; Loop until BF (Buffer FULL) status
		goto	$-1
		call	_pir_spin
	BANKSEL SSP1BUF
		movf	SSP1BUF,W		; Copy results to output address (W)
_nrf_cmd_done
	BANKSEL LATC
		bsf 	NRF_SPI_CS 		; Set the CS
		return

; ----
_nrfWriteReg ; W- contains regId, FSR1 points to the value to write
		call	_nrf_cmd_setup
	BANKSEL SSP1BUF
		movfw	FSR1H			; Now clock out the parameter byte
		movwf	SSP1BUF
		call	_pir_spin
		goto	_nrf_cmd_done

; ----
_nrfFlush ; W has the command to send
		call	_nrf_cmd_setup
		goto	_nrf_cmd_done

; ----
_nrfWriteRegEx ; W contains the regId, FSR0 the srcAddr, and nrfByteCount the numBytes
		call	_nrf_cmd_setup
	; Time to clock out the bytes
	BANKSEL SSP1BUF
		moviw	FSR0++			; Copy byte from source address
		movwf	SSP1BUF			; Write byte to the SPI interface
		call	_pir_spin
		decfsz	nrfByteCount,f	; Decrement the number to send.. loop if not done
		goto 	$-5				; Jump back to BANKSEL SSP1BUF
		goto	_nrf_cmd_done
; ----
_pir_spin
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop until SPI IRQ completed
		goto 	$-1
		bcf		PIR1, SSP1IF	; Clear the interrupt
		return
; ----
_nrf_cmd_setup
	BANKSEL LATC
		bcf		NRF_SPI_CS		; Clear the CS
	BANKSEL SSP1BUF
		movwf 	SSP1BUF			; Hand the WriteREGID command to the SPI interface
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop until SPI IRQ completed
		goto 	$-1
		bcf		PIR1, SSP1IF	; Clear the interrupt flag
	return

; ------------------------------------------------------------
; EEPROM read (retrieve Device ID)
; ------------------------------------------------------------
BL_READ_ID
		clrf	FSR0H 				; BANK 0 variables
		movlw	LOW RXADDR
		movwf	FSR0L 				; Point to ID_HIGH
		movlw	0x05
		movwf	BL_TEMP
	BANKSEL EEADRL 					; Start reading at 0xF000
		movlw	0xF0
		movwf	EEADRH
		clrf	EEADRL

_ee_id_clock
		bcf		EECON1, CFGS  		; Deselect Config space
		bcf		EECON1, EEPGD 		; Point to DATA memory
		bsf		EECON1, RD    		; EE Read
		movf	EEDATL, W     		; W contains the value
		movwi	FSR0++       		; Write to target & continue
		incf	EEADRL,f
		decfsz	BL_TEMP,f
		goto	_ee_id_clock
		return

EndofData:
;	dt "(c) 2021 Andrew Williams"

;===================================================
; Application (loaded) space.
ORG	0x200
APP_IRQ_HANDLER
	retfie

ORG 0x204
APP_EVENT_HANDLER
    goto BL_MAIN_LOOP

ORG 0x300
DUMP_REG
	nrfReadReg NRF_CONFIG
	nop
	nrfReadReg NRF_EN_AA
	nop
	nrfReadReg NRF_EN_RXADDR
	nop
	nrfReadReg NRF_SETUP_AW
	nop
	nrfReadReg NRF_RF_CH
	nop
	nrfReadReg NRF_RF_SETUP
	nop
	nrfReadReg NRF_STATUS
	nop
	nrfReadReg NRF_RX_ADDR_P0
	nop
	nrfReadReg NRF_RX_ADDR_P1
	nop
	nrfReadReg NRF_TX_ADDR
	nop
	nrfReadReg NRF_FIFO_STATUS
	nop
	nop
	goto	DUMP_REG
; ----------------- end! -------
        END
