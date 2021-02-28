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
; A5			2			NC	
; A4			3			PWM3
; A3/MCLR		4			MCLR
; C5			5			NC	
; C4			6			SPI_CE
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
#define USER_SPACE_SIZE  ((0x07FF - USER_SPACE_START)/2)

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
	; -- The following block is a cached copy from the EEPROM
		RXADDR:3
		APP_CSUM_ODD
		APP_CSUM_EVEN
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
		nrfByteCount		; count bytes in the tx or rx transaction
		nrfStatus		; local copy of the nrf STATUS register	

		BL_TEMP
		BL_CMD
		BL_CALC_SUM
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
#define NRF_SPI_CS		LATC,3
#define NRF_CE 		LATA,5	;	LATC,4

#include "nrf_header.inc"

; --- NRF MACROS
; SYNCHRONOUS READ OF A REGISTER
; Results returned in W
nrfReadReg macro regId
		movlw	regId&0x1F
		call	_nrfReadReg
	endm

; SYNCHRONOUS WRTIE OF A REGISTER
; Optimized - store value to write in FSR1H (instead of using FSR1 indirectly
nrfWriteReg macro regId, value
		movfw	value
		movwf	FSR1H
		movlw	(regId&0x1F)|0x20
		call	_nrfWriteReg
	endm

;SYNCHRONOUS WRITE OF ACTIVATE command
nrfActivate macro value
		movfw	value
		movwf	FSR1H
		movlw	0x50
		call	_nrfWriteReg
	endm

; SYNCHRONOUS MULTI-BYTE WRITE OF A REGISTER
; Variable length - need to use the FSR0 with indirection.
nrfWriteRegEx macro regId, srcAddr, numBytes
		movlw	srcAddr			; Set FSR0 to the source address
		movwf	FSR0
		movlw	numBytes		; Set nrfByteCount to the # of bytes to transmit
		movwf	nrfByteCount
		movlw	(regId&0x1F)|0x20 
		call	_nrfWriteRegEx
	endm

; SYNCHRONOUS MULTI-BYTE WRITE OF A PACKET
nrfWritePayload macro regId, srcAddr, numBytes
		movlw	srcAddr			; Set FSR0 to the source address
		movwf	FSR0
		movlw	numBytes		; Set nrfByteCount to the # of bytes to transmit
		movwf	nrfByteCount
		movlw	regId
		call	_nrfWriteRegEx
	endm

; SYNCHRONOUS FLUSH COMMAND
nrfFlush macro cmd
		movlw	cmd
		call	_nrfFlush
	endm

; SYNCHRONOUS READ OF PAYLOAD	
nrfReadPayload macro addr, size
		movlw	addr			; Set FSR0 as the write target address
		movwf	FSR0
		movlw	size
		movwf	nrfByteCount	; Store number of bytes to read
	BANKSEL LATC
		bcf		NRF_SPI_CS		; Clear the CS
	BANKSEL SSP1BUF
		movlw	NRF_READ_PAYLOAD
		movwf	SSP1BUF			; Write the READ PAYLOAD command (0x61)
	; Wait for the result
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop until SPI IRQ completed
		goto 	$-1
		bcf		PIR1, SSP1IF	; Clear the SPI IRQ
	BANKSEL SSP1BUF
		movfw	SSP1BUF
		movwf	nrfStatus		; Store the returned NRF_STATUS
		clrf	SSP1BUF			; Send 0x00 in order to clock in the payloadbyte
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop untilSPI IRQ completed
		goto 	$-1
		bcf		PIR1, SSP1IF	; Clear the interrupt
	BANKSEL SSP1BUF
		movfw 	SSP1BUF			; Read the incoming SPI byte
		movwi	FSR0++			; Write the byte to the target address (increment for next read)
		decfsz	nrfByteCount,f	; Decrement the count and loop if not done
		goto 	$-9				; Jump back to clrf SSP1BUF
	BANKSEL LATC
		bsf		NRF_SPI_CS		; Set the CS
	endm

; ==================
; -- Main Program --
; ==================
	ORG 0x000
        clrf PCLATH
        goto BL_START

	ORG 0x004
; -------Interrupt Routine --------------------
BL_IRQ_HANDLER 
		goto	APP_IRQ_HANDLER

; ---------- Main Program Starts Here ----------------------------
	; BootLoader - Main Loop - Published address
BL_MAIN_LOOP
	BANKSEL PIR1
		; Count # times through the loop, then do an audit
		btfss	PIR1,TMR1IF
		goto	_check_nrf_network
		bcf		PIR1,TMR1IF
		decfsz	BL_WAIT_COUNT,F
		goto	_check_nrf_network
	; Timeout
		call	BL_DO_AUDIT
		btfss	WREG,Z
		goto	BL_CMD_RESET 		; CSUM failure... reset and try again
		goto	APP_EVENT_HANDLER 	;; That's it..hand off the APPLICATION

_check_nrf_network
		nrfReadReg NRF_STATUS
	; WARNING - this method is *NOT* the proper way. Race condition can result
	; in this bit not being set. Instead need to check if the pipe# (see arduino RF24.cpp)
		movwf	nrfStatus	; POLL  NRF for the Rx of payload
		btfss	nrfStatus,6
		goto	BL_MAIN_LOOP    ; Repeat Ad Absurdum.

	; NRF Packet has been received in the queue
nrf_payload_pending
		bcf	NRF_CE				; Payload received - turn off the receiver ??
		nrfReadPayload rxpayload, PAYLOAD_SIZE	; Read the 32 byte payload
		movlw	0x70
		movwf	nrfTempByte
		nrfWriteReg NRF_STATUS, nrfTempByte	; Clear the RX interrupt flags
		; Was this...
		lsrf	nrfStatus,w
		andlw	0x07 			; w contains the pipe index...
		; short cut - only receive on Pipe 1
		decfsz	WREG,W
		goto	BL_CONTINUE		; Re-enable RX mode

	; Appears to be a valid BL packet - reset the timer counter
		clrf	BL_WAIT_COUNT

	BANKSEL rxpayload
        ;; Based on the CMD byte .. take action (is the upper nibble 0x8n?
		movfw	rxpayload
		andlw	0xF0
		xorlw	0x80
		bnz		BL_CONTINUE

		movfw	rxpayload
		andlw	0x07
		addwf	PCL,F
jumptable
		goto	BL_CMD_START 	; 0x80 CMD_START
		goto	BL_CMD_WRITE 	; 0x81 CMD_WRITE
		goto	BL_CMD_COMMIT	; 0x82 CMD_COMMIT
		goto 	BL_CMD_AUDIT	; 0x83 CMD_AUDIT
		goto	BL_CONTINUE		; 0x84 Re-enable RX mode
		goto	BL_CONTINUE		; 0x85 Re-enable RX mode
		goto	BL_CMD_RESET	; 0x86 CMD_BOOT
		goto	BL_BIND			; 0x87 CMD_BCAST

	; --- END OF THE MAIN LOOP

BL_CONTINUE
		bsf		NRF_CE				; Turn on the Receiver
		goto	BL_MAIN_LOOP
; -----------------------------------------------------------------------
; -----------------------------------------------------------------------
;;-----
BL_BIND 	; Reply from server contains PIPE index to write to
		movfw	rxpayload+1
		movwf	TXADDR
		movfw	rxpayload+2
		movwf	TXADDR+1
		movfw	rxpayload+3
		movwf	TXADDR+2
		nrfWriteRegEx NRF_TX_ADDR,    TXADDR, 3
		nrfWriteRegEx NRF_RX_ADDR_P0, TXADDR, 3
		goto	BL_CONTINUE

;;-----------
BL_REPLY_NACK
		movlw	0x01
		goto	_BL_REPLY
BL_REPLY_ACK
		clrf	WREG
_BL_REPLY	; Send the ACK packet back to the server - byte[1] contains
			; 0x00 - success  not 0x00 FAIL CODE
			; Note: WREG holds error code
	BANKSEL rxpayload
		movwf	rxpayload+1
		nrfWritePayload 0xA0, rxpayload,32
		goto	_sendpayload

;;----------
BL_CMD_AUDIT
		call	BL_DO_AUDIT
		goto	_BL_REPLY		; Return AUDIT results to the caller.

BL_DO_AUDIT
		movlw	LOW USER_SPACE_START
		movwf	FSR0L
		movlw	HIGH USER_SPACE_START
		movwf	FSR0H
		movlw	HIGH USER_SPACE_SIZE
		movwf	BL_SIZE_H
		movlw	LOW	USER_SPACE_SIZE
		movwf	BL_SIZE_L
		movfw	APP_CSUM_EVEN	; Seed the CSUM calculation
		call	_calcsum
		btfss	STATUS,Z  	; If Zero continue.. else error
		retlw	0x02		; Error in the EVEN block
		; Check the ODD space
		movlw	LOW USER_SPACE_START+1
		movwf	FSR0L
		movlw	HIGH USER_SPACE_START
		movwf	FSR0H
		movlw	HIGH USER_SPACE_SIZE
		movwf	BL_SIZE_H
		movlw	LOW	USER_SPACE_SIZE
		movwf	BL_SIZE_L
		movfw	APP_CSUM_ODD		; Seed the CSUM calculation
		call	_calcsum
		btfss	STATUS,Z
		retlw	0x01		; Error in the ODD block
		retlw	0x00		; Success!!

_calcsum		; THE CSUM LOOP
		movwf	BL_CALC_SUM	; Re-use the CALC SUM variable
		moviw	FSR0++
		addwf	BL_CALC_SUM,F
		incf	FSR0,F
		decfsz	BL_SIZE_L,F
		goto	$-4
		decfsz	BL_SIZE_H,F
		goto	$-6
		movfw	BL_CALC_SUM	; Return with the CSUM in WREG (should be zero on success)
		return


;; End of BL_CMD_AUDIT
;; ----------
	; Resets the device if the received byte matches the reset character.
BL_CMD_RESET
	reset

;; End of BL_CMD_RESET
;; ----------

	; Sets the write address, expected checksum of the next 32 words,
	; and erases the row at that address if the last byte of the command matches
	; the "erase" character.
BL_CMD_START
	clrf	BL_CALC_SUM	; Clear the calculated CHECKSUM
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
    bcf		EECON1,WREN	; clear write enable flag
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
; If the value in the temp register is 0 after all 64 bytes have been copied
; to the write latches, proceed with the write.
        movlw   LOW rxpayload+NRF_BL_WRITE_START ; Load FSR0 to point to the start of the payload
        movwf   FSR0L
        movlw   HIGH rxpayload
        movwf   FSR0H

	movlw	(1<<LWLO)|(1<<WREN)	; write to latches only
	banksel	EECON1
	movwf	EECON1
; simultaneously compute the checksum of the 16/32 words and copy them to the
; write latches
	movlw	15			; (1823=16, 1825=32) number of words to write 
	movwf	BL_TEMP	; used for loop count
_wloop	moviw	FSR0++			; load lower byte
	addwf	BL_CALC_SUM,f		; add lower byte to checksum
	movwf	EEDATL			; copy to write latch
	moviw	FSR0++			; load upper byte
	addwf	BL_CALC_SUM,f		; add upper byte to checksum
	movwf	EEDATH			; copy to write latch
; still have more words to go

	movf	EEADRL,W 		; Check if lower bits of address are '000'
	andlw	0x07 
	btfss 	STATUS,Z 		; Exit if last of eight words,
	goto	BL_REPLY_ACK

	call	UNLOCK_FLASH		; execute unlock sequence
	incf	EEADRL,f		; increment write address
	decfsz	BL_TEMP,f		; decrement loop count
					; if  0, we're done writing for this payload
	goto	_wloop
	goto	BL_REPLY_ACK

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
	addwf	BL_CALC_SUM,f

; Should not be necessary
;	movlw	(1<<LWLO)|(1<<WREN)	; write to latches only
;	banksel	EECON1
;	movwf	EECON1

;	Write the first WORD
	movlw	1			; Will be 2 for the 16f1825
	movwf	BL_TEMP
	call	_wloop

	clrf	EECON1
	tstf	BL_CALC_SUM		; verify the checksum (should sum to 0)
	skpnz
	goto	BL_REPLY_NACK	; if there's a mismatch, abort the write
; checksum is valid, write the data
	bsf	EECON1,WREN
	call	UNLOCK_FLASH		; stalls until write finishes
	clrf	EECON1			; clear write enable
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
	bsf	EECON1,WR ; Set WR bit to begin erase
	nop 		  ; Any instructions here are ignored as processor
			  ; halts to begin erase sequence
	nop  		  ; Processor will stop here and wait for erase complete.

	return


; --- BOOT LOADER START
;    Init the Chip (incl SPI)
;    Init the NRF
;    Start Listening <-- should be attempting to determine if the NRF upgrade server is out there

BL_START    
; Initialize pins, oscillator, etc
	BANKSEL PORTA	; BANK0
		clrf	PORTA
		clrf	PORTC
		movlw	0x7D
		movwf	T1CON

	BANKSEL LATC 	; BANK 2
		clrf	LATA				; All LATA Pins off
		bsf		NRF_SPI_CS			; Default to HIGH (active LOW)
		bcf		NRF_CE				; NRF CE - standby mode

	BANKSEL WDTCON	; BANK1
	;	bcf		WDTCON,0			; turn off WDT ; We are going to want a Watchdog here, to ensure recovery of failing devices
		movlw	0xF0				; setup internal oscillator 
		movwf	OSCCON				; 32Mhz (8Mhz internal oscillator, 4xPLL )

		clrf 	TRISA				; outputs:PWM 0,1,4
		bsf 	TRISA,2				; SPI_IRQ - INPUT
		movlw 	0x02				; outputs:CS, SDO, NRF_CE
		movwf	TRISC				; inputs: SDI

		movlw	0x08				; Enable internal pull-ups, Interrupt on Falling edge
		movwf	OPTION_REG			; No Prescalar for the TMR0 clock

	BANKSEL ANSELA	; BANK3
		clrf	ANSELA				; Turn off A/D Converters
		clrf	ANSELC
	
  	BANKSEL SSP1STAT ; BANK4  --- SPI INIT
		bcf	SSP1STAT,SMP	; Sample in the middle
		bsf	SSP1STAT,CKE	; from online example with RFM12B  

		movlw	0x01		; Go for divide by 8 = 4Mhz clock
		movwf	SSP1ADD		;

		movlw   0x2A		; Mode 0,0 = CKE=1 CKP=0 
		movwf   SSP1CON1	; Select SPI clock and enable SSP

	; Memory Initialization
		call 	BL_READ_ID 	; Call to read EEPROM data into page 0 variables

	; Global Parameters
		clrf	nrfStatus
		clrf	nrfByteCount                

	;nrfInit
		; Set retries - SETUP_RETR, 0x00
		clrf	nrfTempByte
		nrfWriteReg NRF_SETUP_RETR, nrfTempByte ; SETUP_RETR
		; Set DYN_PD, 0x00
		nrfWriteReg DYNPD, nrfTempByte
		; Set AUTO_ACK, 0x02
		bsf	nrfTempByte,1
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
	
		nrfFlush NRF_FLUSH_RX ; Flush Rx
		nrfFlush NRF_FLUSH_TX ; Flush Tx

		; Set CRC Length - 
		nrfReadReg NRF_CONFIG	; Results left in W
		iorlw	0x7C	; MASK_CRC0 |  EN_CRC
		andlw	0x7F
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte

		; Setup for 3 byte address length
		movlw	0x01
		movwf	nrfTempByte
		nrfWriteReg NRF_SETUP_AW, nrfTempByte

	BANKSEL RXADDR ; Use rxpayload as buffer for Rx/Tx address config data
		movlw	0xC0
		movwf	TXADDR+2	;TxAddr = C0DEC1
		movlw	0xDE
		movwf	TXADDR+1
		movlw	0xC1
		movwf	TXADDR+0
		nrfWriteRegEx NRF_TX_ADDR,    TXADDR, 3
		nrfWriteRegEx NRF_RX_ADDR_P0, TXADDR, 3
		nrfWriteRegEx NRF_RX_ADDR_P1, RXADDR, 3

		; Write the payload size for Pipe 0 & 1
		movlw PAYLOAD_SIZE
		movwf nrfTempByte
		nrfWriteReg NRF_RX_PW_P0, nrfTempByte
		nrfWriteReg NRF_RX_PW_P1, nrfTempByte

		; Enable RX on Pipe 0 & Pipe 1
		movlw 0x03
		movwf nrfTempByte
		nrfWriteReg NRF_EN_RXADDR, nrfTempByte

		; Setup max number of retries
		movlw	0x0F
		movwf	nrfTempByte
		nrfWriteReg	NRF_SETUP_RETR, nrfTempByte

; - end of NRF INIT
; - Time to start transmitting "I am Here" message

		; Turn on Radio in Tx Mode
	;	nrfReadReg NRF_CONFIG	; Results left in W
	;	iorlw 0x02				; SET PWR_UP & PRIM_TX
	;	movwf	nrfTempByte
	;	nrfWriteReg NRF_CONFIG, nrfTempByte

; Announce to Server
	BANKSEL rxpayload
		movlw	NRF_BL_CMD
		movwf	rxpayload
		movfw	RXADDR
		movwf	rxpayload+1
		movfw	RXADDR+1
		movwf	rxpayload+2
		nrfWritePayload 0xB0, rxpayload,32

_sendpayload
		; Turn on Radio in Tx Mode
		nrfReadReg NRF_CONFIG	; Results left in W
		bsf		WREG,1			; SET PWR_UP & PRIM_TX
		bcf		WREG,0
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte

	;Clear any pending flags
		movlw	0x70
		movwf	nrfTempByte
		nrfWriteReg NRF_STATUS, nrfTempByte

	BANKSEL LATA
		; Enable CS for at least 10us
		bsf		NRF_CE	; Enable Transmitter
; Need a better way to tell that the payload was sent.. this sucks
		movlw	8
		decfsz	WREG,f	; 8 instructions to make a 10us spin (@ 32Mhz)
		goto 	$-1

w4ack		nrfReadReg NRF_STATUS
		btfss	WREG,5  ; TX_DS flag
		goto	w4ack

		bcf		NRF_CE  ; Transmitter: Enter Standby mode

		; Turn Radio into LISTEN mode
		nrfReadReg NRF_CONFIG	; Results left in W
		iorlw 0x03				; SET PWR_UP
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte
		;Start receiving
		bsf		NRF_CE			; Start Listening Mode
		goto	BL_MAIN_LOOP



; ------------------------------------------------------------
; NRF/SPI read/write routines 
; ------------------------------------------------------------
_nrfReadReg ; W contains register to read
	BANKSEL LATC
		bcf		NRF_SPI_CS		; Clear the CS
	BANKSEL SSP1BUF
		movwf 	SSP1BUF			; Hand the ReadREGID to the SPI interface
	BANKSEL PIR1
		btfss	PIR1, SSP1IF
		goto 	$-1 			; Loop until SPI IRQ completed
		bcf		PIR1, SSP1IF	; Clear the interrupt flag						
	BANKSEL SSP1BUF
		clrf	SSP1BUF			; Write 0x00 in order to clock out another byte
		btfss 	SSP1STAT,BF		; Loop until BF (Buffer FULL) status
		goto	$-1
	BANKSEL PIR1
		btfss	PIR1, SSP1IF 	; Loop until Interrupt from SPI module
		goto 	$-1
		bcf		PIR1, SSP1IF 	; Clear the interrupt flag	
	BANKSEL SSP1BUF
		movf	SSP1BUF,W		; Copy results to output address (W)
	BANKSEL LATC
		bsf 	NRF_SPI_CS 		; Set the CS
		return

_nrfWriteReg ; W- contains regId, FSR1 points to the value to write
	BANKSEL LATC
		bcf		NRF_SPI_CS		; Clear the CS
	BANKSEL SSP1BUF
		movwf 	SSP1BUF			; Hand the WriteREGID command to the SPI interface
	BANKSEL PIR1
		btfss	PIR1, SSP1IF
		goto 	$-1 			; Loop until SPI IRQ completed
		bcf		PIR1, SSP1IF	; Clear the interrupt flag
	BANKSEL SSP1BUF
		movfw	FSR1H			; Now clock out the parameter byte
		movwf	SSP1BUF
	BANKSEL PIR1
		btfss	PIR1, SSP1IF
		goto 	$-1				; Loop until SPI IRQ completed
		bcf		PIR1, SSP1IF	; Clear the interrupt flag
	BANKSEL LATC
		bsf 	NRF_SPI_CS		; Set the CS	
		return

_nrfFlush ; W has the command to send
		bcf		NRF_SPI_CS		; Clear the CS
	BANKSEL SSP1BUF
		movwf 	SSP1BUF			; Write the command to the SPI interface
	BANKSEL PIR1
		btfss	PIR1, SSP1IF
		goto 	$-1 			; Loop until SPI IRQ completed
		bcf		PIR1, SSP1IF	; Clear the interrupt
		bsf 	NRF_SPI_CS		; Set the CS
		return

_nrfWriteRegEx ; W contains the regId, FSR0 the srcAddr, and nrfByteCount the numBytes
	BANKSEL LATC
		bcf		NRF_SPI_CS			; Clear the CS
	BANKSEL SSP1BUF
		movwf 	SSP1BUF			; Hand the WriteREGID command to the SPI interface
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop until SPI IRQ completed
		goto 	$-1
		bcf		PIR1, SSP1IF	; Clear the interrupt flag
	; Time to clock out the bytes
	BANKSEL SSP1BUF
		moviw	FSR0++			; Copy byte from source address
		movwf	SSP1BUF			; Write byte to the SPI interface
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop until SPI IRQ completed
		goto 	$-1
		bcf		PIR1, SSP1IF	; Clear the interrupt
		decfsz	nrfByteCount,f	; Decrement the number to send.. loop if not done
		goto 	$-9
	BANKSEL LATC
		bsf 	NRF_SPI_CS		; Set the CS
		return

; ------------------------------------------------------------
; EEPROM read (retrieve Device ID)
; ------------------------------------------------------------
BL_READ_ID
		clrf	FSR0H ; BANK 0 variables
		movlw	LOW RXADDR
		movwf	FSR0L ; Point to ID_HIGH
		movlw	0x04
		movwf	BL_TEMP
	BANKSEL EEADRL ; Start reading at 0xF000
		movlw	0xF0
		movwf	EEADRH
		clrf	EEADRL ; 0xF000
_ee_id_clock 	bcf	EECON1, CFGS  ; Deslect Config space
		bcf	EECON1, EEPGD ; Point to DATA memory
		bsf	EECON1, RD    ; EE Read
		movf	EEDATL, W     ; W contains the value

		movwi	FSR0++       ; Write to target & continue
		incf	EEADRL,f
		decfsz	BL_TEMP,f
		goto	_ee_id_clock
		return


EndofData:
	dt "(c) 2021 Andrew Williams"

;===================================================
; Application (loaded) space.
	ORG	0x200
APP_IRQ_HANDLER
	retfie

	ORG 	0x204
APP_EVENT_HANDLER
    goto BL_MAIN_LOOP


; ----------------- end! -------	
        END
