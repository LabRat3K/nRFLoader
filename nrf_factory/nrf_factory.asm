; -------------------------------------------------------------
; NRF Receiver
; (c) 2013-2022 Andrew Williams, Just Some Guy Productions.
;
; 16F1823
;
; CHANGELOG: Version 1 (underway)
; WHEN       WHO WHY
; -------------------------------------------------------------
; 2022-04-20 ADW Restructure to use the Bootloader
; 2013-03-23 ADW New file - based on DMX2IR
;
; ---
; 16F1823 Chip Connections
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
#ifdef __16F1823
       	;list      p=16f1823     		; list directive to define processor
        #include <p16f1823.inc>			; processor specific variable definitions
        errorlevel  -302        		; suppress message 302 from list file
	#define PICID 16F1823
#endif
#ifdef __16F1825
       	;list      p=16f1825     		; list directive to define processor
        #include <p16f1825.inc>			; processor specific variable definitions
        errorlevel  -302        		; suppress message 302 from list file
	#define PICID 16F1825
#endif

#ifndef PICID
  ERROR "Undefined processor  - supported processors are: 16F1823 or 16F1825"
#endif

  list r=dec
#define USE_BOOTLOADER_CODE 

#ifdef __16F1823
; Configuration settings
;   __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF      
    __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON  & _MCLRE_OFF& _CP_OFF & _CPD_OFF & _BOREN_ON  & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF      
    __CONFIG _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_OFF & _BORV_LO & _LVP_ON
#endif

#ifdef __16F1825
; Configuration settings
;   __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
    __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON  & _MCLRE_OFF& _CP_OFF & _CPD_OFF & _BOREN_ON  & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
    __CONFIG _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_OFF & _BORV_LO & _LVP_OFF
#endif

; ** NOTE - change MCLR_EN to MCLR_DIS after debugging completed.
; -------------------------
; User definable options
; -------------------------
;
;#define DEBUG
; Define the default start address
#define START_ADDRESS D'022'
; Invert the incoming DMX signal
;#define INVERT_OUTPUT

#define START_VALUE (0x00) ; Power On default "lighting value" for the PWM
; NOTE: If set to 1 the device has a tendency to hang on startup, and we end up having
; to cycle the power a second (or more) time. Leave this as 0x00 for now!!

; ------------------------------------------------------------------------------
; ------------------------------------------------------------------------------
; System definitions - User should not be messing below this line. 
; (I know you will.. we always do)
;
#define CLOCKRATE 32000000

#define PAYLOAD_SIZE 32
#define NUM_DISPLAY_CHAN 3

; Must be placed before CBLOCK to pickuped definition of start addresses
;


; *******************************************************
; Exported Shared Code from the Bootloader
;
; See export_list for the list of addresses to export
; Creates file import_list.inc which should be used by
; the application code.
; If debugging in MPLAB IDE comment this out, and cut/paste the values 
; manually from import_list.inc, to here. This will allow the necessary
; build with symbols for debugging.

#include "import_list.inc"

; *******************************************************
    
 CBLOCK BL_bank0_vars
	; Take a look at Symbols exported from the BL
	; First 2 bytes are pulled from EEPROM 
		APP_BAUD_RATE
		RESERVED
		chan1val:NUM_DISPLAY_CHAN	; Values actively being displayed
		chan1new:NUM_DISPLAY_CHAN	; Values to be used in next duty cycle
	app_end_bank0_vars:0
 ENDC
 if app_end_bank0_vars > 0x6F
	error "Bank 0 Variable Space overrun"
 endif


 CBLOCK BL_bank1_vars
		PACKET_ID 	; which packet contains dimming values for this device
		OFFSET		; offset within the packet
		NUM_CH1		; Number of channels to be read from packet
		NUM_CH2		; Secondary count *if* our dimming array crossed packet boundary
		SPLIT_PAYLOAD 	; FLAG indicating payload crosses boundary 
		TEMP_CHL	; Cached copies of CHL, CHH so that we don't have to 
		TEMP_CHH	; waste time BANKSELECTING
	app_end_bank1_vars:0
 ENDC
 if app_end_bank1_vars > 0xBF
	error "Bank 1 Variable Space overrun"
 endif

 CBLOCK BL_bank_global_vars
  		; Common memory page
		pwmpinsA
		IntCount 	; Master counter - compare to this to determine when to
				; turn off the PWM pins.
        app_end_global_vars:0	
 ENDC
 if app_end_global_vars > 0x7F
	error "Global Variable Space overrun"
 endif	

; ---------------------------------------------
; Device address - valid range 0 - 20 (decimal)
#define DEVICEADDR (START_ADDRESS-0x01)

; --------------------------------------
#define chan2val (chan1val+1)
#define chan3val (chan1val+2)

#define chan2new (chan1new+1)
#define chan3new (chan1new+2)
; ---------------------------------------

#define NRF_SPI_CS	LATC,3
#define NRF_CE 		LATA,5 

#include "../nrf_header.inc"

#ifdef INVERT_OUTPUT
pwmSET macro varname,pinid
	bcf	varname,pinid
	endm

#else
pwmSET macro varname, pinid
	bsf	varname,pinid
	endm
#endif

; --- NRF MACROS
#include "../nrf_macros.inc"

; -- Main Program --
 	ORG APP_EVENT_HANDLER
		clrf PCLATH
		goto APP_START

 	ORG APP_IRQ_HANDLER	; Interrupt handler - expected  interrupts - SSP1IF
app_irqHandler 
	; ------- Actual Interrupt Routines Here --------------------
	BANKSEL TMR0
		clrf	TMR0	; TMR0 will roll over every 40.0 uS
        	     		; which allows for 97.65 Hz PWM frequency
				; Nice thing here is that even if we're receiving
				; full-on DMX (44, 512-byte frames per second)
				; the PWM routine can easily keep up.  So some 
				; exciting effects should be possible.

DecIntCount                                      
        decfsz  IntCount,F	; Decrement IntCount register.  If it's zero, reset 
        goto    chan1check	; the PWM routine.  Otherwise, check each PWM value

PWMReset
	BANKSEL LATA     
#ifdef INVERT_OUTPUT
		movfw	LATA
		iorlw	0x13
		movwf	LATA
		movwf	pwmpinsA

#else        
        clrf    pwmpinsA	; We've been through 256  cycles of PWM, so it's time
				; pwmpins is a byte  variable. Bytes are cleared
				; here (turning off red,  green and blue drive pins)
        clrf	LATA		; and the entire byte is  pushed out to LATA.
#endif
	BANKSEL chan1val
        movlw   D'255'		; Reset the counter to allow 255 values per channel 
        movwf   IntCount	; Also, since we're resetting, it's time to transfer
        movf    chan1new,w	; the *new variables (which came from the DMX routine)
        movwf   chan1val	; to the working PWM variables.
        movf    chan2new,w	; If the transfer takes place at any time *other* than
        movwf   chan2val	; during a PWM reset, the LEDs flicker unflatteringly
        movf    chan3new,w	; We're guaranteed that new DMX data is made available
        movwf   chan3val	; to the PWM routine as soon as possible
        goto    irqExit		; Exit ISR

; Here we compare each of the duty cycles to the IntCount.  If the values
; match, the corresponding pin is driven high.  It's a bit counter-intuitive
; but it works.  Note that if a value of 255 is received, it won't work.  So 
; the DMX routine limits PWM values to [0 254].  Which is good enough.

chan1check
   BANKSEL chan1val
        movf    chan1val,w
        subwf   IntCount,w
        btfsc   STATUS,Z                ; Are they equal?
        pwmSET  pwmpinsA,0              ; Note that the *val variables are used only within
        				; the PWM routine.  The actual DMX data is stored       

chan2check                              ; in the *new variables, and transferred to these
        movf    chan2val,w              ; working variables only when the PWM rolls over
        subwf   IntCount,w                               
        btfsc   STATUS,Z                ; If the values are equal, set the bit (which turns         
        pwmSET  pwmpinsA,1              ; on the corresponding drive pin)
#ifndef DEBUG
chan3check
        movf    chan3val,w
        subwf   IntCount,w
        btfsc   STATUS,Z     
        pwmSET  pwmpinsA,4 
#endif   

updatePORTS
	BANKSEL LATA
		movfw	LATA		; READ
#ifdef DEBUG
		andlw	0xE8		; A4 | A1 | A0 (clear these bits)
#else
		andlw   0xF8		; Leave A4 for debug notification
#endif
		iorwf	pwmpinsA,W	; MODIFY (OR them back in as required)	
		movwf	LATA		; WRITE

irqExit
		bcf	INTCON,T0IF
        retfie				; return to gathering DMX data


; ----------------------------------------------------------------
; ---------- Main Program Starts Here ----------------------------
; ----------------------------------------------------------------q
APP_START
		call    INIT_CHIP        ; Initialize pins, oscillator, etc
		call	INIT_MEM	; Init memory and default variables
		call	INIT_NRF	; Changes from BL nrf setup
		call	INIT_PWM	
		call	GET_INDEX
		;Start receiving
		bsf		NRF_CE

main_loop
		call	BL_NRF_HDLR ; Returns 1 if there is a non-admin payload
							; 00 - nothing, 01 - not PIPE 1, 02 - Unknown payload, 04- Handled by BL
		addlw	0x00		; get the Z bit set
		btfsc	STATUS,Z
		goto	main_loop	; Zero - no payload

		lsrf	nrfStatus,w	; Read PIPE index from nrfStatus
		andlw	0x07
		btfsc	STATUS,Z
		goto	_PIPE_0   		; PIPE 0
		decfsz	WREG,W
		goto	_check_pipe_2	; Not PIPE 1 - maybe 2
		goto	_PIPE_1			; PIPE 1
_check_pipe_2
		decfsz 	WREG,W
		goto	main_loop		; Unknown PIPE .. reject the request
		goto	_PIPE_2			; PIPE 2 (aka Dimming Data)

_PIPE_0
	; Valid PIPE 0 messages are..  BIND
		call	_test_for_bind 
; ** TO DO ** add a timeout method .. to reset and return to normal mode
		decfsz	WREG,W	; If return was a 1, this is NOT a BIND
		call	_handle_bind_request
		goto	main_loop

_test_for_bind
	BANKSEL rxpayload
		movlw	0x87	; Is Byte[0] 0x87?
		xorwf	rxpayload,W
		btfss	STATUS,Z
		retlw	0x01	; NOT a BIND
		retlw	0x00

_handle_bind_request
		; Disable Listening on all Pipes and re-open listening Pipes
		bcf		nrfTempRX,2
		nrfWriteRegL	NRF_EN_RXADDR, 0x00
		nrfWriteRegEx 	NRF_RX_ADDR_P0, rxpayload,3
		nrfWriteRegEx 	NRF_RX_ADDR_P1, RXADDR,3
		nrfWriteReg		NRF_EN_RXADDR, nrfTempRX
		; Code now matched BL .. let it handle the BIND
		; From here on out the BL code should succesfully handle 
		; BL requests from the device
		goto	BL_CMD_BIND	
		
	
_PIPE_1
	; Valid PIPE 1 messages are.. BEACON and BIND,set BAUD, set CHAN, set START ADDR
		call	_test_for_bind 
; ** TO DO ** add a timeout method .. to reset and return to normal mode
		decfsz	WREG,W	; If return was a 1, this is NOT a BIND
		call	_handle_bind_request
		movlw	0x85
		xorwf	rxpayload,W
		btfsc	STATUS,Z
		goto	BL_CMD_QRY
		; Check Payload Command Byte
		movlw	0x01
		xorwf	rxpayload,W
		btfsc	STATUS,Z
		goto	CMD_NEWCHAN
		
		; Check Payload Command Byte
		movlw	0x03
		xorwf	rxpayload,W
		btfsc	STATUS,Z
		goto	CMD_NEWID

		; To Do -- add BAUD, CHAN, and START ADDR handlers here
	;   BEACON - send the I am Here response
		goto main_loop	

_PIPE_2		; Packet on the dimming channel
	; Prepare to access content 
		MOVLW	HIGH rxpayload
		MOVWF	FSR0H
		MOVLW	LOW rxpayload
		MOVWF	FSR0L
		movlw	HIGH chan1new
		movwf	FSR1H
		movlw	LOW chan1new
		movwf	FSR1L
	BANKSEL PACKET_ID
		movfw	PACKET_ID
		xorwf	INDF0,W   ; Does the packet id match?
		btfss	STATUS,Z
		goto	_check_secondary
		; They Match!
		movfw	OFFSET
		addwf	FSR0,F
		movfw	NUM_CH1
		call	_do_copy

_check_secondary
		movfw	PACKET_ID
		decf	WREG,W
		xorwf	INDF0,W
		btfss	STATUS,W
		goto	main_loop ; No match

		incf	FSR0,F ;Point to 1st byte
		movfw	NUM_CH1
		addwf	FSR1,F
		movfw	NUM_CH2
		call	_do_copy
		goto	main_loop 

_do_copy
		moviw	FSR0++
		movwi	FSR1++
		decfsz	WREG,W
		goto	_do_copy
		return

CMD_NEWCHAN
		movlw	rxpayload+1
		movwf	FSR0L
		movlw	2				;# bytes to copy
		movwf	BL_TEMP
	BANKSEL EEADRL
		movlw   (START_CHL-RXADDR) ; Offset in the EEPROM
		movwf	EEADRL
		call	WRITE_TO_FLASH
	;Write Completed - re-read/re-init device (might be better to reboot)
		clrf	FSR0H 				; BANK 0 variables
		movlw	LOW RXADDR
		movwf	FSR0L 				; Point to ID_HIGH
		call	_read_eeprom
		call	GET_INDEX
		call	_reply_ack
		call	INIT_NRF ; Restart radios in listener mode
		goto	main_loop

CMD_NEWID
		movlw	rxpayload+1
		movwf	FSR0L
		movlw	3
		movwf	BL_TEMP
	BANKSEL EEADRL
		clrf	EEADRL ; START address is at 0x00
		call	WRITE_TO_FLASH
		; Write completed .. restart the device
		call	_reply_ack
		reset

; ------------------------------------------------------------------
; ---------- Initialize the PIC - updates from Boot Loader Config --
; ------------------------------------------------------------------
; Bootloader already deals with:
;    SPI-pins - LATA defaults, OPTION_REG, INTERNAL PULL_UPS
;    OSCILLATOR - 32Mhz internal
;    A/D - Disable Analog to Digital converters
INIT_CHIP
	BANKSEL WDTCON	; BANK1
		bcf	WDTCON,0	; turn off WDT
; LABRAT - enable the HW Watchdog!!

	BANKSEL CM1CON0	; BANK2
		movlw	07h
		movwf	CM1CON0		; disable the comparators
		clrf	CM1CON0
		clrf	CM1CON1

	BANKSEL IOCAN ; BANK7
		clrf	IOCAN

	BANKSEL LATA	; BANK 2
	return

; ------------------------------------
; ----------- PWM Initialize ---------
; ------------------------------------
INIT_PWM
    BANKSEL OPTION_REG
		clrf    OPTION_REG              ; No prescaler for TMR0 needed
		bsf		OPTION_REG,PSA
		bsf		INTCON,5                ; Enable global and TMR0 interrupts
    BANKSEL LATC   
		return

; ---------------------------------------
; ----------- Memory Initialize ---------
; ---------------------------------------
INIT_MEM
		movlw 	chan1val		; Clear the channel data
		movwf	FSR0			; Address to write to
		movlw	NUM_DISPLAY_CHAN	; # of bytes to write
		movwf	nrfByteCount
		movlw	START_VALUE		; value to store
		movwi	NUM_DISPLAY_CHAN[FSR0]  ; init the chanXnew
		movwi	FSR0++			; init the chanXval & increment
		decfsz	nrfByteCount,f
		goto 	$-2			; jump back to movwi

		clrf	nrfStatus
		clrf	pwmpinsA

		return

; ---------------------------------
; ----------------- NrfInit -------	
; ---------------------------------

INIT_NRF
	BANKSEL LATC
		; Pull CE LOW
		bcf	NRF_CE
		; Pull CS High
		bsf	NRF_SPI_CS

 ; LABRAT: Update to pull baud rate from EEPROM - if NONE default to 2MBPS
		; Use default data rate (2MBps) and set POWER to HIGH
		nrfReadReg NRF_RF_SETUP	; Results left in W
		iorlw 0x06
		movwf	nrfTempByte
		nrfWriteReg NRF_RF_SETUP, nrfTempByte


		; Clear any pending STATUS flags, RX_DR | TX_DS | MAX_RT
		nrfWriteRegL NRF_STATUS, 0x70

		; Set RF_CH, 80
	BANKSEL APP_RFCHAN
		movfw	APP_RFCHAN
		xorlw	0xFF
		btfss	STATUS,Z
		goto	_set_rf_chan
		movlw	0x52
		movwf	APP_RFCHAN
_set_rf_chan
		nrfWriteReg NRF_RF_CH, APP_RFCHAN
	
		; Flush Rx
		nrfFlush NRF_FLUSH_RX

		; Flush Tx
		nrfFlush NRF_FLUSH_TX

		; Set AUTO_ACK, 0x00
		nrfWriteRegL NRF_EN_AA, 0x00

		; Set CRC Length - 
		nrfReadReg NRF_CONFIG	; Results left in W
		iorlw 0x7C	; MASK_CRC0 |  EN_CRC
		andlw 0x7F
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte

		; Setup for 3 byte address length
		nrfWriteRegL NRF_SETUP_AW, 0x01

 ; Use 3 byte address PIPE1  0xC0DE42 for traffic
 ; and <my device id> for admin packets, and default
 ; TRANSMIT address to 0xC0DEC1
	BANKSEL rxpayload
		movlw 0xC0
		movwf rxpayload ; Use rxpayload as a temp buffer
		movlw 0xDE
		movwf rxpayload+1
		movlw 0x42
		movwf rxpayload+2
		; Write to ADD_P0
		nrfWriteRegEx NRF_RX_ADDR_P2, rxpayload, 3
	BANKSEL rxpayload
		movlw 0xC1
		movwf rxpayload
		movlw 0xDE
		movwf rxpayload+1
		movlw 0xC0
		movwf rxpayload+2

		nrfWriteRegEx NRF_TX_ADDR, rxpayload, 3
		nrfWriteRegEx NRF_RX_ADDR_P1, rxpayload,3
		nrfWriteRegEx NRF_RX_ADDR_P0, RXADDR,3
		; Write the payload size
		nrfWriteRegL NRF_RX_PW_P0, 0x20 ; 32 byte payloads
		nrfWriteRegL NRF_RX_PW_P1, 0x20 ; 32 byte payloads
		nrfWriteRegL NRF_RX_PW_P2, 0x20 ; 32 byte payloads

 ;Setup Pipe to receive on my own iD (already done in the bootloader)

	BANKSEL rxpayload

		; Enable Rx on Pipes 0,1,2
		nrfReadReg NRF_EN_RXADDR
		iorlw 0x07
		movwf nrfTempRX
		nrfWriteReg NRF_EN_RXADDR, nrfTempRX

		; Turn on Radio
		nrfReadReg NRF_CONFIG	; Results left in W
		iorlw 0x03		; SET PWR_UP & PRIM_RX
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte

		; Flush Tx
		nrfFlush NRF_FLUSH_TX

		; Turn on Radio
		nrfReadReg NRF_CONFIG	; Results left in W
		iorlw 0x02				; SET PWR_UP
		movwf	nrfTempByte
		nrfWriteReg NRF_CONFIG, nrfTempByte
	return

; ---------------------------------------------------------
; ---- GET_INDEX -- convert E.131/DMX channel to ----------
; ----------------- nRF packet ID & offset       ----------
; ---------------------------------------------------------
; It's not pretty.. it overshoots and then adds back... 
;      ... it could be better ..  but it works!
; ---------------------------------------------------------
GET_INDEX
	BANKSEL START_CHH
		movfw	START_CHH
	BANKSEL	TEMP_CHH
		movwf	TEMP_CHH
	BANKSEL START_CHL
		movfw	START_CHL
	BANKSEL TEMP_CHL
		movwf	TEMP_CHL
		movlw	NUM_DISPLAY_CHAN
		movwf	NUM_CH1
		CLRF	NUM_CH2
		CLRF	SPLIT_PAYLOAD
		CLRF	PACKET_ID

_dec_loop
		MOVLW	0x1F			; Loop - subtract 31 and see if result when negative
		SUBWF	TEMP_CHL,F
		BTFSS	STATUS,C	 	; Did we drop below zero?
		GOTO	_roll_over		; Negative result - check if we have to subtract the carry
		BTFSC	STATUS,Z		; Zero result - see if we are done?
		GOTO	_roll_over
		INCF	PACKET_ID,F		; Looping back , incremement that packet counter and continue
		GOTO	_dec_loop

_roll_over
		MOVFW	TEMP_CHH		;See if there is anything left in the MSB to decrement?
		BTFSC	STATUS,Z		;If zero - skip to exit handling
		GOTO	_end_of_loop
		DECF	TEMP_CHH,F		; NOT zero - increment that packet count and continue
		INCF	PACKET_ID,F
		GOTO	_dec_loop

_end_of_loop
		MOVFW	TEMP_CHL		; CHL was negative (or zero) - add 31 back to it.
		ADDLW	0x20			; Includes +1 to skip command byte
		MOVWF	OFFSET
		MOVFW	PACKET_ID
		BTFSS	STATUS,Z		; Counting starts at offset 1 
		DECF	OFFSET,F

		; Offset is known.. do we wrap into next packet?
		; This code checks if channel count spans into the subsequent payloads
		MOVFW	OFFSET
		SUBLW	(32-NUM_DISPLAY_CHAN)+1
		BTFSC	STATUS,C
		RETURN
		; Device range crosses payload boundary
		BSF	SPLIT_PAYLOAD,0
		XORLW	0xFF
		INCF	WREG,W			; Convert -1 number into number of bytes in 2nd packet
		MOVWF	NUM_CH2	
		SUBWF	NUM_CH1,F
		RETURN
; ---------------------------------------------------------
EndofData:
	dt "(c) 2022 Andrew Williams"

; ---------------------------------------------------------
; ---------------------------------------------------------
	ORG 0x6FF
DmxCurveLookup
		addwf	PCL,F
	ORG 0x700
; ---
; DMX Dimming Curve
DmxCurve
		;/* 0x0x */
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x00
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		;/* 0x1x */
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x01
		retlw 0x02
		retlw 0x02
		retlw 0x02
		retlw 0x02
		retlw 0x02
		retlw 0x02
		retlw 0x02
		;/* 0x2x */
		retlw 0x02
		retlw 0x02
		retlw 0x03
		retlw 0x03
		retlw 0x03
		retlw 0x03
		retlw 0x03
		retlw 0x03
		retlw 0x04
		retlw 0x04
		retlw 0x04
		retlw 0x04
		retlw 0x04
		retlw 0x05
		retlw 0x05
		retlw 0x05
		;/* 0x3x */
		retlw 0x05
		retlw 0x06
		retlw 0x06
		retlw 0x06
		retlw 0x07
		retlw 0x07
		retlw 0x07
		retlw 0x08
		retlw 0x08
		retlw 0x08
		retlw 0x09
		retlw 0x09
		retlw 0x0A
		retlw 0x0A
		retlw 0x0B
		retlw 0x0B
		;/* 0x4x */
		retlw 0x0C
		retlw 0x0C
		retlw 0x0D
		retlw 0x0D
		retlw 0x0E
		retlw 0x0F
		retlw 0x0F
		retlw 0x10
		retlw 0x11
		retlw 0x11
		retlw 0x12
		retlw 0x13
		retlw 0x14
		retlw 0x15
		retlw 0x16
		retlw 0x17
		;/* 0x5x */
		retlw 0x18
		retlw 0x19
		retlw 0x1A
		retlw 0x1B
		retlw 0x1C
		retlw 0x1D
		retlw 0x1F
		retlw 0x20
		retlw 0x21
		retlw 0x23
		retlw 0x24
		retlw 0x26
		retlw 0x27
		retlw 0x29
		retlw 0x2B
		retlw 0x2C
		;/* 0x6x */
		retlw 0x2E
		retlw 0x30
		retlw 0x32
		retlw 0x34
		retlw 0x36
		retlw 0x38
		retlw 0x3A
		retlw 0x3C
		retlw 0x3E
		retlw 0x40
		retlw 0x43
		retlw 0x45
		retlw 0x47
		retlw 0x4A
		retlw 0x4C
		retlw 0x4F
		;/* 0x7x */
		retlw 0x51
		retlw 0x54
		retlw 0x57
		retlw 0x59
		retlw 0x5C
		retlw 0x5F
		retlw 0x62
		retlw 0x64
		retlw 0x67
		retlw 0x6A
		retlw 0x6D
		retlw 0x70
		retlw 0x73
		retlw 0x76
		retlw 0x79
		retlw 0x7C
		;/* 0x8x */
		retlw 0x7F
		retlw 0x82
		retlw 0x85
		retlw 0x88
		retlw 0x8B
		retlw 0x8E
		retlw 0x91
		retlw 0x94
		retlw 0x97
		retlw 0x9A
		retlw 0x9C
		retlw 0x9F
		retlw 0xA2
		retlw 0xA5
		retlw 0xA7
		retlw 0xAA
		;/* 0x9x */
		retlw 0xAD
		retlw 0xAF
		retlw 0xB2
		retlw 0xB4
		retlw 0xB7
		retlw 0xB9
		retlw 0xBB
		retlw 0xBE
		retlw 0xC0
		retlw 0xC2
		retlw 0xC4
		retlw 0xC6
		retlw 0xC8
		retlw 0xCA
		retlw 0xCC
		retlw 0xCE
		;/* 0xAx */
		retlw 0xD0
		retlw 0xD2
		retlw 0xD3
		retlw 0xD5
		retlw 0xD7
		retlw 0xD8
		retlw 0xDA
		retlw 0xDB
		retlw 0xDD
		retlw 0xDE
		retlw 0xDF
		retlw 0xE1
		retlw 0xE2
		retlw 0xE3
		retlw 0xE4
		retlw 0xE5
		;/* 0xBx */
		retlw 0xE6
		retlw 0xE7
		retlw 0xE8
		retlw 0xE9
		retlw 0xEA
		retlw 0xEB
		retlw 0xEC
		retlw 0xED
		retlw 0xED
		retlw 0xEE
		retlw 0xEF
		retlw 0xEF
		retlw 0xF0
		retlw 0xF1
		retlw 0xF1
		retlw 0xF2
		;/* 0xCx */
		retlw 0xF2
		retlw 0xF3
		retlw 0xF3
		retlw 0xF4
		retlw 0xF4
		retlw 0xF5
		retlw 0xF5
		retlw 0xF6
		retlw 0xF6
		retlw 0xF6
		retlw 0xF7
		retlw 0xF7
		retlw 0xF7
		retlw 0xF8
		retlw 0xF8
		retlw 0xF8
		;/* 0xDx */
		retlw 0xF9
		retlw 0xF9
		retlw 0xF9
		retlw 0xF9
		retlw 0xFA
		retlw 0xFA
		retlw 0xFA
		retlw 0xFA
		retlw 0xFA
		retlw 0xFB
		retlw 0xFB
		retlw 0xFB
		retlw 0xFB
		retlw 0xFB
		retlw 0xFB
		retlw 0xFC
		;/* 0xEx */
		retlw 0xFC
		retlw 0xFC
		retlw 0xFC
		retlw 0xFC
		retlw 0xFC
		retlw 0xFC
		retlw 0xFC
		retlw 0xFC
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		;/* 0xFx */
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFD
		retlw 0xFE
		retlw 0xFE
		retlw 0xFE
		retlw 0xFE
		retlw 0xFE
		retlw 0xFE
		retlw 0xFE
		retlw 0xFE
; ----------------- end! -------	
        END

