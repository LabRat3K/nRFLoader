; --- NRF MACROS
; Rely on the existence of helper routines
; function _nrfReadReg
; function _nrfWriteReg
; function _nrfWriteRegEx
; function _nrfFlush
;
;


; SYNCHRONOUS READ OF A REGISTER
; Results returned in W
nrfReadReg macro regId
		movlw	regId&0x1F
		call	_nrfReadReg		; Call the common code
	endm

; SYNCHRONOUS WRTIE OF A REGISTER 
; Optimized - store value to write in FSR1H (instead of using FSR1 indirectly
nrfWriteReg macro regId, value
		movfw	value
		movwf	FSR1H
		movlw	(regId&0x1F)|0x20
		call	_nrfWriteReg		; Call the common code
	endm

; SYNCHRONOUS WRTIE OF A LITTERAL INTO NRF REGISTER 
; Optimized - store value to write in FSR1H (instead of using FSR1 indirectly
nrfWriteRegL macro regId, value
		movlw	value
		movwf	FSR1H
		movlw 	(regId&0x1F)|0x20
		call	_nrfWriteReg		; Call the common code
	endm

;SYNCHRONOUS WRITE OF ACTIVATE command
nrfActivate macro value
		movfw	value
		movwf	FSR1H
		movlw	0x50
		call	_nrfWriteReg		; Call the common code
	endm 

; SYNCHRONOUS MULTI-BYTE WRITE OF A REGISTER
; Variable length - need to use the FSR0 with indirection. 
nrfWriteRegEx macro regId, srcAddr, numBytes
		movlw	srcAddr			; Set FSR0 to the source address
		movwf	FSR0
		movlw	numBytes
		movwf	nrfByteCount		; Set nrfByteCount to the # of bytes to transmit
		movlw	(regId&0x1F)|0x20 
		call	_nrfWriteRegEx		; Call the common code
	endm

; SYNCHRONOUS MULTI-BYTE WRITE OF A PACKET
nrfWritePayload macro regId, srcAddr, numBytes
		movlw	srcAddr			; Set FSR0 to the source address
		movwf	FSR0
		movlw	numBytes
		movwf	nrfByteCount		; Set nrfByteCount to the # of bytes to transmit
		movlw	regId
		call	_nrfWriteRegEx		; Call the common code
	endm

; SYNCHRONOUS FLUSH COMMAND
nrfFlush macro cmd
		movlw	cmd
		call	_nrfFlush		; Call the common code
	endm

; SYNCHRONOUS READ OF PAYLOAD	
nrfReadPayload macro addr, size
		movlw	addr			; Set FSR0 as the write target address
		movwf	FSR0
		movlw	size
		movwf	nrfByteCount	; Store number of bytes to read
	BANKSEL LATC
		bcf	NRF_SPI_CS		; Clear the CS
	BANKSEL SSP1BUF
		movlw	NRF_READ_PAYLOAD
		movwf	SSP1BUF			; Write the READ PAYLOAD command (0x61)
	; Wait for the result
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop until SPI IRQ completed
		goto 	$-1
		bcf	PIR1, SSP1IF	; Clear the SPI IRQ
	BANKSEL SSP1BUF
		movfw	SSP1BUF
		movwf	nrfStatus		; Store the returned NRF_STATUS
		clrf	SSP1BUF			; Send 0x00 in order to clock in the payloadbyte
	BANKSEL PIR1
		btfss	PIR1, SSP1IF	; Loop untilSPI IRQ completed
		goto 	$-1
		bcf	PIR1, SSP1IF	; Clear the interrupt
	BANKSEL SSP1BUF
		movfw 	SSP1BUF			; Read the incoming SPI byte
		movwi	FSR0++			; Write the byte to the target address (increment for next read)
		decfsz	nrfByteCount,f	; Decrement the count and loop if not done
		goto 	$-9				; Jump back to clrf SSP1BUF
	BANKSEL LATC
		bsf	NRF_SPI_CS		; Set the CS
	endm
