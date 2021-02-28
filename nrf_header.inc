; ----------------- NrfInit -------	
; Register Definitions
#define NRF_CONFIG 		(0x00)
#define NRF_EN_AA 		(0x01)
; NRF_RXADDR is deprecated in favour of NRF_EN_RXADDR
#define NRF_RXADDR		(0x02)
#define NRF_EN_RXADDR	(0x02)
#define NRF_SETUP_AW	(0x03)
#define NRF_SETUP_RETR	(0x04)
#define NRF_RF_CH		(0x05)
#define NRF_RF_SETUP	(0x06)
#define NRF_STATUS		(0x07)
#define NRF_OBSERVE_TX	(0x08)
#define NRF_CD			(0x09)
#define NRF_RX_ADDR_P0	(0x0A)
#define NRF_RX_ADDR_P1	(0x0B)
#define NRF_RX_ADDR_P2	(0x0C)
#define NRF_RX_ADDR_P3	(0x0D)
#define NRF_RX_ADDR_P4	(0x0E)
#define NRF_RX_ADDR_P5	(0x0F)
#define NRF_TX_ADDR		(0x10)
#define NRF_RX_PW_P0	(0x11)
#define NRF_RX_PW_P1	(0x12)
#define NRF_RX_PW_P2	(0x13)
#define NRF_RX_PW_P3	(0x14)
#define NRF_RX_PW_P4	(0x15)
#define NRF_RX_PW_P5	(0x16)
#define NRF_FIFO_STATUS	(0x17)
#define DYNPD           (0x1C)
#define FEATURE         (0x1D)

; -------------------------
; Commands
#define NRF_READ_PAYLOAD (0x61)
#define NRF_FLUSH_TX     (0xE1)
#define NRF_FLUSH_RX     (0xE2)
#define NRF_READ_STATUS  (0xFF)

; ----------------------------------
; Bootloader NRF packet structures
;
; Constants used by the bootloader communications protocol.
; *All* messages are 32 bytes in length, first byte
NRF_BL_CMD		equ	0x87
NRF_BL_CMD_MASK 	equ	0x0F

; START command: <0x00> <ADRL> <ADRH> <ERASE?>
NRF_BL_CMD_START	equ	0x80
NRF_BL_START_ADRL	equ	1
NRF_BL_START_ADRH	equ	2
NRF_BL_START_ERASE	equ	3

; MIDDLE command:<0x01> <30 bytes (15 words)> (64 byte flash requires 2x this mesage)
NRF_BL_CMD_WRITE	equ	0x81
NRF_BL_WRITE_START	equ	1

; COMMIT command:<0x02><size><cksum>< size*2 bytes (size words)>
; incl size of the DATA payload
NRF_BL_CMD_COMMIT	equ	0x82
NRF_BL_COMMIT_SIZE	equ	1
NRF_BL_CKSUM		equ	2
NRF_BL_COMMIT_DATA	equ	3

; AUDIT command: <0x03><ADRL><ADRH><SIZEL><SIZEH><CKSUM_ODD><CKSUM_EVEN>
NRF_BL_CMD_AUDIT	equ	0x83
NRF_BL_AUDIT_ADRL	equ	1
NRF_BL_AUDIT_ADRH	equ	2
NRF_BL_AUDIT_SIZEL	equ	3
NRF_BL_AUDIT_SIZEH	equ	4
NRF_BL_AUDIT_ODDSUM	equ	5
NRF_BL_AUDIT_EVENSUM	equ	6

; RESET command
NRF_BL_CMD_RESET	equ	0x86
