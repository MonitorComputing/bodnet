;**********************************************************************
;                                                                     *
;    Description:                                                     *
;                                                                     *
;    Author:        Chris White (whitecf@bcs.org.uk)                  *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Copyright (C) 1999  Monitor Computing Services Ltd.              *
;                                                                     *
;    This program is free software; you can redistribute it and/or    *
;    modify it under the terms of the GNU General Public License      *
;    as published by the Free Software Foundation; either version 2   *
;    of the License, or any later version.                            *
;                                                                     *
;    This program is distributed in the hope that it will be useful,  *
;    but WITHOUT ANY WARRANTY; without even the implied warranty of   *
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the    *
;    GNU General Public License for more details.                     *
;                                                                     *
;    You should have received a copy of the GNU General Public        *
;    License (http://www.gnu.org/copyleft/gpl.html) along with this   *
;    program; if not, write to:                                       *
;       The Free Software Foundation Inc.,                            *
;       59 Temple Place - Suite 330,                                  *
;       Boston, MA  02111-1307,                                       *
;       USA.                                                          *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes: Half duplex serial to BodNet interface using 10 KHz       *
;           interrupt to provide pseudo 4K8 baud.                     *
;           1 start bit, 8 data bits, no parity, 1 stop bit           *
;                                                                     *
;           'Up' interface, asynchronous serial @ 4K8 baud            *
;           Port B 0 - Rx, 1 - Tx                                     *
;                                                                     *
;           'Down' interface, synchronous BodNet @ 10K baud           *
;           Port A 0 - Rx/Tx, 1 - Data clock out                      *
;                                                                     *
;**********************************************************************


	list      p=16f84

#include <p16f84.inc>

	__CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _XT_OSC

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.


;**********************************************************************
; Include and configuration directives                                *
;**********************************************************************

#include "../utility_pic/asyn_srl.inc"
#include "../utility_pic/link_hd.inc"


;**********************************************************************
; Variable definitions                                                *
;**********************************************************************

		CBLOCK  0x0C

; Status and accumulator storage registers
w_isr         ; 'w' register, accumulator, store during ISR
status_isr    ; status register store during ISR

; Timing registers
tmrCount      ; Free running interrupt counter, used to generate timing loops
nextMilli     ; Interrupt counter value for millisecond timing
centiCount    ; Millisecond counter for centisecond timing
deciCount     ; Centisecond counter for decisecond timing
secCount      ; Decisecond counter for second timing

; Interface registers
serStatus     ; Serial I/F status flags

; 'Up' side interface registers
serUTmr       ; Interrupt counter for serial bit timing
serUReg       ; Data shift register
serUByt       ; Data byte buffer
serUBitCnt    ; Bit down counter
lnkUSte       ; Up link byte level state register

; 'Down' side interface registers
serDTmr       ; Interrupt counter for serial bit timing
serDReg       ; Data shift register
serDByt       ; Data byte buffer
serDBitCnt    ; Bit down counter
lnkDSte       ; Down link byte level state register

		ENDC


;**********************************************************************
; Constant definitions                                                *
;**********************************************************************

portAStatus   EQU     B'00000000' ; Initial port status, all bits set as output
portBStatus   EQU     B'00000000' ; Initial port status, all bits set as output

rtccInt       EQU     158         ; 1Mhz / 100 (adjusted for RTCC write inhibit) = 10Khz

intSerBit     EQU     2           ; Interrupts per serial bit @ 5K baud (simulated 4K8)
intSerIni     EQU     3           ; Interrupts per initial Rx serial bit (start bit)

intMilli      EQU     10          ; Interrupts per millisecond

txDataBits    EQU     10          ; 1 start, 8 data, no parity, 1 stop (tx an extra zero)
rxDataBits    EQU     9           ; 1 start (not counted), 8 data, no parity, 1 stop

; 'serStatus' bit definitions
serURdy       EQU     0           ; Serial ready flag (0 - not ready, 1 - ready)
rxUErr        EQU     1           ; Rx error, ignored received data
txUClr        EQU     1           ; Tx byte clear
RXUBREAK      EQU     2           ; 'Up' received 'break' status bit
serDRdy       EQU     4           ; Serial ready flag (0 - not ready, 1 - ready)
rxDErr        EQU     5           ; Rx error, ignored received data
txDClr        EQU     5           ; Tx byte clear
RXDBREAK      EQU     6           ; 'Down' received 'break' status bit

MSB           EQU     7

; 'Up' side interface constants
txUTRIS       EQU     TRISB       ; 'Up' side Tx port direction register
txUPORT       EQU     PORTB       ; 'Up' side Tx port data register
txUBit        EQU     1           ; Tx on Port B bit 1
rxUTRIS       EQU     TRISB       ; 'Up' side Rx port direction register
rxUPORT       EQU     PORTB       ; 'Up' side Rx port data register
rxUBit        EQU     0           ; Rx on Port B bit 0

; 'Down' side interface constants
txDTRIS       EQU     TRISA       ; 'Down' side Tx port direction register
txDPORT       EQU     PORTA       ; 'Down' side Tx port data register
txDBit        EQU     0           ; Tx on Port A bit 0
rxDTRIS       EQU     TRISA       ; 'Down' side Rx port direction register
rxDPORT       EQU     PORTA       ; 'Down' side Rx port data register
rxDBit        EQU     0           ; Rx on Port A bit 0
clkTRIS       EQU     TRISA       ; 'Down' side clock port direction register
clkPORT       EQU     PORTA       ; 'Down' side clock port data register
clkBit        EQU     1           ; 'Down' side clock output on Port A bit 1


;**********************************************************************
; Reset vector, initialisation code                                   *
;**********************************************************************

		ORG     0x000             ; processor reset vector
BootVector	goto    Main              ; Jump to beginning of program


;**********************************************************************
; Interrupt vector, interrupt service routine (ISR) code              *
;**********************************************************************

		ORG     0x004             ; interrupt vector location
BeginISR	movwf   w_isr             ; save off current W register contents
		movf	STATUS,W          ; move status register into W register
		movwf	status_isr        ; save off contents of STATUS register

		btfss   INTCON,T0IF       ; Skip if RTCC interrupt flag is set ...
		goto    EndISR            ; ... otherwise jump to end of service routine

		BANKSEL TMR0              ; Ensure register page 0 is selected

		bcf     clkPORT,clkBit    ; Set BodNet clock output low
		bcf     INTCON,T0IF       ; Clear the RTCC interrupt bitflag

		movlw   rtccInt
		addwf   TMR0,F            ; Reload RTCC

		incf    tmrCount,F        ; Increment free running interrupt counter

		call    SrvcULink         ; Perform up interface link service
		call    SrvcDLink         ; Perform down interface link service

EndISR		bsf     clkPORT,clkBit    ; Set BodNet clock output low

		movf    status_isr,W      ; Retrieve copy of STATUS register
		movwf	STATUS            ; Restore pre-isr STATUS register contents
		swapf   w_isr,F
		swapf   w_isr,W           ; Restore pre-isr W register contents
		retfie                    ; Return from interrupt


;**********************************************************************
; Physical layer service routine macro invocations                    *
;**********************************************************************

SrvcULink	SrvcLink   SrvcRxU, SrvcTxU, lnkUSte, intSerIni, serUTmr, EnableTxU, InitTxU, EnableRxU, InitRxU


EnableRxU	EnableRx  rxUTRIS, rxUPORT, rxUBit
		return


InitRxU		InitRx  serUTmr, serStatus, serURdy, rxUErr, RXUBREAK
		return


SrvcRxU	ServiceRx serUTmr, rxUPORT, rxUBit, serUBitCnt, rxDataBits, intSerIni, serStatus, rxUErr, serUReg, serUByt, serURdy, intSerBit


EnableTxU	EnableTx  txUTRIS, txUPORT, txUBit
		return


InitTxU		InitTx  serUTmr, serStatus, txUClr
		return


SrvcTxU	ServiceTx serUTmr, serStatus, serUByt, serUReg, serURdy, txUClr, txDataBits, serUBitCnt, intSerBit, txUPORT, txUBit


SrvcDLink	SrvcLink   SrvcRxD, SrvcTxD, lnkDSte, intSerIni, serDTmr, EnableTxD, InitTxD, EnableRxD, InitRxD


; 'Down' link is synchronous so use #define to simplify compiled code
#define SYNC_SERIAL
EnableRxD	EnableRx  rxDTRIS, rxDPORT, rxDBit
		return


InitRxD		InitRx  serDTmr, serStatus, serDRdy, rxDErr, RXDBREAK
		return


SrvcRxD	ServiceRx serDTmr, rxDPORT, rxDBit, serDBitCnt, rxDataBits, 1, serStatus, rxDErr, serDReg, serDByt, serDRdy, 1


EnableTxD	EnableTx  txDTRIS, txDPORT, txDBit
		return


InitTxD		InitTx  serDTmr, serStatus, txDClr
		return


SrvcTxD	ServiceTx serDTmr, serStatus, serDByt, serDReg, serDRdy, txDClr, txDataBits, serDBitCnt, 1, txDPORT, txDBit
#undefine SYNC_SERIAL


;**********************************************************************
; Link layer service routines                                         *
;**********************************************************************

LinkURx		; Test if a valid byte has been received by the 'Up' link

		btfss   serStatus,serURdy ; Skip if up interface Rx byte 'loaded' flag set ...
		return                    ; ... otherwise return (no new Rx data available)

		btfss   serStatus,rxUErr  ; Skip if up interface Rx error flag set ...
		goto    LinkURxData       ; ... otherwise jump (accept new Rx data)

		bcf     serStatus,serURdy ; Clear up interface Rx byte 'loaded' flag (reject new Rx data)
		return

LinkURxData	; A valid byte has been received, test if the 'Down' link is in the receive state

		movf    lnkDSte,W
		xorlw   2
		btfss	STATUS,Z          ; Skip if down link in state 2 (receive) ...
		goto    LinkDNotRx        ; ... otherwise jump

		; 'Down' link is in the receive state, force it to 'turnaround' to the transmit state

		movlw   3
		movwf   lnkDSte           ; Force down link to state 3 (stop receiving)
		return

LinkDNotRx	; 'Down' link is not in the recieve state, test if it is in the transmit state

		movf    lnkDSte,W
		xorlw   6
		btfss	STATUS,Z          ; Skip if down link in state 6 (transmit) ...
		return                    ; ... otherwise return

		; 'Down' link is in the transmit state, test if it's Tx byte buffer is available

		btfss   serStatus,serDRdy ; Skip if down interface Tx byte 'clear' flag set ...
		return                    ; ... otherwise return (down Tx data already queued)

		; 'Down' link is in the transmit state and it's Tx byte buffer is available, relay the received byte

		movf    serUByt,W         ; Copy up link Rx byte ...
		movwf   serDByt           ; ... to down link Tx byte
		bcf     serStatus,serURdy ; Clear up interface Rx byte 'loaded' flag
		bcf     serStatus,serDRdy ; Clear down interface Tx byte 'clear' flag
		return


;**********************************************************************

LinkUTx		; Test if a byte is queued in the 'Up' link Tx buffer for transmission

		btfss   serStatus,serURdy ; Skip if up interface Tx byte 'clear' flag set ...
		return                    ; ... otherwise return (Tx data queued)

		; Nothing queued, test if the 'Up' link has a transmission in progress

		btfss   serStatus,txUClr  ; Skip if up interface Tx not in progress ...
		return                    ; ... otherwise return (wait for Tx to complete)

		; The 'Up' link has completed all transmission, initiate a 'turnaround' to the receive state

		clrf    lnkUSte           ; Set up link to state 0 (stop transmitting)
		return


;**********************************************************************

LinkDRx		; Test if a valid byte has been received by the 'Down' link

		btfss   serStatus,serDRdy ; Skip if down interface Rx byte 'loaded' flag set ...
		return                    ; ... otherwise return (no new Rx data available)

		btfss   serStatus,rxDErr  ; Skip if down interface Rx error flag set ...
		goto    LinkDRxData       ; ... otherwise jump (accept new Rx data)

		bcf     serStatus,serDRdy ; Clear down interface Rx byte 'loaded' flag (reject new Rx data)
		return

LinkDRxData	; A valid byte has been received, test if the 'Up' link is in the receive state

		movf    lnkUSte,W
		xorlw   2
		btfss	STATUS,Z          ; Skip if up link in state 2 (receiving) ...
		goto    LinkUNotRx        ; ... otherwise jump

		; 'Up' link is in the receive state, force it to 'turnaround' to the transmit state

		movlw   3
		movwf   lnkUSte           ; Force up link to state 3 (stop receiving)
		return

LinkUNotRx	; 'Up' link is not in the recieve state, test if it is in the transmit state

		movf    lnkUSte,W
		xorlw   6
		btfss	STATUS,Z          ; Skip if up link in state 6 (transmitting) ...
		return                    ; ... otherwise return

		; 'Up' link is in the transmit state, test if it's Tx byte buffer is available

		btfss   serStatus,serURdy ; Skip if up interface Tx byte 'clear' flag set ...
		return                    ; ... otherwise return (up Tx data already queued)

		; 'Up' link is in the transmit state and it's Tx byte buffer is available, relay the received byte

		movf    serDByt,W         ; Copy down link Rx byte ...
		movwf   serUByt           ; ... to up link Tx byte
		bcf     serStatus,serDRdy ; Clear down interface Rx byte 'loaded' flag
		bcf     serStatus,serURdy ; Clear up interface Tx byte 'clear' flag
		return


;**********************************************************************

LinkDTx		; Test if a byte is queued in the 'Down' link Tx buffer for transmission

		btfss   serStatus,serDRdy ; Skip if down interface Tx byte 'clear' flag set ...
		return                    ; ... otherwise return (Tx data queued)

		; Nothing queued, test if the 'Down' link has a transmission in progress

		btfss   serStatus,txDClr  ; Skip if down interface Tx not in progress ...
		return                    ; ... otherwise return (wait for Tx to complete)

		; The 'Down' link has completed all transmission, initiate a 'turnaround' to the receive state

		clrf    lnkDSte           ; Set down link to state 0 (stop transmitting)
		return


;**********************************************************************
; Main body of code                                                   *
;**********************************************************************

Main		clrf    PORTA             ; Clear I/O ports
		clrf    PORTB

		BANKSEL OPTION_REG        ; Select register page 1

		movlw   portAStatus       ; Program I/O port bit directions
		movwf   TRISA
		movlw   portBStatus
		movwf   TRISB

		clrf    OPTION_REG
		bsf     OPTION_REG,NOT_RBPU
		bsf     OPTION_REG,INTEDG
		bsf     OPTION_REG,PSA

		BANKSEL TMR0              ; Select register page 0

		movlw   portAStatus       ; For Port A need to write one to each bit ...
		movwf   PORTA             ; ... being used for input

		; Initialise variables

		; Initialise serial links
		SerInit    serStatus, serUTmr, serUReg, serUByt, serUBitCnt, serUTmr, serUReg, serUByt, serUBitCnt
		SerInit    serStatus, serDTmr, serDReg, serDByt, serDBitCnt, serDTmr, serDReg, serDByt, serDBitCnt

		clrf    tmrCount          ; Initialise timing
		clrf    nextMilli
		movlw   10
		movwf   centiCount
		movwf   deciCount
		movwf   secCount

		call    EnableRxU
		call    InitRxU
		movlw   2                 ; Initial up link state ...
		movwf   lnkUSte           ; ... to state 2 (receiving)
		call    EnableRxD
		call    InitRxD
		movlw   2                 ; Initial down link state ...
		movwf   lnkDSte           ; ... to state 2 (receiving)

		movlw   rtccInt
		movwf   TMR0              ; Load RTCC
		clrf    INTCON
		bsf	INTCON,T0IE       ; Enable RTCC interrupts
		bsf	INTCON,GIE        ; Enable interrupts

MainLoop                                  ; Top of main processing loop

		; Relay data between the 'up' and 'down' links

		movf    lnkUSte,W
		xorlw   2
		btfsc	STATUS,Z          ; Skip if up link not in state 2 (receiving)
		call    LinkURx

		movf    lnkUSte,W
		xorlw   6
		btfsc	STATUS,Z          ; Skip if up link not in state 6 (transmitting)
		call    LinkUTx

		movf    lnkDSte,W
		xorlw   2
		btfsc	STATUS,Z          ; Skip if down link not in state 2 (receiving)
		call    LinkDRx

		movf    lnkDSte,W
		xorlw   6
		btfsc	STATUS,Z          ; Skip if down link not in state 6 (transmitting)
		call    LinkDTx

		; Execute the non interrupt timing loops.

Timing		movf    nextMilli,W       ; Compare difference of next interrupt count ...
		subwf   tmrCount,W        ; ... and previous interrupt count ...
		sublw	intMilli          ; ... with number of interrupts per millisecond
		btfsc   STATUS,C          ; Skip if a millisecond has elapsed
		goto    NotMilliSec       ; Jump if a millisecond hasn't elapsed

Millisec	movlw   intMilli          ; Update the next interrupt count ...
		addwf   nextMilli,F       ; ... to the next millisecond interrupt count

	; Any code placed here will be executed approximately every millisecond (1KHz)

		decfsz  centiCount,F      ; Decrement centi second milliseconds counter ...
		goto    NotCentiSec       ; ... skipping this jump if reached zero

CentiSecond	movlw   10                ; Reload centi second milliseconds counter
		movwf   centiCount

	; Any code placed here will be executed approximately every centisecond (100Hz)

		decfsz  deciCount,F       ; Decrement decisecond centiseconds counter ...
		goto    NotDeciSec        ; ... skipping this jump if reached zero

DeciSecond	movlw   10                ; Reload deci second centiseconds counter
		movwf   deciCount

	; Any code placed here will be executed approximately every decisecond (10Hz)

		decfsz  secCount,F        ; Decrement second deciseconds counter ...
		goto    NotSecond         ; ... skipping this jump if reached zero

Second		movlw   10                ; Reload second centiseconds counter
		movwf   secCount

	; Any code placed here will be executed approximately every second (1Hz)

NotSecond
NotDeciSec
NotCentiSec
NotMilliSec

		goto    MainLoop          ; End of main processing loop


		end                       ; directive 'end of program'
