;**********************************************************************
;                                                                     *
;    Filename:	    bod2bod.asm                                       *
;    Date:          8 March 1999                                      *
;    File Version:  1                                                 *
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
;    Notes: Half duplex BodNet serial interface using 10 KHz          *
;           interrupt to provide 10K baud.                            *
;           1 start bit, 8 data bits, no parity, 1 stop bit           *
;                                                                     *
;           'Up' interface,   Port A 3                                *
;           'Down' interface, Port A 0                                *
;                                                                     *
;**********************************************************************


;**********************************************************************
; Include and configuration directives                                *
;**********************************************************************

	list      p=16C84
	#include <c:\mplab\inc\p16C84.inc>
#define SYNC_SERIAL
	#include <\DEV\PROJECTS\PICSRL\asyn_srl.inc>
	#include <\DEV\PROJECTS\BODNET\PIC\BODLINK\bod_link.inc>

	__CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _XT_OSC

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.


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

#ifndef SYNC_SERIAL
portAStatus   EQU     B'00000000' ; Initial port status, all bits set as output
#else
portAStatus   EQU     B'00010000' ; Initial port status, bit 4 set as input all other bits set as output
#endif
portBStatus   EQU     B'00000000' ; Initial port status, all bits set as output

#ifndef SYNC_SERIAL
rtccInt       EQU     158         ; 1Mhz / 100 (adjusted for RTCC write inhibit) = 10Khz
#else
rtccInt       EQU     255         ; Interrupt on RA4/RTCC input (10KHz clock source)
#endif

#ifndef SYNC_SERIAL
intSerBit     EQU     2           ; Interrupts per serial bit @ 5K baud (simulated 4K8)
intSerIni     EQU     3           ; Interrupts per initial Rx serial bit (start bit)
#else
intSerBit     EQU     1           ; Interrupts per serial bit @ 10K baud
intSerIni     EQU     1           ; Interrupts per initial Rx serial bit (start bit)
#endif

intMilli      EQU     10          ; Interrupts per millisecond

txDataBits    EQU     10          ; 1 start, 8 data, no parity, 1 stop (tx an extra zero)
rxDataBits    EQU     9           ; 1 start (not counted), 8 data, no parity, 1 stop

; 'serStatus' bit definitions
serURdy       EQU     0           ; Serial ready flag (0 - not ready, 1 - ready)
rxUErr        EQU     1           ; Rx error, ignored received data
txUBsy        EQU     1           ; Tx byte busy (0 - finished, 1 - in progress)
serDRdy       EQU     4           ; Serial ready flag (0 - not ready, 1 - ready)
rxDErr        EQU     5           ; Rx error, ignored received data
txDBsy        EQU     5           ; Tx byte busy (0 - finished, 1 - in progress)

MSB           EQU     7

; 'Up' side interface constants
txUTRIS       EQU     TRISA       ; 'Up' side Tx port direction register
txUPORT       EQU     PORTA       ; 'Up' side Tx port data register
txUBit        EQU     3           ; Tx on Port A bit 3
rxUTRIS       EQU     TRISA       ; 'Up' side Rx port direction register
rxUPORT       EQU     PORTA       ; 'Up' side Rx port data register
rxUBit        EQU     3           ; Rx on Port A bit 3

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
  		goto    Main              ; Jump to beginning of program


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
;		call    SrvcDLink         ; Perform down interface link service

EndISR		bsf     clkPORT,clkBit    ; Set BodNet clock output low

		movf    status_isr,W      ; Retrieve copy of STATUS register
		movwf	STATUS            ; Restore pre-isr STATUS register contents
		swapf   w_isr,F
		swapf   w_isr,W           ; Restore pre-isr W register contents
		retfie                    ; Return from interrupt


;**********************************************************************
; Physical layer service routine macro invocations                    *
;**********************************************************************

SrvcULink	SrvcLink   PerformRxU, PerformTxU, lnkUSte, intSerIni, serUTmr, EnableTxU, InitTxU, EnableRxU, InitRxU


SrvcDLink	SrvcLink   PerformRxD, PerformTxD, lnkDSte, intSerIni, serDTmr, EnableTxD, InitTxD, EnableRxD, InitRxD


EnableRxU	EnableRx  rxUTRIS, rxUPORT, rxUBit
		return


InitRxU		InitRx  serUTmr, serStatus, serURdy, rxUErr
		return


PerformRxU	PerformRx serUTmr, rxUPORT, rxUBit, serUBitCnt, rxDataBits, intSerBit, serStatus, rxUErr, serUReg, serUByt, serURdy, intSerBit


EnableTxU	EnableTx  txUTRIS, txUPORT, txUBit
		return


InitTxU		InitTx  serUTmr, serStatus, serURdy, txUBsy
		return


PerformTxU	PerformTx serUTmr, serStatus, serUByt, serUReg, serURdy, txUBsy, txDataBits, serUBitCnt, intSerBit, txUPORT, txUBit

EnableRxD	EnableRx  rxDTRIS, rxDPORT, rxDBit
		return


InitRxD		InitRx  serDTmr, serStatus, serDRdy, rxDErr
		return


PerformRxD	PerformRx serDTmr, rxDPORT, rxDBit, serDBitCnt, rxDataBits, intSerBit, serStatus, rxDErr, serDReg, serDByt, serDRdy, intSerBit


EnableTxD	EnableTx  txDTRIS, txDPORT, txDBit
		return


InitTxD		InitTx  serDTmr, serStatus, serDRdy, txDBsy
		return


PerformTxD	PerformTx serDTmr, serStatus, serDByt, serDReg, serDRdy, txDBsy, txDataBits, serDBitCnt, intSerBit, txDPORT, txDBit


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

		btfsc   serStatus,txUBsy  ; Skip if up interface Tx not in progress ...
		return                    ; ... otherwise return (wait for Tx to complete)

		; The 'Up' link has completed all transmission, initiate a 'turnaround' to the receive state

		clrf    lnkUSte           ; Set up link to state 0 (stop transmitting)
		return


;**********************************************************************

; As this is a test program the 'Down' link simply simulates echoing of it's transmit data

LinkD		btfsc   serStatus,serDRdy ; Skip if down interface Tx byte 'clear' flag not set ...
		return                    ; ... otherwise return (no new Tx data available)

		movf    lnkUSte,W
		xorlw   2
		btfss	STATUS,Z          ; Skip if up link in state 2 (receiving) ...
		goto    LinkUNotInRx      ; ... otherwise jump

		movlw   3
		movwf   lnkUSte           ; Force up link to state 3 (stop receiving)
		return

LinkUNotInRx	movf    lnkUSte,W
		xorlw   6
		btfss	STATUS,Z          ; Skip if up link in state 6 (transmitting) ...
		return                    ; ... otherwise return

		btfss   serStatus,serURdy ; Skip if up interface Tx byte 'clear' flag set ...
		return                    ; ... otherwise return (up Tx data already queued)

		movf    serDByt,W         ; Copy down link Rx byte ...
		movwf   serUByt           ; ... to up link Tx byte
		bsf     serStatus,serDRdy ; Set down interface Tx byte 'clear' flag
		bcf     serStatus,serURdy ; Clear up interface Tx byte 'clear' flag
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
#ifdef SYNC_SERIAL
		bsf     OPTION_REG,T0CS
#endif
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
		bsf     serStatus,serDRdy ; Set down interface Tx byte 'clear' flag
		movlw   6                 ; Initial down link state ...
		movwf   lnkDSte           ; ... to state 6 (transmitting)

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

		call    LinkD             ; 'Down' link not really there, simply echoes data

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

