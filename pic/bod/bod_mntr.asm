;**********************************************************************
;                                                                     *
;    Description:                                                     *
;                                                                     *
;    Author:        Chris White (whitecf@bcs.org.uk)                  *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Copyright (C) 2001  Monitor Computing Services Ltd.              *
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
;    Notes: Simple low level monitor program offering terminal        *
;           interface via half duplex serial @ 2K5 baud.              *
;           1 start bit, 8 data bits, no parity, 1 stop bit           *
;                                                                     *
;           'Up' interface, asynchronous serial @ 2K5 baud            *
;           Port A,1 - Data                                           *
;                                                                     *
;           'Down' interface, asynchronous serial @ 2K5 baud          *
;           1 start bit, 8 data bits, no parity, 1 stop bit           *
;           Port A,0 - Data                                           *
;                                                                     *
;**********************************************************************


	list      p=16F84

#include <p16F84.inc>

	__CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _XT_OSC

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.


;**********************************************************************
; Include and configuration directives                                *
;**********************************************************************

#include <\dev\projects\utility\pic\asyn_srl.inc>
#include <\dev\projects\utility\pic\link.inc>


;**********************************************************************
; Constant definitions                                                *
;**********************************************************************

PORTASTATUS   EQU     B'00000000' ; Initial port status, all bits set as output
PORTBSTATUS   EQU     B'00000000' ; Initial port status, all bits set as output

RTCCINT       EQU     158         ; 1Mhz / 100 (adjusted for RTCC write inhibit) = 10Khz

INTSERBIT     EQU     4           ; Interrupts per serial bit @ 2K5 baud
INTSERINI     EQU     6           ; Interrupts per initial Rx serial bit (start bit)
INTLINKDEL    EQU     0           ; Interrupt cycles for link turnaround delays

INTMILLI      EQU     10          ; Interrupts per millisecond

CLKPORT       EQU     PORTA       ; Clock tick output port
CLKBIT        EQU     4           ; Clock tick output on Port A bit 4

; 'Up' interface constants
RXUFLAG       EQU     0           ; 'Up' receive byte buffer 'loaded' status bit
RXUERR        EQU     1           ; 'Up' receive error status bit
RXUBREAK      EQU     2           ; 'Up' received 'break' status bit
TXUFLAG       EQU     0           ; 'Up' transmit byte buffer 'clear' status bit
TXUTRIS       EQU     TRISA       ; 'Up' side Tx port direction register
TXUPORT       EQU     PORTA       ; 'Up' side Tx port data register
TXUBIT        EQU     1           ; Tx on Port A bit 1
RXUTRIS       EQU     TRISA       ; 'Up' side Rx port direction register
RXUPORT       EQU     PORTA       ; 'Up' side Rx port data register
RXUBIT        EQU     1           ; Rx on Port A bit 1

; 'Down' interface constants
RXDFLAG       EQU     4           ; 'Down' receive byte buffer 'loaded' status bit
RXDERR        EQU     5           ; 'Down' receive error status bit
RXDBREAK      EQU     6           ; 'Down' received 'break' status bit
TXDFLAG       EQU     4           ; 'Down' transmit byte buffer 'clear' status bit
TXDTRIS       EQU     TRISA       ; 'Down' side Tx port direction register
TXDPORT       EQU     PORTA       ; 'Down' side Tx port data register
TXDBIT        EQU     0           ; 'Down' Tx on Port A bit 0
RXDTRIS       EQU     TRISA       ; 'Down' side Rx port direction register
RXDPORT       EQU     PORTA       ; 'Down' side Rx port data register
RXDBIT        EQU     0           ; 'Down' Rx on Port A bit 0


;**********************************************************************
; Variable definitions                                                *
;**********************************************************************

		CBLOCK  0x0C

; Status and accumulator storage registers
pclath_isr    ; PCLATH register store during ISR
w_isr         ; 'w' register, accumulator, store during ISR
status_isr    ; status register store during ISR

; Timing registers
tmrCount      ; Free running interrupt counter, used to generate timing loops

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
lnkDSte       ; Link state register

		ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

		ORG     0x2100            ; EEPROM data area

EEdirection	DE      00                ; Data direction mask, 0 - output    1 - input
EEactiveLow	DE      00                ; Input active mask,   0 - high      1 - low
EElatched	DE      00                ; Input latched mask,  0 - unlatched 1 - latched
 

;**********************************************************************
; Reset vector                                                        *
;**********************************************************************

		ORG     0x000             ; Processor reset vector
BootVector	goto    Main              ; Jump to beginning of program


;**********************************************************************
; Interrupt vector                                                    *
;**********************************************************************

		ORG     0x004             ; Interrupt vector location
 		goto    BeginISR          ; Jump to interrupt service routine


;**********************************************************************
; Link and interface routine macro invocations                        *
;**********************************************************************

EnableRxU	EnableRx  RXUTRIS, RXUPORT, RXUBIT
		return


InitRxU		InitRx  serUTmr, serStatus, RXUFLAG, RXUERR, RXUBREAK
		return


SrvcRxU		ServiceRx serUTmr, RXUPORT, RXUBIT, serUBitCnt, INTSERINI, serStatus, RXUERR, RXUBREAK, serUReg, serUByt, RXUFLAG, INTSERBIT


EnableTxU	EnableTx  TXUTRIS, TXUPORT, TXUBIT
		return


InitTxU		InitTx  serUTmr, serStatus, TXUFLAG
		return


SrvcTxU		ServiceTx serUTmr, serStatus, serUByt, serUReg, TXUFLAG, serUBitCnt, INTSERBIT, TXUPORT, TXUBIT, 0, 0


SerURx		SerialRx serStatus, RXUFLAG, serUByt


SerUTx		SerialTx serStatus, TXUFLAG, serUByt


SrvcULink	SrvcLink   SrvcRxU, SrvcTxU, lnkUSte, INTLINKDEL, serUTmr, EnableTxU, InitTxU, EnableRxU, InitRxU


LinkMRx		LinkRx lnkUSte, SerURx


LinkMTx		LinkTx lnkUSte, SerUTx


EnableRxD	EnableRx  RXDTRIS, RXDPORT, RXDBIT
		return


InitRxD		InitRx  serDTmr, serStatus, RXDFLAG, RXDERR, RXDBREAK
		return


SrvcRxD		ServiceRx serDTmr, RXDPORT, RXDBIT, serDBitCnt, INTSERINI, serStatus, RXDERR, RXDBREAK, serDReg, serDByt, RXDFLAG, INTSERBIT


EnableTxD	EnableTx  TXDTRIS, TXDPORT, TXDBIT
		return


InitTxD		InitTx  serDTmr, serStatus, TXDFLAG
		return


SrvcTxD		ServiceTx serDTmr, serStatus, serDByt, serDReg, TXDFLAG, serDBitCnt, INTSERBIT, TXDPORT, TXDBIT, 0, 0


SerDRx		SerialRx serStatus, RXDFLAG, serDByt


SerDTx		SerialTx serStatus, TXDFLAG, serDByt


SrvcDLink	SrvcLink   SrvcRxD, SrvcTxD, lnkDSte, INTLINKDEL, serDTmr, EnableTxD, InitTxD, EnableRxD, InitRxD


LinkHRx		LinkRx lnkDSte, SerDRx


LinkHTx		LinkTx lnkDSte, SerDTx


;**********************************************************************
; Monitor code                                                        *
;**********************************************************************

#define NOMONBANNER
#define MONUSERON
#include <\dev\projects\utility\pic\monitor.inc>


;**********************************************************************
; Interrupt service routine (ISR) code                                *
;**********************************************************************

BeginISR	movwf   w_isr             ; Save off current W register contents
		movf	STATUS,W          ; Move status register into W register
		movwf	status_isr        ; save off contents of STATUS register
		movf	PCLATH,W          ; Move PCLATH register into W register
		movwf	pclath_isr        ; save off contents of PCLATH register

		btfss   INTCON,T0IF       ; Skip if RTCC interrupt flag is set ...
		goto    EndISR            ; ... otherwise jump to end of service routine

		BANKSEL TMR0              ; Ensure register page 0 is selected
#ifdef CLKPORT
		bcf     CLKPORT,CLKBIT    ; Set clock output low
#endif
		bcf     INTCON,T0IF       ; Clear the RTCC interrupt bitflag

		movlw   RTCCINT
		addwf   TMR0,F            ; Reload RTCC

		incf    tmrCount,F        ; Increment free running interrupt counter

		call    SrvcULink         ; Perform up interface link service
		call    SrvcDLink         ; Perform 'down' interface link service

		MonitorISR

#ifdef CLKPORT
		bsf     CLKPORT,CLKBIT    ; Set clock output high
#endif

EndISR		movf    pclath_isr,W      ; Retrieve copy of PCLATH register
		movwf	PCLATH            ; Restore pre-isr PCLATH register contents
		movf    status_isr,W      ; Retrieve copy of STATUS register
		movwf	STATUS            ; Restore pre-isr STATUS register contents
		swapf   w_isr,F
		swapf   w_isr,W           ; Restore pre-isr W register contents

		retfie                    ; Return from interrupt


;**********************************************************************
; Main program code                                                   *
;**********************************************************************

Main		clrf    PORTA             ; Clear I/O ports
		clrf    PORTB

		BANKSEL OPTION_REG        ; Select register page 1

		movlw   PORTASTATUS       ; Program I/O port bit directions
		movwf   TRISA
		movlw   PORTBSTATUS
		movwf   TRISB

		clrf    OPTION_REG
		bsf     OPTION_REG,NOT_RBPU
		bsf     OPTION_REG,INTEDG
		bsf     OPTION_REG,PSA

		BANKSEL TMR0              ; Select register page 0

		movlw   PORTASTATUS       ; For Port A need to write one to each bit ...
		movwf   PORTA             ; ... being used for input

		; Initialise variables

		; Initialise serial link
		SerInit    serStatus, serUTmr, serUReg, serUByt, serUBitCnt, serUTmr, serUReg, serUByt, serUBitCnt
		call    EnableRxU
		call    InitRxU
		clrf    lnkUSte           ; Initialise 'up' link to enter receiving state
		call    EnableRxD
		call    InitRxD
		clrf    lnkDSte           ; Initialise 'down' link to enter receiving state

		; Perform user code initialisation
		call    UserInit

		; Initialise interrupts
		movlw   RTCCINT
		movwf   TMR0              ; Load RTCC
		clrf    INTCON
		bsf	INTCON,T0IE       ; Enable RTCC interrupts
		bsf	INTCON,GIE        ; Enable interrupts


		goto    MonitorMain       ; Run monitor program


;**********************************************************************
; User variable definitions                                           *
;**********************************************************************

		CBLOCK

direction	; Data direction mask, 0 - output    1 - input
activeLow	; Input active mask,   0 - high      1 - low
latched		; Input latched mask,  0 - unlatched 1 - latched

inData		; Processed input bits
seenData	; 'Seen' input bits
outData		; Output bits

		ENDC


;**********************************************************************
; User code                                                           *
;**********************************************************************

UserInit	; User initialisation code

		movlw   low EEdirection
		call    GetEEPROM
		movwf   direction
		BANKSEL OPTION_REG        ; Select register page 1
		movlw   TRISB             ; Program PORTB bit directions
		BANKSEL TMR0              ; Select register page 0
		movlw   low EEactiveLow
		call    GetEEPROM
		movwf   activeLow
		movlw   low EElatched
		call    GetEEPROM
		movwf   latched
		clrf    inData
		clrf    seenData
		clrf    outData

		return


UserInt		return                    ; User interrupt code


UserMain	; User main loop code

		comf    seenData,W        ; Set mask to clear 'seen' input bits
		andwf   direction,W       ; Mask out output bits
		andwf   inData,W          ; Get 'not seen' bits from input
		andwf   latched,W         ; Clear 'non latched', 'unseen' bits
		movwf   inData            ; Maintain any 'latched', 'unseen' bits
		clrf    seenData          ; Clear 'seen' bits register (all now 'unseen')

		movf    PORTB,W           ; Read port
		andwf   direction,W       ; Mask out output bits
		xorwf   activeLow,W       ; Invert 'active low' bits
		iorwf   inData,F          ; Combine any 'latched', 'not seen' bits

		comf    direction,W       ; Set mask to isolate output bits
		andwf   outData,W         ; Isolate output bits
		movwf   PORTB             ; Write output bits to port

		return


		end                       ; directive 'end of program'

