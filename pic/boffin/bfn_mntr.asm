;**********************************************************************
;                                                                     *
; Author: Chris White (whitecf69@gmail.com)                           *
;                                                                     *
; Copyright (C) 1999 by Monitor Computing Services Limited, licensed  *
; under CC BY-NC-SA 4.0. To view a copy of this license, visit        *
; https://creativecommons.org/licenses/by-nc-sa/4.0/                  *
;                                                                     *
; This program is distributed in the hope that it will be useful, but *
; WITHOUT ANY WARRANTY; without even the implied warranty of          *
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes: Simple low level monitor program offering terminal        *
;           interface via full duplex serial @ 4K8 baud.              *
;           1 start bit, 8 data bits, no parity, 1 stop bit           *
;                                                                     *
;           'Up' interface, asynchronous serial @ 4K8 baud            *
;           Port A 2 - Rx, 3 - Tx                                     *
;                                                                     *
;           'Down' interface, asynchronous serial @ 2K5 baud          *
;           Port A 0 - Data                                           *
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
; Constant definitions                                                *
;**********************************************************************

PORTASTATUS EQU     B'00000000' ; Initial port status, all bits set as output
PORTBSTATUS EQU     B'00000000' ; Initial port status, all bits set as output

RTCCINT   EQU     158         ; 1Mhz / 100 (adjusted for RTCC write inhibit) = 10Khz

INT2K5BIT EQU     4           ; Interrupts per serial bit @ 2K5 baud (approximately 2K4)
INT2K5INI EQU     6           ; Interrupts per initial Rx serial bit @ 2K5 (start bit)
INT5KBIT  EQU     2           ; Interrupts per serial bit @ 5K baud (approximately 4K8)
INT5KINI  EQU     3           ; Interrupts per initial Rx serial bit @ 5K (start bit)
INTLINKDEL  EQU     0           ; Interrupt cycles for link turnaround delays

INTMILLI  EQU     10          ; Interrupts per millisecond

CLKPORT   EQU     PORTA       ; Clock tick output port
CLKBIT    EQU     4           ; Clock tick output on Port A bit 4

; 'Up' interface constants
RXUFLAG   EQU     0           ; Receive byte buffer 'loaded' status bit
RXUERR    EQU     1           ; Receive error status bit
RXUBREAK  EQU     2           ; Received 'break' status bit
RXUSTOP   EQU     3           ; Seeking stop bit status bit
TXUFLAG   EQU     4           ; Transmit byte buffer 'clear' status bit
TXUBREAK  EQU     5           ; Send 'break' status bit

TXUTRIS   EQU     TRISA       ; 'Up' side Tx port direction register
TXUPORT   EQU     PORTA       ; 'Up' side Tx port data register
TXUBIT    EQU     3           ; Tx on Port A bit 3
RXUTRIS   EQU     TRISA       ; 'Up' side Rx port direction register
RXUPORT   EQU     PORTA       ; 'Up' side Rx port data register
RXUBIT    EQU     2           ; Rx on Port A bit 2

; 'Down' interface constants
RXDFLAG   EQU     0           ; Receive byte buffer 'loaded' status bit
RXDERR    EQU     1           ; Receive error status bit
RXDBREAK  EQU     2           ; Received 'break' status bit
RXDSTOP   EQU     3           ; Seeking stop bit status bit
;  Half duplex so Rx and Tx flags share status bits
TXDFLAG   EQU     RXDFLAG     ; Transmit byte buffer 'clear' status bit
TXDBREAK  EQU     RXDBREAK    ; Send 'break' status bit

TXDTRIS   EQU     TRISA       ; 'Down' side Tx port direction register
TXDPORT   EQU     PORTA       ; 'Down' side Tx port data register
TXDBIT    EQU     0           ; 'Down' Tx on Port A bit 0
;  Half duplex so Rx and Tx share same IO
RXDTRIS   EQU     TXDTRIS     ; 'Down' side Rx port direction register
RXDPORT   EQU     TXDPORT     ; 'Down' side Rx port data register
RXDBIT    EQU     TXDBIT      ; 'Down' Rx on Port A bit 0


;**********************************************************************
; Variable definitions                                                *
;**********************************************************************

    CBLOCK  0x0C

; Status and accumulator storage registers
pclath_isr  ; PCLATH register store during ISR
w_isr   ; 'w' register, accumulator, store during ISR
status_isr  ; status register store during ISR

; 'Up' side interface registers
serUStatus   ; Serial I/F status flags
serURxTmr    ; Interrupt counter for serial bit timing
serURxReg    ; Data shift register
serURxByt    ; Data byte buffer
serURxBitCnt ; Bit down counter
serUTxTmr    ; Interrupt counter for serial bit timing
serUTxReg    ; Data shift register
serUTxByt    ; Data byte buffer
serUTxBitCnt ; Bit down counter

; 'Down' side interface registers
serDStatus ; Serial I/F status flags
serDTmr    ; Interrupt counter for serial bit timing
serDReg    ; Data shift register
serDByt    ; Data byte buffer
serDBitCnt ; Bit down counter
lnkDSte    ; Link state register

    ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

    ORG     0x2100            ; EEPROM data area

EEdirection DE      00                ; Data direction mask, 0 - output    1 - input
EEactiveLow DE      00                ; Input active mask,   0 - high      1 - low
EElatched DE      00                ; Input latched mask,  0 - unlatched 1 - latched


;**********************************************************************
; Reset vector                                                        *
;**********************************************************************

    ORG     0x000             ; Processor reset vector
BootVector  goto    Main              ; Jump to beginning of program


;**********************************************************************
; Interrupt vector                                                    *
;**********************************************************************

    ORG     0x004             ; Interrupt vector location
    goto    BeginISR          ; Jump to interrupt service routine


;**********************************************************************
; Link and interface routine macro invocations                        *
;**********************************************************************

EnableRxU EnableRx  RXUTRIS, RXUPORT, RXUBIT
    return

InitRxU   InitRx  serUStatus, serURxTmr, serURxBitCnt, serURxReg, RXUFLAG, RXUERR, RXUBREAK, RXUSTOP
    return

SrvcRxU   ServiceRx serUStatus, serURxTmr, serURxBitCnt, serURxReg, serURxByt, RXUPORT, RXUBIT, INT5KINI, INT5KBIT, RXUERR, RXUBREAK, RXUSTOP, RXUFLAG

EnableTxU EnableTx  TXUTRIS, TXUPORT, TXUBIT
    return

InitTxU   InitTx  serUStatus, serUTxTmr, serUTxBitCnt, serUTxReg, TXUFLAG, TXUBREAK
    return

SrvcTxU   ServiceTx serUStatus, serUTxTmr, serUTxBitCnt, serUTxReg, serUTxByt, TXUPORT, TXUBIT, RXUPORT, RXUBIT, INT5KBIT, TXUFLAG, TXUBREAK

LinkMRx
SerURx    SerialRx serUStatus, serURxByt, RXUFLAG

LinkMTx
SerUTx    SerialTx serUStatus, serUTxByt, TXUFLAG

EnableRxD EnableRx  RXDTRIS, RXDPORT, RXDBIT
    return

InitRxD   InitRx  serDStatus, serDTmr, serDBitCnt, serDReg, RXDFLAG, RXDERR, RXDBREAK, RXDSTOP
    return

SrvcRxD   ServiceRx serDStatus, serDTmr, serDBitCnt, serDReg, serDByt, RXDPORT, RXDBIT, INT2K5INI, INT2K5BIT, RXDERR, RXDBREAK, RXDSTOP, RXDFLAG

EnableTxD EnableTx  TXDTRIS, TXDPORT, TXDBIT
    return

InitTxD   InitTx  serDStatus, serDTmr, serDBitCnt, serDReg, TXDFLAG, TXDBREAK
    return

TxDBreak  TxBreak  serDStatus, TXDBREAK
    return

SrvcTxD   ServiceTx serDStatus, serDTmr, serDBitCnt, serDReg, serDByt, TXDPORT, TXDBIT, RXDPORT, RXDBIT, INT2K5BIT, TXDFLAG, TXDBREAK

SerDRx    SerialRx serDStatus, serDByt, RXDFLAG

SerDTx    SerialTx serDStatus, serDByt, TXDFLAG

SrvcDLink SrvcLink   lnkDSte, serDTmr, INTLINKDEL, INTLINKDEL, EnableTxD, InitTxD, SrvcTxD, TxDBreak, EnableRxD, InitRxD, SrvcRxD

LinkHRx   LinkRx lnkDSte, SerDRx

LinkHTx   LinkTx lnkDSte, SerDTx


;**********************************************************************
; Monitor code                                                        *
;**********************************************************************

#include "../utility_pic/eeprom.inc"
#define MONUSERON
#include "../utility_pic/monitor.inc"


;**********************************************************************
; Interrupt service routine (ISR) code                                *
;**********************************************************************

BeginISR  movwf   w_isr             ; Save off current W register contents
    movf  STATUS,W          ; Move status register into W register
    movwf status_isr        ; save off contents of STATUS register
    movf  PCLATH,W          ; Move PCLATH register into W register
    movwf pclath_isr        ; save off contents of PCLATH register

    btfss   INTCON,T0IF       ; Skip if RTCC interrupt flag is set ...
    goto    EndISR            ; ... otherwise jump to end of service routine

    BANKSEL TMR0              ; Ensure register page 0 is selected
#ifdef CLKPORT
    bcf     CLKPORT,CLKBIT    ; Set clock output low
#endif
    bcf     INTCON,T0IF       ; Clear the RTCC interrupt bitflag

    movlw   RTCCINT
    addwf   TMR0,F            ; Reload RTCC

    call    SrvcRxU           ; Perform up interface Rx service
    call    SrvcTxU           ; Perform up interface Tx service
    call    SrvcDLink         ; Perform 'down' interface link service

    MonitorISR

#ifdef CLKPORT
    bsf     CLKPORT,CLKBIT    ; Set clock output high
#endif

EndISR    movf    pclath_isr,W      ; Retrieve copy of PCLATH register
    movwf PCLATH            ; Restore pre-isr PCLATH register contents
    movf    status_isr,W      ; Retrieve copy of STATUS register
    movwf STATUS            ; Restore pre-isr STATUS register contents
    swapf   w_isr,F
    swapf   w_isr,W           ; Restore pre-isr W register contents

    retfie                    ; Return from interrupt


;**********************************************************************
; Main program code                                                   *
;**********************************************************************

Main    clrf    PORTA             ; Clear I/O ports
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

    ; Initialise serial links
    call    EnableRxU
    call    InitRxU
    call    EnableTxU
    call    InitTxU
    call    EnableRxD
    call    InitRxD
    clrf    lnkDSte           ; Initialise 'down' link to enter receiving state

    ; Perform user code initialisation
    call    UserInit

    ; Initialise interrupts
    movlw   RTCCINT
    movwf   TMR0              ; Load RTCC
    clrf    INTCON
    bsf INTCON,T0IE       ; Enable RTCC interrupts
    bsf INTCON,GIE        ; Enable interrupts


    goto    MonitorMain       ; Run monitor program


;**********************************************************************
; User variable definitions                                           *
;**********************************************************************

    CBLOCK

direction ; Data direction mask, 0 - output    1 - input
activeLow ; Input active mask,   0 - high      1 - low
latched   ; Input latched mask,  0 - unlatched 1 - latched

inData    ; Processed input bits
seenData  ; 'Seen' input bits
outData   ; Output bits

    ENDC


;**********************************************************************
; User code                                                           *
;**********************************************************************

UserInit  ; User initialisation code

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


UserInt   return                    ; User interrupt code


UserMain  ; User main loop code

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
