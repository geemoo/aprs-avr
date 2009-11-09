


;=================================================================================================
; MODEMLESS PACKET RADIO Terminal Node Controller
;  c 2004 Bob Ball WB8WGA., See Notice at Bottom of File
; Amateur and Educational Use Encouraged
; Reproduction for Commercial Purposes  Prohibited Without Written Permission
; 
;
; Acknowledgements:
; This program builds on the work of many others.
; It was inspired by the previous work of Mike Berg, N0QBH, on modemless packet receiver design.
; Also thanks to Byon Garrabrant,John Hansen, and others for Work Published on PIC Packet Radio at TAPR site
;  Transmit tone generation interrupt routine from Son of TinyTrak (SoTT) by Mike Pendley, ; K5ATM




;
;=================================================================================================
; Designed to Run on a PIC16F88 at 20 mhz.
;=================================================================================================

; This program does packet receive and digipeat of standard AX25 packets at 1200 Baud. 
; It transmits AX25 beacons with user inputed text
; It will also transmit UI frames in conversation mode as addressed by UNPROTO.
; Serial Port may be connected to either a Terminal or GPS Unit. Controlled by Jumper J4
; With GPS Connected, sends NMEA sentences, as controlled by GPS command option
; MYCALL, UNPROTO, ALIAS, BTEXT,BEACON ON/OFF, & MONITOR ON/OFF parameters are inputted by user and 
; stored in EERAM after PERM command is entered

;=================================================================================================
;Program Organization
;The program consists of 8 processes, each of which are scheduled and called in round robin fashion.
;The processes are:
;		1) Command Interpreter
;		2) Packet Receive
;		3) Beacon Generator
;		4) Monitor
;		5) Software Timer
;		6) Digipeat
;		7) Converse
;		8) GPS
;
; The main loop for the process scheduler is at the END of the file. 
; Defines are first in the program, then followed by many subroutines

;The program runs on a PIC16F88 at 20 Mhz.
;=================================================================================================
	list    p=PIC16F88
	ERRORLEVEL	-302
	ERRORLEVEL	-202
	ERRORLEVEL	-306
    radix   hex
 	#include "P16F88.INC"  
          

;Program Configuration Register 1
		__CONFIG    _CONFIG1, _CP_OFF & _CCP1_RB0 & _DEBUG_OFF & _WRT_ENABLE_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _MCLR_OFF & _PWRTE_OFF & _WDT_OFF & _HS_OSC
;Note, Watch Dog timer will be turned on later in program
;Program Configuration Register 2
		__CONFIG    _CONFIG2, _IESO_OFF & _FCMEN_OFF
;------------------------------------------------------------------
; Some Key Defines
;

#define	XMIT_BUFFER_SIZE	.125		;Buffer to store chars during converse mode
#define	MAX_BEACON_TEXT	.100			;max size of beacon text
#define	BEACON_EE_START	.100	;where in EE data does beacon text start 

#define BAUD_VALUE	.129		;9600 baud, BRGH is set

;=====================================================================
;I/O PIN Layout
	;Comparitor inputs/outputs use up RA0 to RA3
	;#define AUDIO_IN		PORTA,0			;not used in program
	;RA1 is permanently grounded, configure as input
	;RA2 is permanently grounded, configure as input
	;RA3 is the comparitor output, configure as output
	;RA4 is not used
	;RA5 (Master Clear) is disabled and is used for GPS input 
	;RA6,RA7 are Oscillator  
	;
	;Now RB PORT
	;RB4 is used in at external interrupt input on any change (RBIF) 
	#define PTT_OUT			PORTB,0			;    
	#define SERIAL_IN		PORTB,2			;RECEIVE ON UART
	#define UART_TX			PORTB,5			;TX ON UART
	
	;Make RB4 an input
	;RB1, RB3,RB6, and RB7  are Audio Out
	#define TRISA_SET	b'11110111'				;Make these match with above
	#define TRISB_SET	b'00110100'				


;============================================================================
; Memory Management is Configured to Handle a full 255 byte packet
;----------------------------------------------------------------------------
;Memory Management Stuff
;Done by RWMWrite and Read
; Here are parameters
; Bank 0, Use 0x6e to 0x6f -> 0x2 	->.2 bytes		( rest is scratch variables)
; Bank 1, Use 0xA0 to 0xdd -> .62 bytes
; Bank 2, Use 0x110 to 0x16f -> 0x30 ->.96 bytes
; Bank	3, Use 0x190 to 0x1ef -> 0x30 ->96 bytes
;Thus have total 255 bytes to use
; Bank 0 get bytes 0 to 1
; Bank 1 gets bytes 2 to 63
; Bank 2 gets bytes 64 to 159
; Bank 3 gets bytes 160 to 255 ;spare byte here
MEM_0_ST		equ	.0
MEM_0_END		equ .1		;end of bank 0 memory
MEM_1_ST		equ .2
MEM_1_END		equ	.63
MEM_2_ST		equ	.64
MEM_2_END		equ .159
MEM_3_ST		equ	.160
MEM_3_END		equ	.255
MEM_0_ADDR_ST	equ	0x6e
MEM_1_ADDR_ST	equ	0xa0
MEM_2_ADDR_ST	equ	0x110
MEM_3_ADDR_ST	equ	0x190

#define	BOOT_DATA_SIZE BOOT_FLAGS1 - mycall	;data copied to EE RAM


;=============================================================================
;Macros to select the register bank
;Many bank changes can be optimized when only one STATUS bit changes

Bank0		MACRO			;macro to select data RAM bank 0
		bcf	STATUS,RP0
		bcf	STATUS,RP1
		ENDM

Bank1		MACRO			;macro to select data RAM bank 1
		bsf	STATUS,RP0
		bcf	STATUS,RP1
		ENDM

Bank2		MACRO			;macro to select data RAM bank 2
		bcf	STATUS,RP0
		bsf	STATUS,RP1
		ENDM

Bank3		MACRO			;macro to select data RAM bank 3
		bsf	STATUS,RP0
		bsf	STATUS,RP1
		ENDM
;===================================================================
;IF YOU CHANGE THE CLOCK SPEED, ADJUST THIS FOR CLOCK_SPEED CHANGE
; This program does some timing based on instruction counting in routine
;fixed_delay

FIXED_DELAY_LENGTH equ d'250'		;200 us at 20 MHz is .250
;=========================================================================
; Defines for Software Timer Block
; This uses Timer 1 which is at PreScale of 8. 
; Flag is set every 62.5 msec. Switch? counts these to make 15 seconds
;
#define	NUMBER_OF_SOFTWARE_TIMERS	2

;=========================================================================
;Miscellaneous defines	
	#define	TRUE		1
	#define FALSE		0
	#define	CR			0x0d		;carriage return
	#define LF			0x0a		;line feed
	#define	BS			0x8
	#define	CONTROL_C	0x3			;End of transmittion indicator
	#define	SPACE_BAR	0x20
	#define MATCH		0xff		;indicates found our command
	#define	ERROR		0			;Invalid pointer, command interpreter
;
	#define BASE12		.200		; Timer Values to get 1200/2200
	#define	BASE22		.229		; With Tweaks, get 1206/2202
	#define	TWEAK12		1			;
	#define	TWEAK22		1

;==============================================================================
; F register defines	

		cblock	0x20
		cbase1200			
		cbase2200			
		tweaktime12
		tweaktime22
		;*** Start of any vars that should be cleaned on any full init
		Rcd_Char
		timer_block:NUMBER_OF_SOFTWARE_TIMERS
		tic_counter
				
		;*** Start of vars that should  be cleaned up after successful rcv/ or xmit
		var_1                   ;various counts 
        var_2                   ;various counts
		var_3
		count
		temp
        byte_count              ;offset counter
        store_count             ;bytes stored cnt
        bit_stuff_count         ;packet bitstuff counter
        bit_count               ;packet bit counter
        loop_count              ;prgm loop counter	
		offset			;memory store offset
       	send_reg                ;serial data
        byte_in                 ;data byte rx'd
        AX25_crc_hi             ;upper byte CRC word
        AX25_crc_lo             ;lower byte CRC word
        next_to_last_byte       ;2nd from last byte
        last_byte               ;last byte rxed
        ssid_buffer             ;SSID storage (2f)
		unload_counter			;used when unloading received message
		aux_count				;used in digipeat decoding
		mirror_reg				;used in digipeat decoding
		rotor			; 32 rx data compare
		
		;*** Start of Configuration Data, Copied from eeram, set by command_interpreter
		mycall:7
		unproto:7
		first_via:7
		second_via:7
		;put more vias here
		last_via:7
		myalias:7
		last_callsign_ptr		;pointer to last valid callsign above
		timer_init_array:NUMBER_OF_SOFTWARE_TIMERS
		xmit_delay				;number of 200 msec tics to delay when xmitter keyed
		BOOT_FLAGS0
		BOOT_FLAGS1
		
		endc




		cblock	0x70		;Put any variables used by interrupt routine here
							;Prevents any page problems, can only be a total of 16
		w_tmp		
		stat_tmp
		PFLAGS0                   ;various flags
		PFLAGS1					;more flags
		SCHEDULE				;more flags, used by scheduler where timers are needed
		SCHEDULE1				;more flags, just used for scheduling any processes
		base
		r_val
		base_tweak
		tweak
		loop_ctr
		trace_rotor
		
		endc
	
;----------------------------------------------------------------
;The following defines are used by routine clean_up to know where to stop and stop
;Scratch end is always end but start can be varied. 
	#define RX_TX_START		var_1			;Variable to clean out after RX or RCV or packets
	#define SCRATCH_START	Rcd_Char		;ALWAYS MAKE THESE THE FIRST AND LAST TO BE CLEARED
	#define	SCRATCH_END		rotor			;Always end value
;----------------------------------------------------------------
;Here are a bunch of defines for receive and transmit
;================================================================
;First Flags
;Both PFLAGS0 and 1 can be read/set from any page
;they are cleared at init and between successful rcv/xmit sessions

#define	DATA_FLAG		PFLAGS0,0			;says got to data part of packet
#define	BUSY_FLAG		PFLAGS0,1			;busy channel indicator
#define	RAMH_FLAG		PFLAGS0,2			;bank 0,1/2,3 ALSO cleared by routine
#define	XMITTING		PFLAGS0,3			;indicates in normal transmit state								
#define	SENDa0			PFLAGS0,4			;tells packet sender to send a zero, initted by routine										
#define	T_TONE			PFLAGS0,5			;Transmit tone,low or high
#define	R_TONE			PFLAGS0,6	        ;rx freq (hi/lo)
#define	LAST_CALLSIGN	PFLAGS0,7			;Set when last call sign in address field


#define CONVERSE_MODE   PFLAGS1,0         	;set if in converse mode, not command mode
#define	TO_UPPER		PFLAGS1,1			;map lower case to upper case if set
#define	GPS_CONNECTED	PFLAGS1,2			;set if GPS receiver connected to serial port
#define	CHAR_TIMEOUT	PFLAGS1,3			;set if getc_sleep never saw a character 


;These 	flags are not cleared automatically and are loaded in from eeprom
;#define	SPARE			BOOT_FLAGS0,0
#define CRC_OFF			BOOT_FLAGS0,1		;Set to 1 if not doing CRC check on receive
#define	MONITOR_ME		BOOT_FLAGS0,2			;Used to monitor only packets addressed to me
#define	BEACON_ENABLE	BOOT_FLAGS0,3		;Says turn the beacon feature on, must still be scheduled
#define	MONITOR_ALL		BOOT_FLAGS0,4		;Says turn off the monitor function
#define	DIGIPEAT_ENABLE	BOOT_FLAGS0,5		;Says Turn off the digipeat function
#define VALID_MYCALL	BOOT_FLAGS0,6			;Mycall valid
#define	VALID_ALIAS		BOOT_FLAGS0,7			;Alias valid
#define VALID_UNPROTO	BOOT_FLAGS1,0			;Unproto Valid
#define	VALID_BTEXT		BOOT_FLAGS1,1			;Good beacon text
#define	ECHO			BOOT_FLAGS1,2			;Set to 1 if char to be echoed to terminal
#define	TRACE_XMIT		BOOT_FLAGS1,3			;These to indicate trace is on xmit and rcv
#define	TRACE_RCV		BOOT_FLAGS1,4			
#define	GPRMC			BOOT_FLAGS1,5			;Using $GPRMC sentences from GPS
#define	GPGLL			BOOT_FLAGS1,6			;Using $GPGLL sentences from GPS
#define	GPGGA			BOOT_FLAGS1,7			;Using $GPGGA sentences from GPS
;Scheduler Flags, used by Timer Manager and task manager.
;Cleared on Boot Only 
;Each Process has a flag that is set when to run.
;A timer may be associated with a process if needed. They are kept in SCHEDULE0,
;otherwise SCHEDULE1 is used

;Timed processes
;KEEP THESE IN SAME ORDER AS SCHEDULE MASK
#define	RUN_BEACON		SCHEDULE,0		;timed process
#define	RUN_DEBUG		SCHEDULE,1	;timed test process
;Non Timed
#define	RUN_INTERPRETER	SCHEDULE1,0
#define	RUN_CONVERSE	SCHEDULE1,1
;2 is spare
;3 is spare
#define	RUN_STIMER		SCHEDULE1,4
#define	RUN_DIGIPEAT	SCHEDULE1,5
#define	RUN_MONITOR		SCHEDULE1,6
#define	RUN_RECEIVE		SCHEDULE1,7


;

;------------------------------------------------------------

		
;------------------------------------------------------------
;TIMER 0 is used on receive to measure the zero crossing time
; It is also used on transmit, to generate the 32 sample sine wave

;Hardware Timer 0 and 1 values, change if clock changes
;EDITs NEEDED HERE IF CLOCK SPEED CHANGES
; Some Macros for Timer 0, used a receive freq detector in this design
; Timer 0 is an 8 bit timer, runs at Fosc/4 . Prescaler is available, set the value in the next line

; Routine gets interrupted twice per cycle (low to high and high to low)
; 1000 Hz is then 78
; 1200 Hz is then 65
; 2200 Hz is then 35
; 1700 Hz is then 45

CENTER_FREQ		equ	d'45'		;1700 Hz (TMR0/8)(10 mhz and PS of 16)

;------------------------------------------------------------
;TMR1 is the general purpose software timer. Generates an interrupt each 62.5 ms. 16 are counted
; by Switch? to make 1 second. Flag is polled.

;------------------------------------------------------------
;Timer 2 is the the Link Bit Rate Timer. It is run at Prescale of 16. Timer is set to either
;one or 1.5, depending on whether it is a bit transistion

;At 20 mhz, use same values but set post scale of 2/1
	#define ONE_BIT 	.129
	#define	ONE_5_BIT	.194
;Set the Pre and Post Scale at init time. Reset timer in Timer Manager
;Here is the define to poll the timer expired, Assumes used from Page 0

;==================================================================	
;Some packet definitions
PACKET_FLAG		equ	0x7e			;01111110
MINIMUM_LENGTH		equ	d'15'		;smallest packet in bytes
;------------------------------------------------------------
				

			
;==========================================================================================

		ORG     H'0000'
        goto    main
        ORG     H'0004'
		
;---------------------------------------------------------------------------------
; INTERRUPT ROUTINE STARTS HERE, always at location 4

        movwf   w_tmp         ;   Save state
        swapf   STATUS,w      
        movwf   stat_tmp      
		
		;Might have been in any memory bank so put us back in Bank 0. PCLATH should
		;OK since it was set in transmit setup
		Bank0
		;Take care of TMR0, doing sine wave generation
		;Added this instruction so numbers below are off by .4 us
		btfss	XMITTING		;see if we are xmitting
		goto	do_receive

        movf    base,w        ;Lets reset the timer first

        decfsz  tweak,f 
        goto    notweak        

        addlw   0xff          
        movwf   TMR0     
        movf    r_val,w       ;update the sine wave
        call    getsin        
        movwf   PORTB         

        movf    base_tweak,w   ;restore tweak
        movwf   tweak          

        goto    incloop       

notweak: movwf   TMR0         ;  No  tweak needed
        movf    r_val,w       
        call    getsin        
        movwf   PORTB         


incloop:

        incf    r_val,w       
        andlw   0x1f          
        movwf   r_val         

        incf    loop_ctr,f    
		
		bcf		INTCON,T0IF		;get ready to take another interrupt
;
int_restore   
		swapf   stat_tmp,w    ; Restore state and return
        movwf   STATUS        
        swapf   w_tmp,f       
        swapf   w_tmp,w       

        retfie                




;==================================================================
;END OF Main INTERRUPT ROUTINE
;===============================================================


;
; Here is the sine wave profile.  The indicated voltages are measured
; from the top of R5 and assume a nominal 5 volts out of the pic chip
; I/O lines
; If you want to calculate the output, Eo, use this formula
; Eo = (V7/RB7 + V6/RB6 + V5/RB3 + V4/RB1)/(1/RL + 1/RB7 + 1/RB6 + 1/RB3 +1/RB1)
;Value is  left shifted to be able to insert into high bites and skip RB4
;Note RA0 is always set, it is PTT keyup and should be set to keep transmitter keyed
getsin:
        addwf   PCL,f
;                       r-val   volts
        retlw  		0x4b   
        retlw  	 	0x49  
        retlw 		0x43  
        retlw      	0x41  
        retlw      	0x0b  
        retlw      	0x09  
        retlw     	0x03 
        retlw       0x03 
        retlw      	0x03 
        retlw       0x03   
        retlw       0x09 ; 10      
        retlw      	0x0b 
        retlw       0x41 
        retlw       0x43 
        retlw       0x49 
        retlw       0x4b 
        retlw       0x83 
        retlw       0x89 
        retlw       0x8b 
        retlw       0xc1 
        retlw       0xc3 
        retlw       0xc9 
        retlw       0xcb
        retlw      	0xcb
        retlw      	0xcb 
        retlw      	0xcb 
        retlw      	0xc9
        retlw       0xc3
        retlw      	0xc1
        retlw       0x8b
        retlw       0x89
        retlw       0x83

;===============================================================================


schedule_mask		;When a softwaretimer expired, schedule one or more tasks
		addwf	PCL,F
		dt		b'00000001'	;Timer0
		dt		b'00000010'

;================================================================
; bcd_hex is used by hex dump routine. Converts binary to hex ascii equivalent
bcd_hex
	addwf	PCL,F
	dt	"0123456789ABCDEF"
;=================================================================
;Now receive interrupt level stuff
do_receive	
	Bank0			

	btfsc	INTCON,RBIF		;Comparitor Change?
	goto	inte_handler
	goto	int_restore	
;
inte_handler
	movfw	PORTB
	nop
	bcf	INTCON,RBIF				;so we can take another one
	;
	;If temp is lower than center freq, freq is hi tone 
	movlw	CENTER_FREQ		;1700 Hz
	subwf	TMR0,W			;Timer0 Val - center freq
	btfss	STATUS,C		;Carry set on high freq
	goto 	hi_tone
lo_tone
	bcf	R_TONE			;freq < 1700 Hz, > 1000
	clrf	TMR0

	goto	int_restore
hi_tone
	bsf	R_TONE			;freq > 1700 Hz, < 2400
	clrf	TMR0
;
	goto int_restore
                           ;return to program
;--------------------------------------------------------------------
;      Subroutine                                     
;                flipout                  
;                                           
;
; This routine will cause the "other" tone to be
; generated.  Example, if 1200Hz tone is in progress
; then the next tone will be 2200Hz. Done
; by changing the base interrupt delay value.
; In NRZI, everytime a 0 is needed, flipout called
;    



flipout	btfss	T_TONE		; 9 cycles
		goto 	set1200
set2200	bcf		T_TONE

		movfw	cbase2200	;hi
		movwf	base
		movfw	tweaktime22
		movwf	tweak
		movwf	base_tweak

		goto	flipout2
set1200	bsf		T_TONE

		movfw	cbase1200
		movwf	base
		movfw 	tweaktime12
		movwf	tweak
		movwf	base_tweak
		
flipout2	return		
;---------------------------------------------------------------------------------------------
;---------------------------------------------------------------------------------------------

;------------------------------------------------------------------------------
; This subroutine will init all state needed to
; start generating a tone.  
; If channel is clear, Transmitter will be keyed and a  small delay added

init_xmit

			;Timer 0 is used to generate tones on xmit and time
			;bit times on xmit
			;runs without prescaler
			;Timer 0 Interrupt Will be turned on later via T0IE in INTCON
			Bank1
			
			movlw	b'11000000'				;Set PSA to get no prescale, also leave edge bit set
       		movwf   OPTION_REG              ;and set it TMR0/1, Ready to roll at PS of 1

			Bank0
			


								
; Got a clear channel?
; 

channel_check	
				movlw	.10
				movwf	temp
channel_again	call	look_busy
				btfss	BUSY_FLAG
				goto	go_ahead
				decfsz	temp,f		;safety check, don't hang forever
				goto	channel_again		;yes, wait
		
;  All clear, grab channel and do it
go_ahead	bsf	PTT_OUT			;turn on xmitter
			
			clrf	r_val		;start sine wave at zero
			;force a switch to load variables		
			call	flipout		;load up the parameters

			;PCLATH needs to be set correctly for the get sin lookup to work
			movlw	HIGH getsin
			movwf	PCLATH

			bsf		XMITTING	;Tells interrupt routine how to branch

			bcf	INTCON,T0IF		;Clear flag for first interrupt
			bsf	INTCON,T0IE		;Turn it on, tone should start
		
			return
;========================================================================================
init_vars		;any other clears to do

				movlw	BASE12		;init base values
				movwf	cbase1200
				movlw	BASE22
				movwf	cbase2200
				movlw	TWEAK12
				movwf	tweaktime12
				movlw	TWEAK22
				movwf	tweaktime22

				
				

				clrf	SCHEDULE		;No processes initially scheduled
				clrf	SCHEDULE1
				clrf	PFLAGS0			;and zero out pflags
				clrf	PFLAGS1

				call	Get_Params				;restore mycall, unproto, and BOOT_FLAGS

				;start both software timers.
				movlw	0	
				call	Start_timer
				movlw	1
				call 	Start_timer		;test, remove this later

				; A sanity check on the EE PROM data. May have been reset by programmer
				;and, if is this first time, may be 0xff. If so, clear out all the flags to
				;and just set defaults  until it is done in the command interpreter.
				;after reset is done, should never hit this again
				movlw	0xff
				xorwf	BOOT_FLAGS1,w		;could test any, this is the last one
				skpz
				return
				clrf	BOOT_FLAGS0
				clrf	BOOT_FLAGS1
				call	set_defaults		;and set defaults
				return

;Init hardware does PORTS and UART init, Receive and xmit init does anything special
;beyond that
;=========================================================================
init_hardware
				movlw   b'00000110'
				
												;Set up internal comparitor output from C1 on
												;RA3 is feed over to RB4 and generates an interrupt
												;on positive change. All this used to to a freq count
												; (1200/2200) on receive)


        			Bank1
				movwf   CMCON

;set digital and analog ANx pins
				movlw	b'00011111'
				movwf	ANSEL
					
       	 	
;Now do ports			
       			movlw	TRISA_SET
				 movwf   TRISA
				movlw	TRISB_SET
        		movwf   TRISB
				Bank0

				clrf	PORTA
				clrf	PORTB			;and zero the outputs to be sure off
			
				;do uart
				call	init_uart		

				movlw	b'00110001'		;Prescale of 8, Timer 1 on
				movwf	T1CON

				;Now timer 2
				movlw	b'00001111'		;PreS or 16 and Timer On, Post Scale of 2
				movwf	T2CON
				;Timer 2 running.. no interrupt enabled
                 ;
				;Get ready for pheripheral interrupt from uart,timers

				bsf		INTCON,PEIE		;Enable PEIE
				bsf		INTCON,GIE
		
				;Now start Watch Dog Timer. Assumes it was NOT set in the Config Word
				;First see if it went off, if so put out a special message
				btfsc		STATUS,4	;TO will be clear if it went off
				goto		no_timeout
				movlw		.14				;CR,LF
				call output_strings			;put it out
				movlw		.26				;CR,LF
				call output_strings			;put out WDT timeout occured
no_timeout
				Bank2
				movlw	b'00010111'			;sets prescaler for about 2 sec
				movwf	WDTCON
				Bank0

				return
;-----------------------------------------------------------
init_uart
				;do all setup of uart
				Bank1
       			BSF		TXSTA,BRGH
				movlw	BAUD_VALUE		;set data rate
				movwf	SPBRG			;async setup

				BSF		TXSTA,TXEN		;Enable xmit, polled
				Bank0
				BSF		RCSTA,SPEN
				BSF		RCSTA,CREN		;Enable, receive, polled
				return
	
;-----------------------------------------------------------
;Send out a beacon. If gps unit connected do some checking and then let nmea do it. 
beacon
		btfsc	BEACON_ENABLE
		goto	check_addresses					
		goto	beacon_return
		; mycall, btext, and unproto should be valid before sending
		;a beacon. via will be used if it is valid
check_addresses
		btfss	VALID_MYCALL
		goto	beacon_return
		btfss	VALID_UNPROTO
		goto	beacon_return
		;Is there a GPS unit connected? If so, gather the input and process

		btfss	GPS_CONNECTED
		goto	no_gps
		
		;Is a GPS so go ahead and process NMEA
		;turn the uart receive back on
		bsf		RCSTA,CREN

		;should be a stream of chars coming in
nmea_start_again
		call	getc_sleep
		btfsc	CHAR_TIMEOUT			;did a character come in or did we timeout?
		goto	beacon_return			;we timed out, just get out
is_it_start?
		;Sit here waiting around until you get a valid start of a NMEA sentence	
		movfw	Rcd_Char
		xorlw	'$'		;start of sentence?
		skpnz
		goto	buffer_nmea				;saw the $, start buffering			
		goto	nmea_start_again		;look again,no $ 
buffer_nmea
		call	load_tty_buffer		;load the buffer
		;check timeout again
		btfsc	CHAR_TIMEOUT
		goto	beacon_return		;timed out with partial sentence
		call	nmea_sum			;is it the right nmea sentence?
		movwf	temp				;could be either gprmc or gpgll
		btfss	GPRMC
		goto	try_gpgll
		movlw	'G'+'P'+'R'+'M'+'C'
		goto	do_nema_test
try_gpgll
		btfss	GPGLL
		goto	try_gpgga
		movlw	'G'+'P'+'G'+'L'+'L'
		goto	do_nema_test
try_gpgga
		btfss	GPGGA
		goto	beacon_return						;a sentence type is not set, get out
		movlw	'G'+'P'+'G'+'G'+'A'
do_nema_test
		xorwf	temp,w
		skpz
		goto	nmea_start_again

		;nmea complete input, sent it
add_btext
		;but first, turn the uart receive back off
		bcf		RCSTA,CREN
		call	send_a_header

		movlw	'$'						;Put out the $
        call    reg_packet_out			;send it

		;now do the body. Don't want to map these to upper case
		bcf		TO_UPPER
		clrf	offset		;now point back to start of text

send_data_loop
        call	unload_tty_buffer
			
		movfw	Rcd_Char
        call    reg_packet_out			;send it

		movlw	CR
		xorwf	Rcd_Char,w
		skpz
        goto    send_data_loop
		goto	end_of_beacon			;done, finish up

no_gps									;no gps found, do as normal beacon
		btfss	VALID_BTEXT
		goto	beacon_return
		call	send_a_header	
		;do the beacon text
		movlw	BEACON_EE_START
		movwf	var_1
beacon_loop
		movfw	var_1
		Bank2
		movwf	EEADR
		Bank3
		bsf		EECON1,RD		;pull the data from the EE RAM
		Bank2
		movf	EEDATA,w
		Bank0
		;w now has the character, there is a null at the end
		skpnz
		goto	end_of_beacon
		call	reg_packet_out
		incf	var_1,f
		goto	beacon_loop

end_of_beacon
        call finish_up
		call	key_up
beacon_return
		bcf	RUN_BEACON		;deschedule	
		return	

;==========================================================================

;===============================================================
send_a_header
;Using mycall and unproto, send a packet down to body
;Assumes these were checked before entering
		;get transmitter going and packet initialized
fire_up
		;TRACE CODE START
		;If this trace is set will not be transmitting, just printing the output
		;that would have been transmitted, so skip firing up the transmitter
		btfss	TRACE_XMIT
		goto	do_setup_of_xmit
		call	trace_init
		goto	do_addressee
		;TRACE CODE END
do_setup_of_xmit
		call 	init_xmit			
		call	ready_to_tx
		;Put out the address field using unproto and mycall
		;First is unproto as source, always with SSID bit of 0
do_addressee
		movlw	unproto			;do final destination first
		movwf	FSR
		bcf		LAST_CALLSIGN
		call	send_address

		;now mycall. If no additional digis are specified then LAST_CALLSIGN bit must be set.
		; Can found out by looking at last_callsign_ptr
		bcf		LAST_CALLSIGN		;preset it to off

		movlw	mycall
		movwf	FSR					;get FSR pointing to right place

		movlw	unproto
		xorwf	last_callsign_ptr,w
		skpnz
		goto	last_callsign		;just do mycall, set ssid bit on it and finish
		;more to do..
		call	send_address		;does mycall

		;now move onto the digis
		movlw	first_via
		movwf	FSR
go_for_next_digi
		xorwf	last_callsign_ptr,w
		skpnz
		goto	last_callsign
		call	send_address
		movlw	7
		addwf	FSR,f		;bump up to next digi
		movfw	FSR
		goto	go_for_next_digi
last_callsign
		bsf		LAST_CALLSIGN
		call	send_address

do_control
		movlw	0x3				;put in control fields
		call reg_packet_out
		movlw	0xf0
		call reg_packet_out
		
		return

;===============================================================
;Routine build an address field used on transmit
;does 7 bytes at a time, FSR should point to first byte call sign data to send.
;On entry, LAST_CALLSIGN bit will be set if last SSID bit should be set
;On return, leave the FSR pointing where it was at the beginning
send_address
		movlw	7
		movwf	aux_count
address_loop
		movfw	INDF
		movwf	temp	;ascii char to sent now in temp
		;Is this the last char  (SSID)?
		movlw	0x1
		xorwf	aux_count,W
		skpz
		goto	not_the_last
		;now doing the last byte in the address field
		clrc
		rlf		temp,f
		btfsc	LAST_CALLSIGN
		bsf		temp,0			;and set the last callsign bit
		movfw	temp
		call reg_packet_out		;send it
		movlw	6
		subwf	FSR,f
		return			;and get out
not_the_last	
		clrc
		rlf		temp,f
		movfw	temp
		call	reg_packet_out
		incf	FSR,f
		decf	aux_count,f
		goto address_loop
;
;-------------------------------------------------------------
;General Purpose Utilities for Setting Up a packet transmit
;Assumes transmitter is keyed and stable  tone is going
ready_to_tx	
        movlw   0xff
        movwf   AX25_crc_hi             ;reset CRC calc
        movwf   AX25_crc_lo

 ;Zero Out timer 1       
		movlw	ONE_BIT
		Bank1
		movwf	PR2
		Bank0					;Set up the link timer
		clrf	TMR2
        bcf     PIR1,TMR2IF             ;clear flag

		movfw	xmit_delay				;based on TXDELAY setting
        call    tx_flags                ;send flags for tx

		
	return
;------------------------------------------------------------
finish_up
        movlw   0xff
        xorwf   AX25_crc_hi,W
        movwf   temp                    ;inv, store hi byte
        movlw   0xff
        xorwf   AX25_crc_lo,W
        call    reg_packet_out
        movfw   temp
        call    reg_packet_out
        movlw   0x02
        call    tx_flags                ;send end flags
        btfss   PIR1,TMR2IF		;bit done?
        goto    $-1
        return
;------------------------------------------------------------
;------------------------------------------------------------
;		Looks for 26ms of quiet on Link 
;		Sets flag if noise seen

look_busy	
	bcf	BUSY_FLAG		;clear busy flag
	movfw	PORTB		;clears any mismatch on RB4 input
	bcf	INTCON,RBIF		;clear comparitor flag

	movlw	d'20'		;delay 20 msec
	call	delay_x_ms
	btfsc	INTCON,RBIF	;will be set if comparitor changed state

	bsf	BUSY_FLAG		;nope was noise, so busy

	return		

;------------------------------------------------------------
;Send a byte out
;               Assumes byte is in W
reg_packet_out
        movwf   send_reg                ;W to send_reg
		clrwdt							;clear watch dog
		;TRACE POINT START
		;If TRACE xmit only  is enabled, print out the byte to be sent
		btfsc	TRACE_XMIT
		goto	just_trace
		;TRACE POINT END
        movlw   0x08
        movwf   bit_count               ;8 bits
rotor_out
        rrf     send_reg,W              ;calc CRC
        call    crc_core				;
        rrf     send_reg,F              ;get bit to send
        skpc
        goto    zero_out
		bcf		SENDa0
        incf    bit_stuff_count,F       ; b_s_c +1
        call    tx_one_bit              ;send a 1
        movfw   bit_stuff_count         ;check b_s_c
        xorlw   0x05
        skpnz                           ;b_s_f = 5 ?
        goto    zero_out                ;yes
count_the_bits
        decfsz  bit_count,F
        goto    rotor_out
        return
zero_out
        clrf    bit_stuff_count         ;reset b_s_f
       	bsf		SENDa0
        call    tx_one_bit              ;send a 0
        goto    count_the_bits          ;
;TRACE CODE START
just_trace
		movfw	send_reg
		call	trace
		return
;TRACE CODE END
;
;------------------------------------------------------------
;Send Some Flags
; On entry, w has the number of flags to send
;               Sends (7e) b'01111110' w  times
tx_flags
		movwf	var_3
tx_flags_again
        movlw   0x06
        movwf   bit_count               
        bsf		SENDa0
        call    tx_one_bit              ;send 0
flag_twist
		bcf		SENDa0
        call    tx_one_bit              ;send 6 1's
        decfsz  bit_count,F
        goto    flag_twist
        bsf		SENDa0
        call    tx_one_bit              ;send 0
        decfsz  var_3,F
        goto    tx_flags_again
        return                          ;end of flags
;
;------------------------------------------------------------
;       Assumes data bit is in Carry (About 10)
crc_core
        movlw   1
        skpnc
        xorwf   AX25_crc_lo, F          ; crc calculation
        clrc
        rrf     AX25_crc_hi, F
        rrf     AX25_crc_lo, F
        skpc
        return
        movlw   H'08'
        xorwf   AX25_crc_lo, F
        movlw   H'84'
        xorwf   AX25_crc_hi, F
        return
;
;------------------------------------------------------------
; Send one bit
; 

tx_one_bit 
		clrwdt                             
        btfss   PIR1,TMR2IF             ;TMR1 timed out?
        goto    tx_one_bit              ;no
        bcf     PIR1,TMR2IF             ;clear flag 
		btfsc	SENDa0	;2
		call	flipout		;flip the freq (adds 9) 11
nochange	return

;-------------------------------------------------------------
; Stops xmitting. Turns transmitter off and stops Timer 0 interrupt
key_up
		bcf	INTCON,T0IE	;turn off interrupt
		nop				;in case one pending

;
		bcf PTT_OUT		;unkey transmitter
;

		return
;-------------------------------------------------------------	
; Start of receive routines, first is the receive main loop
;
;
;------------------------------------------------------------
; Named registers are cleared and the program looks for the 1st
; packet flag to syncronize on.
;
;===============================================================
;START OF RECEIVE
;------------------------------------------------------------
;       Packet bit receive routine
; If tone change seen, returns right away with carry set to 0
; If tone doesn't change in timeout period, returns with carry set to 1
;	Timed with TMR2	
;	1 bit = 832 uS							
;     
; If it catches a tone change, sets timer to 1.5 normal amount to get synced 
;Otherwise, resets time to 1.0 bit time
 
recv_one_bit 
                         
        btfsc   R_TONE			;test rx freq, will be set by interrupt routine


		goto	tone_set
tone_clear
		btfss	rotor,0	
		goto 	no_change		;rotor bit = 0
		incf	rotor,F			;toggle rotor:0
        goto    bit_edge                ;saw tone change	
tone_set
		btfsc	rotor,0
		goto	no_change
		incf	rotor,F			;toggle rotor:0
		goto	bit_edge	
no_change
        btfss   PIR1,TMR2IF		;no change, check timeout
        goto    recv_one_bit            ;not timed out

		
		movlw	ONE_BIT		;Reset for one bit
		Bank1
		movwf	PR2
		Bank0
		clrf	TMR2
		bcf		PIR1,TMR2IF		;reset TMR1 ovrfl flag
        setc                            ;no toggle,carry is set
        return
		
bit_edge
		movlw	ONE_5_BIT		;Reset for one and half bit
		Bank1
		movwf	PR2
		Bank0
		clrf	TMR2
		bcf		PIR1,TMR2IF		;reset TMR1 ovrfl flag
        clrc                            ;no toggle,carry is set
        return

;------------------------------------------------------------
;	Sends  data w/USART module
; 	Byte should be loaded in w

do_serial_send
        btfss   PIR1,TXIF		;wait for clear register	
      	goto    $-1
		movwf	TXREG			;send it
		clrwdt					;clear watchdog
		return
;=======================================================================
; Routine trace is called with w register loaded with a byte to be displayed. 
;Output the byte using hex_dump and put out a Space. Every 16 bytes, put out a 
;CR,LF
trace
	call	hex_dump		;print it
	movlw	' '
	
	call	do_serial_send	;put in a space

	incf	trace_rotor,f
	movlw	0xf
	andwf	trace_rotor,w
	skpz
	return
	call	send_cr_lf
	return
;=======================================================================
; Trace init just sets up the required things for trace to work
trace_init
	clrf	trace_rotor		;start 16 on new line
	call	send_cr_lf
	return
;======================================================================
;send_cr_lf
send_cr_lf
	movlw	CR
		
	call	do_serial_send
	movlw	LF
	
	call	do_serial_send
	return
;------------------------------------------------------------
; Stores up to 255 bytes of data in 3 separate sections of on board RAM.
; Routine counts the bytes and selects which bank of memory to use.
; Both writes and reads use indirect addressing for memory access.
;
RAMWrite
	movfw	offset			;check if offset in range
	sublw	MEM_3_END			;test for too big
	skpc
	return
	incf	byte_count,F		;< in range, continue AND increment byte_count
	goto	RW_OK
RAMWrite_skip_count
	movfw	offset			;check if offset in range
	sublw	MEM_3_END			;test for too big
	skpc
	return
								;OK, in range and DONT increment byte_count
RW_OK	
	call	RAM_Calc		;sort which bank to store in
	btfsc	RAMH_FLAG		;store in bank 2,3?
	goto	hi_write		;yes
	movfw	next_to_last_byte	;no
	movwf	INDF			;store W at indirect offset
	return
hi_write
	movfw	next_to_last_byte
	bsf	STATUS,IRP		;bank 2,3
	movwf	INDF			;store W at indirect offset
	bcf	STATUS,IRP		;bank 0,1
	bcf	RAMH_FLAG		;clear flag
	return
RAMRead
	decf	byte_count,F		;count the bytes out
RAMRead_skip_count
	call	RAM_Calc		;sort which bank to read from
	btfsc	RAMH_FLAG		;read from bank 2 or 3?
	goto	hi_read			;yes
	movfw	INDF			;no, read from indirect offset
	return
hi_read
	bsf	STATUS,IRP		;bank 2,3
	movfw	INDF			;read from indirect offset
	bcf	STATUS,IRP		;bank 0,1
	bcf	RAMH_FLAG		;clear flag
	return
;
RAM_Calc				;calcs FSR from offset
;
	bcf	RAMH_FLAG		;default to low bank, will set otherwise bel
	movfw	offset
	sublw	MEM_0_END			
	skpnc
	goto	mem_0			
	movfw	offset
	sublw	MEM_1_END		
	skpnc
	goto	mem_1				
	movfw	offset
	sublw	MEM_2_END		
	skpnc
	goto	mem_2
	;must be bank 3
	goto	mem_3			
	;can't overflow since 8 bit offset
;
mem_0
	movfw	offset			
	addlw	MEM_0_ADDR_ST	
	movwf	FSR		
	return
mem_1
	movfw	offset		;how far from starting offset?
	movwf	temp
	movlw	MEM_1_ST
	subwf	temp,w			
	addlw	MEM_1_ADDR_ST		;and what is it address?	
	movwf	FSR	
	return
mem_2
	movfw	offset
	movwf	temp
	movlw	MEM_2_ST
	subwf	temp,w
	addlw	MEM_2_ADDR_ST
	movwf	FSR
	bsf	RAMH_FLAG		;use bank 2
	return
mem_3
	movfw	offset
	movwf	temp
	movlw	MEM_3_ST
	subwf	temp,w
	addlw	MEM_3_ADDR_ST
	movwf	FSR
	bsf	RAMH_FLAG		;use bank 3
	return
;
;------------------------------------------------------------
delay_200us                            ;200 uS
        movlw   FIXED_DELAY_LENGTH      ;4 instructions/loop

        movwf   var_2
	nop
        decfsz  var_2,F
        goto    $-2
        return
;------------------------------------------------------------
delay_0.2_sec                       ;Gives 0.2 sec per each value of w
        movwf   var_3
		movlw	.200			;gives 200 ms
        call    delay_x_ms		;wait 1 ms per count, thus .2 sec
        decfsz  var_3,F
        goto    $-3
        return
;------------------------------------------------------------
delay_x_ms					;gives 1 ms for each value of w
 		movwf   var_1
        call    delay_200us		;wait 200 uS per count
		call    delay_200us	
		call    delay_200us	
		call    delay_200us	
		call    delay_200us	
        decfsz  var_1,F
        goto    $-6
        return
;------------------------------------------------------------
;       Stores all packet data as received
;
AX25_CRC_calc
;
		
        call    RAMWrite                ;store in RAM
		incf	offset,F		;inc store offset
;
        movlw   0x08
        movwf   bit_count               ;8 loops
crc_head
        rrf     next_to_last_byte,F     ;get bit for CRC
        call    crc_core                ;do CRC calc
        decfsz  bit_count,F             ;8 counts
        goto    crc_head
        return
;------------------------------------------------------------

;------------------------------------------------------------
;This routine takes the received packet and prints it out
;------------------------------------------------------------
;       Assumes offset is correct
;
parse_callsign
        movlw   0x06
        movwf   loop_count              ;6 loops
parse_callsign_loop	
        call    RAMRead                 ;get byte from RAM
        movwf   temp                ;byte temporarily in temp
        clrc
        rrf     temp,F              ;right shift to normal
        movfw   temp
        xorlw   ' '                     ;space ?
        skpnz
		goto	skip_sending_it			;yes, skip next
		movfw	temp	
        call    do_serial_send          ;no, send byte
skip_sending_it
		incf	offset,F					;inc offset
        decfsz  loop_count,F            ;loop count - 1
        goto    parse_callsign_loop
;					
		call	RAMRead 					;get stored SSID byte
		movwf	ssid_buffer	                                
        rrf     ssid_buffer,F           ;right shift byte
        movlw   0x0f
        andwf   ssid_buffer,F           ;mask hi nibble
        skpnz
        goto	ssid_is_zero            ;ssid = 0
        movlw   '-'                     ;add dash
        call    do_serial_send          ;send -
        movlw   d'10'                   ;10
        subwf   ssid_buffer,W           ;test magnitude
        skpc
        goto    send_the_ones           ;SSID < 10
        movwf   ssid_buffer             ;ssid buffer => 10
        movlw   "1"                     ;ascii 1 for tens
        call    do_serial_send          ;send 10
send_the_ones
        movfw   ssid_buffer             ;ssid to W
        addlw   0x30                    ;ascii conver
        call    do_serial_send
ssid_is_zero							;do asteric routine
	movfw	var_2
	xorwf	offset,W					;check for loc match to
	bnz	$+3								;sending digi
	movlw	"*"
	call	do_serial_send				;send * to denote 
        return                          ;sending digi
;------------------------------------------------------------
;Process for Receiving packets
Packet_Receive
	
		bcf	XMITTING		;reset flag to allow interrupt to do freq counting on receive
		bcf	INTCON,T0IE		;Turn interrupt off for Timer 0, we will poll the timer in receive
		; Have to change the rate on timer 0 since will be using it to measure the time
		;between zero crossings
		;So, get timer 0 running at correct prescale
		Bank1
		movlw	b'10000100'				
        movwf   OPTION_REG              ;and set it TMR0/16
		Bank0
		bcf	INTCON,RBIF		;make sure flag cleared
		bsf	INTCON,RBIE		; Start Up the interrupt for the zero crossing detector
							;GIE should already be enabled

;Start of basic receive loop, stay in it until have to do any xmitting (beacon, digipeat,
;text in, etc		
	
house_clean                             ;new packet

		movlw	RX_TX_START
		movwf	FSR
		
		call	clean_up				;clean up all scratch memory
		;clear the data flag
		bcf		DATA_FLAG
		
		
        movlw   0xff
        movwf   AX25_crc_hi             ;reset CRC calc
        movwf   AX25_crc_lo
		;Set up link bit timer1 to one bit time to try to get synced up
		clrf		TMR2
		movlw	ONE_5_BIT
		movwf	TMR2
						
		bcf	PIR1,TMR2IF		;reset TMR2  flag

		;All set up to start receiving, just sit here looking
find_start_flag				;look for 7e b'01111110'
	
		;got a little time here while waiting for a valid flag
		;so, check to see if anyone else needs some CPU cycles
		;will do a switch if a character has arrived or timer has expired
		call	switch?
		skpnc
		goto	give_it_up						;Give up control if someone else needs it
	
		;Nothing found keep looking for a flag
        call    recv_one_bit            ;check for toggle
        rrf     byte_in,F               ;roll carry in (0 or 1)
        movfw   byte_in                 ;byte_in to W
        xorlw   PACKET_FLAG             ;is it a flag ?
        skpz

        goto    find_start_flag         ;no
	
;------------------------------------------------------------
; This routine creates a received byte from 8 received bits.
; no tone change (1) setc	tone change  (0) clrc
; The carry bit is rotated into byte_in 8 times.
;
get_next_byte				;saw start flag
        clrf    byte_in                 ;clear input buffer
        movlw   0x08
        movwf   bit_count               ;get 8 bits
get_next_bit
        call    recv_one_bit
        skpc
        goto    zero_in                 ;0 rx'd
        incf    bit_stuff_count,F       ;inc counter, 1 rx'd
roll_em_in
        rrf     byte_in,F               ;store carry bit
        movfw   bit_stuff_count
        xorlw   0x05                    ;5 1's in a row ?
        skpnz
        goto    check_stuff             ;yes
count_em_out                            ;no
        decfsz  bit_count,F             ;8 bits ?
        goto    get_next_bit            ;no
        goto    CRC_save_store          ;yes
zero_in
        clrf    bit_stuff_count         ;0 seen, clear bit_stuff_count
        goto    roll_em_in		;store the zero

; Bit stuff decode routine
; This routine handles the bit stuffing which occurs when more
; than 5 bits elapse without a tone change (1's). The sender
; inserts a 0 that is discarded on the receive end. 
; Flags are also detected here (01111110).					
;
check_stuff
        call    recv_one_bit            ;wait 1 bit
        skpnc                           ;if data=0, skip next
        goto    sixth_one               ;unchanged, flag ?
        clrf    bit_stuff_count         ;0 seen, clear count
        goto    count_em_out            ;don't count this bit
sixth_one
        call    recv_one_bit            ;get 6th bit
        skpnc                           ;0 = flag ?
        goto    house_clean             ;no, 7th 1, reset
        movfw   bit_count
        xorlw   0x03					;is the 0 in
        skpz                            ;the right place ?
        goto    house_clean             ;no
        clrf    bit_stuff_count         ;yes, flag, clear bstc
        btfss   DATA_FLAG               ;header/tail flag ?
        goto    get_next_byte           ;header flag
        movfw   store_count             ;tail flag
        sublw   MINIMUM_LENGTH          ;how long is packet ?
        skpnc
        goto    house_clean             ; < 16 bytes rx'd
        goto    packet_end              ;it's long enough
;
; Two bytes are buffered before starting CRC calculations.
; When the end flag is seen, the buffered bytes become the
; sent checksum.
;
CRC_save_store
        btfss   DATA_FLAG               ;test data flag		;CHECK THIS OUT!
        bsf     DATA_FLAG               ;set data flag
		incf	store_count,F			;buffer byte count
		movfw	store_count
        xorlw   0xff                    ;how long ?
        skpnz
        goto    house_clean             ;255 bytes, reset
        movfw   store_count             ;how long again
        sublw   0x02
        skpc                            ; >2 ?
        call    AX25_CRC_calc           ;calc CRC, store data
        movfw   last_byte               ;last byte to W
        movwf   next_to_last_byte       ;to 2nd to last
        movfw   byte_in                 ;get new data byte
        movwf   last_byte               ;store in last byte
        goto    get_next_byte           ;get next byte
;
;------------------------------------------------------------
;       Invert CRC data and compare to value sent
;		
packet_end
				
		;ending calculation

		btfsc	CRC_OFF
		goto	terminus		;skip CRC test
        movlw   0xff
        xorwf   AX25_crc_lo,F
        xorwf   AX25_crc_hi,F           ;invert  CRC data
        movfw   next_to_last_byte       ;check lobyte match
        xorwf   AX25_crc_lo,W
        skpz
        goto    house_clean             ;mismatch, reset
        movfw   last_byte
        xorwf   AX25_crc_hi,W           ;check hibyte match
        skpz
        goto    house_clean             ;mismatch, reset
terminus
		movlw	d'2'
		subwf	store_count,F		;realign total count
		;valid packet received. Give up control and schedule digipeat and monitor
		;processes to do their thing
		bsf	RUN_MONITOR
		bsf	RUN_DIGIPEAT


;Give up control
give_it_up	bcf	INTCON,RBIE				;turn off the interrupt
			return						;back to round robin

;END OF RECEIVE
;
;===============================================================
;MONITOR PROCESS
;Outputs the packet that was received
;Is scheduled by the receive process and runs once
;Assumes valid packet is in RAM starting at offset 0
;Should not trash the packet in anyway
;------------------------------------------------------------
; Check the digi SSID's for the last "H bit" set which indicates
; the sending digi.
;
monitor
		;Just run once so deschedule
		bcf		RUN_MONITOR
		
		; Check if monitor is enabled. Either Monitor ALL or ME must be set
		btfsc   MONITOR_ALL
		goto	proceed_with_output
		
		;see if monitor me is set, ie just look at packets addressed to me
	
		btfss	MONITOR_ME
		return		;neither all or me, must be off

		;doing match me, destination must match mycall or alias
	
		btfss	VALID_MYCALL			;mycall must be valid
		return							;else get out
		clrf 	offset					;point at mycall
		movlw	mycall
		movwf	FSR	
		call	call_sign_match
		skpc
		goto	check_alias_me				;that didn't match, try alias
		goto	proceed_with_output			;matched on callsign, proceed
check_alias_me
		btfss	VALID_ALIAS		;if alias is valid, check it
		return	;callsign didn't match.. alias is not valid, so get out
		clrf	offset			;restore pointer to destination 
								;and do match again
		movlw	myalias
		movwf	FSR	
		call	call_sign_match		;check alias
		skpc
		return		;neither matched.. get out

proceed_with_output

		;START OF TRACE CODE
		;Is receive trace on?
		btfsc	TRACE_RCV
		goto	trace_only
		;End of TRACE CODE
		
		
		;first find the sending digi
		movlw	d'255'
		movwf	var_2	  	        ;reset sending digi offset  
		movlw	d'13'
		movwf	offset			;org address SSID loc
;
fsd_loop_up
		call	RAMRead_skip_count
        movwf   ssid_buffer
        btfsc	ssid_buffer,0
		goto	mark_last_digi		;found last digi
		movlw	d'7'
		addwf	offset,F		;next SSID
		goto	fsd_loop_up
mark_last_digi
		movfw	offset
		movwf	var_1			;end of addresses in memory
fsd_loop_down
		movlw	d'13'
		xorwf	offset,W
		bz		read_back_data		;heard direct
		btfsc	ssid_buffer,7		;bit 7 is "H" bit
		goto	sd_found
		movlw	d'7'
		subwf	offset,F
		call	RAMRead_skip_count
        movwf   ssid_buffer
		goto	fsd_loop_down
sd_found
		movfw	offset
		movwf	var_2	 	        ;store loc of sending digi SSID					
;------------------------------------------------------------
read_back_data                          ;send data serially
        movlw   0x07                    ;start of org call
        movwf   offset		        	;in memory
;
        call    parse_callsign			;send org call & SSID
;
        movlw   '>'                     ;insert r_arrow
        call    do_serial_send          ;send >
;
        clrf    offset		       		;start of destination call
;
        call    parse_callsign			;send destination call & SSID
;
		movfw	var_1
		xorlw	d'13'					;end of addresses?
		bnz	first_digi					;no
        movlw   0x0d                    ;yes, setup for info field
        movwf   offset                  ;starting address skips packet
        goto    info_field_begin        ;type bytes
first_digi
        movlw   ','                     ;send 1st digi
        call    do_serial_send          ;send comma
        movlw   d'14'
        movwf   offset                  ;start of 3rd call (1st digi)
;
display_digipeaters
        call    parse_callsign          ;parse / send next
;
		movfw	var_1
		xorwf	offset,W
        bz      info_field_begin        ;end of addresses
        movlw   ','                     ;next address
        call    do_serial_send          ;send comma 
;
		incf	offset,F				;align w/next callsign
        goto    display_digipeaters
;
info_field_begin
        movlw   ':'
        call    do_serial_send          ;send :
		movlw	d'2'
		addwf	offset,F				;account for two
		subwf	byte_count,F			;skipped bytes
info_field_loop
		movfw	byte_count				;end of data?
        bz	wrap_it_up					;yes
		incf	offset,F				;no, keep reading
        call    RAMRead
        call    do_serial_send          ;send byte
        goto    info_field_loop         ;end not seen
;Start of TRACE_CODE
trace_only
		clrf	offset					;start at beginning and dump all
		call	trace_init
trace_output_loop
		movfw	byte_count				;end of data?
        bz		wrap_it_up					;yes
        call    RAMRead
		incf	offset,F				;no, keep reading
        call    trace          			;send byte
        goto    trace_output_loop       ;end not seen
;End of TRACE CODE
wrap_it_up
        movlw   0x0d                    ;>, send end
        call    do_serial_send
        movlw   0x0a                    ;<LF>
        call    do_serial_send
        return
;------------------------------------------------------------
;=============================================================
; Digipeat Process Routines
;------------------------------------------------------------
;=============================================================
;Routine digipeat repeats packet if we are the digipeater. Relies
;on packet to be in buffer from receive routine
;Must also have valid mycall, valid alias is optional
digipeat
		bcf	RUN_DIGIPEAT	;just run once
		;See if digipeating is enabled
		btfss	DIGIPEAT_ENABLE
		return		;nope, get out
		;must have a valid mycall to digipeat, if not just get out
		btfss	VALID_MYCALL
		return		;nope, get out
		
		;Alias will be used if it is valid, otherwise ignored

		
		;byte_count has the number of bytes in the received packet
		; don't trash
		;variable offset is used to get to right part of packet
		movfw	byte_count		;save byte counter in temp variable
		movwf	unload_counter

valid_digi_check
        movlw   .13
        movwf   offset
        call    RAMRead_skip_count
        movwf   ssid_buffer
        btfsc   ssid_buffer,0           ;any digis ?
        goto    digipeat_return             ;no
        movlw   .20                    ;yes
        movwf   offset           ;1st digi ssid

F_flag_loop
        call    RAMRead_skip_count
        movwf   ssid_buffer             ;W to ssid buffer
        btfss	ssid_buffer,7
		goto	locate_address			;f bit not set.. see if I am
										;to digipeat
		btfsc	ssid_buffer,0
		return							;f and ssid both set, do nothing. This packet has already
										;been done on all digis
		;f bit is set and ssid clear, bump up to next address and
		;try again
        movlw   0x07                    ;F is set.Bit 0 wasn't. 
										;So this was already digipeated. go to next address
        addwf   offset,f
        sublw   .68					;8 digis max allowed plus source and destination is 70 bytes
        skpc                            ; 
        return			             ;yes
        
        goto    F_flag_loop
locate_address
		
        movlw   0x06                    ;no
        subwf   offset,F         ;go back 6 bytes
		;offset now should be pointing at start of sign call of next digipeater
		;also save it in a temp variable for later use
		movfw	offset
		movwf	var_1

do_the_match
		movlw	mycall
		movwf	FSR	
		call	call_sign_match
		skpc
		goto	check_alias_match
		goto	set_f_bit			;matched on callsign , set f bit and go
check_alias_match
		btfss	VALID_ALIAS		;if alias is valid, check it
		return	;callsign didn't match.. alias is not valid, so get out
		movfw	var_1
		movwf	offset			;restore pointer to start of first digipeater
								;and do match again
		movlw	myalias
		movwf	FSR	
		call	call_sign_match		;check alias
		skpc
		return		;neither matched.. get out
;  Alias matched, now overwrite callsign with my alias

overwrite_address
        movfw	var_1
		movwf	offset			;restore the pointer to start of the address
        movlw	7
		movwf	aux_count		;do all 7 characters
		movlw	mycall
		movwf	FSR				;point at mycall

my_call_loop
		movfw	INDF			;get byte
		movwf	mirror_reg
        clrc
        rlf     mirror_reg,W            ;left shift 1
		movwf	next_to_last_byte		;RAMWrite requires it in this register
		movfw	FSR
		movwf	var_2		;temp save FSR so RAMWrite doesn't trash it
		;is bit 0 set on the SSID? if so don't overwrite it
		call	RAMRead_skip_count	
		andlw	1
		iorwf	next_to_last_byte,f		;resets it if set				
        call    RAMWrite_skip_count		;puts it back in the buffer
		movfw	var_2
		movwf	FSR			;restore FSR
		decfsz	aux_count,f
		goto	overwrite_next
		goto	set_f_bit
overwrite_next
        incf    offset,F
        incf    FSR,f
        goto    my_call_loop
set_f_bit
		movfw	var_1
		addlw	6								;pointing at SSID digi callsign
		movwf	offset
        call    RAMRead_skip_count             ;get SSID
        iorlw   0x80                    ;set F bit
		movwf	next_to_last_byte		;RAMWrite requires it in this register
        call    RAMWrite_skip_count
transmit_set_up

      	call		init_xmit
		call	ready_to_tx
;
        clrf    offset           ;start of data loc
        
       
transmit_data_loop
        call    RAMRead_skip_count
        call    reg_packet_out
		incf	offset,f
        decfsz  unload_counter,F            ;count down to 0
        goto    transmit_data_loop
		;Wrap it up
        call finish_up
       
digipeat_return
		call	key_up					;shut down xmitter
		return
;============================================================
;=======================================================================
;called with offset to address field in received packet	
;and FSR set to the address parameter being matched
;return carry set if found and clear if not
;=======================================================================							
call_sign_match
		movlw	7
		movwf	aux_count		;Always look at all 7 bytes
look_again
		movfw	INDF				;get a byte from parameter field
        movwf   mirror_reg              ;and store it
		;save FSR, Ram read uses it. will restore when return
		movfw	FSR
		movwf	count

        clrc                            
        rlf     mirror_reg,F					;right shift it,since received data is shifted
        call    RAMRead_skip_count             ;read 1 byte of received data
		;w had the byte received. It may bit 0 set if it is the 
		;last SSID. So clear it in case
		movwf	byte_in
		bcf		byte_in,0
		movfw	byte_in

        xorwf   mirror_reg,W            ;same ?
        skpz
        goto    cs_match_failed            ;no match

		;restore FSR
		movfw	count			;match
		movwf	FSR
		
        
		decfsz	aux_count,f		;all 7 match?
        goto    some_more
		setc	
		return
some_more
		incf    offset,F         
        incf	FSR,f
		goto look_again
cs_match_failed
		clrc
		return       
;------------------------------------------------------------
;Puts out a string from the string table
;Is called with an offset in w that corresponds to the command string to be output
;Takes this and gets an offset into the table and then goes again to get the actual
;string
output_strings
		movwf	var_1				;save W
		Bank2
		movlw	HIGH	string_table
		movwf	EEADRH
		movlw	LOW		string_table
		movwf	EEADR
		Bank0
		movfw	var_1		;restore offset
		call	increment_address
		call	table_read				;will load EEDATA and EEDATAH, also put EEDATA in w
		;now move the address of the string to be printed over to the ADDRESS Registers
		Bank2
		movwf	EEADR	;does low
		movfw	EEDATH	;get high bits
		movwf	EEADRH	;and replant it in the address
		Bank0
		;am now poised to print out the string

startup_loop
		call	table_read
        movwf   temp                ;store w temporarily
		
        xorlw   0x00                    ;test for end
        skpnz
        return                          ;done, get out
		movfw	temp
        call    do_serial_send          ;send byte
        movlw	1
		call	increment_address
        goto    startup_loop            ;loop
;============================================================
;------------------------------------------------------------
help
		Bank2
		movlw	HIGH help_file
		movwf	EEADRH
		movlw	LOW	 help_file
		movwf	EEADR
		Bank0
        
help_again
		call	table_read
        movwf   temp	            ;store
        xorlw   0x00                ;test for end
        skpnz
        return                      ;done, get out
		movfw	temp
        call    do_serial_send          ;print byte
        movlw	1
		call	increment_address
        goto    help_again            ;loop

		
;------------------------------------------------------------
;
;clean_loop should be called with the FSR pointing at the 
;starting location to be cleared 
;It will always clear to SCRATCH_END


clean_up
		
cagain	clrf	INDF			;clear named registers
		movlw	SCRATCH_END
		xorwf	FSR,W
		btfsc	STATUS,Z
		return	
		incf	FSR,f
		goto	cagain
;============================================================
;
Clear_RAM		clrf	offset
				;Load character to be inserted in buffer
				movlw	0x81			;Why 0x81, Just a precaution
										;If running without CRC, and loop in program starts
										;looking for bit 0 in SSID or H bit in digi and cannot
										;find it, it loops a long time. So to be sure, preinit it set
				movwf	next_to_last_byte	;inserts an 81	
				movlw	.255 							;
				movwf	count
Rclr_loop		call	RAMWrite
				decfsz	count,f
				goto	once_more
				return
once_more		incf	offset,f
				goto	Rclr_loop
;
;============================================================
; Getc takes a character from the UART and handles any errors. If available, stuffs the character
; in Rcd_Char. Comes in two variaties, sleep and non-sleep mode.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
getc_sleep
		clrf	temp
		bcf		CHAR_TIMEOUT
char_check
		clrwdt	;reset watch dog
		btfsc	PIR1,RCIF
		goto	char_arrived

		btfsc	PIR1,TMR1IF
		goto	check_tics
		goto	char_check
check_tics
		movlw	HIGH	.26414		;set to 62.5 msec
		movwf	TMR1H
		movlw	LOW		.26414
		movwf	TMR1L
		bcf	PIR1,TMR1IF		;and set to go again
		incf	temp,f

		;If a GPS receiver is connected, shorten up the timer. The GPS should send a char within a couple of 
		;seconds, else get out. 
		btfss	GPS_CONNECTED
		goto	longer_time
		movlw	.32			;about 2 secs
		goto	set_it
longer_time
		movlw	.112		;62.5 msec.. need 7 sec
set_it
		xorwf	temp,w
		skpz
		goto	char_check
		bsf		CHAR_TIMEOUT	
		return

getc_nosleep		
		clrf	Rcd_Char				
		btfss	PIR1,RCIF	;check if data
		return			;return if no data

char_arrived		
		movf	RCREG,W		;get received data
		movwf	Rcd_Char		 ;no stuff into
		;was there an Error?
		btfsc	RCSTA,OERR	;if overrun error occurred
		goto	ErrSerialOverr	; then go handle error
		btfsc	RCSTA,FERR	;if framing error occurred
		goto	ErrSerialFrame	; then go handle error
		;Nope, got good character
			
		;Don't do any echos with CRs
		movlw	CR
		xorwf	Rcd_Char,W
		skpnz
		goto	getc_out

		;OR, If echo flag is not set, don't do any echo
		btfss	ECHO
		goto	getc_out
		movfw	Rcd_Char
		call	do_serial_send
	
getc_out		
		return

ErrSerialOverr:	bcf	RCSTA,CREN	;reset the receiver logic
				bsf	RCSTA,CREN	;enable reception again
				return
ErrSerialFrame:	movf	RCREG,W		;discard received data that has error
				return
;============================================================
; to_upper maps lower case character to upper case
to_upper_case
		movlw	'a'
		subwf	Rcd_Char,W
		skpnc
		goto	greater_than_a
		return	;not between a and z.. get out
greater_than_a
		movlw	'z' + 1
		subwf	Rcd_Char,w
		skpc
		goto	map_it
		return	;greater than z, don't map
map_it
		movlw	0x20
		subwf	Rcd_Char,f
		return
;============================================================
;Getbcd takes up to 3  digits and converts to binary. Binary output can be from
;0 to 255..  Trashes the temp variable
;Sets the carry bit if there was an error



getbcd_sleep

		clrf	var_3
		;first character must not be a cr.. if so error
		call	unload_tty_buffer		;puts ascii number in Rcd_Char
		movlw	CR
		xorwf	Rcd_Char,W
		skpnz
		goto	bcd_first_time
		
		movlw	' '			;likewise, first character can't be a space
		xorwf	Rcd_Char,W
		skpz
		goto	bcd_first_time
		goto	bcd_error
		
bcd_again
		call	unload_tty_buffer		;puts ascii number in Rcd_Char
		movlw	CR				;is this a CR?, if so am done
		xorwf	Rcd_Char,W
		skpnz
		goto	bcd_done		;is it a space? Also am done. Return with space in Rcd_Char
		movlw	' '				
		xorwf	Rcd_Char,W
		skpnz
		goto	bcd_done		;is not a carriage return or sp, is between 0 and 9 ?
bcd_first_time
		movlw	'0'
		subwf	Rcd_Char,W
		skpc	
		goto	bcd_error
		movlw	0x3a
		subwf	Rcd_Char,W
		skpnc
		goto	bcd_error
		movlw	0x30			;ok, it is valid 0-9, convert to binary
		subwf	Rcd_Char,f
		;now multiply previous result by 10, by adding it to itself 10 times
		;then add in the units digit. First time through temp is zero so doesn't matter
		movlw	.9
		movwf	count			;preset loop,10 times, 9 to 0
		movfw	var_3
mul_loop
		addwf	var_3,f			;add number to itself
		skpnc
		goto	bcd_error		;overflow... set error
		decfsz	count,f
		goto	mul_loop
		;and now add in units digit
		movfw	Rcd_Char
		addwf	var_3,f
		skpnc		;overflow?
		goto	bcd_error		;yes, set error
		goto	bcd_again		;look for next digit
bcd_error
		clrw
		setc
		return
bcd_done

		movfw	var_3
		clrc
		return		
;===============================================================
;flush any characters out of the input stream and pitch them
flush
	movlw	.10
	call	delay_x_ms		;wait a bit for another character
	btfss	PIR1,RCIF
	goto	flush_check_error
	movfw	RCREG			;dump it
	goto	flush			;wait, then do it again
flush_check_error
	btfsc	RCSTA,OERR	;if overrun error occurred
	goto	flush_overflow	; then go handle error
	btfsc	RCSTA,FERR	;if framing error occurred
	goto	flush_frame		; then go handle error
	return					;all clear
flush_overflow
				bcf	RCSTA,CREN	;reset the receiver logic
				bsf	RCSTA,CREN	;enable reception again
				return
flush_frame
				movf	RCREG,W		;discard received data that has error
				return

		 	
;===========================================================
; hex_dump
;prints out a data location in hex format. Used for debugging
; When called, w has the binary to be printed
hex_dump
	movwf	var_3
	movwf	count
	movlw	0xf0
	andwf	count,f
	clrc
	rrf		count,f
	rrf		count,f
	rrf		count,f
	rrf		count,f

	movlw	HIGH bcd_hex		
	movwf	PCLATH

	movfw	count
	call 	bcd_hex
	call	do_serial_send
	movlw	0xf
	andwf	var_3,w
	call	bcd_hex
	call	do_serial_send
	return

	
;=======================================================================
clean_all_scratch
		movlw	SCRATCH_START		
		movwf	FSR
		
		call clean_up				;Does all the Scratch Variables
		return
;======================================================================

;=======================================================================
;routine start timer is called with w set to the software timer id to start.
;It resets the timer value to be counted down. There is an array of numbers for
;the set value and an array for the timer block that is counted down
Start_timer
		movwf	temp			;save timer number
		
		movlw	timer_init_array
		movwf	FSR
		movfw	temp
		addwf	FSR,f
		movfw	INDF			;gets load value
		movwf	var_1			;and save it

		movlw	timer_block
		movwf	FSR
		movfw	temp
		addwf	FSR,f			;FSR points at right timer
		
		movfw	var_1
		movwf	INDF			;and store it

		return
;===========================================================
;Software Timer Manager
; Structure for the Timer Block is:
;	Schedule Flags			one flag per task, set when scheduled
;	Timer 0 Low Counter		counted down by timer manager
;	Timer 0 High Counter	counted down by timer manager
;	
;		|
;		|
;	Timer n Low Counter
;	Timer n High Counter
	
; Run through each of the timers. If timer expired, schedule task
; then reset hardware timer
Stimer_mgr
		bcf		RUN_STIMER
		movlw	HIGH schedule_mask	;make sure PCLATH in right page, high and low in same page
		movwf	PCLATH
		clrf	count
		
next_timer		;Is the timer already zero? If so just skip it

		movlw	timer_block		;Point at First Timer Block
		addwf	count,w			;and then offsets
		movwf	FSR
		movf	INDF,W
		skpnz
		goto	timer_again
		decf	INDF,f
		movf	INDF,w
		skpz
		goto	timer_again
		movfw	count
		call	schedule_mask		;Returns the correct mask for scheduler word in w
		iorwf	SCHEDULE,f			;Scheduled
		;reset timer
		movfw	count
		call	Start_timer

timer_again
		incf	count,f
		movlw	NUMBER_OF_SOFTWARE_TIMERS
		xorwf	count,w
		skpz
		goto	next_timer
		return
;===================================================================================
load_tty_buffer


		;load up the tty buffer on chars inputted from the terminal
		;quits when either a control c or a CR is detected.
		

		;stuffs a NULL in the buffer just past the last character to signify the end

	

		clrf	offset			;used by Ram Write routine

	
load_tty_loop
		call	getc_sleep

		btfsc	CHAR_TIMEOUT			;did we not every get a char?
		goto	load_tty_end			;yes, we did stop processing
		;now do backspace
		movfw	Rcd_Char
		xorlw	BS		;is it a backspace?
		skpnz
		goto	process_backspace
		goto	stuff_it
process_backspace
		clrw
		xorwf	offset,w		;if at the beginning, ignore BS, just overwrite
		skpnz
		goto	load_tty_loop
		decf	offset,f			;back up
		goto	load_tty_loop
		
stuff_it
		;stuff the character in the buffer
		movfw	Rcd_Char
		movwf	next_to_last_byte
		call	RAMWrite
		incf	offset,f			;and bump up
	

		;Was it a CR,LF you just stuffed?  Assume CR comes first
		movlw	LF
		xorwf	Rcd_Char,W
		skpnz	
		goto	load_tty_end
		;nope, wasn't CR,LF  

		;Was it a ctl-c you just stuffed?  If so, get out
		movlw	CONTROL_C
		xorwf	Rcd_Char,W
		skpnz
		goto 	load_tty_end		

		;exceeded max count?
		
		movlw	XMIT_BUFFER_SIZE - 1	
		xorwf	offset,w
		skpz
		goto	load_tty_loop
		;exceeded  the count, bail out
load_tty_end
		clrf	next_to_last_byte
		call	RAMWrite			;stuff the null
		return
;------------------------------------------------------------------------------------
;Corresponding Unload TTY Buffer Routine. Returns with next character in RcdChar. Also maps to upper
;if necessary

unload_tty_buffer
			call	RAMRead
			movwf	Rcd_Char		;get character and store it
			
			incf	offset,f			;and bump up to next one

			btfsc	TO_UPPER		;map to upper if requested
			call	to_upper_case

			return
;====================================================================================
;===========================================================================================
output_callsign
			;call with FSR pointing to the 7 characters to be output
			;returns with FSR where is was on entry
			movfw	FSR
			movwf	var_2			;temp storage
			movlw	6
			movwf	count
cs_loop
			movfw	INDF
			movwf	temp
			movlw	' '		;is it a space?
			xorwf	temp,W
			skpnz
			goto	cs_loop_space
			movfw	temp
			call	do_serial_send
cs_loop_space
			decf	count,f
			skpnz
			goto	do_dash
			incf	FSR,f
			goto	cs_loop
do_dash		movlw	'-'
			call	do_serial_send
			incf	FSR,f			;now points to SSID
			;SSID is 0x30 & SSID. Strip off 0x30, and convert SSID to ascii and output
			;Is the SSID 10 or greater?, if so do proper translation
			movlw	0x0f
			andwf	INDF,W
			movwf	temp	;temp has SSID, in binary
			
			movlw	.10
			subwf	temp,w		;is it less than 10
			skpc
			goto	less_than_ten
			;put out the 1
			movlw	'1'
			call	do_serial_send
			movlw	.10
			subwf	temp,f
		
			
less_than_ten
			movlw	0x30		;convert to ascii
			addwf	temp,w
			
			call	do_serial_send
			movfw	var_2
			movwf	FSR			;restore FSR
			return
;=================================================================================================

;====================================================================================
;Commmand Interpreter Process
;Used to receive commands from terminal, parse them, and set up TNC
; Gets call by scheduler each round to check if a Control-C has been inputted on the terminal
;If it has, it siezes control, accepts commands, and set processor variables
;It then gives up control when  another Control-C has been inputted


command_interpreter
	
			
		;Has someone typed something at the terminal? If so, must be the start of a command
		;go process it

		btfss	PIR1,RCIF
		return			;nope, let other processes receive continue to run

		;tell load routine to map to upper case
		bsf	TO_UPPER

		call	load_tty_buffer		;will return with a buffer full of characters
									;with a null at the end

	
ci_again
		
		call	command_parser
		skpnc
		goto good_command
		movlw	2
		goto	s_out	
		;w has offset of command
good_command
		movwf	temp		;temp has command 
		movlw	HIGH cbranch		;make sure PCLATH set correctly
		movwf	PCLATH
		movfw	temp			;restore it
cbranch
		addwf	PCL,F			;these must match with command table
		goto	c_beacon_every		;0
		goto	c_convers
		goto	c_unproto
		goto	c_alias
		goto	c_digi_off
		goto	c_digi_on			;5
		goto	c_mon_all			;6
		goto	c_mon_me			;7
		goto	c_mycall
		goto	c_btext
		goto	c_perm
		goto	c_mon_off			;11
		goto	c_disp
		goto	c_mycall_cr		;13
		goto	al_error		;14
		goto	btext_clear		;15		;just used when you type btext. clears string
		goto	unproto_bad		;16
		nop		;17 is unused	
		goto	c_help			;18	
		goto	c_trace_off		;19  trace debugging command
		goto	c_echo_on		;20	 echo on
		goto	c_echo_off		;21		echo off
		goto	c_trace_receive	;22		trace receive
		goto	c_trace_xmit	;23		trace xmit
		nop						;24 is spare
		goto	c_rmc		;25		gps $gprmc sentence
		goto	c_gll		;26		gps $gprmc sentence
		goto	c_gpgga		;27		gps $gpgga sentence
		goto	c_calibr	;28		calibrate
		goto	c_txd		;29		transmitter delay
c_txd
			call	getbcd_sleep
			skpnc
			goto	s_out				;error or something wrong
			movwf	temp
			
			movlw	CR
			xorwf	Rcd_Char,W			;should have been a CR after the number
			skpz
			goto	s_out
			movlw	.201
			subwf	temp,w
			skpnc
			goto	s_out				;over 200 inputted
			movlw	0
			xorwf	temp,w
			skpnz
			goto	s_out				;0 inputted
			;valid number
			movfw	temp
			movwf	xmit_delay
			goto	c_ok

c_calibr					;turn on calibrate mode
			movlw	.29
			call	output_strings		;does startup message
			call	calibrate
			goto	c_ok
	
c_alias		
			movlw	myalias			;set up pointer where alias to go
			movwf	FSR
			call	load_callsign

			;always should end up at a CR, no more characters
			clrw
			xorwf	FSR,W
			skpnz
			goto	al_error
			movlw	CR
			xorwf	Rcd_Char,W
			skpz
			goto	al_error
			bsf		VALID_ALIAS
			goto	c_ok
al_error	bcf	VALID_ALIAS		;so know it is trashed
			goto	s_out		;put out error message

c_beacon_every
			call	getbcd_sleep
			skpnc
			goto	s_out				;error or something wrong
			movwf	temp
			
			movlw	CR
			xorwf	Rcd_Char,W			;should have been a CR after the number
			skpz
			goto	s_out
			movlw	.61
			subwf	temp,w
			skpnc
			goto	s_out				;over 60 inputted
			;beacon every 0 should shut off beacon
			movlw	0
			xorwf	temp,w
			skpnz
			goto	zero_entered

			movlw	.15
			subwf	temp,w
			skpc
			clrc
			rlf		temp,f				;x2
			rlf		temp,w				;x4 convert tics to minutes inputted (since 15 secs/tic)
			movwf	timer_init_array	;is zero element
			clrw	
			call	Start_timer	
			bsf	BEACON_ENABLE			;and turn on	
			goto	c_ok
zero_entered
			bcf	BEACON_ENABLE
			clrf	timer_init_array
			goto	c_ok

c_help
			call	help
			goto	c_ok
c_convers
			
			;Put out  a CR,LF and message
			movlw	.14
			call	output_strings

			; mycall and unproto should be valid before entering this mode
			btfss	VALID_MYCALL
			goto	no_go_mycall
			btfss	VALID_UNPROTO
			goto	no_go_unproto
			;everything ok.. go to converse mode. exit interpreter
			;but first flush out any left over tty input
			call	flush
			bsf	RUN_CONVERSE	;go to converse mode
			bcf	RUN_INTERPRETER	;and turn off interpreter
			bcf	TO_UPPER		;don't map to upper in converse mode
			movlw	4
			call	output_strings
			return				;don't want another prompt here...
								;enter convers mode, don't come back until ctl c seen in that
								;process
no_go_mycall	;tried to enter converse mode but invalid mycall
			movlw	.11
			call	output_strings
			goto	s_out
no_go_unproto	;tried to enter converse mode but invalid unproto
			movlw	.10
			call	output_strings
			goto	s_out
c_unproto	
			movlw	unproto			;set up pointer where unproto to go
			movwf	var_2
			;unproto  string should have dest_callsign v via_callsign_first v ... v via_callsign_last . 
			;must have at least dest_callsign  and can't go beyond via_callsign_last
			;So do a loop doing repeated loads
			; A pointer will be kept to the last valid callsign and it will be used when the header
			;is transmitted. Note the callsigns are loaded in order they were received 
			;from the terminal and then vias need
			;be reversed when packet send is done
next_unproto
			movfw	var_2
			movwf	FSR				;pointing to place to load the callsign
			call	load_callsign
			clrw
			xorwf	FSR,W
			skpnz
			goto	unproto_bad		;Something Wrong, FSR of 0 is error, so do no more

		
			;is this the last one?
			movlw	last_via
			xorwf	var_2,w
			skpz
			goto	could_be_more
			
			;yep, we are at the last via, next character better be a CR
			movlw	CR				
			xorwf	Rcd_Char,W
			skpnz
			goto	unproto_ok
			goto	unproto_bad

could_be_more
			movlw	CR				;More to come? If not will be a Carriage Return
			xorwf	Rcd_Char,W
			skpnz
			goto	unproto_ok
		
			movlw	0x20			;Is more to come, so better be a space
			xorwf	Rcd_Char,W
			skpz
			goto	unproto_bad	;error, should have either been a CR or SP
			
			;now  make sure "V " arrives
			call	unload_tty_buffer
			movlw	'V'
			xorwf	Rcd_Char,W
			skpz
			goto	unproto_bad
			call	unload_tty_buffer		;and V should be followed by a space
			movlw	' '
			xorwf	Rcd_Char,W 
			skpz
			goto	unproto_bad

			movlw	7				;bump up to next location
			addwf	var_2,f
			goto	next_unproto
			
			
unproto_bad
			bcf		VALID_UNPROTO
			clrf	last_callsign_ptr 		
			goto	s_out		;put out error message	
			
unproto_ok
			;FSR should point at top of last valid address
			bsf		VALID_UNPROTO
			movfw	var_2
			movwf	last_callsign_ptr
			goto	c_ok
			
		

			
		
c_digi_on	bsf	DIGIPEAT_ENABLE
			goto c_ok
c_digi_off	bcf	DIGIPEAT_ENABLE
			goto c_ok
c_mon_all	bsf	MONITOR_ALL
			bcf	MONITOR_ME
			goto	c_ok
c_mon_me	bsf	MONITOR_ME
			bcf	MONITOR_ALL
			goto	c_ok
c_mon_off	bcf	MONITOR_ALL
			bcf	MONITOR_ME
			goto	c_ok
c_mycall	movlw	mycall			;set up pointer where callsign to go
			movwf	FSR
			call	load_callsign
			clrw
			xorwf	FSR,W
			skpz
			goto	mycall_good
c_mycall_cr				
			bcf		VALID_MYCALL
			goto	s_out		;error message out
mycall_good
			bsf		VALID_MYCALL
			goto	c_ok
			

c_btext		
			movlw	BEACON_EE_START			;starting offset in EE Ram
			movwf	count					;needed by EE_Write
			
			bcf		TO_UPPER		;don't want to map beacon text, leave whatever they typed
			
		
			;so just put in EE Ram, keep going until you hit the null
		
btext_next
			call	unload_tty_buffer		;get the character
			movfw	Rcd_Char
			
			call	EE_Write				;put in EERAM
			incf	count,f
			
			movfw	Rcd_Char				;did you just stuff a NULL, if so, done
			xorlw	0
			skpz
			goto	btext_next				;and do another

			bsf		VALID_BTEXT
			goto	c_ok
btext_clear
			bcf		VALID_BTEXT
			goto	c_ok

c_perm		call	Save_Params
			goto	c_ok


;---------Start of the display (DISP) output
c_disp
			;first do echo on or off
			btfsc	ECHO
			goto 	c_echo_disp
			movlw	.18
			call	output_strings
			goto	c_txdelay
c_echo_disp	movlw	.17
			call	output_strings
c_txdelay
			movlw	.30		;puts out "txdelay"
			call	output_strings
			movlw	HIGH binary_to_ascii		;get PCLATH set right 
			movwf	PCLATH
			movfw	xmit_delay
			call	binary_to_ascii
			movlw	HIGH output_strings		;reset PCLATH
			movwf	PCLATH
c_gps_test
			movlw	.14		;CR,LF				;first a cr,lf
			call	output_strings
			movlw	.22
			call	output_strings				;then "GPS "
			btfss	GPRMC
			goto	c_gps_gll
			movlw	.23
			call	output_strings
			goto	c_trace_test
c_gps_gll
			btfss	GPGLL
			goto	c_gps_gga
			movlw	.27
			call	output_strings
			goto	c_trace_test
c_gps_gga
			movlw	.28
			call	output_strings

c_trace_test
			;now trace, not that only one can be set at once
			movlw	.14		;CR,LF				;first a cr,lf
			call	output_strings
			btfsc	TRACE_XMIT
			goto	c_is_trace_xmit
			btfsc	TRACE_RCV	
			goto	c_trace_rcv
			;neither set
			movlw	.19			;trace off
			call	output_strings
			goto	c_monitor_test
c_trace_rcv	
			movlw	.20			;trace receive buffer set
			call	output_strings
			goto	c_monitor_test
c_is_trace_xmit
			movlw	.21
			call	output_strings
			goto	c_monitor_test
c_monitor_test
			movlw	.14		;CR,LF
			call	output_strings
			movlw	.6
			call	output_strings	;monitor with a space
			btfsc	MONITOR_ALL			;is this monitor all
			goto	c_is_mon_all
			btfsc	MONITOR_ME
			goto	c_is_mon_me
			;neither, so must be off
			movlw	.25
			goto	c_mon_finish
c_is_mon_all	
			movlw	.7
			goto	c_mon_finish
c_is_mon_me
			movlw	.24
c_mon_finish
			call	output_strings
			

c_digi_test
			btfsc	DIGIPEAT_ENABLE
			goto c_digi_on1
			movlw	.9
			call	output_strings
			goto	c_beacon_test
c_digi_on1	movlw	.8
			call	output_strings

c_beacon_test
			
			btfss	BEACON_ENABLE
			goto	c_beacon_off
			movlw	.16		;puts out "beacon on every "
			call	output_strings
			movfw	timer_init_array
			movwf	temp
			movlw	HIGH binary_to_ascii		;get PCLATH set right 
			movwf	PCLATH
			clrc
			rrf		temp,f
			rrf		temp,w		;divide by 4 to make mins
			
			call	binary_to_ascii
			movlw	HIGH output_strings		;reset PCLATH
			movwf	PCLATH
			movlw	.14			;and a cr,lf
			goto	c_beacon_end
c_beacon_off
			movlw	.15
c_beacon_end
			call	output_strings
			
;now unproto
			movlw	.10
			call	output_strings		;outputs unproto with no CR,LF			
			btfsc	VALID_UNPROTO
			goto 	put_out_unproto
			;invalid, just put out a CR,LF
			movlw	.14
			call	output_strings
			goto	c_mycall_test
put_out_unproto
			movlw	unproto
			movwf	FSR
c_output_next_callsign
			call	output_callsign		;no CR,LF
	
			;now print out the digipeaters until FSR is bigger than the last_callsign parameter
			movlw	7
			addwf	FSR,f

			movfw	FSR
			subwf	last_callsign_ptr,w			;is positive until get to last one
			skpc
			goto 	c_unproto_done
			;get ready to print next one
			movlw	' '
			call	do_serial_send
			movlw	'V'
			call	do_serial_send
			movlw	' '
			call	do_serial_send
			goto	c_output_next_callsign
c_unproto_done
			movlw	.14			;cr and lf
			call	output_strings
c_mycall_test
			movlw	.11
			call	output_strings		;no cr,lf sent
			btfss	VALID_MYCALL
			goto	c_mycall_off1
			;valid call here
			movlw	mycall
			movwf	FSR
			call	output_callsign
c_mycall_off1	
			movlw	.14		;put out CR,LF
			call	output_strings

c_alias_test
			movlw	.12
			call	output_strings
			btfss	VALID_ALIAS
			goto	c_alias_off1
			;valid alias here
			movlw	myalias
			movwf	FSR
			call	output_callsign
c_alias_off1	
			movlw	.14		;put out CR,LF
			call	output_strings

c_btext_test
			movlw	.13
			call	output_strings
			btfss	VALID_BTEXT
			goto	c_btext_off	

			movlw	BEACON_EE_START
			movwf	count
btext_loop
			movfw	count
			Bank2
			movwf	EEADR
			Bank3
			bsf		EECON1,RD
			Bank2
			movf	EEDATA,w
			Bank0
			skpnz
			goto	c_ok		;hit the null.. all done
			;char from the read now in W
			
			call	do_serial_send		;put it out
			incf	count,f
			goto	btext_loop



c_btext_off
			movlw	.14			;put of CR,LF
			call output_strings
			goto	s15_out
			
c_trace_receive
			bsf		TRACE_RCV
			bcf		TRACE_XMIT			;can only trace one thing at a time		
			goto	c_ok
c_trace_xmit
			bsf		TRACE_XMIT
			bcf		TRACE_RCV
			goto	c_ok
c_trace_off	
			bcf		TRACE_XMIT			;turn both off
			bcf		TRACE_RCV
			goto	c_ok
								
c_echo_on	bsf	ECHO
			goto c_ok
c_echo_off	bcf	ECHO
			goto c_ok


c_rmc
			bsf		GPRMC
			bcf		GPGLL
			goto	c_ok
c_gll
			bsf		GPGLL
			bcf		GPRMC
			goto	c_ok
c_gpgga
			bsf		GPGGA
			bcf		GPGLL
			bcf		GPRMC
			goto	c_ok

			;add more here commands here
s_out		movlw	5
			goto	s1_out
c_ok		movlw	3					;job done, ack by printing OK
s1_out		call	output_strings
s15_out		movlw	1
			call	output_strings		;and another prompt
			call	flush				;clean out any spurious chars, must be clean
										;before we leave or might hang on one character
			bcf		TO_UPPER			;turn off character mapping
			return						;give back control	
;===========================================================================================


;===========================================================================================
set_defaults
			;put any defaults that are set after a reset command
			;all other features are turned off
			bsf		MONITOR_ALL
			bsf		DIGIPEAT_ENABLE
			bsf		ECHO
			movlw	40
			movwf	xmit_delay				;transmit delay
			return
;============================================================================================
;load call sign is used to load mycall, alias, etc. Called with FSR pointing to the right 7
;bytes to load
;
;On return... FSR is loaded with the pointer to the callsign that was loaded. FSR=0 means error detected, what was loaded may be bad. 
;
;Also on return... Rcd_Char has the last character received

load_callsign	;format is call sign-x ; no padding needed
			;prepad with spaces
			
			movfw	FSR
			movwf	var_1	;var_1 also points to right starting place in memory
		
			movlw	7		;number of bytes per call sign
			movwf	loop_count
pre_padded
			movlw	' '
			movwf	INDF			;pre pad with spaces
			incf	FSR,f	
			decfsz	loop_count,f
			goto	pre_padded
			;and get  the characters	
			movfw	var_1
			movwf	FSR			;reset to front of string storage

			;loop count should go from 0 to 6
			clrf	loop_count

get_next_letter		
			call	unload_tty_buffer
			movlw	'-'				;is this the dash before the SSID?
			xorwf	Rcd_Char,W
			skpnz
			goto	do_SSID			;yep, process SSID
			movlw	CR				;is this an early CR? (ie shortcall with no SSIDspecified)
			xorwf	Rcd_Char,W
			skpnz
			goto	short_call
			;
			movlw	0x20				;is this an early sp? (ie shortcall with via afterwards)
			xorwf	Rcd_Char,W
			skpnz
			goto	short_call
			;No - early space or CR, just load
			;get FSR set up
			movfw	var_1		;start of the load point
			addwf	loop_count,w	;add offset
			movwf	FSR				;and set FSR

			movfw	Rcd_Char
			movwf	INDF		;and load character
			
			incf	loop_count,f
			movfw	loop_count
			xorlw	7
			skpz
			goto	get_next_letter
			goto	callsign_error		;should have hit a '-' first
			
short_call	;either got here by a early CR or sp, it still loaded in Rcd_Char
			movlw	6
			addwf	var_1,W
			movwf	FSR			;now points to SSID
			movlw	0x30		;ssid 0
			movwf	INDF		;store it, FSR points to correct SSID
			goto 	is_loaded
do_SSID		call	getbcd_sleep
			skpnc
			goto	callsign_error

			; w has result of the input, in binary. Rcd_Char has either a SP or CR
			addlw	0x30
			movwf	temp	;and temporarily store it, makes it ascii
			;result should be less than 0x40 since the largest digi is 0xf
			movlw	0x40
			subwf	temp,W
			skpnc
			goto	callsign_error	;too large
			;ok, proceed
			movlw	6
			addwf	var_1,W
			movwf	FSR			;now points to SSID

			movfw	temp
			movwf	INDF		;store it		
			;now should either had an ending CR or SP (says more coming)
			;it is still in Rcd_Char
			
			movlw	CR
			xorwf	Rcd_Char,W
			skpnz
			goto 	is_loaded

			movlw	0x20
			xorwf	Rcd_Char,W
			skpnz
			goto 	is_loaded
			goto	callsign_error
is_loaded	;FSR is pointing to the loaded SSID, if CR nothing else coming, If space
			;then caller might expect more to come
			;Rcd_Char also has the last character is received
			;
			;move FSR back to last valid callsign 
			movfw	var_1
			movwf	FSR
		
			
			return
callsign_error
			clrf	FSR			;shows error
			return	
;============================================================================================
command_parser
			;The table_read function is used to walk the tree. It returns the byte from the table 
		;in the w register. Before calling, theEEADR must be loaded with the location to read

			Bank2
		;Set up for table read

		movlw	HIGH root			
		movwf	EEADRH

		movlw	LOW root			
		movwf	EEADR
			Bank0

		clrf	offset		;start back at beginning of tty buffer

next_char
		call	unload_tty_buffer		;leaves the next char in RcdChar

		;Traverse the binary tree to look for a match of the character
		;If you find a match on a given entry, keep bumping to the next 
		;If in doing this you run into 0xff in the character field,
		;you have found a command string match
		;If the character, doesn't match, check 2 things:
		;If it is less than the char, is an illegal string, since the table
		;is in increasing lexigraphical order. If the forward pointer is NULL
		;it is also illegal.
		;Keep doing this until you 
		;match something or you hit a NULL pointer. In this case, the
		;command string has an error.

	
walk_tree
		
		call	table_read
		
		movwf	temp					;stuff it away for later use

		;does the character in the binary list and the Received Character match?
		
		xorwf	Rcd_Char,W
		skpnz	
		goto char_match
		movfw	temp			;didn't match, so try to keep on going
		subwf	Rcd_Char,W
		skpc
		goto 	is_error		;less than, must not match... bad command inputted
		;value is greater, so see if further matches are possible
		movlw	1
		call	increment_address
		call	table_read

		movwf	temp			;Got Pointer, is it ERROR.....0?
		
		movlw	ERROR
		xorwf	temp,w
		skpnz
		goto	is_error			;nope, bad command
		;go on to next character in tree
		;but first need to subtract one since incremented it above
		movlw	1
		call	decrement_address	
		movfw	temp
		call	increment_address		;and bump up to where pointer shows	
		goto	walk_tree		;and do it again
		;command did not match, clr the carry bit and get out
is_error
		clrc
		return
char_match
		;bump up to next node
		movlw	2
		call	increment_address

		call 	table_read		;"peak" ahead and see if did a match
		movwf	temp
		
		movlw	MATCH
		xorwf	temp,W
		skpz
		goto	next_char			;Matched, but not done with entire string
string_match
		movlw	1
		call	increment_address
		call	table_read			;get the command number
		setc
		return
;============================================================
;Table read subroutines reads data from program memory. Is called with EEADRH and L preloaded with
;the address to read. It returns the low 8 bits of the 13 bit instruction in the w register
;Also leaves the high and low bits in the EEDATA register that may be accessed by the caller
;Caller must reset to correct bank if not bank0
;
; increment_address just increments the EEDATH and L registers by the amount in the w register 
; on entry

table_read
		Bank3
		bsf	EECON1,EEPGD		;Point to program memory
		bsf	EECON1,RD
		nop
		nop						;data now in EEDATA register
		bcf		EECON1,EEPGD	;Disable pointing to program memory
		Bank2
		movf	EEDATA,W
		
		Bank0
		return
;----------------------------------------------------------------------
increment_address
		Bank2
		addwf	EEADR,f
		skpnc
		incf	EEADRH,f
		Bank0
		return
;---------------------------------------------------------------------
; and the corresponding decrement address
decrement_address
		Bank2
		subwf	EEADR,f
		skpc
		decf	EEADRH,f
		Bank0
		return	
;=====================================================================

;Converse Process
;Send out what is typed using the UNPROTO as a dest and mycall as a source
;if digis specified, will use them also

;Shares same buffer used on receive. Doesn't tie it up until a valid character is seen then holds here
;until either a CR or ctl C is seen (or hit 255 character max).
converse
		
	
		;are there any characters to process? if not give it back to scheduler. After first 
		;one has been seen, hold here to buffer filled, CR, or ctl c seen


		btfss	PIR1,RCIF	;check if data
		return			;return if no data, but remain an active process
		;Got a character, so keep process running until got it all


		call	load_tty_buffer
		;returns with a null in last character, char before null is last char inputted
		
		;Was there a timeout?  If so, exit converse mode
		btfsc	CHAR_TIMEOUT
		goto	converse_end
		;If control C is last character inputted in this line, don't even start

		movlw	CONTROL_C						;look for ctl-c to know to end
		xorwf	Rcd_Char,w
		skpnz
		goto	converse_end				;found a ctl-c
		
		
time_to_send


		;first put terminal on next line (since not echoed)
		movlw	CR
		call	do_serial_send
		movlw	LF
		call	do_serial_send
	
		;text is now in the buffer, send it
		;first do the header, including the control
		;create a loop counter
		
		clrf	offset		;now point back to start of text

		call	send_a_header

		;now do the body. Don't want to map these to upper case
		bcf		TO_UPPER
converse_data_loop
        call	unload_tty_buffer			;will have a null at end of string

		movlw	CONTROL_C						;look for ctl-c to know to end
		xorwf	Rcd_Char,w
		skpnz
		goto	converse_end				;found a ctl-c

		clrw
		xorwf	Rcd_Char,w
		skpnz
		goto	end_of_string			;found null
		movfw	Rcd_Char
        call    reg_packet_out			;send it
		
        goto    converse_data_loop
		;Wrap it up
end_of_string
        call finish_up
		call	key_up
		
		return		;sent that one give up for a while, till next char seen
					;don't go back to command interpreter
converse_end
		call	key_up			;just to be sure xmitter is off
		bcf	RUN_CONVERSE		;exit this process for good
		bsf	RUN_INTERPRETER		;and restart interpreter
		call	flush
		movlw	1
		call	output_strings
		return					;back to command mode
;===========================================================


;============================================================
;sums up all the chars in the nema string upto the , and returns the number in w
		
nmea_sum
		clrf	offset
		clrf	aux_count			;temp sum counter
nema_read_again
		call	RAMRead
		movwf	Rcd_Char		;get character and store it
		call	to_upper_case	;map it to upper case

		movlw	','
		xorwf	Rcd_Char,w
		skpz
		goto 	nema_sum_again
		movfw	aux_count
		return
nema_sum_again
		incf	offset,f
		movfw	Rcd_Char
		addwf	aux_count,f
		goto	nema_read_again
;============================================================
; Get_Params reads all the EE data block and deposits it into the
; right variables. All variables from mycall to BOOT_FLAGS1 are
;moved
Get_Params
		clrf	count	;will be loaded into EE_ADDR
		movlw	mycall	;FSR will point to memory to be loaded
		movwf	FSR
boot_loop
		movfw	count
		Bank2
		movwf	EEADR
		Bank3
;Make Sure EECON register is cleared so am looking at EE Ram
;If using bootloader might not be cleared
		clrf	EECON1

		bsf		EECON1,RD
		Bank2
		movf	EEDATA,w
		Bank0
		movwf	INDF		;load variable
		movlw	BOOT_DATA_SIZE
		xorwf	count,w
		skpnz
		return
		incf	FSR,f
		incf	count,f
		goto	boot_loop
;=============================================================
;Save_Params does the opposite of Get_Params. Gets called from PERM
;command
Save_Params		
		clrf	count		;offset in EE memory to write
		movlw	mycall
		movwf	FSR			;points to data to get
Write_Data_loop
		Bank0
		movfw	INDF	;got the byte
	
		call	EE_Write
		;
		movlw	BOOT_DATA_SIZE
		xorwf	count,w
		skpnz
		return
		incf	FSR,f
		incf	count,f
		goto	Write_Data_loop
;================================================================
;EE write fills EE data. Called with data to write in W. Count has offset in EE Ram
EE_Write

		bcf		PIR2,EEIF		;just to make sure, clear EEIF flag before starting

	Bank2
		movwf	EEDATA
		Bank0
		movfw	count
		Bank2
		movwf	EEADR

		;Don't alter the next sequence
		Bank3

		bsf	EECON1,WREN	
		bcf	INTCON,GIE
		movlw	0x55
		movwf	EECON2
		movlw	0xaa
		movwf	EECON2
		bsf		EECON1,WR
		;end of sequence
		bcf		EECON1,WREN
		Bank0
		btfss	PIR2,EEIF		;wait till done
		goto $-1

		bcf		PIR2,EEIF
		bsf		INTCON,GIE
		return
;================================================================
;Routine switch? decides if a process switch is necessary. Sets the
;carry bit if it is, other clears the carry bit
; A switch can be needed if timer 2 has expired 10 times, signifying a second
; OR if a character is waiting to be processed

switch?
		clrwdt	;reset watch dog
		btfsc	PIR1,TMR1IF
		goto	timer_did_expire
		goto	check_chars
timer_did_expire
		movlw	HIGH	.26414		;set to 62.5 msec
		movwf	TMR1H
		movlw	LOW		.26414
		movwf	TMR1L
		bcf	PIR1,TMR1IF		;and set to go again
		incf	tic_counter,f
		movlw	.239		;62.5 msec.. need 15 sec
		xorwf	tic_counter,w
		skpz
		goto	check_chars
		bsf		RUN_STIMER		;schedule the software timer
		clrf	tic_counter
		setc	
		return
check_chars
		clrc
		btfsc	PIR1,RCIF
		setc	;chars are present, just relinquish control
		return


;==================================================================
;Calibrate Routine. Used to send alternate tones. Brings up the transmitter and 
;toggles the tone each time space bar is hit
calibrate
		call	init_xmit			;prepare to bring up the transmitter
		call	ready_to_tx

cal_loop
		clrwdt						;reset watch dog
		call	getc_nosleep		;get a char
		
		movlw	SPACE_BAR			;if space bar toggle
		xorwf	Rcd_Char,w
		skpnz
		goto	cal_toggle			;Control C to exit
		movlw	CONTROL_C
		xorwf	Rcd_Char,w
		skpnz
		goto	cal_wrapup
		goto	cal_loop
cal_toggle
		call	flipout				;do the tone toggle
		goto	cal_loop
cal_wrapup
		call	key_up				;turn off xmitter
		return
;============================================================
; Scheduling Algorithm
; "Processes" are called in round robin fashion in the schedule loop
; below. After a process is called, it should either run to completion
; or give up execution when signaled by the timer manager. If the
; process requires to be restarted in the middle of an activity, it must
; keep it's own state info for restart. 

;Processes may or may not have a software timer associated with them. If they do
;the timer will restart them after some interval, as defined by the time
;in the TIMER_BLOCK. If they don't, they can give up control forever 
;(unless started by someone else), or return and request to be started up
;again on next round of the "roundrobin".
;============================================================
;START OF MAIN
main	
		
		call 	clean_all_scratch	
		call	Clear_RAM			;Clears the managed RAM sections
		call	init_vars			;Now reset any  variable inits
		
		call	init_hardware		;And init the  Ports and UART
		
		bsf		RUN_RECEIVE			;Schedule the receive process to run

		;If a GPS receiver is conncected to the serial port, don't need the command interpreter
		;to do anything. Let the beacon process take care of accepting any sentences from the
		;GPS unit

		btfss	PORTA,5			;Read GPS switch
		goto	set_gps
		
		;Command Interpreter Connected, Put out startup message			
		movlw	0
		call	output_strings		;Puts out startup message
		movlw	1
		call	output_strings		;and cmd: prompt
		bsf		RUN_INTERPRETER		;and schedule command interpreter to run
		goto	round_robin
set_gps
		bsf		GPS_CONNECTED
		bcf		MONITOR_ALL			;Don't need monitor
		bcf		MONITOR_ME
round_robin	
		
		clrwdt		;reset watchdog
		btfsc	RUN_INTERPRETER
		call	command_interpreter		;Receives commands, Sets up options, etc

		
		btfsc	RUN_CONVERSE		;processes inputs for converse mode
		call	converse			;in converse mode, sends out UI frames when typed
	
		btfsc	RUN_STIMER
		call	Stimer_mgr			;Manage timer blocks, started once every 15 sec
;
		
		btfsc	RUN_BEACON
		call	beacon				;Puts out a beacon, scheduled by STIMER
		
		btfsc	RUN_RECEIVE			;runs all the time
		call	Packet_Receive
	
		btfsc	RUN_DIGIPEAT		;scheduled from receive process when packet
									;with matching alias or mycall arrives to be digipeated
		call	digipeat
		
		btfsc	RUN_MONITOR			;scheduled from receive process
		call	monitor
		
		goto	round_robin

		org	0x800					;routines after this are in top page

;=============================================================
;Converts binary to ascii and prints it. Used repeated subtraction method.
;divisor looked up in divisor table
binary_to_ascii
		movwf	temp
		clrf	count		;indexes divisor (100,10,1)
		clrf	var_1		;used as a flag to suspress leading 0
		clrf	var_2

		
convert_loop

		movlw	HIGH divisor		;get PCLATH set right 
		movwf	PCLATH
		movfw	count
		call	divisor
		;returns 0 when done
		xorlw	0
		skpnz
		goto	convert_done
		subwf	temp,w
		skpc
		goto	not_that_big    ;don't subtract
		movwf	temp		;is that big, subtract
		incf	var_2,f
		goto	convert_loop	;subtract again		

not_that_big
		;suspress leading 0
		clrw
		xorwf	var_2,w
		skpnz
		goto	check_supress
		goto	convert_it
check_supress
		;is a zero
		btfss	var_1,0
		goto	try_again		;leading 0 don't print
convert_it
		movlw	HIGH do_serial_send		;get PCLATH set right 
		movwf	PCLATH
		bsf		var_1,0
		movlw	0x30
		addwf	var_2,w		;is now ascii
		
		call	do_serial_send
		;Put PCLATH back
		movlw	HIGH try_again
		movwf	PCLATH
try_again
		clrf	var_2
		incf	count,f		;index into subs
		goto	convert_loop

convert_done
		return
;Divisor must stay on same page as binary_to_ascii routine
divisor	
		addwf	PCL,F
		dt	.100
		dt	.10
		dt	.1
		dt	.0		;says to end
; Here is help file. Output when "help" is inputted

		
help_file
	dt	CR,LF
	dt	"Commands are Case Insensitive",CR,LF
	dt	"Use Backspace Key (BS) for Correction",CR,LF
	dt	"Use the DISP command to display all options",CR,LF
	dt	"Insert Jumper J4 and Connect GPS for APRS Operation",CR,LF
	dt	"Remove Jumper J4 and "
	dt	"Connect to Terminal for Command Interpreter",CR,LF,CR,LF
	dt	"Commands (with example): ",CR,LF	
	dt	"MYCALL (mycall wb8wga-2)",CR,LF
	dt	"UPPROTO (unproto wb8wga-14 v wb8wga-1) - 3 digis max",CR,LF
	dt	"BTEXT (btext Bob)-100 chars max",CR,LF
	dt	"BEACON (beacon every n)- n=0 is off and 1<n<60 mins",CR,LF
	dt	"MONitor (mon all,mon me, or mon off)",CR,LF
	dt	"DIGIpeat (digi on or digi off)",CR,LF
	dt	"MYALIAS (myalias RELAY)",CR,LF
	dt	"PERM (PERM)",CR,LF
	dt	"ECHO (echo on or echo off)",CR,LF
	dt	"GPS (gps $GPGGA or gps $GPGLL or gps $GPRMC)",CR,LF
	dt	"TRace (tr xmit or tr rcv) - For debugging only",CR,LF
	dt	"TXDELAY (txdelay n 0<n<201 n is number of delay flags to send)",CR,LF
	dt	"CALIBRATE (Calibrate Mode - Testing Only)",CR,LF,CR,LF
	
	dt	0
		
;==================================================================
		
string_table
		da		s0
		da		s1
		da		s2
		da		s3
		da		s4
		da		s5
		da		s6
		da		s7
		da		s8
		da		s9
		da		s10
		da		s11
		da		s12
		da		s13
		da		s14
		da		s15
		da		s16
		da		s17
		da		s18
		da		s19
		da		s20
		da		s21
		da		s22
		da		s23
		da		s24
		da		s25
		da		s26
		da		s27
		da		s28
		da		s29
		da		s30
s0		dt      CR,LF, "WB8WGA MODEMLESS TNC  V 1.08",CR,LF
		dt		"Type HELP for Info",CR,LF,0
s1		dt		CR,LF,"cmd: ",0
s2		dt		'?',0
s3		dt		CR,LF,"OK",0
s4		dt		CR,LF,"***  Converse Mode, ctl C to Exit",CR,LF,0
s5		dt		CR,LF,"?",0
s6		dt		"MONitor ",0
s7		dt		"ALL",CR,LF,0
s8		dt		"DIGIpeater ON",CR,LF,0
s9		dt		"DIGIpeater OFF",CR,LF,0
s10		dt		"UNPROTO ",0
s11		dt		"MYCALL ",0
s12		dt		"MYALIAS ",0
s13		dt		"BTEXT ",0
s14		dt		CR,LF,0			;just CR,LF
s15		dt		"BEACON Off",CR,LF,0
s16		dt		"BEACON On EVERY ",0
s17		dt		CR,LF,"ECHO ON",CR,LF,0
s18		dt		CR,LF,"ECHO OFF",CR,LF,0
s19		dt		"TRace OFF",0
s20		dt		"TRace RCV",0
s21		dt		"TRace	XMIT",0
s22		dt		"GPS ",0
s23		dt		"$GPRMC",0
s24		dt		"ME",CR,LF,0
s25		dt		"OFF",CR,LF,0
s26		dt		"Watch Dog Timer Failure",CR,LF,0
s27		dt		"$GPGLL",0
s28		dt		"$GPGGA",0
s29		dt		"Calibrate Mode. SP to toggle; ctl C to Exit",CR,LF,0
s30		dt		"TXDELAY ",0
;------------------------------------------------------------

;==============================================================================================
;Here is the parse table. It is a binary tree. Here are rules:
;1) Start with offset 0. If the received character matches the entry, bump to next entry and wait for
;next character. If the next entry character is MATCH (oxff), then we have matched a string.  to see if it matches.
;2) If it doesn't match see if it is greater or less than the value go to the less than offset 
;and look again, otherwise go to the less than entry. If this pointer is a zero, then we have an
;illegal command.
;3)		
	
;command_list follows

			
root			dt	'B',c - root
be				dt	'E',bt - be
				dt	'A',ERROR
				dt	'C',ERROR
				dt	'O',ERROR
				dt	'N',ERROR
				dt	' ',ERROR
			    dt	'E',ERROR
				dt	'V',ERROR
				dt	'E',ERROR
				dt	'R',ERROR
				dt	'Y',ERROR
				dt	' ',ERROR
				dt	MATCH,0		; "beacon every"   0
bt				dt	'T',ERROR
				dt	'E',ERROR
				dt	'X',ERROR
				dt	'T',ERROR
btc				dt	CR,btext_sp - btc
				dt	MATCH,0xf		;btext with just CR
btext_sp		dt	' ',ERROR		;btext with a space
				dt	MATCH,9		;btext

c				dt	'C',d-c
ca				dt	'A',co - ca
				dt	'L',ERROR
				dt	'I',ERROR
				dt	'B',ERROR
				dt	'R',ERROR
				dt	'A',ERROR
				dt	'T',ERROR
				dt	'E',ERROR
				dt	MATCH,.28
co				dt	'O',ERROR
				dt	'N',ERROR
				dt	'V',ERROR
				dt	'E',ERROR
				dt	'R',ERROR
				dt	'S',ERROR
				dt	'E',ERROR
				dt	CR,ERROR
				dt	MATCH,1			;converse mode
d				dt	'D',e - d
				dt	'I',ERROR
dig				dt	'G',dis - dig
				dt	'I',ERROR
				dt	' ',ERROR
				dt	'O',ERROR
digio			dt	'F',digi_o - digio
				dt	'F',ERROR
				dt	CR,ERROR
				dt	MATCH,4			;"digi off" match
digi_o			dt	'N',ERROR
				dt	CR,ERROR
				dt	MATCH,5			;"digi on" match
dis				dt	'S',ERROR
				dt	'P',ERROR
				dt	CR,ERROR
				dt	MATCH,0xc

e				dt	'E',g - e
				dt	'C',ERROR
ech				dt	'H',ERROR
				dt	'O',ERROR
				dt	' ',ERROR
				dt	'O',ERROR
echoo			dt	'F',echoon - echoo
				dt	'F',ERROR
				dt	CR,ERROR
				dt	MATCH,.21			;"echo off" match
echoon			dt	'N',ERROR
				dt	CR,ERROR
				dt	MATCH,.20			;"echo on" match
g				dt	'G',h - g
				dt	'P',ERROR
				dt	'S',ERROR
				dt	' ',ERROR
				dt	'$',ERROR
				dt	'G',ERROR
				dt	'P',ERROR
gp				dt	'G',gpr - gp
gpg				dt	'G',gpgl - gpg
				dt	'A',ERROR
				dt	MATCH,.27
gpgl			dt	'L',ERROR
				dt	'L',ERROR
				dt	MATCH,.26
gpr				dt	'R',ERROR
				dt	'M',ERROR
				dt	'C',ERROR
				dt	MATCH,.25
h				dt	'H',m - h
				dt	'E',ERROR
				dt	'L',ERROR
				dt	'P',ERROR
				dt	CR,ERROR
				dt	MATCH, .18 
m				dt	'M',p - m
mo				dt	'O',my - mo
				dt	'N',ERROR
				dt	' ',ERROR
mon_a			dt	'A',mon_m - mon_a
				dt	'L',ERROR
				dt	'L',ERROR
				dt	MATCH,.6
mon_m			dt	'M',mon_o - mon_m
				dt	'E',ERROR
				dt	MATCH,.7
mon_o			dt	'O',ERROR
monof			dt	'F',mon_on - monof
				dt	'F',ERROR
				dt	CR,ERROR
				dt	MATCH,.11			;"mon off"
mon_on 			dt	'N',ERROR
				dt	CR,ERROR
				dt	MATCH,7				;"mon on" match	
my				dt	'Y',ERROR
mya				dt	'A',myc - mya
				dt	'L',ERROR
				dt	'I',ERROR
				dt	'A',ERROR
				dt	'S',ERROR
myaliasc		dt	CR,myalias_sp - myaliasc
				dt	MATCH,0xe			;myalias with CR
myalias_sp		dt 	' ',ERROR			;myalias with space
				dt	MATCH,3			;myalias
myc				dt	'C',ERROR
				dt	'A',ERROR
				dt	'L',ERROR
				dt	'L',ERROR
mycallc			dt	CR,mycall_sp - mycallc		;mycall with a space following
				dt	MATCH,0xd		;mycall with CR
mycall_sp		dt	' ',ERROR
				dt	MATCH,8				;"mycall" match
p				dt	'P',t - p
				dt	'E',ERROR
				dt	'R',ERROR
				dt	'M',ERROR
				dt	CR,ERROR
				dt	MATCH,0xa
t				dt	'T',u - t
tr				dt	'R',tx - tr
				dt	' ',ERROR
tr_o			dt	'O',tr_r - tr_o
				dt	'F',ERROR
				dt	'F',ERROR
				dt	CR,ERROR
				dt	MATCH,.19
tr_r			dt	'R',tr_x - tr_r
				dt	'C',ERROR
				dt	'V',ERROR
				dt	CR,ERROR
				dt	MATCH,.22
tr_x			dt	'X',ERROR
				dt	'M',ERROR
				dt	'I',ERROR
				dt	'T',ERROR
				dt	CR,ERROR
				dt	MATCH,.23
tx				dt	'X',ERROR
				dt	'D',ERROR
				dt	'E',ERROR
				dt	'L',ERROR
				dt	'A',ERROR
				dt	'Y',ERROR
				dt	' ',ERROR
				dt	MATCH,.29	
u				dt	'U',ERROR
				dt	'N',ERROR
				dt	'P',ERROR
				dt	'R',ERROR
				dt	'O',ERROR
				dt  'T',ERROR
				dt	'O',ERROR
unprotoc		dt	CR,unproto_sp - unprotoc
				dt	MATCH,0x10		;unproto with CR
unproto_sp		dt 	' ',ERROR		;unproto with sp
				dt	MATCH,2				;unproto match
;================================================================
; Modification History 8/6/04 Added Boot Loader Capability Beta 1.3

;------------------------------------------------------------
				end
Copyright (c) 2004, Bob Ball WB8WGA 
Adapted from work provided by others, particularly Mike Berg N0QBH
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer. 
Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution. 
Neither the name of the <ORGANIZATION> nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission. 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
;------------------------------------------------------------
