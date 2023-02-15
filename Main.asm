
#include <xc.inc>
    
   
#include "Opcodes.asm"
#include "EQU.asm"
; CONFIG
  CONFIG  WDTE = OFF            ; Watchdog Timer (WDT enabled)
  CONFIG  CP = OFF              ; Code Protect (Code protection off)
  CONFIG  MCLRE = OFF            ; Master Clear disabled 
 
ORG 0x0000


SCL     MACRO 
	GPIO,SCL_PIN 
	ENDM
SDA     MACRO 
	GPIO,SDA_PIN
	ENDM
;
;SDA    --
;           \
;            \______

;
;SCL    -------
;               \
;
;           S

START_CONDITION MACRO   
			BSF SDA     ; must be high
			NOP
                        BSF SCL     ; pulse clock
			NOP
                        BCF SDA     ; transition to low specifies start/restart condition
			NOP
			NOP
                ENDM
;
;SDA         ______
;           /
;     ____ /

;
;SCL    -------
;               \
;
;           S
STOP_CONDITION  MACRO   
			BCF SDA
			NOP
			NOP
                        BSF SCL
			NOP
			NOP
                        BSF SDA
                ENDM


goto_		MACRO   label
		    pagesel label
		    goto label
		    pagesel $
		ENDM
		
call_		MACRO   label
		    pagesel label
		    call label 
		    pagesel $
		ENDM

;this must be linked to the reset vector
    PSECT   resetVec,class=CODE,reloc=2
resetVec:
    goto    main
    PSECT appcode,class=APP,delta=2 ;this makes the program work, i have no idea why

main:
    movlw 0		    
    movwf CMCON0	    ; disable comparator to enable digital IO
   
    
    MOVLW ~((1<<5)|(1<<6))  ; initialise TMR0 to use internal oscillator
    OPTION		    ; and disable pullups
    CLRW
    TRIS GPIO
    BCF GPIO, 2
    
    CLRF VPCL_H
    CLRF VPCL_L
    goto GET_NEXT_INSTRUCTION
    
    
; check if the "d" bit indicates stroing in WREG or in FILE
CHECK_DESTINATION:
    ; only file commands will come here
    ; bit 5 of arg holds the destination
    ; by default we use the file as the desination
    ; if this is incorrect, we will return the s_INDF to INDF
    ; an copy to WREG
    btfsc Arg,5	    ; 1 = file, 0 = WREG
    goto CHECK_VPCL
    
    ; temp = INDF
    movf INDF
    movwf tempReg
    
    ; INDF = s_INDF
    movf s_INDF
    movwf INDF
    
    ; WREG = temp
    movf tempReg
    
; Check If the Virtual Program Counter has been altered 
; by a conditional command (DECFSZ etc.)
CHECK_VPCL:
    movf VPCL_Update_Req,f
    btfsc STATUS,2
	call_ UPDATE_VPCL
	
; Tidy up after completing a generic command
END_CMD:
    ;movf s_WREG				; restore WREG

    
   
GET_NEXT_INSTRUCTION:
    BCF GPIO,2
    ; setup 16 bit address read with 16 bits data
    movlw 0xff & ( (AddressSize16Bits) | (I2C_Select_READ) | (1<<IncomingSize))
    movwf I2C_CTRL
    
    movlw slaveI2CAddr
    movwf I2C_Adddress_REG
    
    movf VPCL_H,W
    movwf I2C_ADDR_H_REG
    
    movf VPCL_L,W
    movwf I2C_ADDR_L_REG
    
; Parameters -I2C_ADDRESS_REG
;	     -I2C_ADDR_H_REG | I2C_ADDR_L_REG -> 16 bit address    
I2C_READ_2_BYTES:
    movf I2C_Adddress_REG,W		; Set to 'Write' command by default
    call_ I2C_WRITE_PREP		; send the slave address
    START_CONDITION
    call_ I2C_TX
    
    movf I2C_ADDR_H_REG, W
    call_ I2C_WRITE_PREP		; send program counter
    call_ I2C_TX
    
    ; continue for 16 bit only
  ;  btfsc I2C_CTRL,AddressSize
  ;  goto BEGIN_READING

    movf I2C_ADDR_L_REG, W
    call_ I2C_WRITE_PREP		; send program counter
    call_ I2C_TX
    
    BEGIN_READING:
    
    INCF I2C_Adddress_REG,W
    call_ I2C_WRITE_PREP		; send the slave address
    START_CONDITION			; send restart condition
    call_ I2C_TX
    
    movlw I2C_ACK_BIT
    call_ I2C_READ_PREP		; read command
    call_ I2C_RX
    
    movf I2C_RR,W		; read register holds our next command 
    movwf NextCommand
    
    movlw I2C_NACK_BIT
    call_ I2C_READ_PREP		; read arguments
    call_ I2C_RX
    
    movf I2C_RR,W		; read register holds our next command argument
    movwf Arg
    
    ; end the madness
    STOP_CONDITION
    
    ; increment counter
    call_ UPDATE_VPCL

   ; BSF I2C_CTRL,QuickRW	; set quick read for the next instruction to reduce overhead
    goto  EXECUTE

UPDATE_VPCL:
    ;BCF I2C_CTRL,QuickRW	; disable quick read/write, since the address may now be changed
    incf VPCL_L, f		; increment the program counter
    incf VPCL_L, f		; by two
    btfss STATUS,2		; zero flag
    RETLW 0
    incf VPCL_H,f		; increment upper byte
    RETLW 0


I2C_ERROR:

   BSF GPIO,2
I2C_WRITE_PREP:
    
    ;load argument from WREG
    movwf I2C_WR
    
    ; set SDA and SCL as outputs
    movlw 0xf & (0<<2 | 0<<SCL_PIN | 0<<SDA_PIN)
    TRIS GPIO

    ; set up our counter
    movlw 8		; address is 7 bits;
    movwf tempReg
 
    RETLW 0
    
    
    
    // 10 instructions to send one bit
    // 1Mhz /10 = 100 Khz
I2C_TX:
    
    BCF SCL

    ; send a one or zero
    btfss I2C_WR,7
    BCF SDA
    btfsc I2C_WR,7
    BSF SDA
    
    

    BSF SCL
    
    rlf I2C_WR,f	; I2C_WR <<= 1
			; this will set carry flag if bit 8 was set

    decfsz tempReg	; if(--tempReg == 0) break;
    
    GOTO I2C_TX


    I2C_ACK:
   ;movlw 0xf & (0<<LED | 1<<SDA_PIN | 0<<SCL_PIN)
;	TRIS GPIO
	; change SDA to input so that ACK can be read
	;movlw 0xff
	;movwf tempReg
	NOP
	BCF SCL
	
	movlw 0xff
	movwf tempReg
	
	
	; wait for ack
	
	NOP
	NOP
	BSF SCL	
	WAIT_FOR_ACK:
    /*
	    btfss GPIO, SDA_PIN
	    goto I2C_EXIT
	    
	    decfsz tempReg
	    goto WAIT_FOR_ACK
	    BCF SCL
	    ; timeout error
	    goto I2C_ERROR*/
	    NOP
	    NOP
	    NOP
	    NOP;*/
	    ;NOP
I2C_EXIT:
    BCF SCL
    RETLW 0   

I2C_READ_PREP:
    movwf I2C_WR	; holds ack or nack bit
    movlw 0xf & (0<<LED | 0<<SCL_PIN | 1<<SDA_PIN)
    TRIS GPIO
    movlw 8		; address is 7 bits;
    movwf tempReg
    clrf I2C_RR
    RETLW 0
    
    
I2C_RX:
    NOP
    BCF SCL
    NOP
    ; read value at pin
    rlf I2C_RR,f
    
    btfsc SDA	    
    bsf I2C_RR,0
    
    
    ; we have spent 7 cycles to get here
    BSF SCL
   
    
    ; if(--tempReg == 0) break;
    decfsz tempReg
    
    ; 10 instructions per byte
    GOTO I2C_RX
    
    ; send ack or nack
    movlw 0xf & (0<<LED | 0<<SCL_PIN | 0<<SDA_PIN)
    TRIS GPIO
    movf I2C_WR, F  ; ack = 0, nack = 1 
    BCF SCL
    
    
    btfss STATUS , 2	; zero flag
    BSF SDA
    btfsc STATUS , 2	; zero flag
    BCF SDA
    
    BSF SCL
    NOP
    NOP
    NOP
    NOP
    BCF SCL
    RETLW 0

EXECUTE:
    clrf VPCL_Update_Req    
;
;Check the type of command
;;;
;	-File oriented
;
;	-Bit oriented;
;
;	-literal and Control
;   
    
    
    ; nextCommand contains the upper byte of the instruction
    ; the top nibble of Next Command will be empty
    ; the first bit of the instruction code will be bit 3 (MSBit of lower nibble)
	btfsc NextCommand, 3
	goto CONTROL_COMMAND_LIST	; first bit of control and literal commands is always a one

    ; there are some control commands that are completely empty
	movf NextCommand, f
	btfss STATUS, 2			;zero flag
	goto FILE_AND_BIT_COMMON	
    
    ; Movwf can alias for one of these commands, so we need to do a test first
    ; it can be distinguished by bit 5 being set in "Arg"
	btfss NextCommand, 5
	goto SPECIAL_COMMAND_LIST

    FILE_AND_BIT_COMMON:
	
    ; this must be either a file command or a bit command
    ; point to the correct location in ram
	movwf s_WREG,W			; save WREG to shadow WREG

	movf Arg, W
	andlw 0b11111	;the file address is the bottom 5 bits

					; copy address of FILE into WREG
	movwf FSR			; set pointer from WREG

	movf INDF, W			; create a backup of "FILE"; because we don't know the destination
	movwf s_INDF
	

	; test for a bit command
	btfsc NextCommand, 2
	goto BIT_COMMAND_LIST
    
    
    FILE_COMMAND_LIST:
    
	; FILE commands need data from the lower byte to fully decipher
	; so we will now move this into "NextCommand"
	; bit shift into upper nibble
	; the top 4 bits are now the instruction
	swapf NextCommand, F

	; most commands only need the first two bits of the "Arg" parameter
	; MOVWF and CLRF need 3 bits
	; CLRF will alias with CLRW
	; MOVWF will alias with NOP
	swapf Arg, W 
	andlw 0b1100

	; the top 6 bits are now the instruction
	addwf NextCommand, F
	

	;compare to list of commands
	
	// these commands increment by one
	// onlt CLRF and CLRW are aliased
	                    ;Relevant nibble
	;    MOVWF_OC	0b00    0000            00 001f ffff
	;    CLRF_OC	0b00	0001		00 001f ffff 
	;    CLRW_OC	0b00	0001		00 0000 0000
	;    SUBWF_OC	0b00	0010		00 00df ffff
	;    DECF_OC	0b00	0011		00 00df ffff
	;    IORWF_OC	0b00	0100		00 00df ffff
	;    ANDWF_OC	0b00	0101		00 00df ffff
	;    XORWF_OC	0b00	0110		00 00df ffff
	;    ADDWF_OC	0b00	0111		00 00df ffff
	;    MOVF_OC	0b00	1000		00 00df ffff
	;    COMF_OC	0b00	1001		00 00df ffff
	;    INCF_OC	0b00	1010		00 00df ffff
	;    DECFSZ_OC	0b00	1011		00 00df ffff 
	;    RRF_OC	0b00	1100		00 00df ffff
	;    RLF_OC	0b00	1101		00 00df ffff
	;    SWAPF_OC	0b00	1110		00 00df ffff
	;    INCFSZ_OC	0b00	1111		00 00df ffff
	;
	rrf NextCommand,W
	rrf NextCommand,W
	; so we increment the program counter by the instruction value 
	addwf PCL,F
	goto MOVWF_INSTRUCTION	;adds nothing to program counter
	goto CLRW_INSTRUCTION
	goto SUBWF_INSTRUCTION
	goto DECF_INSTRUCTION
	goto IORWF_INSTRUCTION
	goto ANDFW_INSTRUCTION
	goto XORWF_INSTRUCTION
	goto ADDFW_INSTRUCTION
	goto MOVF_INSTRUCTION
	goto COMF_INSTRUCTION
	goto INCF_INSTRUCTION
	goto DECFSZ_INSTRUCTION
	goto RRF_INSTRUCTION
	goto RLF_INSTRUCTION
	goto SWAPF_INSTRUCTION
	goto INCFSZ_INSTRUCTION
    
	
    CONTROL_COMMAND_LIST:
;
;			  NextCommand|Arg
;	    RETLW_OC    0b0000  1000 kkkk kkkk   
;	    CALL_OC	0bkkkk  1001 kkkk kkkk
;	    GOTO_OC	0bkkkk  101k kkkk kkkk
;	    MOVLW_OC    0b0000  1100 kkkk kkkk
;	    IORLW_OC    0b0000  1101 kkkk kkkk    
;	    ANDLW_OC    0b0000  1110 kkkk kkkk
;	    XORLW_OC    0b0000  1111 kkkk kkkk
;	     observations
;	     
;	     The lower 3 bits of NextCommand are incremented by one
;	
	movlw 0b111
	andwf NextCommand, W
	
	movwf PCL
	
	goto RETLW_INSTRUCTION
	goto CALL_INSTRUCTION
	goto GOTO_INSTRUCTION
	goto GOTO_INSTRUCTION	; can be two or three
	goto MOVLW_INSTRUCTION
	goto IORLW_INSTRUCTION
	goto ANDLW_INSTRUCTION
	goto XORLW_INSTRUCTION

	

    SPECIAL_COMMAND_LIST:
	//            |Arg starts here
	// NOP    00 0000 0000
	// CLRWDT 00 0000 0100
	// OPTION 00 0000 0010
	// SLEEP  00 0000 0011
	// TRIS   00 0000 0fff where f = 6 or 7 (for some reason)
	//		     
	; bit 2 being set will show that this is either a CLRWDT or a TRIS command
	movf Arg,F
	btfsc STATUS,2		    ;Zero Flag indicates NOP
	goto END_CMD
	
	btfsc Arg,2
	goto OPTION_OR_SLEEP
	
	
	
	OPTION_OR_SLEEP:
	    btfss Arg,0
	    goto OPTION_INSTRUCTION
	    goto SLEEP_INSTRUCTION
	

// EXECUTE ENDS HERE
; ADDFW REG, d
ADDFW_INSTRUCTION:	    ; complete
    movf s_WREG, W	    ; restore WREG
    addwf INDF, F
    goto CHECK_DESTINATION

; ANDFW REG, d
ANDFW_INSTRUCTION:	    ; complete
    movf s_WREG, W	    ; restore WREG
    andwf INDF, F
    goto CHECK_DESTINATION

; clrf f
CLRW_INSTRUCTION:	    ; complete
CLRF_INSTRUCTION:
    ; clrf and clrw are aliased
    ; bit 5 will distinguish them
    ; it is set for CLRF and cleared for CLRW
    btfsc Arg,5
    clrf INDF
    btfss Arg,5
    clrf s_WREG 
    goto CHECK_DESTINATION    
COMF_INSTRUCTION:		; complete
    comf INDF, F
    goto CHECK_DESTINATION

DECF_INSTRUCTION:		; complete
    decf INDF, F
    goto CHECK_DESTINATION
DECFSZ_INSTRUCTION:		; complete
    decfsz INDF
    comf VPCL_Update_Req	; setf VPCL_Update_Req will remain zero if --INDF != 0
    goto CHECK_DESTINATION

INCF_INSTRUCTION:		; complete
    incf INDF
    goto CHECK_DESTINATION
    
INCFSZ_INSTRUCTION:		; complete
    incfsz INDF
    comf VPCL_Update_Req
    goto CHECK_DESTINATION
    
IORWF_INSTRUCTION:		; complete
    ;restore WREG
    movf s_WREG,W
    IORWF INDF
    goto CHECK_DESTINATION
MOVF_INSTRUCTION:		; complete
    movf INDF, W
    movwf s_WREG
    goto CHECK_DESTINATION
MOVWF_INSTRUCTION:
    movf s_WREG
    movwf INDF
    goto END_CMD
NOP_INSTRUCTION:		; complete
    goto END_CMD
   
RLF_INSTRUCTION:		; complete
    rlf INDF,F
    goto CHECK_DESTINATION
    
RRF_INSTRUCTION:		; complete
    rrf INDF,F
    goto CHECK_DESTINATION
SWAPF_INSTRUCTION:
    swapf INDF,F
    goto CHECK_DESTINATION
SUBWF_INSTRUCTION:		; complete
    movf s_WREG,W
    subwf INDF
    goto CHECK_DESTINATION
XORWF_INSTRUCTION:		; complete
    movf s_WREG,W
    xorwf INDF
    goto CHECK_DESTINATION
    
BIT_COMMAND_LIST:
BCF_INSTRUCTION:
BSF_INSTRUCTION:  
BTFSC_INSTRUCTION:
BTFSS_INSTRUCTION:
    ;BSF GPIO,2
    //       NextCommand | Arg
    // opcode 0b0000 0100 bbbf ffff BCF
    //	      0b0000 0101 bbbf ffff BSF
    //        0b0000 0110 bbbf ffff BTFSC
    //        0b0000 0111 bbbf ffff BTFSS
    ;FILE &= ~BITS
    
    ;find file
    movf Arg,W
    andlw 0x1f
    movwf FSR
    
    
    clrf tempReg
    INCF tempReg
    ;find bits
    swapf Arg, f
    rrf Arg,W
    andlw 0b111
    movwf Arg
    
    btfsc STATUS, 2	; check if bit zero is being set/cleared
    goto SKIP_FIND_BIT
    ; convert the bit number into a bit
    ; 7 -> 1000 0000
    FIND_BIT:
    rlf tempReg	    ; tempReg = 1 << WREG
    decfsz Arg
    goto FIND_BIT
    
    SKIP_FIND_BIT:
    ; distinguish bit set/clear from bit tests
    btfss NextCommand,1	    
    goto FINISH_BIT_TESTS
    
    btfss NextCommand,0
    goto FINISH_BSF
    ; Bit Clear instruction
    ;Bits = ~Bits
    comf tempReg, w
    ; INDF = INDF & ~BITS
    andwf INDF,f
    goto END_CMD
    
    FINISH_BSF:
    
        ; INDF = INDF | BITS
	iorwf  INDF, F
	goto END_CMD
	
    FINISH_BIT_TESTS:
        btfss NextCommand,0
	goto FINISH_BTFSS
	; btfsc
	FINISH_BTFSS:
	    ;TODO
	    
	    

ANDLW_INSTRUCTION:
    movf Arg,W
    andwf INDF
    goto END_CMD



GOTO_INSTRUCTION:
    // set virtual program counter
    // OPCODE	0000 101k kkkk kkkk
    // EXT	kkkk 1011 kkkk kkkk - goto
    //		kkkk 1001 kkkk kkkk - call
    clrf VPCL_H
    btfsc NextCommand,0
    incf VPCL_H
    goto EXTENDED_ADDRESS
    CALL_INSTRUCTION:
	clrf VPCL_H
	EXTENDED_ADDRESS:
	    swapf NextCommand,W	; if extended address is not used, this will just be zero
	    andlw 0xf
	    addwf VPCL_H,f

	    movlw Arg
	    movwf VPCL_L

	    goto END_CMD
    
IORLW_INSTRUCTION:  ; complete
    movf Arg,W
    iorlw s_WREG
    goto END_CMD
    
MOVLW_INSTRUCTION:
    movf Arg,W
    movwf s_WREG
    goto END_CMD
OPTION_INSTRUCTION:
    movf s_WREG,W
    OPTION
    goto END_CMD
RETLW_INSTRUCTION:
    movf Arg,W
    movwf s_WREG
    goto END_CMD
SLEEP_INSTRUCTION:
    SLEEP
    ;goto END_CMD
    
CLRWDT_INSTRUCTION:	    ; complete
    clrwdt
    goto END_CMD	
TRIS_INSTRUCTION:
    movf s_WREG,W
    TRIS GPIO
    goto END_CMD
XORLW_INSTRUCTION:
    movf Arg,W
    XORWF INDF
    goto END_CMD
    

RAM_DUMP:
    movlw systemRam
    movwf FSR               ; set pointer to address of the highest byte of ram

    RAM_TX_LOOP:
        call_ I2C_TX
        decfsz FSR, f       ; decrement until we run out of memory
        goto RAM_TX_LOOP    ; note that zero is no implemented in RAM (INDF - virtual register)
END 
