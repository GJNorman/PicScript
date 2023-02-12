
#include <xc.inc>
#include "Opcodes.asm"
#include "EQU.asm"
; CONFIG
  CONFIG  WDTE = OFF            ; Watchdog Timer (WDT enabled)
  CONFIG  CP = OFF              ; Code Protect (Code protection off)
  CONFIG  MCLRE = ON            ; Master Clear Enable (GP3/MCLR pin function  is MCLR)
 
ORG 0x0000
//PSECT   udata_acs


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
                        BSF SCL     ; pulse clock
                        BCF SDA     ; transition to low specifies start/restart condition
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
                        BSF SCL
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
    PSECT   code
    PSECT appcode,class=APP,delta=2 ;this makes the program work, i have no idea why
ORG 0x000
main:
    goto GET_NEXT_INSTRUCTION
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
    
CHECK_VPCL:
    movf VPCL_Update_Req,f
    btfsc STATUS,2
	call_ UPDATE_VPCL
END_CMD:
    ;restore WREG
    movf s_WREG
    
GET_NEXT_INSTRUCTION:
    
    ; grab the first instruction
    movlw slaveI2CAddr	
    call_ I2C_WRITE		; send the slave address
    
    movf VPCL_L, W
    call_ I2C_WRITE		; send program counter
    movf VPCL_H, W
    
    call_ I2C_READ		; read command
    movf I2C_RR,W		; read register holds our next command 
    movwf NextCommand
    
    call_ I2C_READ		; read arguments
    movf I2C_RR,W		; read register holds our next command argument
    movwf Arg
    
    ; end the madness
    STOP_CONDITION
    
    ; increment counter
    call_ UPDATE_VPCL
    
    goto EXECUTE
    
UPDATE_VPCL:
    incf VPCL_L, f		; increment the program counter
    incf VPCL_L, f		; by two
    btfss STATUS,2		; zero flag
    RETLW 0
    incf VPCL_H,f		; increment upper byte
    incf VPCL_H,f
    RETLW 0
; the internal oscialltor runs at 4Mhz, each instruciton takes 4 cycles
; so we have effectively 1Mhz clock
; we want I2c 100 Khz, so can have 10 instructions per clock cycle

;load argument into WREG
I2C_WRITE:
    movwf I2C_WR
    movlw 0xf & (0<<SCL_PIN | 0<<SDA_PIN)
    TRIS GPIO
    movlw 8		; address is 7 bits;
    movwf tempReg
    
    START_CONDITION
    
I2C_TX:
    ; pulse clock low on even, high on odd
    btfsc tempReg,0
    BCF SCL
    btfss tempReg,0
    BSF SCL
    ; we have spent 4 cycles to get here
    ; send a one or zero
    btfsc I2C_WR,7
    BSF SDA
    btfss I2C_WR,7
    BCF SDA

    ; we have spent 8 cycles to get here
    ; I2C_WR <<= 1
    rlf I2C_WR,f	; this will set carry flag if bit 8 was set
    
    ; if(--tempReg == 0) break;
    decfsz tempReg
    ; 12 instructions per byte
    GOTO I2C_TX
    
    RETLW 0

I2C_READ:
    movlw 0xf & (0<<SCL_PIN | 1<<SDA_PIN)
    TRIS GPIO
    BCF STATUS,2	; zero flag
    movlw 8		; address is 7 bits;
    movwf tempReg
    
    START_CONDITION
I2C_RX:
    ; pulse clock low on even, high on odd
    btfsc tempReg,0
    BCF SCL
    btfss tempReg,0
    BSF SCL
    ; we have spent 4 cycles to get here
    ; send a one or zero
    
    rlf I2C_RR,f
    btfsc SDA	    
    bsf I2C_RR,0
    ; we have spent 7 cycles to get here

    ; if(--tempReg == 0) break;
    decfsz tempReg
    ; 10 instructions per byte
    GOTO I2C_TX
    
    RETLW 0

EXECUTE:
    clrf VPCL_Update_Req    
    /*
	Check the type of command

	-File oriented

	-Bit oriented

	-literal and Control
    */
    
    ; nextCommand contains the upper byte of the instruction
    ; the top nibble of Next Command will be empty
    ; the first bit of the instruction code will be bit 3
	btfss NextCommand, 3
	goto CONTROL_COMMAND_LIST	; first bit of control and literal commands is always a one

    ; there are some control commands that are completely empty
	movf NextCommand, f
	btfss STATUS, 2			;zero flag
	goto FILE_COMMAND_LIST	
    
    ; Movwf can alias for one of these commands, so we need to do a test first
    ; it can be distinguished by bit 5 being set in "Arg"
	btfss NextCommand, 5
	goto SPECIAL_COMMAND_LIST

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
	CHECK_COMMAND_OC	NextCommand,ADDWF_OC,	    ADDFW_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,ANDWF_OC,	    ANDFW_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,CLRW_OC,	    CLRW_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,COMF_OC,	    COMF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,DECF_OC,	    DECF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,DECFSZ_OC,	    DECFSZ_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,INCF_OC,	    INCF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,INCFSZ_OC,	    INCFSZ_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,IORWF_OC,	    IORWF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,MOVF_OC,	    MOVF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,RLF_OC,	    RLF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,RRF_OC,	    RRF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,SUBWF_OC,	    SUBWF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,XORWF_OC,	    XORWF_INSTRUCTION


    BIT_COMMAND_LIST:
	CHECK_COMMAND_OC	NextCommand,BCF_OC,	    BCF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,BSF_OC,	    BSF_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,BTFSC_OC,	    BTFSC_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,BTFSS_OC,	    BTFSS_INSTRUCTION
    
    CONTROL_COMMAND_LIST:
	CHECK_COMMAND_OC	NextCommand,ANDLW_OC,	    ANDLW_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,CALL_OC,	    CALL_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,GOTO_OC,	    GOTO_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,IORLW_OC,	    IORLW_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,MOVLW_OC,	    MOVLW_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,RETLW_OC,	    RETLW_INSTRUCTION
	CHECK_COMMAND_OC	NextCommand,XORLW_OC,	    XORLW_INSTRUCTION
    
    SPECIAL_COMMAND_LIST:
	//            |Arg starts here
	// NOP    00 0000 0000
	// CLRWDT 00 0000 0100
	// OPTION 00 0000 0010
	// SLEEP  00 0000 0011
	// TRIS   00 0000 0fff where f = 6 
	//		      0110

	; we will ignore NOP
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
NOP_INSTRUCTION:		; complete
    goto END_CMD
   
RLF_INSTRUCTION:		; complete
    rlf INDF,F
    goto CHECK_DESTINATION
    
RRF_INSTRUCTION:		; complete
    rrf INDF,F
    goto CHECK_DESTINATION
SUBWF_INSTRUCTION:		; complete
    movf s_WREG,W
    subwf INDF
    goto CHECK_DESTINATION
XORWF_INSTRUCTION:		; complete
    movf s_WREG,W
    xorwf INDF
    goto CHECK_DESTINATION
    
BCF_INSTRUCTION:
BSF_INSTRUCTION:  
BTFSC_INSTRUCTION:
BTFSS_INSTRUCTION:
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
    
    ;find bits
    swapf Arg, f
    rrf Arg,W
    andlw 0x7
    movwf Arg
    clrf tempReg
    
    ; convert the bit number into a bit
    ; 7 -> 1000 0000
    FIND_BIT:
    bsf STATUS, 0   ; carry bit
    rlf tempReg	    ; tempReg = 1 << WREG
    decfsz Arg
    goto FIND_BIT
    
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
CALL_INSTRUCTION:
    // set virtual program counter
    // OPCODE  0000 1001 kkkk kkkk
    clrf VPCL_H
    goto SET_NEW_ADDRESS
CLRWDT_INSTRUCTION:
    clrwdt
    goto END_CMD

GOTO_INSTRUCTION:
    // set virtual program counter
    // OPCODE  0000 101k kkkk kkkk
    // EXT     kkkk 1011 kkkk kkkk - goto
    //	       kkkk 1001 kkkk kkkk - call
    clrf VPCL_H
    btfsc NextCommand,0
    incf VPCL_H

    SET_NEW_ADDRESS:
	swapf NextCommand,W	; if extended address is not used, this will just be zero
	andlw 0xf
	addwf VPCL_H,f

	movlw Arg
	movwf VPCL_L

	goto END_CMD
    
IORLW_INSTRUCTION:
    movf Arg,W
    iorlw INDF
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
    goto END_CMD
TRIS_INSTRUCTION:
    movf Arg,W
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
