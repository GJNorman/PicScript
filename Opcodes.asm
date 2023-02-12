;OP-CODEs
#ifndef OPCODES_H
#define OPCODES_H	
			    
#define ADDWF_OC    0b00011100
#define ANDWF_OC    0b00010100
#define CLRF_OC	    0b00000100 ;1 -> note set to alias with CLRW on purpose
#define CLRW_OC	    0b00000100
#define COMF_OC	    0b00100100
#define DECF_OC	    0b00001100
#define DECFSZ_OC   0b00101100 
#define INCF_OC	    0b00101000
#define INCFSZ_OC   0b00111100
#define IORWF_OC    0b00010000
#define MOVF_OC	    0b00100000
#define NOP_OC	    0b000000	;special
#define RLF_OC	    0b00110100
#define RRF_OC	    0b00110000
#define SUBWF_OC    0b00001000
#define SWAPF_OC    0b00111000
#define XORWF_OC    0b00011000
#define BCF_OC	    0b010000
#define BSF_OC	    0b010100
#define BTFSC_OC    0b011000
#define BTFSS_OC    0b011100
#define ANDLW_OC    0b111000
#define CALL_OC	    0b100100
#define CLRWDT_OC   0b000000	;special
#define GOTO_OC	    0b101000
#define IORLW_OC    0b110100
#define MOVLW_OC    0b110000
#define OPTION_OC   0b000000	;special
#define RETLW_OC    0b100000
#define SLEEP_OC    0b000000	;special
#define TRIS_OC	    0b000000	;special
#define XORLW_OC    0b111100
  

			
CHECK_COMMAND_TYPE  MACRO   REG, MASK, jmp_label
			    movf REG, W
			    andlw MASK
			    btfss STATUS,2  ;zero flag
			    goto jmp_label
		    ENDM
CHECK_COMMAND_OC  MACRO  REG, CMD_MASK, jmp_label
			movf REG, W
			xorlw CMD_MASK
			btfss STATUS, 2 ;Zero flag
			goto jmp_label
		  ENDM

CHECK_SPECIAL_OC  MACRO  REG, CMD_MASK, Instruction
			movf REG, W
			xorlw CMD_MASK
			btfss STATUS, 2 ;Zero flag
			Instruction
		    ENDM
;use the last nibble to decipher special commands
#define SPECIAL_OC  0b00000000
#define SLEEP_NI    0b00000011	;special
#define TRIS_NI	    0b00000111	;special -> is hardcoded as 6
#define OPTION_NI   0b00000010	;special  
#define CLRWDT_NI   0b00000100	;special
#define NOP_NI	    0b00000000	;special


#endif
