SCL_PIN	equ	1
SDA_PIN equ	0
LED	equ     2

SFR_registers	equ	8
userRAM		equ	24
systemRam	equ     SFR_registers+userRAM
slaveI2CAddr    equ	0b11001100	; use 8 bits, MS 7 bits are address, last bit should be one (for read command)
I2C_ACK_BIT	equ     0
I2C_NACK_BIT	equ	1
tempReg		equ     0+SFR_registers
/*
		2 and 3 currently empty
*/

VPCL_H		equ	3+SFR_registers		; virtual program counter high/low
VPCL_L		equ	4+SFR_registers
s_WREG		equ	5+SFR_registers		; shadow WREG	
s_INDF		equ     6+SFR_registers	
s_STATUS	equ     7+SFR_registers
; for incoming instructions
NextCommand	equ	8+SFR_registers
Arg		equ	9+SFR_registers		; file or literal
VPCL_Update_Req equ	10+SFR_registers

;I2C Loading
I2C_Adddress_REG    equ	    11+SFR_registers
I2C_ADDR_H_REG	    equ	    12+SFR_registers
I2C_ADDR_L_REG	    equ	    13+SFR_registers
I2C_CTRL	    equ	    14+SFR_registers
I2C_RR		    equ     15+SFR_registers		; I2C read register
I2C_RR1		    equ     16+SFR_registers		; I2C read register
I2C_WR		    equ     17+SFR_registers		; I2C Write Register
I2C_WR2		    equ     18+SFR_registers		; I2C Write Register
 ;control whether this is a read or write
 ;and whether it uses an 8 or 16 bit address
    AddressSize	    equ 7   ; 0 = 8 bit, 1 = 16 bit
    CommandType	    equ 6   ; 0 = R, 1 = W
    IncomingSize    equ 5   ; 0 = 8 bit, 1 = 16 bit
    QuickRW	    equ 4   ; 0 = false, 1 = true; Quick Read or Write
	    
	    
AddressSize16Bits equ (1<<AddressSize)
AddressSize8Bits  equ (0<<AddressSize)
I2C_Select_READ	  equ (0<<CommandType)
;12C_16bit_Read	  equ (1<<IncomingSize)
;I2C_8bit_Read     equ (0<<IncomingSize)
