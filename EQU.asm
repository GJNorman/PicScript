SCL_PIN	equ	1
SDA_PIN equ	0
 

SFR_registers	equ	8
userRAM		equ	24
systemRam	equ     SFR_registers+userRAM
slaveI2CAddr    equ	0b11001101	; use 8 bits, MS 7 bits are address, last bit should be one (for read command)

tempReg		equ     0+SFR_registers
I2C_RR		equ     1+SFR_registers		; I2C read register
I2C_WR		equ     2+SFR_registers		; I2C Write Register

VPCL_H		equ	3+SFR_registers		; virtual program counter high/low
VPCL_L		equ	4+SFR_registers
s_WREG		equ	5+SFR_registers		; shadow WREG	
s_INDF		equ     6+SFR_registers		
; for incoming instructions
NextCommand	equ	7+SFR_registers
Arg		equ	8+SFR_registers		; file or literal
VPCL_Update_Req equ	9+SFR_registers


