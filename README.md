# PicScript
Assembly interpreter for Pic10 Microcontrollers, intended for debugging and learning

### Goals 
The goal of this project is to have a Pic10f202/206/322 microcontroller (or any with 512 words of FLASH memory) read an assembly instruction over I2C and correctly interpret it

The instructions will be read from either a parent MCU or external memory chip

The secondary goal is to build a debugging tool using an arduino

The arduino will send a single instruction at a time (sent by the user over serial console)

after each isntruction is completed, the arduino will read the entire RAM contents and display to the user. 

It will also connect each output pin to an ADC (for example - to measure the result of any bitbanging), for easier verification of scripts

It's also intended to have the PIC be able to read instructions off of external EEPROM/Flash memory.

### The benefits of this

- Potentially larger program memory (optional ability to extend CALL/GOTO instruction addresses by 4 bits (12 - 13 bit address in total))
- Easier reprogrammability - the PIC firware can be bundled with a parent microcontroller on the same board (If present)
It also improves flexibility and development time, since the MCUs could be programmed with the interpreter (and therefore, the PCBS also manufactured)
before the main application had been written. 
- Better debugging

### The cons
- Much slower execution. Just a few percent of what the MCU is otherwise capable
- Requires about half of available RAM on pic10f202/206 
- Requires two pins for I2C, but i'm looking into 1-Wire as an alternative

at the moment, each instruction is processed as fast as it can be. But this means that each instruction has different timing. I may add some delays later to even these out
so that execution is more predictable (but slower)

### Applications
As a learning or debugging Tool

Commercial tool in very niche applications 

### How it works

The program has a software I2C implementation (Master mode)

The MCU will keep a "virtual program counter" (16 bits) - it uses this as a Read address during I2C exchanges

It will read two bytes of data from the host, then increment the virtual address. CALL and GOTO instructions will directly modify the Virtual address

It will interpret and excute the instruction

There is also the option for adding some custom commands - since we pack each 14-bit instruction into 16 bits. For Example
- Reading and Writing RAM Addresses
- Relative Branching instructions (to deal with potentially larger address space)
- Convenience Instructions (movff, movlf etc.)

but my goal is to maintain compatibility with the existing instruction OPCODES

### Current Status

The I2C implementation is working with an arduino. I still need to test that all 35 built in instructions are being interpreted correctly

I will be testing first on the PIC10F206. If this is successful I will move onto the PIC10F322. I plan to attempt to implement Interrupt routines by storing the instructions in RAM
