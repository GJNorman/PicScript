#include <Wire.h>

constexpr int i2cBufferSize = 2;
volatile byte i2cRegister[i2cBufferSize];
size_t counter=0;

void setup() {
    // put your setup code here, to run once:
    
    Wire.begin(B1100110);         // join i2c bus (address optional for master)
    Serial.begin(9600);           // start serial for output
    Wire.onReceive(i2cReceive);   // stores address of program counter
    Wire.onRequest(requestEvent); // responds with opcode at program counter
}
void i2cReceive(int bytesReceived) 
{
       
      i2cRegister[counter++] = Wire.read();
      if(counter>=i2cBufferSize)
      {
            counter=0;
      }

      // onReceive will not be invoked unless rxBuffer is empty.
      while (Wire.available()) 
      { 
        i2cRegister[counter++]  = Wire.read();

        if(counter>=i2cBufferSize)
        {
            counter=0;
        }
      }
}
void requestEvent() 
{

      uint16_t address = i2cRegister[0]<<8;
      address += i2cRegister[1];
      //if(address == 127)
      {
        // test code - 0x546 - BSF GPIO, 2 (the LED)
        //           - 0x446 - BCF GPIO, 2 (the LED)
        // This code will blink the LED
        static int flipflop = 0;

        byte value = 0x5;
        flipflop++;
        if(flipflop<=500)
        {
            value = 4;
        }
        else if( flipflop ==1000)
        {
            flipflop=0;
        }
        Wire.write(value);
        Wire.write(0x46);
      }
 }
void loop() 
{
  // do nothing
}
