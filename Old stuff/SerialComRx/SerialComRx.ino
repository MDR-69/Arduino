/*
For more details see: http://projectsfromtech.blogspot.com/

Connect the Rx pin of this board to the Tx of the board running Serial_Comm_Tx.ino
Connect the Grounds of the two boards
Open Serial Monitor


Receives integer value and prints it to the serial monitor
*/

#include <SPI.h>
#include "printf.h"

int val;



void setup()
{
  Serial.begin(57600);
  Serial.println("Serial Monitor Connected");
  printf_begin();

  printf("\n\rSerial monit active \n\r");  
}

void loop()
{
  int incoming = Serial.available();
  
  if (incoming > 0)
  {
   val = Serial.parseInt();  //Reads integers as integer rather than ASCI. Anything else returns 0
   //Serial.println(val);
   printf("\n\rGot a value : %d",val);
  }
}
