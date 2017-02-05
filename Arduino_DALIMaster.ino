// DALI Master Arduino Code
// by Florian Heptner https://github.com/FHdB

/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
der GNU General Public License, wie von der Free Software Foundation,
Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
veröffentlichten Version, weiterverbreiten und/oder modifizieren.
Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
Siehe die GNU General Public License für weitere Details.
Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
*/
/*
DALI Master Arduino Code
Made for being used in combination with seven segment and rotary encoder module
https://github.com/FHdB/7Segment-Display-with-Rotary-Encoder/blob/master/SevSegment_RotaryEncoder/SevSegment_RotaryEncoder.ino
*/

#include <Wire.h>

#define  CHLVL1 A0
#define  CHLVL2 A1
#define  CHLVL3 A2
#define  CHLVL4 A3

uint8_t i2c_rec[6];
uint8_t dimValue[] = {0 , 0 , 0 ,0 };
uint8_t address[]  = {233 , 233 , 233 ,233 };

// Warrior defines
#define warrior_addr 0b100000

// dim button
#define INTERRUPTPIN_DIM 2
boolean button_pressed = false;

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(50000L);
  Serial.begin(9600);  // start serial for output
  Serial.println("Start");
  pinMode(2,INPUT);
  
  //change level pins
  pinMode(CHLVL1, INPUT);
  digitalWrite(CHLVL1 ,LOW);
  pinMode(CHLVL2, INPUT);
  digitalWrite(CHLVL2 ,LOW);
  pinMode(CHLVL3, INPUT);
  digitalWrite(CHLVL3 ,LOW);
  pinMode(CHLVL4, INPUT);
  digitalWrite(CHLVL4 ,LOW);
  
  //interrupt pin
  pinMode(INTERRUPTPIN_DIM, INPUT);
  pciSetup(INTERRUPTPIN_DIM);
}

void loop()
{
  int pin_1_state = digitalRead(A0);
  int pin_2_state = digitalRead(A1);
  int pin_3_state = digitalRead(A2);
  int pin_4_state = digitalRead(A3);
  if(pin_1_state==1u)
  {
    Serial.println("requesting");
    requestDimAndAddr(2u, 0u);
    Serial.println("finished");
  }
  
//  if(pin_2_state==1u)
//  {
//    Serial.println("requesting");
//    requestDimAndAddr(3u, 1);
//    Serial.println("finished");
//  }
//  if(pin_3_state==1u)
//  {
//    Serial.println("requesting");
//    requestDimAndAddr(4u, 2);
//    Serial.println("finished");
//  }
//  if(pin_4_state==1u)
//  {
//    Serial.println("requesting");
//    requestDimAndAddr(5u, 3);
//    Serial.println("finished");
//  }
    
  if(button_pressed)
  {
    dali_dim_dev(address[0], dimValue[0]);
//    dali_dim_dev(address[1], dimValue[1]);
//    dali_dim_dev(address[2], dimValue[2]);
//    dali_dim_dev(address[3], dimValue[3]);
    button_pressed = false;
    //pciSetup(INTERRUPTPIN_DIM);
  }
}

void requestDimAndAddr(uint8_t i2c_addr, uint8_t index)
  {
    Wire.requestFrom(i2c_addr, 6);    // request 6 bytes from slave device #2
  
    uint8_t i = 0;
    while (Wire.available())   // slave may send less than requested
    {
      uint8_t c = Wire.read(); // receive a byte as character
      i2c_rec[i] = 0;
      i2c_rec[i] = c;
      i++;
    }
    
    if(i2c_rec[0] == 'a')
    {
      address[index]  = i2c_rec[1];
    }
  
    if(i2c_rec[3] == 'd')
    {
      dimValue[index] = i2c_rec[4];
    }
    Serial.print("address: ");
    Serial.println(address[index],DEC);
    Serial.print("dimValue: ");
    Serial.println(dimValue[index],DEC);
  }
  
void setDimAndAddr(uint8_t i2c_addr, uint8_t index)
{
  uint8_t sendArray[] = {'a',0,' ','d',0,' '};
  Serial.println("Sending: ");
  sendArray[1] = (address[index]+1)&(0xFF);
  Serial.print("address: ");
  Serial.println(sendArray[1]);
  sendArray[4] = (dimValue[index]+1)&(0xFF);
  Serial.print("dimValue: ");
  Serial.println(sendArray[4]);

  Serial.println("-------");

  Wire.beginTransmission(2); // transmit to device #8
  Wire.write(sendArray,6);        // sends five bytes
  boolean ok = Wire.endTransmission();    // stop transmitting

  if(ok!=0)
  {
    Serial.print("Transmission success: ");
    Serial.println(ok);
  }
}


/*------------------------------*/

// DALI Adapter Warrior09 commands

boolean dali_dim_dev(uint8_t dali_addr, uint8_t dali_dim){
  //dali_addr is expected to be a value from 0 to 64
  //if dali_addr = 0 : broadcast
  if(dali_addr==0u)
  {
    dali_addr = 0b01111111;
  }else{
    //make sure that dali_addr is max 63
    dali_addr = dali_addr&0b00111111;
    dali_addr = dali_addr-1;
  }
  
  //dali_dim is expected to be a value from 0 to 100
  // map to 0:254
  dali_dim = map(dali_dim,0,100,0,254);  
  
    uint8_t dali_bus_status = 1;
    
    Wire.requestFrom(warrior_addr,1);
    while(Wire.available()){
      dali_bus_status = Wire.read();
      Serial.println(dali_bus_status,BIN);
      if(dali_bus_status)
         {
           Serial.println("DALI BUS not ready");   
         }
    }

    //if DALI BUS is ready (==0) send dim command to addr       
    if(!dali_bus_status){
        Serial.print("Writing DALI Addr: ");
        Serial.println(dali_addr);
        
        Wire.beginTransmission(warrior_addr); // transmit to device #4
        Wire.write(0x01);
        Wire.write((dali_addr<<1));
        Wire.write(dali_dim);
        boolean bus_ack = Wire.endTransmission();    // stop transmitting
        if(bus_ack==0){
            Serial.print("Command: ");
            Serial.println((dali_addr<<1),BIN);
            Serial.println("Succesfully sent");         
        }
    }
}



/*Inhterrupt Section*/
// enable interrupt for pin
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

//disable interrupt
void pciTurnOff(byte pin)
{
    *digitalPinToPCMSK(pin) = *digitalPinToPCMSK(pin) &~ (bit (digitalPinToPCMSKbit(pin)));  // disable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}



// Interrupt service routine for "dim" button
ISR (PCINT2_vect) // handle pin change interrupt for 1 to 7 (I think...) here
{
  Serial.println("Pin 2 interrupt");
  //pciTurnOff(INTERRUPTPIN_DIM);
  button_pressed = true;
}
