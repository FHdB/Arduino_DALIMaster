// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


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
