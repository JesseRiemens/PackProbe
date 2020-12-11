/************************************************************************************

PackProbe

by Erik Speckman, modified by Jesse Riemens

For documentation and the latest version see: http://powercartel.com/projects/packprobe/
Source Repository: git@github.com:PowerCartel/PackProbe.git

Depends on SoftI2CMaster http://playground.arduino.cc/Main/SoftwareI2CLibrary

PackProbe allows you to obtain detailed history and health information on laptop battery 
packs without access to the corresponding laptop.

This is is a proof of concept. It doesn't handle error conditions and spurious values

PackProbe is based on:
  http://linuxehacking.blogspot.com/2014/03/recharging-and-reusing-acer-laptop.html 
  and http://forum.arduino.cc/index.php?topic=62955.0

*************************************************************************************/
#ifndef PackProbeJesse_h
#define PackProbeJesse_h


#include <Arduino.h>
#include <Wire1.h>


// Standard and common non-standard Smart Battery commands

#define BATTERY_MODE            0x03
#define TEMPERATURE             0x08
#define VOLTAGE                 0x09
#define CURRENT                 0x0A
#define RELATIVE_SOC            0x0D
#define ABSOLUTE_SOC            0x0E
#define REMAINING_CAPACITY      0x0F
#define FULL_CHARGE_CAPACITY    0x10
#define TIME_TO_FULL            0x13
#define CHARGING_CURRENT        0x14
#define CHARGING_VOLTAGE        0x15
#define BATTERY_STATUS          0x16
#define CYCLE_COUNT             0x17
#define DESIGN_CAPACITY         0x18
#define DESIGN_VOLTAGE          0x19
#define SPEC_INFO               0x1A
#define MFG_DATE                0x1B
#define SERIAL_NUM              0x1C
#define MFG_NAME                0x20      // String
#define DEV_NAME                0x21      // String
#define CELL_CHEM               0x22     // String
#define MFG_DATA                0x23      // String
#define CELL4_VOLTAGE           0x3C // Indidual cell voltages don't work on Lenovo and Dell Packs
#define CELL3_VOLTAGE           0x3D
#define CELL2_VOLTAGE           0x3E
#define CELL1_VOLTAGE           0x3F
#define STATE_OF_HEALTH         0x4F

#define SDA_PIN                 PIN_WIRE_SDA1
#define SCL_PIN                 PIN_WIRE_SCL1
#define deviceAddress           11U

void setupPackProbe();

int fetchWord(byte func);

uint8_t i2c_smbus_read_block ( uint8_t command, uint8_t* blockBuffer, uint8_t blockBufferLen );

void scan();

void setupPackProbe(){
  /*


  Comment out the next two lines following lines for use with Serial output
  

  // Bridge.begin();
  // Console.begin();



  Uncomment the following lines for use with Serial output
  

  //Serial.begin(115200);  // start serial for output
  //Serial.println(i2c_init());
  //pinMode(22,INPUT_PULLUP);
  //pinMode(23,INPUT_PULLUP);


  // while (!Console) {    
  //   ; // wait for Console port to connect.
  // }

  // Serial.println("Console Initialized");
  */
  Wire1.begin();
  Serial.println("I2C Inialized");
  scan();
}

int fetchWord(byte func)
{
    byte b1, b2;
    Wire1.beginTransmission((uint8_t)(deviceAddress<<1));
    Wire1.write(func);
    Wire1.endTransmission(false);
    Wire1.requestFrom(deviceAddress, 2);
    Wire1.endTransmission(true);
    if(2 <= Wire1.available()) {
        b1 = Wire1.read();
        b2 = Wire1.read();
    }
    return (int)b1|((( int)b2)<<8);
}

uint8_t i2c_smbus_read_block ( uint8_t command, uint8_t* blockBuffer, uint8_t blockBufferLen ) 
{
    uint8_t x, num_bytes;
    Wire1.beginTransmission((uint8_t)deviceAddress);
    Wire1.write(command);
    Wire1.endTransmission(false);
    Wire1.requestFrom(deviceAddress, blockBufferLen);
    num_bytes = Wire1.available();
    for ( x = 0; x < num_bytes; x++)
    {
        blockBuffer[x] = Wire1.read();
    }
    Wire1.endTransmission(true);
    blockBuffer[++x] = 0;
    return num_bytes;
}

void scan()
{
    byte i = 0;
    for ( i= 0; i < 127; i++ )
    {
        Serial.print("Address: 0x");
        Serial.print(i,HEX);
        Wire1.beginTransmission(i);
        bool ack = Wire1.endTransmission(); 
        if ( ack ) {
            Serial.println(": OK");
            Serial.flush();
        }
        else {
            Serial.println(": -");
            Serial.flush();      
        }
        
    }
}


#endif