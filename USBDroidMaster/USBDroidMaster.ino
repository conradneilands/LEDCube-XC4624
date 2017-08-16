//
//START SHARED INCLUDES AND INIT
//
unsigned int device0Level = 8;
unsigned int device0Offset = 1;
unsigned long device0LastTx = 0;
unsigned int device1Level = 8;
unsigned int device1Offset = 0;
unsigned long device1LastTx = 0;
unsigned int device2Level = 8;
unsigned int device2Offset = 2;
unsigned long device2LastTx = 0;
unsigned int device3Level = 8;
unsigned int device3Offset = 1;
unsigned long device3LastTx = 0;

//
//END SHARED INCLUDES AND INIT
//

//
//START USB HOST INLCUDES AND INIT
//
//will need to modify libraries/USB_HOST_SHEILD/cdcftdi.h
//to add PID and VIDs
//Bus 001 Device 010: ID 0403:6015 Future Technology Devices International, Ltd 
//#define FTDI_VID                        0x0403  // FTDI VID
//#define FTDI_PID                        0x6015  // FTDI PID
#include <Wire.h>
#include <cdcftdi.h>
#include <usbhub.h>
#include "pgmstrings.h"
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
class FTDIAsync : public FTDIAsyncOper
{
public:
    virtual uint8_t OnInit(FTDI *pftdi);
};
uint8_t FTDIAsync::OnInit(FTDI *pftdi)
{
    uint8_t rcode = 0;

    rcode = pftdi->SetBaudRate(115200);

    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetBaudRate"), rcode);
        return rcode;
    }
    rcode = pftdi->SetFlowControl(FTDI_SIO_DISABLE_FLOW_CTRL);

    if (rcode)
        ErrorMessage<uint8_t>(PSTR("SetFlowControl"), rcode);

    return rcode;
}
USB              Usb;
//USBHub         Hub(&Usb);
FTDIAsync        FtdiAsync;
FTDI             Ftdi(&Usb, &FtdiAsync);
uint32_t next_time;
//
//END USB HOST INLCUDES AND INIT
//

//
//START LED CONTROL INCLUDES
//
#include <Wire.h>
// Example of using I2C to drive a Chromoduino/Funduino LED matrix slave
// Mark Wilson Dec '16
#define I2C_ADDR 0x05  // the Chromoduino/Funduino I2C address
#define RST_PIN A0     // this pin is connected to DTR on the Chromoduino/Funduino
byte balanceRGB[3] = {22, 63, 63};
byte colourRGB[3] = {128, 0, 128};
unsigned int ledMap[64]      = {0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,2,2,3,3,3,3,2,2,2,2,3,3,3,3,2,2,2,2,3,3,3,3,2,2,2,2,3,3,3,3,2,2};
unsigned int ledLevelMap[64] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0};
byte offRGB[3] = {0,0,0};
byte redRGB[3] = {255,0,0};
byte greenRGB[3] = {0,128,0};
byte blueRGB[3] = {0,0,128};
byte yellowRGB[3] = {255,255,0};
//This array is read 4 elements at a time, as sound levels increase we move closer to zero
byte* levelColourState[13] = {redRGB,redRGB,redRGB,redRGB,redRGB,yellowRGB,greenRGB,blueRGB,offRGB,offRGB,offRGB,offRGB,offRGB};
//byte* levelColourState[11] = {offRGB,offRGB,offRGB,offRGB,blueRGB,greenRGB,yellowRGB,redRGB,redRGB,redRGB,redRGB};

bool Configure()
{
  // write the balance, true if got expected response from matrix
  StartBuffer();
  WriteRGB(balanceRGB);
  return SetBalance();
}
inline void StartBuffer()
{
  // start writing
  Wire.beginTransmission(I2C_ADDR);
  Wire.write((byte)0x00);
  Wire.endTransmission();
  delay(1);
}
inline void WriteRGB(byte* pRGB)
{
  // write a triple
  Wire.beginTransmission(I2C_ADDR);
  for (int idx = 0; idx < 3; idx++, pRGB++)
    Wire.write(*pRGB);
  Wire.endTransmission();
  delay(1);
}
inline void ShowBuffer()
{
  // flip the buffers
  Wire.beginTransmission(I2C_ADDR);
  Wire.write((byte)0x01);
  Wire.endTransmission();
  delay(1);
}
bool SetBalance()
{
  // true if there are 3 bytes in the slave's buffer
  Wire.requestFrom(I2C_ADDR, 1);
  byte count = 0;
  if (Wire.available())
    count = Wire.read();
    
  Wire.beginTransmission(I2C_ADDR);
  Wire.write((byte)0x02); // set the 3 bytes to be balance
  Wire.endTransmission();
  delay(1);
  return count == 3;  // the slave got 3 bytes
}
/*
void DisplayBuffer()
{
  // fill the buffer and display it
  StartBuffer();
  for (int row = 0; row < 8; row++)
    for (int col = 0; col < 8; col++)
      WriteRGB(colourRGB);
  ShowBuffer();
}
*/
inline void SetColumnDisplayBuffer()
{
  // fill the buffer and display it
  StartBuffer();
  //colours written in sequence 0 to 63 then pushed to display 
  for (int i = 0; i < 64; i++)
  {
    unsigned int thislevel=ledLevelMap[i];
    unsigned int thiscoloumn=ledMap[i];
    if(thiscoloumn == 0)
    {
        unsigned int thisledval = thislevel+device0Level;
        if(thisledval == 8)
        {
          byte twinkleRGB[3] = {0,0,random(0,255)};
          WriteRGB(twinkleRGB);
        }
        else
        {
          WriteRGB(levelColourState[thisledval]);
        }
    }
    else if(thiscoloumn == 1)
    {
        unsigned int thisledval = thislevel+device1Level;
        if(thisledval == 8)
        {
          byte twinkleRGB[3] = {0,0,random(0,255)};
          WriteRGB(twinkleRGB);
        }
        else
        {
          WriteRGB(levelColourState[thisledval]);
        }
    }
    else if(thiscoloumn == 2)
    {
        unsigned int thisledval = thislevel+device2Level;
        if(thisledval == 8)
        {
          byte twinkleRGB[3] = {0,0,random(0,255)};
          WriteRGB(twinkleRGB);
        }
        else
        {
          WriteRGB(levelColourState[thisledval]);
        }
    }
    else if(thiscoloumn == 3)
    {
        unsigned int thisledval = thislevel+device3Level;
        if(thisledval == 8)
        {
          byte twinkleRGB[3] = {0,0,random(0,255)};
          WriteRGB(twinkleRGB);
        }
        else
        {
          WriteRGB(levelColourState[thisledval]);
        }
    }          
  }
  ShowBuffer();
}

inline void CheckDeviceTimeouts()
{
    unsigned long timeout = millis()-5000;
    if(device0LastTx < timeout)
    {
        device0Level = 9;
        //Serial.println("D0:Timeout");
    }
    if(device1LastTx < timeout)
    {
        device1Level = 9;
        //Serial.println("D1:Timeout");
    }
    if(device2LastTx < timeout)
    {
        device2Level = 9;
        //Serial.println("D2:Timeout");
    }
    if(device3LastTx < timeout)
    {
        device3Level = 9;
        //Serial.println("D3:Timeout");
    } 
}
//
//END LED CONTROL INCLUDES
//

void setup()
{
  Serial.begin( 115200 );
  Serial.println("BEGIN SETUP");
  //START USB HOST SETUP
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  Serial.println("USB HOST SETUP START");
  if (Usb.Init() == -1)
  {
      Serial.println("OSC did not start.");
  }
  delay( 200 );
  next_time = millis() + 5000;
  Serial.println("USB HOST SETUP END");
  //END USB HOST SETUP

  //START LED CONTROL SETUP
  Wire.begin();
  Serial.println("LED CONTROL SETUP START");
  // reset the board
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(1);
  digitalWrite(RST_PIN, HIGH);
  // keep trying to set the balance until it's awake
  do
  {
    delay(100);
  } while (!Configure());
  SetColumnDisplayBuffer();
  Serial.println("LED CONTROL SETUP END");
  //END LED CONTROL SETUP

  Serial.println("END SETUP");
  //Comment out for further debug
  //Serial.end();
}

void loop()
{
  
    //START USB HOST LOOP TASK 
    Usb.Task();
    if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
    {
        uint8_t  rcode;
        uint8_t  buf[64];
        String response = "";
        uint8_t thisDeviceId = 0;
        for (uint8_t i=0; i<64; i++)
        {
            buf[i] = 0;
        }
        uint16_t rcvd = 64;
        rcode = Ftdi.RcvData(&rcvd, buf);
        if (rcode && rcode != hrNAK)
        {
            ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
        }
        // The device reserves the first two bytes of data
        //   to contain the current values of the modem and line status registers.
        if (rcvd > 2)
        {
            //Serial.print((char*)(buf+2));
            response = (char*)(buf+2);
            //Serial.println(response);
            if(response.substring(0,1) == "D")
            {
              if(isDigit(response.charAt(1)))
              {
                String thisDevId = response.substring(0,2);
                uint8_t this1sdB = response.substring(3).toInt();
                uint8_t thisMapped = map(this1sdB,65,102,7,0);
                if(0 <= thisMapped && thisMapped <=7)
                {
                  if(thisDevId == "D0")
                  {
                      thisMapped= thisMapped + device0Offset;//crude calibration
                      device0Level = thisMapped;
                      device0LastTx = millis();
                  }
                  else if(thisDevId == "D1")
                  {
                      thisMapped = thisMapped + device1Offset;//crude calibration
                      device1Level = thisMapped;
                      device1LastTx = millis();
                  }
                  else if(thisDevId == "D2")
                  {
                      thisMapped = thisMapped + device3Offset;//crude calibration
                      device2Level = thisMapped;
                      device2LastTx = millis();
                  }
                  else if(thisDevId == "D3")
                  {
                      thisMapped = thisMapped + device3Offset;//crude calibration
                      device3Level = thisMapped;
                      device3LastTx = millis();
                  }   
                  
                  //Set and display LEDs       
                  CheckDeviceTimeouts();               
                  SetColumnDisplayBuffer();

                  //Debug
                  Serial.print(thisDevId);Serial.print(":");Serial.println(thisMapped);
                }
              }
            }
        }
        else
        {
          //Write the buffer even if we don't need to refresh the twinkle leds
          CheckDeviceTimeouts();  
          SetColumnDisplayBuffer();
        }
   }
  else
  {
    //Write the buffer even if we don't need to refresh the twinkle leds
    CheckDeviceTimeouts();  
    SetColumnDisplayBuffer();
  }

   //END USB HOST LOOP TASK
}

