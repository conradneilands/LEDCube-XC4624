
#include <RFduinoGZLL.h>
device_t role = HOST;
unsigned long resetTimer = 3600000; // 1hour
// Define my custom packet of no more than 20 bytes.
struct myCustomPacket_t
{
  int this1sdB;    // 4 bytes
  int this1mdB;    // 4 bytes
  int this1sV;

 // NOTE: compiler will reserve a byte here not defined
};

struct myCustomPacket_t packet;

void setup()
{
    override_uart_limit = true;

  Serial.begin(115200);
  // start the GZLL stack  
  RFduinoGZLL.begin(role);
  NRF_WDT->CRV = 32768; // Timeout period of 1 s
  NRF_WDT->TASKS_START = 1; // Watchdog start
  
}

void loop()
{
   if(millis() < resetTimer)
   {
      //Should stop tickling the watchdog after the timeout
      NRF_WDT->RR[0] = WDT_RR_RR_Reload;
   }
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  memcpy( &packet, &data[0], len);
  if (device == DEVICE0)
  {
   Serial.print("D0:"); Serial.println(packet.this1sdB);
  }
  else if (device == DEVICE1)
  {
   Serial.print("D1:"); Serial.println(packet.this1sdB);
  }
  else if (device == DEVICE2)
  {
   Serial.print("D2:"); Serial.println(packet.this1sdB);
  }
  else if (device == DEVICE3)
  {
   Serial.print("D3:"); Serial.println(packet.this1sdB);
  }
  else if (device == DEVICE4)
  {
   Serial.print("D4:"); Serial.println(packet.this1sdB);
  }             
}

String doubleToString( double val, unsigned int precision)
{
   String returnval = "";
   returnval += int(val);  //prints the int part
   returnval += "."; // print the decimal point
   unsigned int frac;
   if(val >= 0)
   {
       frac = (val - int(val)) * precision;
   }
   else
   {
       frac = (int(val)- val ) * precision;
   }
   returnval += frac;
   return returnval;
}
