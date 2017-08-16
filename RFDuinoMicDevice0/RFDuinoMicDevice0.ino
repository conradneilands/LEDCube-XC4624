//Code acknowledgements
//dB calculation for Max4466
//https://forum.arduino.cc/index.php?topic=435395.0
//double to string
//http://forum.arduino.cc/index.php?topic=44216.0

//const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
const int sampleWindow = 1000; // Sample window width in mS (1000 mS = Hz)
unsigned int sample;
unsigned int micPin = 6;
unsigned long avgPeriod = 60000;
unsigned long sampleCount = 0;
unsigned long bootMillis = millis();
unsigned long peakToPeakSum = 0 ;

#include <RFduinoGZLL.h>
device_t role = DEVICE0;

// Define my custom packet of no more than 20 bytes.
struct myCustomPacket_t
{
  int this1sdB;    // 4 bytes
  int this1mdB;
  int this1sV;

 // NOTE: compiler will reserve a byte here not defined
};

struct myCustomPacket_t packet;


void setup() 
{
  override_uart_limit = true;

   Serial.begin(115200);
   RFduinoGZLL.txPowerLevel = 0;
   // start the GZLL stack
   RFduinoGZLL.begin(role);
   
}


void loop() 
{
   unsigned long loopStartMillis= millis();  // Start of sample window

   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 1023;

   // collect data for 50 mS
   while (millis() - loopStartMillis < sampleWindow)
   {
      sample = analogRead(micPin);
      
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   peakToPeakSum = peakToPeakSum + peakToPeak;
   sampleCount++;
   
   if(loopStartMillis - bootMillis >= avgPeriod)
   {
      //Calculate and Clear timing variables
      int peakToPeakAvg = (int) (peakToPeakSum / sampleCount);
      //Serial.println(sampleCount);
      sampleCount = 0;
      peakToPeakSum = 0;
      bootMillis = loopStartMillis;
      int dBPeak = caculatedB(peakToPeakAvg);
      //String dBAvg = doubleToString(dBPeak, 2);
      //Serial.println("60s SPL: " + dBAvg + "dB");     
      //RFduinoGZLL.sendToHost(char* (dBAvg), 5);
      packet.this1sdB = dBPeak;
      packet.this1mdB = caculatedB(peakToPeakAvg);
      packet.this1sV = peakToPeak;
      RFduinoGZLL.sendToHost((char *)&packet, sizeof(packet));

   }
   else
   {
      int dBPeak = caculatedB(peakToPeak);
      //String dB1s = doubleToString(dBPeak, 2);
      //Serial.print("1s dB: "); Serial.println(dBPeak);
      //Serial.println("1sV: " + peakToPeak );
      //RFduinoGZLL.sendToHost(char* (dB1s), 5);
      packet.this1sdB = dBPeak;
      packet.this1sV = peakToPeak;
      RFduinoGZLL.sendToHost((char *)&packet, sizeof(packet));
   }
}

int caculatedB(int peakToPeakVal)
{
      double volts = ((peakToPeakVal * 3.3) / 1023) * 0.707;  // convert to volts
      double first = log10(volts/0.00631)*20; // The microphone sensitivity is -44 ±2 so V RMS / PA is 0.00631
      double second = (int) first + 94 - 44; // The microphone sensitivity is -44 ±2
      return second;
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

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
}
