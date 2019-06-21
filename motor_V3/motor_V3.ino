#include <MsTimer2.h>
#include "config.h"
#include <mcp_can.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);

TinyGPSPlus gps;
// Build an ID or PGN
const int SPI_CS_PIN = 10;
long unsigned int txID = 0x1881ADBA; 
long unsigned int txID_LAT = 0x1881AEBA; 
long unsigned int txID_LONG = 0x1881AFBA; 
// This format is typical of a 29 bit identifier.. the most significant digit is never greater than one.
unsigned char stmp[8] = {1, 1, 1, 1, 2, 2, 2, 2};
unsigned char stmp_LAT[8] = {1, 1, 1, 1, 2, 2, 2, 2};
unsigned char stmp_LONG[8] = {1, 1, 1, 1, 2, 2, 2, 2};

//Temperature:
//Ventilator should start at a temperature of 45°C 
//and has to do the following operation: 
//30s cleaning - 10min cooling - 30s cleaning - 10min cooling -
//... while the temperature is to high (between 45°C and 55°C is ok) 
//Accuracy of the temperature: +- 1°C will do

int temPin = A0;  // for temp
double valA0=0;
float ntc=0;
float tem=0;
//int tem=0;
float tem_old=20;
//String message = "Temperature : ";

 float GPS_LONG; 
 float GPS_LAT; 

enum state_enum {STOP,SSTART1,CLEAN,SSTART2,COOL};
uint8_t state = STOP;
uint8_t laststate = STOP;
int delayforgps=0;
int Hpin1=5;
int Hpin2=6;
int n1=0;
int n2=0;
int delta=5;
int slowtime=SStime/50;
int cleanstart=0;
int cleanend=0;
int coolstart=0;
int coolend=0;
    String message = "Current Temperature : ";
    String message1 = "Temperature sent: ";
    String message3 = "Temperature after cooling: ";
    String message2 = "Temperature before cooling: ";



MCP_CAN CAN(SPI_CS_PIN);                            

void timer10min()
{
  MsTimer2::stop();
  coolend = 1;
  Serial.println("interrupt: timer10min");
}

void timer30s()
{
  Serial.println("interrupt: timer30");
  MsTimer2::stop();
  cleanend =1;
  //Serial.println("cleanend:");
        // Serial.println(cleanend);
}

void slowstart1()
{
  cleanstart=0;
  MsTimer2::stop();
  if(n1+5< 250)
  {
    n1=n1+5;    //50 times to get 245 
  }
  analogWrite(Hpin1, n1);
  //Serial.println(n1);
}
void slowstart2()
{
  coolstart=0;
  MsTimer2::stop();
  if(n2+5< 250)
  {
    n2=n2+5;    //50 times to get 250 
  }
  analogWrite(Hpin2, n2);
  //Serial.println(n2);
}


void setup() {
    
    Serial.begin(9600);
      ss.begin(GPSBaud);
  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println(F("Edited By www.maxphi.com"));
  Serial.println();
  
  // In the begining ventilator stop
    analogWrite(Hpin1, 0);
    analogWrite(Hpin2, 0);
    
  // checking tem setting
    if (tem_H < tem_L)
     {
      Serial.println("ERROR FOR TEM SETTING");
      int tem_H1=tem_H;
      tem_H=tem_L;
      tem_L=tem_H1;
     }
     
  // checking CANBUS   
    while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 250K
    {
         Serial.println("CAN BUS Module Failed to Initialized");
        Serial.println("Retrying....");
        delay(200);
        
    }
    Serial.println("CAN BUS Shield init ok!");
     
}
void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    GPS_LAT= (float)(gps.location.lat());
    dtostrf(GPS_LAT,3, 4, stmp_LAT);
    
    Serial.print(F(","));
    GPS_LONG= (float)(gps.location.lng());
    dtostrf(GPS_LAT,3, 4, stmp_LONG);
    Serial.print(gps.location.lng(), 6);
    
    
  }
  else
  { 
    Serial.print(F("INVALID"));
  }  
  if(delayforgps< 45)
  {delayforgps++;}
  else{
    CAN.sendMsgBuf(txID_LAT,1, 8, stmp_LAT); 
    CAN.sendMsgBuf(txID_LONG,1, 8, stmp_LONG); 
    delayforgps=0;
    Serial.println("gps send");
    }
  Serial.println();
}
void loop() {
    while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  
    //detect temperature
    valA0 = analogRead(temPin);
    ntc=2762000/valA0-2700;
    tem = 3700/(log(ntc)+4.73)-273.15;
   // tem=temd;
    //Serial.println(message+tem);
    if(state != laststate)
    {
      Serial.print("current state:");
      Serial.println(state);
      laststate= state;
    }
    if(tem > tem_old+1 || tem < tem_old-1)
    {
       stmp[1]=tem;
       Serial.println(message1+tem);
       CAN.sendMsgBuf(txID,1, 8, stmp); 
       tem_old= tem;
    }
       
    switch (state)
    {
      case STOP:
            
            analogWrite(Hpin1, 0);
            analogWrite(Hpin2, 0);
            if(enableTem == 0)
              {
               Serial.println("tempreture is too high");
               state= SSTART1;
               Serial.println(message2+tem);
              }
            if(enableTem ==1 && tem > tem_H)
              {               
               Serial.println("tempreture is too high");
               state= SSTART1;
               Serial.println(message2+tem);
              }
           break;

      case SSTART1:
           if(n1 != 245 && cleanstart==0)
           {
             MsTimer2::set(slowtime,slowstart1);
             MsTimer2::start();
             cleanstart=1;
           }
           if(n1 == 245)
           {
             state= CLEAN;
             cleanstart=1;
           }
           break;
           
      case CLEAN:
           if(cleanstart == 1)
            {     
             //start a timing which generate an intterupt 30s later and set cleanend=1
             MsTimer2::set(cleantime, timer30s);
             MsTimer2::start();
             cleanstart = 0; 
             Serial.println("ceaning");
            }
           
           if(cleanend == 1 )
             {
               n1=0;
               analogWrite(Hpin1, n1);
               cleanend = 0;
               coolstart = 0;
               Serial.println("cleanend"); 
               delay(slowdowntime);
               state=SSTART2;
             }
           break; 
                 
      case SSTART2:
           if(n2 != 245 && coolstart==0)
           {
             MsTimer2::set(slowtime,slowstart2);
             MsTimer2::start();
             coolstart=1;
           }
           if(n2 == 245)
           {
             state= COOL;
             coolstart=1;
           }
           break;     
           
      case COOL:
           if(coolstart == 1)
            {
             //start a timing which generate an intterupt 10min later and set cleanend=1
             MsTimer2::set(cooltime, timer10min);
             MsTimer2::start();
             coolstart = 0;  
             Serial.println("cooling");
            }
            
           if(coolend == 1 )
             {
              n2=0;
              analogWrite(Hpin2, n2);
              coolend = 0;
              state=STOP;
              Serial.println("coolend");  
              delay(slowdowntime);
              if(enableTem ==0)
                {
                  state = SSTART1;
                }
              if(enableTem ==1 && tem >= tem_L)
                {state=SSTART1;Serial.println(message3+tem);}
              if(enableTem == 1 && tem < tem_L)  
                {state=STOP;Serial.println(message3+tem);}          
             }                     
           break;
    }    
}
