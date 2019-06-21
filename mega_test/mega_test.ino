#include <analogShield.h>   //Include to use analog shield.
#include <SPI.h>
#include "config.h"
#include "mcp_can.h"、
#include <SoftwareSerial.h>

SoftwareSerial BTserial(15, 14); // RX | TX
long unsigned int rxId;

unsigned long rcvTime;

unsigned char len = 0;
unsigned char buf[8];

unsigned long IMU1_w;
unsigned long IMU1_x;
unsigned long IMU1_y;
unsigned long IMU1_z; 
unsigned long IMU2_w;
unsigned long IMU2_x;
unsigned long IMU2_y;
unsigned long IMU2_z; 
unsigned long IMU3_w;
unsigned long IMU3_x;
unsigned long IMU3_y;
unsigned long IMU3_z; 
unsigned long OilTem; 
 String GPS_LONG = ""; 
 String GPS_LAT = ""; 


const int SPI_CS_PIN = 53;

int sensorPin = A0;
float toLow=40957;
float toHigh=57340;
int sensorValue = 0;

MCP_CAN CAN(SPI_CS_PIN);  
void setup()
{
  Serial.begin(38400);           //  setup serial
  BTserial.begin(9600);

    while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Module Failed to Initialized");
        Serial.println("Retrying....");
        delay(200);
    }    
    Serial.println("CAN BUS Module Initialized!");
    Serial.println("Time\t\tPGN\t\tByte0\tByte1\tByte2\tByte3\tByte4\tByte5\tByte6\tByte7");
}

unsigned int count = 0;
int temPin = A0; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
int x_axisPin= A1;
int y_axisPin= A2;
int z_axisPin= A3;
int button1Pin= A4;  // right one  orange wire
int button2Pin= A5;  // left one   violet wire
double valA0 = 0;  // variable to store the value read
unsigned int valA1 = 0; 
unsigned int valA2 = 0;
unsigned int valA3 = 0;
unsigned int valA4 = 0;
unsigned int valA5 = 0;
double Vx = 0; 
double Vy = 0;
double Vz = 0;
unsigned int Vbutton1 = 0; 
unsigned int Vbutton2 = 0;
char lat_0;
char lat_1; 
char lat_2;
char lat_3;
char lat_4;
char lat_5;
char lat_6;
char lat_7;

char long_0;
char long_1; 
char long_2;
char long_3;
char long_4;
char long_5;
char long_6;
char long_7;
void loop()
{
  for(int i = 0; i<9; i++){

          //Serial.print(i);

  valA0 = analogRead(temPin);  // read the input pin
  valA1 = analogRead(x_axisPin);
  valA2 = analogRead(y_axisPin);
  valA3 = analogRead(z_axisPin);
  valA4 = analogRead(button1Pin);
  valA5 = analogRead(button2Pin);

 // String message1 = "x_axis : ";
  //String message2 = "y_axis : ";
  //String message3 = "z_axis : ";
  //String message4 = "dac result : ";
  //Serial.println(message+tem);          // debug value

  
  if(mode==1)
  {
    toLow=40057;   //40957
    toHigh=55540;  // 57340
  }
  if(mode==0)
  {
    toLow=outputL*6553+31000;
    toHigh=outputH*6553+31000; 
  
  }
  valA1=map(valA1, 0, 1023, toLow, toHigh);
  valA2=map(valA2, 0, 1023, toLow, toHigh);
  valA3=map(valA3, 0, 1023, toLow, toHigh);
 //   Serial.println(message1+valA1);
  //Serial.println(message2+valA2);
  //Serial.println(message3+valA3); 
  if(mode == 0)
  {
  Vx=(valA1-toLow)*(outputH-outputL)/(toHigh-toLow) + outputL;
  Vy=(valA2-toLow)*(outputH-outputL)/(toHigh-toLow) + outputL;
  Vz=(valA3-toLow)*(outputH-outputL)/(toHigh-toLow) + outputL;
  /*Serial.println(message1+Vx);
  Serial.println(message2+Vy);
  Serial.println(message3+Vz); */
  }

  if(mode ==1 )
  {
  Vx=(valA1-toLow)*6/(toHigh-toLow) + 3;
  Vy=(valA2-toLow)*6/(toHigh-toLow) + 3;
  Vz=(valA3-toLow)*6/(toHigh-toLow) + 3;
  /*Serial.println(message1+Vx);
  Serial.println(message2+Vy);
  Serial.println(message3+Vz); */
  }
  //valA1=65535;
  //valA4=map(valA1, 0, 1023, toLow, toHigh);
  // 65535--5V  32766--0V  0-- -5V   32766/4*3=24574  24574+32766=57340 3.75V  8191+32766=40957 --1.25V
 /* Serial.println("Button1：");
  Serial.println(valA4);
  Serial.println("Button2：");
  Serial.println(valA5);*/
  if(valA5< 10 && valA4 >200)
    {
      Serial.println("Button2: ON");
      analog.write(3, toLow);
      Vbutton2=1;
    }
   else if(valA4 < 10)
   {
      Serial.println("Button1: ON");
      analog.write(3, toHigh);
      Vbutton1=1;
    }
    
  if(valA5 > 50 && valA4 > 50)
    {
      Vbutton1=0;
      Vbutton2=0;
      analog.write(3, (toLow+toHigh)/2);
    }
  analog.write(0, valA1);
  analog.write(1, valA2);
  analog.write(2, valA3);
  //analog.write(3, 32766);

    //Serial.println("CAN BUS inloop !");
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        rcvTime = millis();
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        rxId= CAN.getCanId();

        Serial.print(rcvTime);
        Serial.print("\t\t");
        Serial.print("0x");
        Serial.print(rxId, HEX);
        Serial.print("\t");

        if(rxId == 0x1881ABBA)
        {
          IMU1_w = buf[0];
          IMU1_x = buf[1];
          IMU1_y = buf[2];
          IMU1_z = buf[3];
        } else if(rxId == 0x1881ACBA){
          IMU2_w = buf[0];
          IMU2_x = buf[1];
          IMU2_y = buf[2];
          IMU2_z = buf[3]; 
        } else if(rxId == 0x1881AABA){
          IMU3_w = buf[0];
          IMU3_x = buf[1];
          IMU3_y = buf[2];
          IMU3_z = buf[3]; 
        }
          else if(rxId == 0x1881ADBA){
          OilTem = buf[1];
          //GPS_LAT = buf[1];
          //GPS_LONG = buf[2];
          //IMU3_z = buf[3];
        }
          else if(rxId == 0x1881AEBA){
            //String a = buf[0];
            //strcat(
              //GPS_LAT=(char)buf[1]+(char)buf[2]+(char)buf[3]+(char)buf[4]+(char)buf[5]+(char)buf[6]+(char)buf[7];
            Serial.println(" ");
            Serial.print((char)buf[0]);
            Serial.print(" ");
            Serial.print((char)buf[1]);
            Serial.print(" ");
            Serial.print((char)buf[2]);
            Serial.print(" ");
            Serial.print((char)buf[3]);
            Serial.print(" ");
            Serial.print((char)buf[4]);
            Serial.print(" ");
            Serial.print((char)buf[5]);
            Serial.print(" ");
            Serial.print((char)buf[6]);
            Serial.print(" ");
            Serial.print((char)buf[7]);

            Serial.println(" ");
            long_0 = buf[0]; 
            long_1 = buf[1];
            long_2 = buf[2];
            long_3 = buf[3];
            long_4 = buf[4];
            long_5 = buf[5];
            long_6 = buf[6];
            long_7 = buf[7];
        }
          else if(rxId == 0x1881AFBA){
            //GPS_LONG=(char)buf[1]+(char)buf[2]+(char)buf[3]+(char)buf[4]+(char)buf[5]+(char)buf[6]+(char)buf[7];
          
            
             Serial.println(" ");
            Serial.print((char)buf[0]);
            Serial.print(" ");
            Serial.print((char)buf[1]);
            Serial.print(" ");
            Serial.print((char)buf[2]);
            Serial.print(" ");
            Serial.print((char)buf[3]);
            Serial.print(" ");
            Serial.print((char)buf[4]);
            Serial.print(" ");
            Serial.print((char)buf[5]);
            Serial.print(" ");
            Serial.print((char)buf[6]);
            Serial.print(" ");
            Serial.print((char)buf[7]);

            Serial.println(" ");
            lat_0 = buf[0]; 
            lat_1 = buf[1];
            lat_2 = buf[2];
            lat_3 = buf[3];
            lat_4 = buf[4];
            lat_5 = buf[5];
            lat_6 = buf[6];
            lat_7 = buf[7];
            
            
            
            }
        


        for(int i = 0; i<len; i++)    // print the data
        {
            if(buf[i] > 15){
              //Serial.print("0x");
              Serial.print(buf[i]);    
            }
          else{
              //Serial.print("0x0");
              Serial.print(buf[i]);
          }  
            
            //Serial.print("0x");
            //Serial.print(buf[i], HEX);
            
            Serial.print("\t");            
        }
        Serial.println();
        
    }
  }
//METHODS TO GET DATA
//METHOD GPS X
//METHOD GPS Y
//float GPSY=methodGPSY();
//METHOD JOYSTICK
//Serial.println("BT sending!!");
BTserial.print("#>");// Start Data Packet
BTserial.print("*1");//Mower_Id: 
BTserial.print("*05.02.2019");//Date
BTserial.print("*13:12:00");//Time: 

BTserial.print("*");//GPS_x: 
BTserial.print(lat_0); 
BTserial.print(lat_1); 
BTserial.print(lat_2); 
BTserial.print(lat_3); 
BTserial.print(lat_4); 
BTserial.print(lat_5); 
BTserial.print(lat_6); 
BTserial.print(lat_7); 

BTserial.print("*");//GPS_y: //GPSY
BTserial.print(long_0); 
BTserial.print(long_1); 
BTserial.print(long_2); 
BTserial.print(long_3); 
BTserial.print(long_4); 
BTserial.print(long_5); 
BTserial.print(long_6); 
BTserial.print(long_7); 
//GPS_y: //GPSY

BTserial.print("*");//Joystick_x; 
BTserial.print((int)(Vx*100));

BTserial.print("*");//Joystick_y; 
BTserial.print((int)(Vy*100));

BTserial.print("*");//Joystick_z; 
BTserial.print((int)(Vz*100));

BTserial.print("*");//Joystick_b1; 
BTserial.print(Vbutton1);

BTserial.print("*");//Joystick_b2; 
BTserial.print(Vbutton2);
 
BTserial.print("*");//Oil_Temperature:
BTserial.print(OilTem);

BTserial.print("*");//IMU_1_w;
BTserial.print(IMU1_w);

BTserial.print("*");//IMU_1_x;
BTserial.print(IMU1_x);

BTserial.print("*");//IMU_1_y;
BTserial.print(IMU1_y);
BTserial.print("*");//IMU_1_z;
BTserial.print(IMU1_z);

BTserial.print("*");//IMU_2_w;
BTserial.print(IMU2_w);

BTserial.print("*");//IMU_2_X;
BTserial.print(IMU2_x);

BTserial.print("*");//IMU_2_y;
BTserial.print(IMU2_y);

BTserial.print("*");//IMU_2_z;
BTserial.print(IMU2_z);

BTserial.print("*");//IMU_3_w;
BTserial.print(IMU3_w);

BTserial.print("*");//IMU_3_x;
BTserial.print(IMU3_x);

BTserial.print("*");//IMU_3_y;
BTserial.print(IMU3_y);

BTserial.print("*");//IMU_3_z;
BTserial.print(IMU3_z);
 
BTserial.print("*%");//End Data Packet %%

    }//delay(1);
