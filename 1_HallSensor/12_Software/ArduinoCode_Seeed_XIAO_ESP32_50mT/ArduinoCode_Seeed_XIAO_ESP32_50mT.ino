/**********************************************************************/
//     LOW FIELD HALL SENSOR - 8 Channel
//     
//    Using the Chip ALS31313 from Allegro Microsystems and ESP32 µC (Seeed XIAO)  
//    Parts of this Software (Readout of Hallsensor in Fast Mode) is based on Example Code from Allegro
//
//    One other good example is published here by vintlabs: https://github.com/vintlabs/ESP32-Arduino-3dHall-Example/tree/master
// 
//    Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
//    MIT LICENSED
//    Have fun guys!
/**********************************************************************/

// Define I2C Protocol
#include <Wire.h>
// #define SDA 4
// #define SCL 5

// Variables used I2C Transmit

#define kNOERROR 0
#define kDATATOOLONGERROR 1
#define kRECEIVEDNACKONADDRESSERROR 2
#define kRECEIVEDNACKONDATAERROR 3
#define kOTHERERROR 4


// I2C Baud Rate
const long i2cclock = 100000;  // Baud/Hz 

// LEDs
#define LED_H1 D0
#define LED_H2 D1
#define LED_H3 D2
#define LED_H4 D3
#define LED_H5 D6
#define LED_H6 D10
#define LED_H7 D9
#define LED_H8 D9 //  1 LED too few -> H7 and H8 share the LED
#define LED_Sts D8


// Hall Sensirs
#define Hall_Sensitivity_1000  2.0  // 1000 Gauss Type
#define Hall_Sensitivity_500  4.0   // 500  Gauss Type

const float Hall_Sensitivity = Hall_Sensitivity_500; // Here: 50mT Type

const int Address_H1 = 96;  // Hall 1
const int Address_H2 = 99;   // Hall 2
const int Address_H3 = 100;  // Hall 3
const int Address_H4 = 103;  // Hall 4
const int Address_H5 = 104;  // Hall 5
const int Address_H6 = 107;  // Hall 6
const int Address_H7 = 108;  // Hall 7
const int Address_H8 = 110;  // Hall 8

// Number of averaged measurements
const int Repeat_Steps = 1000; //Number of internal repetitions, these values are averaged arithmetically.

// Hall Mode: Single-Ended Operation, reports mag data from X/Y/Z
const int hall_mode = 0;  




// Serial Connection to Computer (USB)
const long baud_rate = 115200;
const int num_decimal = 6; //  Number of decimal places in the serial output

// Measure Data - Return Value 
int64_t xd = 0;
int64_t yd = 0;
int64_t zd = 0;
int64_t td = 0;

// Prototype Functions
String Hall_Read_Full_Loop(int,String);
uint16_t Hall_Read(int, uint8_t, uint32_t&);
uint16_t Hall_Write(int, uint8_t, uint32_t);
long SignExtendBitfield(uint32_t, int);
void setLED(int);
void Hall_Read_Single(int);
void Hall_SetCustomer_Access(int, String);

// Structure 1 Serialline
String Text_H1="";
String Text_H2="";
String Text_H3="";
String Text_H4="";
String Text_H5="";
String Text_H6="";
String Text_H7="";
String Text_H8="";
String Text_Total = "";

void setup() {

  // Start I2C
  Wire.begin(SDA, SCL);
  Wire.setClock(i2cclock);

  // Serial Communication with PC
  Serial.begin(baud_rate);
  while (!Serial){}
  delay(10); //  Wait 10ms to be on the safe side, so that the Hall sensor has started safely and is listening to commands


  // LED Pins as Output
  pinMode(LED_Sts, OUTPUT);
  pinMode(LED_H1, OUTPUT);
  pinMode(LED_H2, OUTPUT);
  pinMode(LED_H3, OUTPUT);
  pinMode(LED_H4, OUTPUT);
  pinMode(LED_H5, OUTPUT);
  pinMode(LED_H6, OUTPUT);
  pinMode(LED_H7, OUTPUT);
  

  // Set Customer Write Access on ALS31300 (p20 Datasheet)
  Hall_SetCustomer_Access(Address_H1, "Hall 1");
  Hall_SetCustomer_Access(Address_H2, "Hall 2");
  Hall_SetCustomer_Access(Address_H3, "Hall 3");
  Hall_SetCustomer_Access(Address_H4, "Hall 4");
  Hall_SetCustomer_Access(Address_H5, "Hall 5");
  Hall_SetCustomer_Access(Address_H6, "Hall 6");
  Hall_SetCustomer_Access(Address_H7, "Hall 7");
  Hall_SetCustomer_Access(Address_H8, "Hall 8");


  
}

void loop() {

    bool succ = false;

    // Readout Hall 1
    setLED(LED_H1);
    //Serial.print("H1;   ");
    Text_H1 = Hall_Read_Full_Loop(Address_H1,"H1");  // Data is stored in xd,yd,zd,td
    setLED(LED_H1);
    // Readout Hall 2
    setLED(LED_H2);
    //Serial.print("H2;   ");
    Text_H2 = Hall_Read_Full_Loop(Address_H2,"H2");  // Data is stored in xd,yd,zd,td
    setLED(LED_H2);
    // Readout Hall 3
    setLED(LED_H3);
    //Serial.print("H3;   ");
    Text_H3 = Hall_Read_Full_Loop(Address_H3,"H3");  // Data is stored in xd,yd,zd,td
    setLED(LED_H3);
    // Readout Hall 4
    setLED(LED_H4);
    //Serial.print("H4;   ");
    Text_H4 = Hall_Read_Full_Loop(Address_H4,"H4");  // Data is stored in xd,yd,zd,td
    setLED(LED_H4);
    // Readout Hall 5
    setLED(LED_H5);
    //Serial.print("H5;   ");
    Text_H5 = Hall_Read_Full_Loop(Address_H5,"H5");  // Data is stored in xd,yd,zd,td
    setLED(LED_H5);
    // Readout Hall 6
    setLED(LED_H6);
    //Serial.print("H6;   ");
    Text_H6 = Hall_Read_Full_Loop(Address_H6,"H6");  // Data is stored in xd,yd,zd,td
    setLED(LED_H6);
    // Readout Hall 7
    setLED(LED_H7);
    //Serial.print("H7;   ");
    Text_H7 = Hall_Read_Full_Loop(Address_H7,"H7");  // Data is stored in xd,yd,zd,td
    setLED(LED_H7);
    //Readout Hall 8
    setLED(LED_H8);
    //Serial.print("H8;   ");
    Text_H8 = Hall_Read_Full_Loop(Address_H8,"H8");  // Data is stored in xd,yd,zd,td
    setLED(LED_H8);

    //Send Data via UART
    setLED(LED_Sts);
    Text_Total = Text_H1 + "|" + Text_H2 + "|" + Text_H3+ "|" + Text_H4+ "|" + Text_H5+ "|" + Text_H6+ "|" + Text_H7+ "|" + Text_H8;
    Serial.println(Text_Total);
    setLED(LED_Sts);   
   
}


