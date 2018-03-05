#include <Servo.h>
#include <RunningAverage.h>
#include <math.h>
#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>




#define PI 3.1415926535897932384626433832795
// Used for software SPI pins for accelerometer
#define LIS3DH_CLK 9
#define LIS3DH_MISO 14
#define LIS3DH_MOSI 10
#define LIS3DH_CS 15
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK); //setup accelerometer



int led = 3; //control led
int alphadezi = 0; //0-9999 Angle of attack
int elevDeg = 0; //0-180 servo elevator degrees

RunningAverage alphaRA(70);//setup running average for accelerometer
int samples = 0;

Servo elevServo;//create servo for elevator

/****************** User Config Radio ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 5 CSN & 4 CE*/
RF24 radio(5,4);
byte addresses[][6] = {"1Node","2Node"};

void setup() {

  lis.begin(0x18); //start accelerometer
  lis.setRange(LIS3DH_RANGE_4_G); //set accelerometer range
  
  pinMode(led, OUTPUT); //control led

  alphaRA.clear(); // explicitly start clean with running average

  elevServo.attach(6); //activate servo elevator

  //setting up radio
  radio.begin();
  radio.setChannel(10);
  // Set the PA Level low to prevent power supply related issues, RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  
  lis.read(); //read all accelerometer levels
//  alphadezi=lis.x; //testing
  if (lis.z == 0){lis.z = 0.01;} //avoid division by zero
  if (lis.x == 0){lis.x = 0.01;} //avoid zero
  alphadezi = round(100 * atan2(lis.y, lis.z) * 180 / PI); //all the real math in here. calculate angle from acceleration vectors

  //simple moving average filter
  alphaRA.addValue(alphadezi);
  samples++;
  alphadezi=round(alphaRA.getAverage());
  alphadezi = (int)alphadezi; //typecast int because running average delivers float
  if (samples == 300) //clear filter from time to time
  {
    samples = 0;
    alphaRA.clear();
  }
  
  digitalWrite(led, LOW);
  if( radio.available()){
                                                                  // Variable for the received timestamp
    while (radio.available()) {                                   // While there is data ready
      radio.read( &elevDeg, sizeof(int) );                        // Get the payload
      digitalWrite(led, HIGH);                                    //set control led high
    }
   
    radio.stopListening();                                        // stop listening to send 
    radio.write( &alphadezi, sizeof(int) );                       // Send alpha      
    radio.startListening();                                       // Now, resume listening so we catch the next packets.     
    
 }
elevServo.write(elevDeg);                                         //set servo angle



} // Loop

