#include <RunningAverage.h>
#include <SPI.h>
#include "RF24.h"
#include "LedControl.h"

/* LEDCONTROL initialization
 pin 9 is connected to the MOSI
 pin 10 is connected to the CLK 
 pin 6 is connected to Slave Select 
 */
LedControl lc=LedControl(9,10,6,1);

//initializing the Runing average filter
RunningAverage potiRA(2);
int samples=0; //for the RA filter

int elevPoti=0;           //raw potentiometer value
int elevDeg=0;            //converted to servo degrees to be sent to the plane
int led=3;                //the connection led
int alphadezi;            //alpha in dezidegrees from plane
int deltaElevator=0;      //converted to elevator degrees for display
byte displayCounter=0;    //to downsample the led update rate


/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 5 CSN & 4  */
RF24 radio(5,4);
/**********************************************************/

byte addresses[][6] = {"1Node","2Node"};

void setup() {

    /*LCDCONTROL
   The MAX7219 is in power-saving mode on startup,
    do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,8);
  /* and clear the display */
  lc.clearDisplay(0);
  
  pinMode(led, OUTPUT); //set pinmode for connection led

  potiRA.clear();//start clean with moving average filter

  //set up radio
  radio.begin();
  radio.setChannel(10);
  // Set the PA Level low to prevent power supply related issues RF24_PA_MAX is default.
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
}//setup


void printNumberAlpha(int v) {      //the function to print alpha
    int ones;
    int tens;
    int hundreds;
    int thousands;  

    if(v < 0 || v > 9999) 
       return;
    
    ones=v%10;
    v=v/10;
    tens=v%10;
    v=v/10;
    hundreds=v%10;
    v=v/10;
    thousands=v;     
    //Now print the number digit by digit
    lc.setDigit(0,0,(byte)thousands,false);
    lc.setDigit(0,1,(byte)hundreds,false);
    lc.setDigit(0,2,(byte)tens,false);
    lc.setDigit(0,3,(byte)ones,false);
}

void printNumberElevator(int v) {   //the function to to print elevator
    int ones;  
    int tens;  
    int hundreds; 

    boolean negative=false;

    if(v < -999 || v > 999)  
        return;  
    if(v<0) {  
        negative=true; 
        v=v*-1;  
    }
    ones=v%10;  
    v=v/10;  
    tens=v%10;  
    v=v/10; hundreds=v;  
    if(negative) {  
        //print character '-' in the leftmost column  
        lc.setChar(0,4,'-',false);  } 
    else {
        //print a blank in the sign column  
        lc.setChar(0,4,' ',false);  
    }  
    //Now print the number digit by digit 
    lc.setDigit(0,5,(byte)hundreds,false);
    lc.setDigit(0,6,(byte)tens, false); 
    lc.setDigit(0,7,(byte)ones,false); 
} 


void loop() {
  elevPoti=analogRead(A4); //read the potentiometer values 
    //simple moving average filter
  potiRA.addValue(elevPoti);
  samples++;
  elevPoti = round(potiRA.getAverage());
  elevPoti = (int)elevPoti; //typecast to int because from doc a float comes back
  if (samples == 300) //just to clear the filter once in a while
  {
    samples = 0;
    potiRA.clear();
  }
  elevDeg=map(elevPoti, 0, 1023, 30, 100); //map to servo degrees
  deltaElevator=map(elevPoti, 0, 1023, -180, 180); //map to display degrees
  printNumberElevator(deltaElevator);
  
  
//Radio
  
  radio.stopListening();                                    // stop listening to send
  
   if (!radio.write( &elevDeg, sizeof(int) )){              //try to send, set connection led off if it doesnt work
    digitalWrite(led, LOW);
   }
      
  radio.startListening();                                    // Now, continue listening
  
  unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
  
  while ( ! radio.available() ){                             // While nothing is received
    if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
        timeout = true;
        digitalWrite(led, LOW);                               //set connection led to off
        break;
    }      
  }
      
  if ( timeout ){                                             // if timeout, do nothing
      
  }else{
      radio.read( &alphadezi, sizeof(int) );                  //get response
      digitalWrite(led, HIGH);                                //set connection LED high because no timeout and response received
      if(displayCounter > 5){                                 //downsample display update rate to improve readability
        printNumberAlpha(alphadezi);                          //print AoA to display
        displayCounter = 0;
      }else{
        displayCounter ++;
      }
    }
 delay(50);
} // Loop

