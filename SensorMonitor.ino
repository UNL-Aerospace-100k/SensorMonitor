/*
 * 100k Sensor Monitor V1.1
 * 2/16/2024
 * Jack Shaver
 */

/*
 * This code takes an input from the Sub-Master Controller and toggles between
 * opening the sd card and writting sensor data, and closing the sd card and waiting
 * 
 * Written for use with the TEENSY4.1, and the Sub-Master Controller V1 PCb
 * Indicator LED for when data is being actively collected
 * Inputs from array of pins, to be configured.
 * 4 bit parrallel transmission from sub-master
 * Auxillary Load Cell Amplifier HX711 needed on pins 31 and 32
 * 
 * Todo:
 * Change sensors on the analog pins to be the sensors needed for the static fire
 * Change file write format to best log data
 * Add support for digital output sensors (if needed)
 */

#include <Arduino.h>
#include "HX711.h"
//SD card preamble
#include <SPI.h>
#include <SD.h>
//const int chipSelect = BUILTIN_SDCARD;
File myFile;

HX711 loadCell;

//GPIO preamble
#define ActiveWriteLED 2
#define Error_Comm_LED 3

#define ParallelBit0 7
#define ParallelBit1 6
#define ParallelBit2 5
#define ParallelBit3 4

#define LoadCellDataPin 31
#define LoadCellClockPin 32

#define LOADCELL_CALIBRATION_FACTOR 15.94



int pin7State = 0;

int Switchstate = 0;
int toggle = 0;
int edge = 0;

long collectInterval = 100; //interval between sensor reads in milliseconds
long previousMillis = 0;


void setup() { //---------------------------------setup------------------------------------
  
  pinMode(Error_Comm_LED, OUTPUT);
  pinMode(ActiveWriteLED, OUTPUT);

  pinMode(ParallelBit0, INPUT_PULLDOWN);
  pinMode(ParallelBit1, INPUT);
  pinMode(ParallelBit2, INPUT);
  pinMode(ParallelBit3, INPUT);

  Serial.begin(9600);//begin UART comm with computer, 9600 baud
    //while (!Serial){}
    
  //initialize onboard sd card, make sure its initialized
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1){
      digitalWrite(Error_Comm_LED, HIGH);
      delay(100);
      digitalWrite(Error_Comm_LED, LOW);
      delay(100);
      }
    }
  Serial.println("initialization done.");

  //create and open a file on the sd card
  //myFile = SD.open("DataCollection1.txt", FILE_WRITE);
  //myFile.println("Measuring the Load Cell, Units = Grams");

  //Load cell initialization
  loadCell.begin(LoadCellDataPin, LoadCellClockPin);
  loadCell.set_scale(LOADCELL_CALIBRATION_FACTOR);
  loadCell.power_up();

  //GPIO initialization
  digitalWrite(Error_Comm_LED, HIGH);
  delay(1000);
  digitalWrite(Error_Comm_LED, LOW);
}


void loop() { //----------------------------------------loop(main)--------------------------
  
  if(checkpin(ParallelBit0, &pin7State) == 1){
    Switchstate = 1;
    digitalWrite(Error_Comm_LED, HIGH);
    delay(50);
    digitalWrite(Error_Comm_LED, LOW);
  }else{
    Switchstate = 0;
  }
  //0: no change, 1: pressed, 2: released

  if(Switchstate == 1){//on pressed cyle, toggle the state
    toggle = !toggle;
    edge = 1;
  }

  if(toggle == 1){ //toggle state active
    if(edge == 1){ //first cycle after state change
      openfile();
      loadCell.tare();
      delay(1000);
      edge = 0;
    }
    //active toggle state runtime here

    //collect data every specified interval
    if((millis() - previousMillis) > collectInterval){
      previousMillis = millis();
      writefile();
    }
    
  }else{ //toggle state inactive
    if(edge == 1){ //first cycle after state change
      closefile();
      edge = 0;
    }
  }
}

void openfile(void){ //------------------------------Open the file-------------------------
  digitalWrite(ActiveWriteLED, HIGH);
  Serial.println("File Opened");

  int counter = 1;
  char fileName[30];
  memset(fileName, 0, sizeof(fileName));
  sprintf(fileName, "DataCollection%d", counter);
  while(SD.exists(fileName))
  {
    counter++;
    memset(fileName, 0, sizeof(fileName));
    sprintf(fileName, "DataCollection%d", counter);
  }
  myFile = SD.open(fileName, FILE_WRITE);
}

void closefile(void){ //----------------------------Close the file--------------------------
  digitalWrite(ActiveWriteLED, LOW);
  Serial.println("File Closed");
  myFile.close();
}

void writefile(void){ //-----------------------------Write to File--------------------------
  float seconds = millis()/1000.0;
  myFile.print(seconds);
  myFile.print(",");
  myFile.println(loadCell.get_units(), 1);
 
}

//check desired pin, edge triggered output. Debounce and anti-transient design
int checkpin(int pinNumber, int *pinState){ //------------------------------Check Switch-------------------------
  int currentValue;
  currentValue = digitalRead(pinNumber); //check switch, 1: not pressed, 0: pressed
  
  if(currentValue != *pinState){ //if they dont match, possible button state change
    delay(5); //debounce delay
    currentValue = digitalRead(pinNumber); //check switch again, transient buffer
    
    if(currentValue != *pinState){ //if the dont match, actual button state change
      *pinState = currentValue; //realize state change

      if(*pinState == HIGH){ 
        return 1; //low to high
      }else if(*pinState == LOW){
        return 2; //high to low
      }
    }
  }
  return 0; //inactive
}
