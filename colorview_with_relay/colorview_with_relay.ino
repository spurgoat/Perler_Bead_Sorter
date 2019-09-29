#include <Wire.h>
#include "Adafruit_TCS34725.h"


// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
#define stepperOut 8 // creating an output for 12v relay on pin 2
#define kill 7 // kill process button
int analogPin = A3;
int a3val = 0;
int a3inival = 0;
int counter = 0;
int currentState = 0;
int previousState = 0;
int ctu = 0;
// kill button states setup for debouncing read
int killButtonState;
int killLastButtonState = HIGH;

// debounce for kill button
unsigned long killLastDebounceTime = 0;
unsigned long killDebounceDelay = 1;

// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  
  // use these three pins to drive an LED
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(stepperOut, OUTPUT); //output to stepper relay
  pinMode(kill, INPUT); // input for kill process button

  
  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
      
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;      
    }
    //Serial.println(gammatable[i]);
  }
}


void loop() {
  
  int killReading = digitalRead(kill); //getting millisecond reading established for kill button debounce
  if (killReading != killLastButtonState) {
      //reset the debounce timer
      killLastDebounceTime = millis();
    }
  if ((millis() - killLastDebounceTime) > killDebounceDelay) {
      Serial.print("killed"); // prints message if killed
      while(1) { // creates killed state
      }
  }
  
  else {
    a3inival = analogRead(analogPin);  // reads value of potentiometer and save as variable
    a3val = ((a3inival/32)+1); // takes a3inival and converts it into non-zero value divisible by 1024
    Serial.print("Current Step Speed:  "); Serial.println(a3val);
    
    digitalWrite(stepperOut, HIGH); // sets pin 2 relay ON
    delay((1000/(a3val*3))   );  // takes 1000ms to read 
    digitalWrite(stepperOut, LOW); // sets pin 2 relay OFF
    delay((1000/a3val))   ;  //takes 1000ms to read
    
    stepperOut == HIGH;
    currentState = ++ previousState;
    ctu = currentState;  
    
    uint16_t clear, red, green, blue;
  
    tcs.setInterrupt(false);      // turn on LED
 
    tcs.getRawData(&red, &green, &blue, &clear);
  
    tcs.setInterrupt(true);  // turn off LED
    
    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:\t"); Serial.print(blue);
  
    // Figure out some basic hex code for visualization
    uint32_t sum = clear;
    float r, g, b;
    r = red; r /= sum;
    g = green; g /= sum;
    b = blue; b /= sum;
    r *= 256; g *= 256; b *= 256;
    Serial.print("\t");
    Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
    Serial.println();
    Serial.println();
    Serial.print((int)ctu);
    Serial.print(" step counts");
    Serial.println();
    Serial.println();
  
    //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );
  
    analogWrite(redpin, gammatable[(int)r]);
    analogWrite(greenpin, gammatable[(int)g]);
    analogWrite(bluepin, gammatable[(int)b]);
  }
}
