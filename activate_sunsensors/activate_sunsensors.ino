//************************//
//   Author Jon Goodrum   //
// jongoodrum01@gmail.com //
//************************//

#include <Wire.h>
#include "TSL2561.h"

// connect SCL (YELLOW) to analog 5
// connect SDA (PURPLE) to analog 4
// connect VDD to 3.3V DC
// connect GROUND to common ground
// ADDR can be connected to ground, 3.3V, or left floating

TSL2561 tsllow(TSL2561_ADDR_LOW);
TSL2561 tslfloat(TSL2561_ADDR_FLOAT);
TSL2561 tslhigh(TSL2561_ADDR_HIGH);

const int lumbuff = 2;


void setup(void) {
  
  //open up a serial port to communicate with teh PC
  Serial.begin(9600);
  
  //search for all 3 sensors, output error code if not found
  if (!tsllow.begin()){Serial.println("5");}    //ERROR CODE 7: low not found
  if (!tslfloat.begin()){Serial.println("6");}  //ERROR CODE 8: float not found
  if (!tslhigh.begin()){Serial.println("7");}   //ERROR CODE 9: high not found
    
  //amount of amplification to measured luminosity values
  tsllow.setGain(TSL2561_GAIN_0X);
  tslfloat.setGain(TSL2561_GAIN_0X);
  tslhigh.setGain(TSL2561_GAIN_0X);
  
  //duration of light absorption. Low times are good with bright lights
  tsllow.setTiming(TSL2561_INTEGRATIONTIME_13MS);
  tslfloat.setTiming(TSL2561_INTEGRATIONTIME_13MS);
  tslhigh.setTiming(TSL2561_INTEGRATIONTIME_13MS);
}

void loop(void) {

  uint16_t xlow = tsllow.getLuminosity(TSL2561_VISIBLE);
  uint16_t xfloat = tslfloat.getLuminosity(TSL2561_VISIBLE);
  uint16_t xhigh = tslhigh.getLuminosity(TSL2561_VISIBLE);


  ///////////////////////////////
  //
  //  A buffer is needed to discriminate 
  //  noticeable luminosity differences
  //
  //  directions
  //  (relative to pad looking at sun):
  //  9 = don't move LEFT or RIGHT
  //  1 = left
  //  3 = right
  //  2 = up
  //  4 = down
  //
  ////////////////////////////////
  
  
  //Determines azimuthal angle (X-AXIS)
  if( ((xfloat - xhigh) > lumbuff) or ((xhigh - xfloat) > lumbuff) ){
    if(xfloat > xhigh){Serial.println("3");}//then move right
    else if(xfloat < xhigh){Serial.println("1");}//or move left
    else                   {Serial.println("9");}//or don't move at all
  }

  //Determines altitudal angle (Y-AXIS)
  if( ((xfloat - xlow) > lumbuff) or ((xlow - xfloat) > lumbuff) ){
  //if there is a large enough difference between top/bottom sensors:
    if(xfloat > xlow){Serial.println("2");}//then move up
    else if(xfloat < xlow){Serial.println("4");}//or move down
    else                  {Serial.println("9");}//or don't move at all
  }

}
