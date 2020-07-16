#include "Arduino.h"
#include "LineFollower.h"
#include <Wire.h>


LineFollower::LineFollower()
{
	
	//SDA=D2 and SCL=D1
    Wire.begin();
    //Calibration Min and Max
    int  cal_max[8];
    int  cal_min[8];
      
}

void LineFollower::calibrate()
{
  int raw[8];
  Wire.requestFrom(9, 16);
    for (int x=0; x <= 100; x++) {
	for (int i=0; i <= 7; i++) {
	  raw[i] = Wire.read() << 8 | Wire.read();
	  
	  if (cal_min[i] < 1){
		  cal_min[i] = raw[0];
	   }	  
	  cal_min[i] = ((raw[i] > 0) && (raw[i] < cal_min[i])) ? raw[i] : cal_min[i];
	  cal_max[i] = (raw[i] > cal_max[i]) ? raw[i] : cal_max[i];
	}
  }
}

void LineFollower::calibrate_show()
{
  Serial.print("cal_show min\t");
  	for (int i=0; i <= 7; i++) {
	    Serial.print(cal_min[i]);
        Serial.print("\t"); 
	}    
	Serial.println("");

  Serial.print("cal_show max\t");
  	for (int i=0; i <= 7; i++) {
	    Serial.print(cal_max[i]);
        Serial.print("\t"); 
	}
	Serial.println("");    
 }

void LineFollower::calibrate_reset()
{
  	for (int i=0; i <= 7; i++) { 
		cal_min[i]=0; 
		cal_max[i]=0;
	}
 }

void LineFollower::test()
{
  Serial.print("Test: "); 
  int  r[8];
  unsigned char _data[16];
  int reqlen=Wire.requestFrom(9, 16);
  for (int i=0; i <= reqlen-1; i=i+2){
    _data[i] = Wire.read(); 
    _data[i+1] = Wire.read();
    r[i/2] = _data[i] << 8 | _data[i+1];
    Serial.print(r[i/2]);
    Serial.print("\t"); 
  }
  Serial.println("");
}

uint8_t LineFollower::get_line_reading(int s) {
  int r = 0;
  Wire.requestFrom(9, 16);
  for (int i=0; i <= 7; i++) (( Wire.read() << 8 | Wire.read() ) > s)?r|=1<<i:r&=~(1<<i);
  return r;
}

int16_t LineFollower::get_distance(uint8_t line_value) {
    //Array holding the distance from each sensor from the mid point
    int16_t distances[] = {3938, 2813, 1688, 563, -563, -1168, -2813, -3938};
    int16_t sum = 0;
    int16_t divisor = 0;
    int16_t weight;
    uint8_t i;

    //Find the average distance
    for (i = 0; i < 7; i++)
    {
        weight = ((line_value >> i) & 0x1);

        if (weight) {
          divisor++;
        }
        sum += distances[i] * weight;
    }

    return sum / divisor;
}