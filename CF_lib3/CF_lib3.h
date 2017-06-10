/*
  CF_lib3.h
  Created by Enaitz Otazua, April 8, 2017.
  Released into the public domain.
*/

#ifndef CF_lib3_h
#define CF_lib3_h

#include "Arduino.h"

class CF_lib3 {
  public:
    CF_lib3();
    void begin(int* accel);
	float Compute_Roll(int* accel, int gyro_x, int data_noise, float _compCoeff);
	
  private:
	float compCoeff;
	int prev_gyro_x;
	float roll;
	float rollGyro;
    unsigned long timer;
};

#endif