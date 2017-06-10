/*
  IMU_lib2.h
  Created by Oskar Casquero, March 28, 2017.
  Released into the public domain.
*/

#ifndef IMU_lib2_h
#define IMU_lib2_h

#include "Arduino.h"
#include <LSM6.h>

class IMU_lib2 {
  public:
    IMU_lib2();
    
	void begin();
	
	int* Read_Accel();
	int* Read_Gyro();
	int* Get_Noise();
	
  private:
    LSM6 imu;
	
	int accel[3];
	int accel_data[3];
	int gyro_data[3];

	int data[6];  
	int data_offset[6];
	int data_noise[6];

    void getOffsetNoise();
};

#endif