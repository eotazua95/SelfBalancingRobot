/*
  IMU_lib2.h
  Created, March 28, 2017.
  Released into the public domain.
*/

#include "IMU_lib2.h"
#include "Wire.h"
#include <LSM6.h>

IMU_lib2::IMU_lib2() {
}

void IMU_lib2::begin() {
	Wire.begin(); //I2C komunikazioa hasi
	
	if (!imu.init()) {  //dispositiboa ez bada detektatzen bukatzeko
		Serial.println("Failed to autodetect imu type!");
		while (1);
	}
	imu.enableDefault(); //erregistroen baloreak por defecto jarri
	
	Serial.println("IMU_lib2 enableDefault");
	
	// miniIMU_lib2 9 v5 (https://www.pololu.com/product/2736)
	// 4 g scale, 1.66kHz high performance: 1000 10xx
	byte lsm6_ctrl_xl = imu.readReg(LSM6::CTRL1_XL);  //irakurri aldatu nahieko erregistroa
	lsm6_ctrl_xl |= _BV(3) | _BV(7); //1xxx 1xxx
	lsm6_ctrl_xl &= ~(_BV(2)) & ~(_BV(4)) & ~(_BV(5)) & ~(_BV(6)); //1000 10xx
	imu.writeReg(LSM6::CTRL1_XL, lsm6_ctrl_xl); //berridatze balio berria erregistroan
	Serial.println("IMU_lib2 accel 4g");

	// 500 dps scale, 1.66kHz high performance: 1000 01xx
	byte lsm6_ctrl2_g = imu.readReg(LSM6::CTRL2_G); //irakurri aldatu nahieko erregistroa
	lsm6_ctrl2_g |= _BV(2) | _BV(7);  //1xxx x1xx
	lsm6_ctrl2_g &= ~(_BV(3)) & ~(_BV(4)) & ~(_BV(5)) & ~(_BV(6)); //1000 01xx
	imu.writeReg(LSM6::CTRL2_G, lsm6_ctrl2_g); //berridatze balio berria erregistroan
	Serial.println("IMU_lib2 gyro 500dps!");
	
	delay(100); //informazioa pasatzeko bezain besteko denbora
  
	getOffsetNoise();
}

void IMU_lib2::getOffsetNoise() {
  int sampleNum = 100;

  for(int i=0; i<sampleNum; i++) {
    Read_Accel();
    Read_Gyro();
    for(int j=0; j<6; j++) {
      data_offset[j] += data[j];
	}
    delay(10);
  }
    
  for(int j=0; j<6; j++)  //100 muestreoen ax,ay,az,gx,gy,gz aldagaien bataz bestekoa.
    data_offset[j] = data_offset[j]/sampleNum;  

  for(int j=0; j<sampleNum; j++) {  
    Read_Accel();
    Read_Gyro();
    for(int j=0; j<6; j++) {  
      int noise = abs(data[j] - data_offset[j]);  
      // zarata: imu-tik lortutako balio ken aurreko 100 muestretan lortutako baloreen
      // bb-aren balio absolutua. Balio diferentzia hau offset hau zarata izango da
      // ax,ay,az,gx,gy,gz aldagaiek bakoitzak bere zarata izango dute
      if(noise > data_noise[j])
        data_noise[j] = noise;
        // Adb: data_offset[j]=62
        //      data[j]=64 -. noise=4
        //      noise>data_noise[j]
        //      data_noise[j]=4
        // aurreko data_noise baino txikiagoa bada oraingoa,ez da zaratatzat kontzideratuko,
        // horrek zarataren balioa balore handiago batean dagoela esan nahiko baitu       
    }
    delay(10);
  }
}

int* IMU_lib2::Read_Accel() {
  imu.readAcc();
  
  //getOffsetNoise()en erabiltzekoak:
  data[0] = imu.a.x; 
  data[1] = imu.a.y;
  data[2] = imu.a.z;

  //pitchComplementaryFilter() en erabiltzekoak:
  int a_x = data[0] - data_offset[0];
  int a_y = data[1] - data_offset[1];
  int a_z = data[2] - data_offset[2];
  //pitchComplementaryFilter()-en erabili beharreko accel_x, accel_y,
  //accel_y kalkulatzeko data_offset[]; getOffsetNoise()-en lortutako bb
  //data_offset[]-aren baloreak izango dira
  
  //Low Pass Filter
  float alpha = 0.5;
  accel_data[0] = accel_data[0] * alpha + (a_x * (1.0 - alpha));
  accel_data[1] = accel_data[1] * alpha + (a_y * (1.0 - alpha));
  accel_data[2] = accel_data[2] * alpha + (a_z * (1.0 - alpha));
  
  return accel_data;
}

int* IMU_lib2::Read_Gyro() {
  imu.readGyro();

  //getOffsetNoise()en erabiltzekoak:
  data[3] = imu.g.x;
  data[4] = imu.g.y;
  data[5] = imu.g.z;

  //pitchComplementaryFilter() en erabiltzekoak:
  gyro_data[0] = data[3] - data_offset[3];
  gyro_data[1] = data[4] - data_offset[4];
  gyro_data[2] = data[5] - data_offset[5];
  //pitchComplementaryFilter()-en erabili beharreko accel_x, accel_y,
  //accel_y kalkulatzeko data_offset[]; getOffsetNoise()-en lortutako bb
  //data_offset[]-aren baloreak izango dira
  
  return gyro_data;
}

int* IMU_lib2::Get_Noise() {
    
	return data_noise;
}
