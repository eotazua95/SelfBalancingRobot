#include <IMU_lib2.h>
#include <CF_lib3.h>

IMU_lib2 imu_lib2;
CF_lib3 cf_lib3;
  
void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	Serial.println("setup()");

	imu_lib2.begin();
}

void loop() {
	// put your main code here, to run repeatedly:
	int* accel = imu_lib2.Read_Accel();
	int* gyro = imu_lib2.Read_Gyro();
	int* noise = imu_lib2.Get_Noise();

	float roll = cf_lib3.Compute_Roll(accel, *gyro, *(noise+3), 0.95);
	Serial.print("roll: "); Serial.println(roll);
}
