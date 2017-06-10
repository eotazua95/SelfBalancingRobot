#include <IMU_lib2.h>

IMU_lib2 imu_lib2;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("setup()");

  imu_lib2.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  int* accel = imu_lib2.Read_Accel();
  Serial.print("accel_x: "); Serial.println(*accel);
  Serial.print("accel_y: "); Serial.println(*(accel+1));
  Serial.print("accel_z: "); Serial.println(*(accel+2));
  
  int* gyro = imu_lib2.Read_Gyro();
  Serial.print("gyro_x: "); Serial.println(*gyro);
  Serial.print("gyro_y: "); Serial.println(*(gyro+1));
  Serial.print("gyro_z: "); Serial.println(*(gyro+2));

  delay(3000);
}
