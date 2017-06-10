/*
  CF_lib3.h
  Created by Enaitz Otazua, April 8, 2017.
  Released into the public domain.
*/

#include "CF_lib3.h"

CF_lib3::CF_lib3() {
	prev_gyro_x = 0;
	roll = 0;
	rollGyro = 0;
	timer = micros();
}
void CF_lib3::begin(int* accel) {
	int accel_x = *accel;
	int accel_y = *(accel+1);
	int accel_z = *(accel+2);
	
    long squaresum = (long)accel_x*accel_x + (long)accel_z*accel_z;
	roll = atan(accel_y/sqrt(squaresum))*RAD_TO_DEG;
}
float CF_lib3::Compute_Roll(int* accel, int gyro_x, int data_noise, float _compCoeff) {
	int accel_x = *accel;
	int accel_y = *(accel+1);
	int accel_z = *(accel+2);
	
	
	compCoeff = _compCoeff; //gyroak eragin handiagoa izateko, accel-arekin alderatuz; 
							//gyroan denbora luzean eragingo dugu eta accel ean laburrean
							//accel-ak deriban eragin eta gyroak zaratan, horrela konpentsatu
							//ARAZOAK:accel-ak zaratarekiko sentsibleak; gyro-ak deribak sortu

	float dt = (micros() - timer) / 1000000.0f;
	if(gyro_x > data_noise || gyro_x < (-data_noise)) { 
		//gyro_x-ko balioa zarataren umbralean ez badago, zarataren balorea ez badu, ontzat eman!
    
		// Trapezoidal rule for integrating angular velocity: [f(a)+f(b)]/2
		// 17.50mdps/digit is LSM6 sensitivity for 500 dps scale --> kobertzio faktorea angelua lortzeko (rotAng)
		// [degree/s*digit]*digit*s=degree (angelua)
		float rotAng = 0.0175f * ((gyro_x + prev_gyro_x) / 2) * dt; 
		roll += rotAng;  //imu-aren osotasunean angelua emateko beharrezkoa, aurreko _roll, gure azkeneko neurketako angelua baita
		rollGyro += rotAng;  //gyro-aren angelua jakiteko
		prev_gyro_x = gyro_x; //arau trapezoidala burutzeko, oraingo gyro_x, aurrekora pasa hurrengo irakurketarako
	}
	timer = micros();//denbora hau aurrekoa bilakatzeko hurrengo irakurketarako

	//angulo Y =atan(x/[erro(x^2+y^2)])
	//angulo X =atan(y/[erro(x^2+z^2)])
	long squaresum = (long)accel_x*accel_x + (long)accel_z*accel_z;
	//_rollAceel radianetan, segundotara pasatzeko *180/pi
	float rollAccel = atan(accel_y/sqrt(squaresum))*RAD_TO_DEG;

	//angelua=compCoeff*(giroskopioarekin lortutako angelua)+(1-CompCoeff)*(azelerometroarekin lortutako angelua)
	//angelua=compCoeff*(aurrekoangelua+gyro*dt)+(1-CompCoeff)*(accel)
	//angelua=compCoeff*(arau trapezoidala=rotAng)+(1-CompCoeff)*(accel)
	roll = compCoeff*roll + (1.0f-compCoeff)*rollAccel;
  
	/*Serial.print(roll,3); //float zenbakietan 3 dezimal lortzen ditugu horrela
	Serial.print(","); Serial.print(rollAccel,3); 
	Serial.print(","); Serial.println(rollGyro,3);*/
	return roll;
}