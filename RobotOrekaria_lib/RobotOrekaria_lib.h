#include <dogm_7036.h>
#include <PID_v1.h>
#include <IMU_lib2.h>
#include <CF_lib3.h>
#include <Motor_ktrl_lib.h>

void LCD_init(dogm_7036* DOG, 
			  double Kp, double Ki, double Kd, 
			  double Setpoint, double Input, double Output);
			  
void Param_select(dogm_7036* DOG, double Setpoint, 
				  double Kp, double Ki, double Kd,
                  volatile double* encoderValue, int* aux);

void Param_update(dogm_7036* DOG, double* Setpoint, 
				  double* Kp, double* Ki, double* Kd, 
                  volatile double encoderValue, int aux);

void PID_init(PID* myPID, unsigned long SampleTime, double outMin, double outMax);

void IO_update(dogm_7036* DOG, double Input, double Output, int aux);