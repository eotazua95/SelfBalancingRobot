#include <RobotOrekaria_lib.h>

void LCD_init(dogm_7036* DOG, 
			  double Kp, double Ki, double Kd, 
			  double Setpoint, double Input, double Output) {
	DOG->initialize(49,0,0,48,47,1,DOGM163); //SS, 0, 0 = HW SPI, RS, 47 = RESET, 1 = 5V, EA DOGM163-A (=3 lines)
	DOG->displ_onoff(true);
	DOG->cursor_onoff(false);	
	DOG->contrast(0x08);
	DOG->clear_display();

	char c[5];
	dtostrf(Kp, 5, 1, c);
	DOG->position(1,1);             
	DOG->string("Kp="); DOG->string(c);			
	            
	dtostrf(Ki, 5, 1, c);
	DOG->position(1,2);             
	DOG->string("Ki="); DOG->string(c);
	
	dtostrf(Kd, 5, 1, c);
	DOG->position(1,3);             
	DOG->string("Kd="); DOG->string(c);
	
	dtostrf(Setpoint, 5, 1, c);
	DOG->position(10,1);             
	DOG->string("S="); DOG->string(c);
	
	dtostrf(Input, 5, 1, c);
	DOG->position(10,2);             
	DOG->string("I="); DOG->string(c);
	
	dtostrf(Output, 5, 0, c);
	DOG->position(10,3);             
	DOG->string("O="); DOG->string(c);
}

void PID_init(PID* myPID, unsigned long SampleTime, double outMin, double outMax) {
	myPID->SetControllerDirection(DIRECT);
	myPID->SetMode(AUTOMATIC);
	myPID->SetOutputLimits(outMin, outMax); 
	myPID->SetSampleTime(SampleTime);
}

void Param_select(dogm_7036* DOG, double Setpoint, 
				  double Kp, double Ki, double Kd,
                  volatile double* encoderValue, int* aux) {
	delay(150); // antirrebote
	
	(*aux)++;
	switch(*aux) {
		case 1:
			*encoderValue = Kp;
			DOG->cursor_onoff(true);
			DOG->position(1,1);
			break;
		case 2:
			*encoderValue = Ki;
			DOG->position(1,2);
			break;
		case 3:
			*encoderValue = Kd;
			DOG->position(1,3);
			break;
		case 4:
			*encoderValue = Setpoint;
			DOG->position(10,1);
			break;
		default:
			break;
	}
}

void Param_update(dogm_7036* DOG, double* Setpoint, 
				  double* Kp, double* Ki, double* Kd, 
                  volatile double encoderValue, int aux) {
	char c[5];
    
	switch(aux) {
		case 1:
			*Kp = encoderValue;
			dtostrf(*Kp, 5, 2, c);
			DOG->position(4,1);             
			DOG->string(c);
			DOG->position(1,1); 
			break;
		case 2:
			*Ki = encoderValue;
			dtostrf(*Ki, 5, 2, c);
			DOG->position(4,2);             
			DOG->string(c);
			DOG->position(1,2); 
			break;
		case 3:
			*Kd = encoderValue;
			dtostrf(*Kd, 5, 2, c);
			DOG->position(4,3);             
			DOG->string(c);
			DOG->position(1,3); 
			break;
		case 4:
			*Setpoint = encoderValue;
			dtostrf(*Setpoint, 5, 1, c);
			DOG->position(12,1);             
			DOG->string(c);
			DOG->position(10,1); 
			break;
		default:
			break;
	}
}

void IO_update(dogm_7036* DOG, double Input, double Output, int aux) {
	char c[5];

	dtostrf(Input, 5, 1, c);
	DOG->position(12,2);             
	DOG->string(c);
	
	dtostrf(Output, 5, 0, c);
	DOG->position(12,3);             
	DOG->string(c);
	
	switch(aux) {
		case 1:
			DOG->position(1,1); 
			break;
		case 2:
			DOG->position(1,2);
			break;
		case 3:
			DOG->position(1,3);
			break;
		case 4:
			DOG->position(10,1);
			break;
		default:
			break;
	}
}