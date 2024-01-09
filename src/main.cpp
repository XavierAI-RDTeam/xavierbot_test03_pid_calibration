#include <Arduino.h>
#include <definitions.h>
#include <pid.h>
#include <pwmSpeedConverter.h>

PIDEstimator leftFront	(K_P, K_I, K_D, 2.4);
PIDEstimator leftRear	(K_P, K_I, K_D, 2.4);
PIDEstimator rightFront (K_P, K_I, K_D, 2.4);
PIDEstimator rightRear	(K_P, K_I, K_D, 2.4);

PwmSpeedConverter leftFrontConverter	(2.4, 0.00, 0.46, 255.0, 0.0, 225.0, 9.0, 25.0);
PwmSpeedConverter leftRearConverter		(2.4, 0.00, 0.46, 255.0, 0.0, 225.0, 9.0, 25.0);
PwmSpeedConverter rightFrontConverter	(2.4, 0.00, 0.46, 255.0, 0.0, 225.0, 9.0, 25.0);
PwmSpeedConverter rightRearConverter	(2.4, 0.00, 0.18, 255.0, 0.0, 240.0, 5.0, 10.0);

// encoder pulse count
volatile int frontLeftPulseCount  = 0;
volatile int frontRightPulseCount = 0;
volatile int rearLeftPulseCount   = 0;
volatile int rearRightPulseCount  = 0;

// protected encoder pulse count
volatile int frontLeftProtectedPulseCount  = 0;
volatile int frontRightProtectedPulseCount = 0;
volatile int rearLeftProtectedPulseCount   = 0;
volatile int rearRightProtectedPulseCount  = 0;

// tracked values related to PID
float front_left_distance		= 0;
float front_right_distance		= 0;
float rear_left_distance		= 0;
float rear_right_distance		= 0;

float front_left_target			= 0;
float front_left_feedback		= 0;
float left_front_corrected		= 0;
float front_right_target		= 0;
float front_right_feedback		= 0;
float right_front_corrected		= 0;

float rear_left_target			= 0;
float rear_left_feedback		= 0;
float left_rear_corrected		= 0;
float rear_right_target			= 0;
float rear_right_feedback		= 0;
float right_rear_corrected		= 0;

long old_time  = 0;
float duration = 0.0;

/*******************************************************************************
** Interrupts for Encoder
*******************************************************************************/

void isrFrontLeftA() 
{
	if(digitalRead(FRONT_LEFT_ENCODER_A) != digitalRead(FRONT_LEFT_ENCODER_B))  frontLeftPulseCount ++; 
	else frontLeftPulseCount --;
}

void isrFrontLeftB() 
{
	if (digitalRead(FRONT_LEFT_ENCODER_A) == digitalRead(FRONT_LEFT_ENCODER_B)) frontLeftPulseCount ++; 
	else frontLeftPulseCount --;
}

void isrFrontRightA() 
{
	if(digitalRead(FRONT_RIGHT_ENCODER_A) != digitalRead(FRONT_RIGHT_ENCODER_B))  frontRightPulseCount ++; 
	else frontRightPulseCount --;
}

void isrFrontRightB() 
{
	if (digitalRead(FRONT_RIGHT_ENCODER_A) == digitalRead(FRONT_RIGHT_ENCODER_B)) frontRightPulseCount ++;
  else frontRightPulseCount --;
}

void isrRearLeftA() 
{
	if(digitalRead(REAR_LEFT_ENCODER_A) != digitalRead(REAR_LEFT_ENCODER_B))  rearLeftPulseCount ++; 
	else rearLeftPulseCount --;
}

void isrRearLeftB() 
{
	if (digitalRead(REAR_LEFT_ENCODER_A) == digitalRead(REAR_LEFT_ENCODER_B)) rearLeftPulseCount ++; 
	else rearLeftPulseCount --;
}

void isrRearRightA() 
{
	if(digitalRead(REAR_RIGHT_ENCODER_A) != digitalRead(REAR_RIGHT_ENCODER_B))  rearRightPulseCount ++; 
	else rearRightPulseCount --;
}

void isrRearRightB() 
{
	if (digitalRead(REAR_RIGHT_ENCODER_A) == digitalRead(REAR_RIGHT_ENCODER_B)) rearRightPulseCount ++;
  else rearRightPulseCount --;
}

/*******************************************************************************
** Interrupt safe data transfer
*******************************************************************************/

void transferDataFromInterrupts()
{
	noInterrupts();

	frontLeftProtectedPulseCount  = frontLeftPulseCount;
	frontRightProtectedPulseCount = frontRightPulseCount;
  	rearLeftProtectedPulseCount   = rearLeftPulseCount;
	rearRightProtectedPulseCount  = rearRightPulseCount;

	frontLeftPulseCount  = 0;
	frontRightPulseCount = 0;
  	rearLeftPulseCount   = 0;
	rearRightPulseCount  = 0;

	interrupts();
}

void calculateCurrentValues()
{
	front_left_distance  = 2 * PI * WHEEL_RADIUS * ((((float)frontLeftProtectedPulseCount)/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
	front_right_distance = 2 * PI * WHEEL_RADIUS * ((((float)frontRightProtectedPulseCount)/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
	rear_left_distance	 = 2 * PI * WHEEL_RADIUS * ((((float)rearLeftProtectedPulseCount)/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
	rear_right_distance  = 2 * PI * WHEEL_RADIUS * ((((float)rearRightProtectedPulseCount)/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);

	front_left_feedback   = front_left_distance/duration;
	front_right_feedback  = front_right_distance/duration;
	rear_left_feedback    = rear_left_distance/duration;
	rear_right_feedback   = rear_right_distance/duration;
}

void setPWM()
{
	int lfpwm = leftFrontConverter.convert(abs(left_front_corrected));
	int lrpwm = leftRearConverter.convert(abs(left_rear_corrected));
	int rfpwm = rightFrontConverter.convert(abs(right_front_corrected));
	int rrpwm = rightRearConverter.convert(abs(right_rear_corrected));

	if (left_front_corrected >= 0)
	{
		analogWrite(FRONT_LEFT_PWM_FORWARD, lfpwm);
		analogWrite(FRONT_LEFT_PWM_REVERSE, 0);
	}
	else
	{
		analogWrite(FRONT_LEFT_PWM_FORWARD, 0);
		analogWrite(FRONT_LEFT_PWM_REVERSE, lfpwm);
	}

	if (left_rear_corrected >= 0)
	{
		analogWrite(REAR_LEFT_PWM_FORWARD,  lrpwm);
		analogWrite(REAR_LEFT_PWM_REVERSE,  0);
	}
	else
	{
		analogWrite(REAR_LEFT_PWM_FORWARD,  0);
		analogWrite(REAR_LEFT_PWM_REVERSE,  lrpwm);
	}
	if (right_front_corrected >= 0)
	{
		analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
		analogWrite(FRONT_RIGHT_PWM_REVERSE, rfpwm);
	}
	else
	{
		analogWrite(FRONT_RIGHT_PWM_FORWARD, rfpwm);
		analogWrite(FRONT_RIGHT_PWM_REVERSE, 0);
	}

	if (right_rear_corrected >= 0)
	{
		analogWrite(REAR_RIGHT_PWM_FORWARD,  0);
		analogWrite(REAR_RIGHT_PWM_REVERSE,  rrpwm);
	}
	else
	{
		analogWrite(REAR_RIGHT_PWM_FORWARD,  rrpwm);
		analogWrite(REAR_RIGHT_PWM_REVERSE,  0);
	}
}

void runPID()
{
	left_front_corrected  = leftFront.evaluate(front_left_target, front_left_feedback, duration);
	right_front_corrected = rightFront.evaluate(front_right_target,front_right_feedback, duration);
	left_rear_corrected   = leftRear.evaluate(rear_left_target, rear_left_feedback, duration);
	right_rear_corrected  = rightRear.evaluate(rear_right_target, rear_right_feedback, duration);
}

void setup() {
	Serial.begin(115200);

	pinMode(FRONT_LEFT_ENCODER_A, 	INPUT);
	pinMode(FRONT_LEFT_ENCODER_B, 	INPUT);
	pinMode(FRONT_RIGHT_ENCODER_A, 	INPUT);
	pinMode(FRONT_RIGHT_ENCODER_B, 	INPUT);
  	pinMode(REAR_LEFT_ENCODER_A, 	INPUT);
	pinMode(REAR_LEFT_ENCODER_B, 	INPUT);
	pinMode(REAR_RIGHT_ENCODER_A, 	INPUT);
	pinMode(REAR_RIGHT_ENCODER_B, 	INPUT);

	attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCODER_A), isrFrontLeftA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCODER_B), isrFrontLeftB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_A), isrFrontRightA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_B), isrFrontRightB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_A), isrRearLeftA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_B), isrRearLeftB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCODER_A), isrRearRightA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCODER_B), isrRearRightB, CHANGE);

	analogWrite(FRONT_LEFT_PWM_FORWARD, 	0);
	analogWrite(FRONT_LEFT_PWM_REVERSE, 	0);
	analogWrite(FRONT_RIGHT_PWM_FORWARD, 	0);
	analogWrite(FRONT_RIGHT_PWM_REVERSE, 	0);
	analogWrite(REAR_LEFT_PWM_FORWARD, 		0);
	analogWrite(REAR_LEFT_PWM_REVERSE, 		0);
	analogWrite(REAR_RIGHT_PWM_FORWARD, 	0);
	analogWrite(REAR_RIGHT_PWM_REVERSE, 	0);

	front_left_target	= 0.5;
	front_right_target	= 0.5;
	rear_left_target	= 0.5;
	rear_right_target	= 0.5;

	old_time = micros();
}

void loop() 
{
	duration	= (micros() - old_time )/1000000.0;

	transferDataFromInterrupts();

	calculateCurrentValues();

	runPID();

	setPWM();

	Serial.print(front_left_feedback);
	Serial.print(", ");
	Serial.print(front_right_feedback);
	Serial.print(", ");
	Serial.print(rear_left_feedback);
	Serial.print(", ");
	Serial.println(rear_right_feedback);
	
	delay(100);
}