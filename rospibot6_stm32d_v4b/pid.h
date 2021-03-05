//
// PID CONTROL
//
#include <PID_v1.h>

// steering PID
#define STEERING_PID_TARGET 0
#define STEERING_PID_SAMPLE_TIME 50 // 5
#define STEERING_PID_MIN_OUTPUT -100
#define STEERING_PID_MAX_OUTPUT 100
#define STEERING_PID_ADAPTATIVE_LIMIT 20
#define STEERING_PID_ADAPTATIVE_DELAY 100
double steeringPidKp0 = 1.0;
double steeringPidKi0 = 0.01;
double steeringPidKd0 = 0.0;
double steeringPidKp1 = 100.0;
double steeringPidKi1 = 10.0;
double steeringPidKd1 = 0;
boolean steeringPidAdaptativeStart = false;
unsigned long steeringPidAdaptativeTimer;
double steeringPidInput = 0;
int steeringPidInputLast = 0;
double steeringPidOutput; // Power supplied to the motor PWM value.
double steeringPidSetPoint;
PID steeringPid(&steeringPidInput, &steeringPidOutput, &steeringPidSetPoint, steeringPidKp0, steeringPidKi0, steeringPidKd0, REVERSE); 
boolean steeringdPidResult = false;
boolean useSteeringPid = false;

#define SPEED_PID_TARGET 40
#define SPEED_PID_SAMPLE_TIME 25.0
#define SPEED_PID_ADAPTATIVE_LIMIT1 15
#define SPEED_PID_ADAPTATIVE_LIMIT2 30

// speed change process
#define SPEED_PID_CHANGE_DELAY 300   // time to wait before start change speed (SPEED_PID_SAMPLE_TIME * 12)

unsigned long speedPidChangeTimer;
int speedPidStepInterval = 250; // time between speed change steps
byte speedPidChangeStatus = 0;

// left speed PID control
#define LEFT_SPEED_PID_MIN_OUTPUT 13000
#define LEFT_SPEED_PID_MAX_OUTPUT 64000
double leftSpeedPidKp0 = 6000;
double leftSpeedPidKi0 = 500;
double leftSpeedPidKd0 = 0;
/*
double leftSpeedPidKp1 = 0;
double leftSpeedPidKi1 = 0;
double leftSpeedPidKd1 = 0;
double leftSpeedPidKp2 = 0;
double leftSpeedPidKi2 = 0;
double leftSpeedPidKd2 = 0;
*/
double leftSpeedPidKp1 = leftSpeedPidKp0;
double leftSpeedPidKi1 = leftSpeedPidKi0;
double leftSpeedPidKd1 = leftSpeedPidKd0;
double leftSpeedPidKp2 = leftSpeedPidKp1;
double leftSpeedPidKi2 = leftSpeedPidKi1;
double leftSpeedPidKd2 = leftSpeedPidKd1;

double leftSpeedPidInput = 0;
byte leftSpeedPidInputLast = 0; // debug
double leftSpeedPidOutput; // Power supplied to the motor PWM value.
double leftSpeedPidSetPoint;
PID leftSpeedPid(&leftSpeedPidInput, &leftSpeedPidOutput, &leftSpeedPidSetPoint, leftSpeedPidKp0, leftSpeedPidKi0, leftSpeedPidKd0, DIRECT); 
boolean leftSpeedPidResult = false;

// right speed PID control
#define RIGHT_SPEED_PID_MIN_OUTPUT 13000
#define RIGHT_SPEED_PID_MAX_OUTPUT 64000
double rightSpeedPidKp0 = 6000;
double rightSpeedPidKi0 = 500;
double rightSpeedPidKd0 = 0;
/*
double rightSpeedPidKp1 = 0;
double rightSpeedPidKi1 = 0;
double rightSpeedPidKd1 = 0;
double rightSpeedPidKp2 = 0;
double rightSpeedPidKi2 = 0;
double rightSpeedPidKd2 = 0;
*/
double rightSpeedPidKp1 = rightSpeedPidKp0;
double rightSpeedPidKi1 = rightSpeedPidKi0;
double rightSpeedPidKd1 = rightSpeedPidKd0;
double rightSpeedPidKp2 = rightSpeedPidKp1;
double rightSpeedPidKi2 = rightSpeedPidKi1;
double rightSpeedPidKd2 = rightSpeedPidKd1;

double rightSpeedPidInput = 0;
byte rightSpeedPidInputLast = 0; // debug
double rightSpeedPidOutput; // Power supplied to the motor PWM value.
double rightSpeedPidSetPoint;
PID rightSpeedPid(&rightSpeedPidInput, &rightSpeedPidOutput, &rightSpeedPidSetPoint, rightSpeedPidKp0, rightSpeedPidKi0, rightSpeedPidKd0, DIRECT); 
boolean rightSpeedPidResult = false;

// added for twist messages
double leftSpeedPidSetPointTmp;
int leftSpeedPidSetPointDirection;
double rightSpeedPidSetPointTmp;
int rightSpeedPidSetPointDirection;

//#define PID_MIN_SAMPLE_PULSES 480.0
//#define PID_MAX_SAMPLE_PULSES 2400.0
double minPidVel = 0.40;
double maxPidVel = 1.40;
