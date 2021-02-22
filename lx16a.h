//
// Created by Zhengzh on 2020/11/21.
//

#ifndef CTYPES_LX16A_H
#define CTYPES_LX16A_H

#define SERVO_ID_ALL  0xfe

#define SERVO_MOVE_TIME_WRITE  1
#define SERVO_MOVE_TIME_READ  2
#define SERVO_MOVE_TIME_WAIT_WRITE  7
#define SERVO_MOVE_TIME_WAIT_READ  8
#define SERVO_MOVE_START  11
#define SERVO_MOVE_STOP  12
#define SERVO_ID_WRITE  13
#define SERVO_ID_READ  14
#define SERVO_ANGLE_OFFSET_ADJUST  17
#define SERVO_ANGLE_OFFSET_WRITE  18
#define SERVO_ANGLE_OFFSET_READ  19
#define SERVO_ANGLE_LIMIT_WRITE  20
#define SERVO_ANGLE_LIMIT_READ  21
#define SERVO_VIN_LIMIT_WRITE  22
#define SERVO_VIN_LIMIT_READ  23
#define SERVO_TEMP_MAX_LIMIT_WRITE  24
#define SERVO_TEMP_MAX_LIMIT_READ  25
#define SERVO_TEMP_READ  26
#define SERVO_VIN_READ  27
#define SERVO_POS_READ  28
#define SERVO_OR_MOTOR_MODE_WRITE  29
#define SERVO_OR_MOTOR_MODE_READ  30
#define SERVO_LOAD_OR_UNLOAD_WRITE  31
#define SERVO_LOAD_OR_UNLOAD_READ  32
#define SERVO_LED_CTRL_WRITE  33
#define SERVO_LED_CTRL_READ  34
#define SERVO_LED_ERROR_WRITE  35
#define SERVO_LED_ERROR_READ  36

#define SERVO_ERROR_OVER_TEMPERATURE  1
#define SERVO_ERROR_OVER_VOLTAGE  2
#define SERVO_ERROR_LOCKED_ROTOR  4

// Lower level Serial Bus control functions
int IO_init(char *filename);

void setServoID(char id, char new_id);
void move(char id, short position, short time);
void movePrepare(char id, short position, short time);
int getPreparedMove(char id);
void moveStart(char id);
void moveStop(char id);
void setPositionOffset(char id, char deviation);
char getPositionOffset(char id);
void setPositionLimits(char id, short minPosition, short maxPosition);
int getPositionLimits(char id);
void savePositionOffset(char id);
void setVoltageLimits(char id, short min_volt, short max_volt);
int getVoltageLimits(char id);
void setMaxTemp(char id, char temp);
char getMaxTemp(char id);
char getTemp(char id);
short getVoltage(char id);
void setServoMode(char id);
char getMode(char id);
void motorOn(char id);
void motorOff(char id);
char isMotorOn(char id);
void setLED(char id, char status);
char isLEDOn(char id);
void setLEDErrors(char id, char error);
char getLEDErrors(char id);
void setSpeed(char id, short speed);
short getSpeedSetting(char id);
short posRead(char id);


#endif
