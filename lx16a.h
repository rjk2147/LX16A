/*
char SERVO_ID_ALL = 0xfe;

char SERVO_MOVE_TIME_WRITE = 1;
char SERVO_MOVE_TIME_READ = 2;
char SERVO_MOVE_TIME_WAIT_WRITE = 7;
char SERVO_MOVE_TIME_WAIT_READ = 8;
char SERVO_MOVE_START = 11;
char SERVO_MOVE_STOP = 12;
char SERVO_ID_WRITE = 13;
char SERVO_ID_READ = 14;
char SERVO_ANGLE_OFFSET_ADJUST = 17;
char SERVO_ANGLE_OFFSET_WRITE = 18;
char SERVO_ANGLE_OFFSET_READ = 19;
char SERVO_ANGLE_LIMIT_WRITE = 20;
char SERVO_ANGLE_LIMIT_READ = 21;
char SERVO_VIN_LIMIT_WRITE = 22;
char SERVO_VIN_LIMIT_READ = 23;
char SERVO_TEMP_MAX_LIMIT_WRITE = 24;
char SERVO_TEMP_MAX_LIMIT_READ = 25;
char SERVO_TEMP_READ = 26;
char SERVO_VIN_READ = 27;
char SERVO_POS_READ = 28;
char SERVO_OR_MOTOR_MODE_WRITE = 29;
char SERVO_OR_MOTOR_MODE_READ = 30;
char SERVO_LOAD_OR_UNLOAD_WRITE = 31;
char SERVO_LOAD_OR_UNLOAD_READ = 32;
char SERVO_LED_CTRL_WRITE = 33;
char SERVO_LED_CTRL_READ = 34;
char SERVO_LED_ERROR_WRITE = 35;
char SERVO_LED_ERROR_READ = 36;

char SERVO_ERROR_OVER_TEMPERATURE = 1;
char SERVO_ERROR_OVER_VOLTAGE = 2;
char SERVO_ERROR_LOCKED_ROTOR = 4;
*/
// Lower level Serial Bus control functions
int initialize(char* portname);
int command(int fd, char servo_id, char cmd_id, char paramc, char* params);
int wait_for_response(int fd, char servo_id, char cmd_id, char* buf);
int query(int fd, char servo_id, char cmd_id, char* buf);
// Untested functions
int setServoID(int fd, char id, char new_id);
char getServoID(int fd, char id);
void move(int fd, char id, int position, int time);
int getPreparedMove(int fd, char id, char* ret);
void movePrepare(int fd, char id, int position, int time);
void moveStart(int fd, char id);
void moveStop(int fd, char id);
char getPositionOffset(int fd, char id);
void setPositionOffset(int fd, char id, char deviation);
void savePositionOffset(int fd, char id);
void getPositionLimits(int fd, char id, int* ret);
void setPositionLimits(int fd, char id, int minPosition, int maxPosition);
void getVoltageLimits(int fd, char id, char* ret);
void setVoltageLimits(int fd, char id, int min_volt, int max_volt);
char getMaxTemp(int fd, char id);
void setMaxTemp(int fd, char id, char temp);
char getTemp(int fd, char id);
char getVoltage(int fd, char id);
char getMode(int fd, char id);
void setServoMode(int fd, char id);
char isMotorOn(int fd, char id);
void motorOn(int fd, char id);
void motorOff(int fd, char id);
char isLEDOn(int fd, char id);
void LEDOn(int fd, char id);
void LEDOff(int fd, char id);
char getLEDErrors(int fd, char id);
void setLEDErrors(int fd, char id, char error);
// Tested functions
int posRead(int fd, char id);
void setSpeed(int fd, char id, int speed);
int speedRead(int fd, char id);
