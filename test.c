#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/signal.h>


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

void signal_handler_IO (int status);   /* definition of signal handler */

int word(char low, char high) {
	int word = ((int)low) + (((int)high)<<8);
	return word;
}

char lower_byte(int value) {
	return (char)(value%256);
}

char higher_byte(int value) {
	return (char)((value>>8)%256);
}

int clamp(int min, int max, int val) {
	if (val > max)
		return max;
	else if (val < min)
		return min;
	else
		return val;
}

int sum(char paramc, char* params) {
	int i = 0;
	int sum = 0;
	while (i < paramc) {
		sum += params[i];
		i++;
	}
	return sum;
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

int initialize(char* portname) {
	int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
	        printf("error %d opening %s: %s", errno, portname, strerror (errno));
	        return 1;
	}
	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking
	return fd;
}

char calculate_checksum(char servo_id, char length, char cmd_id, char paramc, char* params) {
	char checksum = 255 - (servo_id+length+cmd_id+sum(paramc, params)) % 255;
}

int command(int fd, char servo_id, char cmd_id, char paramc, char* params) {
	char length = 3+paramc;
	u_char message[paramc+6];
	int i = 0;
	char checksum = 255-((servo_id+length+cmd_id+sum(paramc, params)) % 256);
	message[0] = 0x55;
	message[1] = 0x55;
	message[2] = servo_id;
	message[3] = length;
	message[4] = cmd_id;
	while (i < paramc) {
		message[5+i] = params[i];
		i++;
	}
	message[paramc+5] = checksum;
	write(fd, message, paramc+6);
	//usleep((25+paramc+6)*100);
	return paramc+6;
}

int wait_for_response(int fd, char servo_id, char cmd_id, char* buf) {
	char bytes_read, sid, len, cmd, checksum, test_checksum;
	char start = 0;
	char head[3] = {0};
	while (start != 0x55)	{
		read(fd, &start, 1);
		//printf("Start 0x%x\n", start);
	}
	start = 0;
	read(fd, &start, 1);
	//printf("Start 0x%x\n", start);
	if (start == 0x55) {
		read(fd, head, 3);
		sid = head[0];
		len = head[1];
		cmd = head[2];
		//printf("sid 0x%x\n", sid);
		//printf("len 0x%x\n", len);
		//printf("cmd 0x%x\n", cmd);
		if (sid != servo_id)
			fprintf(stderr, "Warning: Message has unexpected servo ID.\n");
		if (cmd != cmd_id)
			fprintf(stderr, "Warning: Message has unexpected command ID.\n");

		bytes_read = read(fd, buf, len-2);
		if (bytes_read != len-2)
			fprintf(stderr, "Warning: Unexpected number of bytes read.\n");
				
		// checksum = 255-((sid+len+cmd_id+sum(len-3, buf))%256);
		checksum = calculate_checksum(sid, len, cmd_id, len-3, buf);
		test_checksum = buf[len-3];

		if (checksum != test_checksum) {
			fprintf(stderr, "Error: Invalid checksum.\n");
			return 1;
		}
		return 0;
	}
	return 1;
}

int query(int fd, char servo_id, char cmd_id, char* buf) {
	//char bytes_written;
	char bytes_written = command(fd, servo_id, cmd_id, 0, NULL);
	//usleep((25+bytes_written)*100);
	return wait_for_response(fd, servo_id, cmd_id, buf);
}

// Untested 
// Needs Testing
int setServoID(int fd, char id, char new_id) {
	char params[1];
	params[0] = new_id;
	command(fd, id, SERVO_ID_WRITE, 1, params);
	return 0;
}

char getServoID(int fd, char id) {
	char buf[5];
	char resp = query(fd, id, SERVO_ID_READ, buf);
	if (resp == 1)
		return -1;
	return buf[0];
}

void move(int fd, char id, int position, int time) {
	int clamped_pos = clamp(0, 1000, position);
	int clamped_time = clamp(0, 30000, time);
	char params[4] = {0};
	params[0] = lower_byte(clamped_pos);
	params[1] = higher_byte(clamped_pos);
	params[2] = lower_byte(clamped_time);
	params[3] = higher_byte(clamped_time);
	command(fd, id, SERVO_MOVE_TIME_WRITE, 4, params);
}

int getPreparedMove(int fd, char id, char* ret) {
	char buf[5];
	char resp = query(fd, id, SERVO_MOVE_TIME_WAIT_READ, buf);
	if (resp == 1)
		return 0;
	ret[0] = word(buf[0], buf[1]);
	ret[1] = word(buf[2], buf[3]);
	return 1;
}

void movePrepare(int fd, char id, int position, int time) {
	int clamped_pos = clamp(0, 1000, position);
	int clamped_time = clamp(0, 30000, time);
	char params[4] = {0};
	params[0] = lower_byte(clamped_pos);
	params[1] = higher_byte(clamped_pos);
	params[2] = lower_byte(clamped_time);
	params[3] = higher_byte(clamped_time);
	command(fd, id, SERVO_MOVE_TIME_WAIT_WRITE, 4, params);
}

void moveStart(int fd, char id) {
	command(fd, id, SERVO_MOVE_START, 0, NULL);
}

void moveStop(int fd, char id) {
	command(fd, id, SERVO_MOVE_STOP, 0, NULL);
}

char getPositionOffset(int fd, char id) {
	char resp, deviation;
	char buf[2];
	resp = query(fd, id, SERVO_ANGLE_OFFSET_READ, buf);
	if (resp == 1)
		return 0;
	deviation = buf[0];
	if (deviation > 127)
		deviation -= 256;
	return deviation;
}

void setPositionOffset(int fd, char id, char deviation) {
	char params[1] = {0};
	params[0] = clamp(-125, 125, deviation);
	if (params[0] < 0)
		params[0] += 256;
	command(fd, id, SERVO_ANGLE_OFFSET_ADJUST, 1, params);
}

void savePositionOffset(int fd, char id) {
	command(fd, id, SERVO_ANGLE_OFFSET_WRITE, 0, NULL);
}

void getVoltageLimits(int fd, char id, char* ret) {
	char buf[5];
	char resp = query(fd, id, SERVO_VIN_LIMIT_READ, buf);
	if (resp == 1)
		return;
	ret[0] = word(buf[0], buf[1]);
	ret[1] = word(buf[2], buf[3]);
}

void setVoltageLimits(int fd, char id, int min_volt, int max_volt) {
	int clamped_min_volt = clamp(4500, 12000, min_volt);
	int clamped_max_volt = clamp(4500, 12000, max_volt);
	char params[4] = {0};
	params[0] = lower_byte(clamped_min_volt);
	params[1] = higher_byte(clamped_min_volt);
	params[2] = lower_byte(clamped_max_volt);
	params[3] = higher_byte(clamped_max_volt);
	command(fd, id, SERVO_VIN_LIMIT_WRITE, 1, params);
}

char getMaxTemp(int fd, char id) {
	char buf[2];
	char resp = query(fd, id, SERVO_TEMP_MAX_LIMIT_READ, buf);
	if (resp == 1)
		return 0;
	return buf[0];
}

void setMaxTemp(int fd, char id, char temp) {
	char params[1] = {0};
	params[0] = temp;
	command(fd, id, SERVO_TEMP_MAX_LIMIT_WRITE, 1, params);
}

char getTemp(int fd, char id) {
	char buf[2];
	char resp = query(fd, id, SERVO_TEMP_READ, buf);
	if (resp == 1)
		return 0;
	return buf[0];
}

char getVoltage(int fd, char id) {
	char buf[3];
	char resp = query(fd, id, SERVO_VIN_READ, buf);
	if (resp == 1)
		return 0;
	return word(buf[0], buf[1]);
}

char getMode(int fd, char id) {
	char buf[2];
	char resp = query(fd, id, SERVO_OR_MOTOR_MODE_READ, buf);
	if (resp == 1)
		return 0;
	return buf[0];
}

void setServoMode(int fd, char id) {
	char params[4] = {0};
	command(fd, id, SERVO_OR_MOTOR_MODE_WRITE, 4, params);
}

char isMotorOn(int fd, char id) {
	char buf[2];
	char resp = query(fd, id, SERVO_LOAD_OR_UNLOAD_READ, buf);
	if (resp == 1)
		return 0;
	return buf[0];
}

void motorOn(int fd, char id) {
	char params[1] = {1};
	command(fd, id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, params);
}

void motorOff(int fd, char id) {
	char params[1] = {0};
	command(fd, id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, params);
}

char isLEDOn(int fd, char id) {
	char buf[2];
	char resp = query(fd, id, SERVO_LED_CTRL_READ, buf);
	if (resp == 1)
		return 0;
	return buf[0];
}

void LEDOn(int fd, char id) {
	char params[1] = {1};
	command(fd, id, SERVO_LED_CTRL_WRITE, 1, params);
}

void LEDOff(int fd, char id) {
	char params[1] = {0};
	command(fd, id, SERVO_LED_CTRL_WRITE, 1, params);
}

char getLEDErrors(int fd, char id) {
	char buf[2];
	char resp = query(fd, id, SERVO_LED_ERROR_READ, buf);
	if (resp == 1)
		return 0;
	return buf[0];
}

void setLEDErrors(int fd, char id, char error) {
	char clamp_error = clamp(0, 7, error);
	char params[1] = {0};
	params[0] = clamp_error;
	command(fd, id, SERVO_LED_ERROR_WRITE, 1, params);
}

// Tested
int posRead(int fd, char id) {
	char buf[3];
	int position;
	char resp;
	resp = query(fd, id, SERVO_POS_READ, buf);
	if (resp == 1)
		return 0;
	position = word(buf[0], buf[1]);
	if (position > 32767)
		position -= 65536;
	return position;
}

void setSpeed(int fd, char id, int speed) {
	char params[5];
	int clamped_speed = clamp(-1000, 1000, speed);
	params[0] = 1;
	params[1] = 0;
	params[2] = lower_byte(clamped_speed);
	params[3] = higher_byte(clamped_speed);
	if (speed < 0) 
		speed += 65536;
	command(fd, id, SERVO_OR_MOTOR_MODE_WRITE, 4, params);
}

int speedRead(int fd, char id) {
	char buf[4];
	int speed, resp;
	resp = query(fd, id, SERVO_OR_MOTOR_MODE_READ, buf);
	if (resp == 1)
		return 0;
	if (buf[0] != 1)
		return 0;
	speed = word(buf[2], buf[3]);
	if (speed > 32767)
		speed -= 65536;
	return speed;
}

void getPositionLimits(int fd, char id, int* ret) {
	char buf[5];
	char resp = query(fd, id, SERVO_ANGLE_LIMIT_READ, buf);
	if (resp == 1)
		return;
	ret[0] = word(buf[0], buf[1]);
	ret[1] = word(buf[2], buf[3]);
}

void setPositionLimits(int fd, char id, int minPosition, int maxPosition) {
	int clamped_min = clamp(0, 1000, minPosition);
	int clamped_max = clamp(0, 1000, maxPosition);
	char params[4] = {0};
	params[0] = lower_byte(clamped_min);
	params[1] = higher_byte(clamped_min);
	params[2] = lower_byte(clamped_max);
	params[3] = higher_byte(clamped_max);
	command(fd, id, SERVO_ANGLE_LIMIT_WRITE, 4, params);
}


int fd;
struct termios termAttr;
struct sigaction saio;
char flag = 0;
int cmm = 0;
int T1=0,T2=0;
int main(int argc, char *argv[])
{
     fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
     if (fd == -1)
     {
        perror("open_port: Unable to open /dev/ttyO1\n");
        exit(1);
     }

     saio.sa_handler = signal_handler_IO;
     saio.sa_flags = 0;
     saio.sa_restorer = NULL; 
     sigaction(SIGIO,&saio,NULL);

     fcntl(fd, F_SETFL, FNDELAY);
     fcntl(fd, F_SETOWN, getpid());
     fcntl(fd, F_SETFL,  O_ASYNC ); /**<<<<<<------This line made it work.**/

     tcgetattr(fd,&termAttr);
     cfsetispeed(&termAttr,B115200);
     cfsetospeed(&termAttr,B115200);
     termAttr.c_cflag &= ~PARENB;
     termAttr.c_cflag &= ~CSTOPB;
     termAttr.c_cflag &= ~CSIZE;
     termAttr.c_cflag |= CS8;
     termAttr.c_cflag |= (CLOCAL | CREAD);
     termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
     termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
     termAttr.c_oflag &= ~OPOST;
     tcsetattr(fd,TCSANOW,&termAttr);
     printf("UART1 configured....\n");
	printf("CLOCKS_PER_SEC: %ld\n", CLOCKS_PER_SEC);//1000000
	sleep(3);
	static clock_t t1,t2;
	t1=clock();
	char bytes_written;
     while(1){
		t2=clock();
		if((double)(t2-t1)/1000 > 3)
		{
			
			switch(cmm%2)
			{
				case 0: bytes_written = command(fd, 2, SERVO_POS_READ, 0, NULL); break;
			
				case 1: setSpeed(fd,2,0); break;
			}
			
			
			//printf("send: %f\n", (double)(t2-t1));
			t1 = clock();
			cmm ++;
		}
		
		
		//sleep(1);
		if(flag == 1)
		{
			sleep(1);
		}
     }

     close(fd);
     exit(0);             
}



u_char USART_RX_BUF[256] = {0};     
char sid,len = 5,cmd;
short pos[8],spd[8];
short pos1,spd1;
int count1=0,count2 = 0;
clock_t t3,t4;	
static void analyse_data2(u_char data_t)
{ 
	static u_char state = 0;
	

	USART_RX_BUF[state] = data_t;

	
    if(state == 0)
	{
		if(USART_RX_BUF[state] == 0x55)
			state ++ ;
	}
	else if(state == 1)
	{
		if(USART_RX_BUF[state] == 0x55)
			state ++;
		else
			state = 0;
	}
	else if(state == 5)
	{
		sid = USART_RX_BUF[2];
		len = USART_RX_BUF[3];
		cmd = USART_RX_BUF[4];
		if(len <= 4)
		{
			printf("wrong length!\n");
			flag = 1;
			state = 0;
		}
		else
		{
			state ++;
		}
			
	}
	else if(state >= len + 2)
	{
		switch(cmd)
		{
			case 28:
				pos[sid] = (USART_RX_BUF[6]<<8 | USART_RX_BUF[5]);
				pos1 = (USART_RX_BUF[6]<<8 | USART_RX_BUF[5]);
				//printf("pos = %d\n", pos1);
				
				count1++;
				
				if(count1 >= 1000)
				{
					T1 ++;
					double time_gap = clock() - t3;
					printf("1000 motor1, num = %d, pos = %d\n, gap = %f\n", T1,pos1,time_gap);
					t3 = clock();
					count1 = 0;
						
				}
				if(count2 >= 1000)
				{
					count2 = 0;
					T2 ++;
				}
	
			break;
			
			case 30:
				spd[sid] = (USART_RX_BUF[8]<<8 | USART_RX_BUF[7]);
				spd1 = (USART_RX_BUF[8]<<8 | USART_RX_BUF[7]);
				//count ++;
			
			break;
				
		}
		state = 0;
	}
	else
		state ++;
	
	
}
int count=0;
void signal_handler_IO (int status)
{
	u_char res;
	read(fd, &res, 1);
	if(flag == 1)
	{
		printf("res: %d\t",res);
	}
	count++;
	analyse_data2(res); 
	// if(count >= 1000)
	// {
	// 	printf("rec 1000 bytes!\n");
	// 	count = 0;
	// }
	
    //printf("received data from UART.\n");
}

// int main() {
// 	char *portname = "/dev/ttyUSB0";
// 	int servo_id = 1;
// 	int servo_speed;
	
// 	servo_speed = 0;
// 	int fd = initialize(portname);
// 	int lim[2] = {0};
// 	printf("Pos %d\n", posRead(fd, servo_id));
// 	setSpeed(fd, servo_id, servo_speed);

// 	return 0;
	
// }

