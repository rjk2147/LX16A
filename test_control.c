#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h> 
#include <sys/types.h> 
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include "lx16a.h"
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/signal.h>
char SERVO_POS_READ = 28;
char SERVO_OR_MOTOR_MODE_READ = 30;

void signal_handler_IO (int status);   /* definition of signal handler */

int fd;
struct termios termAttr;
struct sigaction saio;
char flag = 0;
int cmm = 0;
int T1=0,T2=0;
short pos[8],spd[8];
short act[8] = {0,0,0,0, 0,0,0,0};

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
			
			switch(cmm%24)
			{
				case 0: setSpeed(fd,2,act[0]); break;
				case 1: setSpeed(fd,3,act[1]); break;
				case 2: setSpeed(fd,4,act[2]); break;
				case 3: setSpeed(fd,5,act[3]); break;
				case 4: setSpeed(fd,6,act[4]); break;
				case 5: setSpeed(fd,7,act[5]); break;
				case 6: setSpeed(fd,8,act[6]); break;
				case 7: setSpeed(fd,9,act[7]); break;

				case 8: bytes_written = command(fd, 2, SERVO_POS_READ, 0, NULL); break;
				case 9: bytes_written = command(fd, 3, SERVO_POS_READ, 0, NULL); break;
				case 10: bytes_written = command(fd,4, SERVO_POS_READ, 0, NULL); break;
				case 11: bytes_written = command(fd, 5, SERVO_POS_READ, 0, NULL); break;
				case 12: bytes_written = command(fd, 6, SERVO_POS_READ, 0, NULL); break;
				case 13: bytes_written = command(fd, 7, SERVO_POS_READ, 0, NULL); break;
				case 14: bytes_written = command(fd, 8, SERVO_POS_READ, 0, NULL); break;
				case 15: bytes_written = command(fd, 9, SERVO_POS_READ, 0, NULL); break;

				case 16: bytes_written = command(fd, 2, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
				case 17: bytes_written = command(fd, 3, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
				case 18: bytes_written = command(fd, 4, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
				case 19: bytes_written = command(fd, 5, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
				case 20: bytes_written = command(fd, 6, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
				case 21: bytes_written = command(fd, 7, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
				case 22: bytes_written = command(fd, 8, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
				case 23: bytes_written = command(fd, 9, SERVO_OR_MOTOR_MODE_READ, 0, NULL); break;
			
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
short pos1,spd1;
int count1=0,count2 = 0;
	
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
			printf("wrong lenth!\n");
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
			  //printf("pos = %d\t%d\t%d\t%d\n", pos[0],pos[1],pos[2],pos[3]);
			  
				count1++;
				
				if(count1 >= 1000)
				{
					T1 ++;
					printf("1000 motor1!!, T1 = %d, pos = %d\n", T1,pos1);
			  		printf("pos = %d\t%d\t%d\t%d\n", pos[0],pos[1],pos[2],pos[3]);
					count1 = 0;
						
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
