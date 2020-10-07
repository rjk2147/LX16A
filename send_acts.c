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
extern char SERVO_POS_READ;
extern char SERVO_OR_MOTOR_MODE_READ;

void signal_handler_IO (int status);   /* definition of signal handler */

int fd;
struct termios termAttr;
struct sigaction saio;
char flag = 0;
int cmm = 0;
int T1=0,T2=0;
short pos[8],spd[8];
int state[16];
short act[8] = {0,0,0,0, 0,0,0,0};
int act_in[8];

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
    mkfifo("/tmp/act_pipe", 0666);
    printf("Opening act pipe...\n");
    int act_fd = open("/tmp/act_pipe", O_RDONLY);

    int delaty = 4; // in ms
    while(1){
		t2=clock();
		if((double)(t2-t1)/1000 > 4)
		{
			if (cmm%24 == 23) {
				// Reading Actions
    				read(act_fd, act_in, sizeof(int)*8);
				// Copying pos and spd to state and act_in to action
				for (int i=0; i<8; i++) {
					act[i] = (short)act_in[i];
					state[i] = (int)pos[i];
					state[i+8] = (int)spd[i];
	
					/**** protection ****/
					if(i%2 == 1)
					{
						if((pos[i] < 10 && act[i] < 0) || (pos[i] > 950 && act[i] > 0))
						{
							act[i] = 0;
							printf("motor %d\t out of range!\n",i+2);
						}	
					}
					else
					{
						if((pos[i] < 300 && act[i] < 0) || (pos[i] > 700 && act[i] > 0))
						{
							act[i] = 0;
							printf("motor %d\t out of range!\n",i+2);
						}
					    
					}
				}
				printf("act = %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", act[0],act[1],act[2],act[3],act[4],act[5],act[6],act[7]);
				// Writing State
			}
			switch(cmm%8)
			{
				case 0: setSpeed(fd,2,act[0]); break;
				case 1: setSpeed(fd,3,act[1]); break;
				case 2: setSpeed(fd,4,act[2]); break;
				case 3: setSpeed(fd,5,act[3]); break;
				case 4: setSpeed(fd,6,act[4]); break;
				case 5: setSpeed(fd,7,act[5]); break;
				case 6: setSpeed(fd,8,act[6]); break;
				case 7: setSpeed(fd,9,act[7]); break;
			
			}
						
			//printf("send: %f\n", (double)(t2-t1));
			t1 = clock();
			cmm ++;
			//printf("Cmd: %d\n", cmm%24);
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
short pos1 = 500,spd1;
int count1=0,count2 = 0;
clock_t t3;
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
				pos[sid-2] = (USART_RX_BUF[6]<<8 | USART_RX_BUF[5]);
				pos1 = (USART_RX_BUF[6]<<8 | USART_RX_BUF[5]);

				/**** protection ****/
				if(sid%2 == 1)
				{
					if((pos1 < 10 && act[sid-2] < 0) || (pos1 > 950 && act[sid-2] > 0))
					{
						act[sid-2] = 0;
						printf("motor %d\t out of range!\n",sid);
					}
						
				}
				else
				{
					if((pos1 < 300 && act[sid-2] < 0) || (pos1 > 700 && act[sid-2] > 0))
					{
						act[sid-2] = 0;
						printf("motor %d\t out of range!\n",sid);
					}
					    
				}
				
			  
			    if(sid == 2) {
				count1++;
				
				if(count1 >= 100)
				{
					T1 ++;
					double gap = (double)(clock() - t3)/CLOCKS_PER_SEC;
					//printf("100 motor1!!, T1 = %d, gap = %f\n", T1,gap);
					//printf("pos = %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],pos[7]);
					//printf("spd = %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", spd[0],spd[1],spd[2],spd[3],spd[4],spd[5],spd[6],spd[7]);
					t3 = clock();
					count1 = 0;
						
				}
			    }
				
	
			break;
			
			case 30:
				spd[sid-2] = (USART_RX_BUF[8]<<8 | USART_RX_BUF[7]);
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
