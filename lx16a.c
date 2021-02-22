
/* Author: Zhihao Zheng (Arthur) */

#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <pthread.h>
#include "lx16a.h"
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/signal.h>

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>


int socket_fd_ = -1;
int baudrate_ = 115200;
double  tx_time_per_byte = 0.0;

int cmm = 0;
double time_use,time_use2,time_use1;
struct timeval start,t1;
struct timeval end,t2;

short clamp(short min, short max, short val) {
    if (val > max)
        return max;
    else if (val < min)
        return min;
    else
        return val;
}

uint8_t lower_byte(short value) {
    return (uint8_t)(value%256);
}

uint8_t higher_byte(short value) {
    return (uint8_t)((value>>8)%256);
}


void closePort()
{
    if(socket_fd_ != -1)
        close(socket_fd_);
    socket_fd_ = -1;
}

void clearPort()
{
    tcflush(socket_fd_, TCIFLUSH);
}

int writePort(uint8_t *packet, int length)
{
     return write(socket_fd_, packet, length);
}

int readPort(uint8_t *packet, int length)
{
    return read(socket_fd_, packet, length);
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

char txPacket(uint8_t servo_id, uint8_t cmd_id, uint16_t paramc, uint8_t *params)
{
    int i = 0;
    uint16_t length = paramc + 3;
    uint16_t total_packet_length = paramc+6;

    uint8_t *txpacket = (uint8_t *)malloc(total_packet_length);
    uint8_t checksum = 255-((servo_id+length+cmd_id+sum(paramc, params)) % 256);

    if (txpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

//    if (port->is_using_)
//      return COMM_PORT_BUSY;
//    port->is_using_ = true;

    // make packet header
    txpacket[0]   = 0x55;
    txpacket[1]   = 0x55;
    txpacket[2]   = servo_id;
    txpacket[3] = length;
    txpacket[4] = cmd_id;
    while (i < paramc) {
        txpacket[5+i] = params[i];
        i++;
    }
    txpacket[paramc+5] = checksum;

    // tx packet
    clearPort();
    uint16_t written_packet_length = writePort(txpacket, total_packet_length);
    if (total_packet_length != written_packet_length)
    {
        //is_using_ = false;
        printf("writing length error!\n");
        free(txpacket);
        return -1;
    }

    free(txpacket);
    return 1;
}

char rxPacket(uint8_t *rxpacket)
{
    char     result         = 0;

    uint16_t rx_length     = 0;
    uint16_t wait_length   = 7; // minimum length 4+3

    while(1)
    {
        rx_length += readPort(&rxpacket[rx_length], wait_length - rx_length);
        if (rx_length >= wait_length)
        {
            uint16_t idx = 0;

            // find packet header
            for (idx = 0; idx < (rx_length - 1); idx++)
            {
                if ((rxpacket[idx] == 0x55) && (rxpacket[idx+1] == 0x55) )
                    break;
            }

            if (idx == 0)   // found at the beginning of the packet
            {
                // re-calculate the exact length of the rx packet
                if (wait_length != rxpacket[3] +3 )
                {
                    wait_length = rxpacket[3] +3;
                    continue;
                }

                if (rx_length < wait_length)    //packet not complete
                {
                    ////////// check timeout！！！！！
                    continue;    //breakout and receive again

                }

                // verify checksum
                uint16_t rx_checksum =rxpacket[wait_length-1];
                uint16_t checksum = 255-(sum(wait_length-1, rxpacket) -0x55-0x55) % 256;
                if ( checksum == rx_checksum)
                {
                    result = 1;
                }
                else
                {
                    printf("checksum error!\n");
                    for(uint16_t j = 0; j < wait_length; j++)
                        printf("received packet: %x\t",rxpacket[j]);
                    printf("cal checksum: %x\n",checksum);
                    result = -1;
                }
                break;
            }
            else
            {
                // remove unnecessary packets
                for (uint16_t s = 0; s < rx_length - idx; s++)
                    rxpacket[s] = rxpacket[idx + s];
                //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
                rx_length -= idx;
            }
        }
//        else
//        {
//            //////// check timeout
//            //printf("rxlength < waitlength!!!\n");
//
//        }

        usleep(0);

    }
//    port->is_using_ = false;
//
//    if (result == COMM_SUCCESS)
//    removeStuffing(rxpacket);

    return result;
}

char txrxPacket(uint8_t servo_id, uint8_t cmd_id, uint16_t paramc, uint8_t *params, uint8_t *rxpacket)
{

    char result = txPacket(servo_id, cmd_id, paramc, params);
    if(result != 1)
    {
        printf("readpos error\n");
        return -1;
    }


    // rx packet
    do {
        result = rxPacket(rxpacket);
    } while (result == 1 && rxpacket[2] != servo_id);

    if (result == 1 && servo_id == rxpacket[2])
    {
        return 1;
    }
    else
    {
        printf("wrong id rxpacket\n");
        return -1;
    }

}

void setServoID(char id, char new_id) {
    uint8_t params[1];
    params[0] = new_id;
    char result = txPacket(id, SERVO_ID_WRITE, 1, params);
    if(result != 1)
        printf("SERVO_ID_WRITE error\n");
}

void move(char id, short position, short time) {
    short clamped_pos = clamp(0, 1000, position);
    short clamped_time = clamp(0, 30000, time);
    uint8_t params[4] = {0};
    params[0] = lower_byte(clamped_pos);
    params[1] = higher_byte(clamped_pos);
    params[2] = lower_byte(clamped_time);
    params[3] = higher_byte(clamped_time);
    char result = txPacket(id, SERVO_MOVE_TIME_WRITE, 4, params);
    if(result != 1)
        printf("SERVO_MOVE_TIME_WRITE error\n");
}

void movePrepare(char id, short position, short time) {
    short clamped_pos = clamp(0, 1000, position);
    short clamped_time = clamp(0, 30000, time);
    uint8_t params[4] = {0};
    params[0] = lower_byte(clamped_pos);
    params[1] = higher_byte(clamped_pos);
    params[2] = lower_byte(clamped_time);
    params[3] = higher_byte(clamped_time);
    char result = txPacket(id, SERVO_MOVE_TIME_WAIT_WRITE, 4, params);
    if(result != 1)
        printf("SERVO_MOVE_TIME_WAIT_WRITE error\n");
}

int getPreparedMove(char id) {
    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_MOVE_TIME_WAIT_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        short pos = (rxpacket[6]<<8 | rxpacket[5]);
        short time = (rxpacket[8]<<8 | rxpacket[7]);
        int data = (pos << 16 | time);
        free(rxpacket);
        return data;
    }
    else
    {
        printf("SERVO_MOVE_TIME_WAIT_READ error!\n");
        free(rxpacket);
        return (2000<< 16) ;
    }

    free(rxpacket);
    return 1;
}

void moveStart(char id) {

    char result = txPacket(id, SERVO_MOVE_START, 0, NULL);
    if(result != 1)
        printf("SERVO_MOVE_START error\n");
}

void moveStop(char id) {

    char result = txPacket(id, SERVO_MOVE_STOP, 0, NULL);
    if(result != 1)
        printf("SERVO_MOVE_STOP error\n");
}

void setPositionOffset(char id, char deviation) {
    char params[1] = {0};
    params[0] = clamp(-125, 125, deviation);
    if (params[0] < 0)
        params[0] += 256;
    char result = txPacket(id, SERVO_ANGLE_OFFSET_ADJUST, 1, params);
    if(result != 1)
        printf("SERVO_ANGLE_OFFSET_ADJUST error\n");
}

char getPositionOffset(char id) {

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -127;
    }

    char result = txrxPacket(id, SERVO_ANGLE_OFFSET_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        char offset = rxpacket[5];
        free(rxpacket);
        return offset;
    }
    else
    {
        printf("read pos error!\n");
        free(rxpacket);
        return 127;
    }

    free(rxpacket);
    return 1;
}

void setPositionLimits(char id, short minPosition, short maxPosition) {
    short clamped_min = clamp(0, 1000, minPosition);
    short clamped_max = clamp(0, 1000, maxPosition);
    uint8_t params[4] = {0};
    params[0] = lower_byte(clamped_min);
    params[1] = higher_byte(clamped_min);
    params[2] = lower_byte(clamped_max);
    params[3] = higher_byte(clamped_max);
    char result = txPacket(id, SERVO_ANGLE_LIMIT_WRITE, 4, params);
    if(result != 1)
        printf("SERVO_ANGLE_LIMIT_WRITE error\n");
}

int getPositionLimits(char id) {

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_ANGLE_LIMIT_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        short minpos = (rxpacket[6]<<8 | rxpacket[5]);
        short maxpos = (rxpacket[8]<<8 | rxpacket[7]);
        int data = (minpos << 16 | maxpos);
        free(rxpacket);
        return data;
    }
    else
    {
        printf("SERVO_ANGLE_LIMIT_READ error!\n");
        free(rxpacket);
        return (2000 << 16 | 2000);
    }

    free(rxpacket);
    return 1;
}

void savePositionOffset(char id) {
    char result = txPacket(id, SERVO_ANGLE_OFFSET_WRITE, 0, NULL);
    if(result != 1)
        printf("SERVO_ANGLE_OFFSET_WRITE error\n");
}

void setVoltageLimits(char id, short min_volt, short max_volt) {
    short clamped_min_volt = clamp(4500, 12000, min_volt);
    short clamped_max_volt = clamp(4500, 12000, max_volt);
    uint8_t params[4] = {0};
    params[0] = lower_byte(clamped_min_volt);
    params[1] = higher_byte(clamped_min_volt);
    params[2] = lower_byte(clamped_max_volt);
    params[3] = higher_byte(clamped_max_volt);
    char result = txPacket(id, SERVO_VIN_LIMIT_WRITE, 4, params);
    if(result != 1)
        printf("SERVO_VIN_LIMIT_WRITE error\n");
}

int getVoltageLimits(char id) {

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_VIN_LIMIT_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        short min_volt = (rxpacket[6]<<8 | rxpacket[5]);
        short max_volt = (rxpacket[8]<<8 | rxpacket[7]);
        int data = (min_volt << 16 | max_volt);
        free(rxpacket);
        return data;
    }
    else
    {
        printf("SERVO_VIN_LIMIT_READ error!\n");
        free(rxpacket);
        return (13000<<16);
    }

    free(rxpacket);
    return 1;
}

void setMaxTemp(char id, char temp) {
    uint8_t params[1] = {0};
    params[0] = temp;
    char result = txPacket(id, SERVO_TEMP_MAX_LIMIT_WRITE, 1, params);
    if(result != 1)
        printf("SERVO_TEMP_MAX_LIMIT_WRITE error\n");
}

char getMaxTemp(char id) {

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_TEMP_MAX_LIMIT_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        char maxtemp = rxpacket[5];
        free(rxpacket);
        return maxtemp;
    }
    else
    {
        printf("SERVO_TEMP_MAX_LIMIT_READ error!\n");
        free(rxpacket);
        return 127;
    }

    free(rxpacket);
    return 1;
}

char getTemp(char id) {

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_TEMP_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        char temp = rxpacket[5];
        free(rxpacket);
        return temp;
    }
    else
    {
        printf("SERVO_TEMP_READ error!\n");
        free(rxpacket);
        return 127;
    }

    free(rxpacket);
    return 1;
}

short getVoltage(char id) {
    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_VIN_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        short vol = (rxpacket[6]<<8 | rxpacket[5]);
        free(rxpacket);
        return vol;
    }
    else
    {
        printf("SERVO_VIN_READ error!\n");
        free(rxpacket);
        return 20000;
    }

    free(rxpacket);
    return 1;
}

void motorOn(char id) {
    uint8_t params[1] = {1};
    char result = txPacket(id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, params);
    if(result != 1)
        printf("SERVO_LOAD_OR_UNLOAD_WRITE error\n");
}

void motorOff(char id) {
    uint8_t params[1] = {0};
    char result = txPacket(id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, params);
    if(result != 1)
        printf("SERVO_LOAD_OR_UNLOAD_WRITE error\n");
}

char isMotorOn(char id) {
    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_LOAD_OR_UNLOAD_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        char motoron = rxpacket[5];
        free(rxpacket);
        return motoron;
    }
    else
    {
        printf("SERVO_LOAD_OR_UNLOAD_READ error!\n");
        free(rxpacket);
        return 127;
    }

    free(rxpacket);
    return 1;
}

void setLED(char id, char status) {
    uint8_t params[1] = {0};
    params[0] = status;

    char result = txPacket(id, SERVO_LED_CTRL_WRITE, 1, params);
    if(result != 1)
        printf("SERVO_LED_CTRL_WRITE error\n");
}

char isLEDOn(char id) {

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_LED_CTRL_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        char ledOn = rxpacket[5];
        free(rxpacket);
        return ledOn;
    }
    else
    {
        printf("SERVO_LED_CTRL_READ error!\n");
        free(rxpacket);
        return 127;
    }

    free(rxpacket);
    return 1;
}

void setLEDErrors(char id, char error) {
    char clamp_error = clamp(0, 7, error);
    uint8_t params[1] = {0};
    params[0] = clamp_error;
    char result = txPacket(id, SERVO_LED_ERROR_WRITE, 1, params);
    if(result != 1)
        printf("SERVO_LED_ERROR_WRITE error\n");
}

char getLEDErrors(char id) {

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }
    char result = txrxPacket(id, SERVO_LED_ERROR_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        char ledErr = rxpacket[5];
        free(rxpacket);
        return ledErr;
    }
    else
    {
        printf("SERVO_LED_ERROR_READ error!\n");
        free(rxpacket);
        return 127;
    }

    free(rxpacket);
    return 1;
}

void setServoMode(char id){
    uint8_t params[4];

    params[0] = 0;
    params[1] = 0;
    params[2] = 0;
    params[3] = 0;

    char result = txPacket(id, SERVO_OR_MOTOR_MODE_WRITE, 4, params);
    if(result != 1)
        printf("setServoMode error\n");
}

char getMode(char id){
    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -1;
    }

    char result = txrxPacket(id, SERVO_OR_MOTOR_MODE_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        char mode = rxpacket[5];
        free(rxpacket);
        return mode;
    }
    else
    {
        printf("SERVO_OR_MOTOR_MODE_READ error!\n");
        free(rxpacket);
        return 127;
    }

    free(rxpacket);
    return 1;
}

void setSpeed(char id, short speed) {
    uint8_t params[4];
    short clamped_speed = clamp(-1000, 1000, speed);

    params[0] = 1;
    params[1] = 0;
    params[2] = lower_byte(clamped_speed);
    params[3] = higher_byte(clamped_speed);

    char result = txPacket(id, SERVO_OR_MOTOR_MODE_WRITE, 4, params);
    if(result != 1)
        printf("setspeed error\n");
}

short getSpeedSetting(char id)
{
    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -10000;
    }

    char result = txrxPacket(id, SERVO_OR_MOTOR_MODE_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        short spd = (rxpacket[8]<<8 | rxpacket[7]);
        free(rxpacket);
        return spd;
    }
    else
    {
        printf("SERVO_OR_MOTOR_MODE_READ error!\n");
        free(rxpacket);
        return 10000;
    }

    free(rxpacket);
    return 1;
}

short posRead(char id)
{
//  //test one python circle time
//    gettimeofday(&end,NULL);
//    time_use2=(end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);
//    printf("one gap = %lf\n", time_use2);
//    gettimeofday(&start,NULL);

    uint8_t *rxpacket = (uint8_t *)malloc(10);
    if (rxpacket == NULL)
    {
        printf("malloc error!!!\n");
        return -10000;
    }

    char result = txrxPacket(id, SERVO_POS_READ, 0, NULL, rxpacket);
    if(result == 1)
    {
        short pos = (rxpacket[6]<<8 | rxpacket[5]);
        free(rxpacket);
        return pos;
    }
    else
    {
        printf("read pos error!\n");
        free(rxpacket);
        return 10000;
    }

    free(rxpacket);
    return 1;
}

int IO_init(char *filename)
{
    struct termios newtio;
    //gettimeofday(&start,NULL);
    closePort();

    socket_fd_ = open(filename, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(socket_fd_ < 0)
    {
        printf(" Error opening serial port!\n");
        return -1;
    }

    bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;

    // clean the buffer and activate the settings for the port
    tcflush(socket_fd_, TCIFLUSH);
    tcsetattr(socket_fd_, TCSANOW, &newtio);

    tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;

    printf("UART configured....\n");
    //printf("CLOCKS_PER_SEC: %ld\n", CLOCKS_PER_SEC);//1000000
    sleep(3);
    return 1;
}

//int main(int argc, char *argv[])
//{
//    short pos1 = 0;
//    int cmm =0;
//    char filename[] = "/dev/ttyUSB0";
//    if (IO_init(filename)<0)
//    {
//        exit(1);
//    }
//
//    setSpeed(1, 150);
//    sleep(0.2);
//
////    gettimeofday(&t1,NULL);
//    while(1)
//    {
//        //gettimeofday(&t1,NULL);
//
//        if( cmm <= 2000 )
//        {
//            if(cmm <= 1000)
//            {
//                char result = posRead(1, &pos1);
//                if(result > 0)
//                    printf("run pos = %d\n", pos1);
//                else
//                    printf("read pos1 error!\n");
//            }
//            else if(cmm <= 1002)
//            {
//                setSpeed(1,0);
//                //sleep(0.1);
//            }
//            else
//            {
//                char result = posRead(1, &pos1);
//                if(result > 0)
//                    printf("stop pos1 = %d\n", pos1);
//                else
//                    printf("read pos1 error!\n");
//            }
////            gettimeofday(&start,NULL);
//            //setSpeed(fd,1,0);
//
//
////            gettimeofday(&t2,NULL);
////            time_use1=(t2.tv_sec-t1.tv_sec)*1000000+(t2.tv_usec-t1.tv_usec);
////            printf("one gap = %lf, cmm = %d\n", time_use1,cmm);
//
//            cmm++;
//
//            //t1 = clock();
//        }
//
//
//    }
//
//    setSpeed(1, 0);
//    close(socket_fd_);
//    exit(0);
//}