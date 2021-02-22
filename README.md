# Fast LX16A Control Code
## How to run
Before running the program for the first time:

Please compile lx16a.c to .so file first!

You can use the command as:  gcc -g -o lx16a.so -shared -fPIC lx16a.c

The name of lx16a.so can be changed as you like, remember to modify the value 'lib'.
If you don't modify lx16a.c and lx16a.h, there's no need to compile any more.

It is also highly recommended for the user to read the protocol file of the servo motor
very carefully before using this code.

## Class Instruction:
1. After init the class, init the usb port first
2. All the read/get functions will return the data you called, like readPosition(id = 1) will return the current position of servo id = 1. If something wrong happen, the python function will return False and the C function will print the error info. Notice that different get functions might return different numbers of params. Read the Class ServoMotor() carefully first.
3. After testing, each read command would take around 3.7ms on linux. So if the user find it takes much longer than that on linux, please read USBLATENCY.txt file to adjust the latency timer of linux.
4. After using setSpeed() function, the mode of lx16a would be set to motor mode. If the user wants to use move() function, lx16a must be set to servo mode firstly, which means to use setServoMode() before move().

Author: Zhihao Zheng (Arthur), Robert Kwiatkowski
email: zhengzh1220@outlook.com, robert.kwiatkowski@columbia.edu
