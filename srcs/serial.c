/*
* serial port programming
*/
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include <sys/time.h>
#include <time.h>	
#include "gps.h"
#include "gpsparser.h"


// void gps_init(double noise);
// void gps_update(double lat, double lon, double seconds_since_last_timestep);
// void gps_read(double* lat, double* lon);
// bool gpsparser(char* data, double* lon, double* lat, double* HDOP, int* numSV)

using namespace std;

const double timestep = 0.100;
const double noise = 1.000;

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0) {
		perror("SetupSerial 1");
		return -1;
	}
	memset(&newtio, 0, sizeof(newtio));

	/*
	* Enable the receiver and set local mode...
	*/
	newtio.c_cflag |= CLOCAL | CREAD;

	/*
	* Set Data Bits
	*/
	newtio.c_cflag &= ~CSIZE;
	switch (nBits) {
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

	/*
	* Set Parity Bit
	*/
	switch (nEvent) {
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	}

	/*
	*	Set Stop Bit
	*/
	if (nStop == 1)
		newtio.c_cflag &= ~CSTOPB;
	else if (nStop == 2)
		newtio.c_cflag |= CSTOPB;
	/*
	*	Set BaudRate
	*/
	switch (nSpeed) {
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 460800:
		cfsetispeed(&newtio, B460800);
		cfsetospeed(&newtio, B460800);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}

	/*
	* Disable Hardware Flow Control
	*/
	newtio.c_cflag &= ~CRTSCTS;

	/*
	* Disable Software Flow Control
	*/
	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

	/******************** Noncanonical Mode***********************/
	// 
	// // Set Min Character & Waiting Time
	// 
	// newtio.c_cc[VTIME] = 1; //segement group
	// newtio.c_cc[VMIN] = 68;

	
	// // use raw input and output
	
	// newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
	// newtio.c_oflag &= ~OPOST; /*Output*/
	/********************************************/

	/******************** Canonical Mode***********************/
	/*
	 Raw 模式输出.
	*/
 	newtio.c_oflag = 0;
 
	/*
	  ICANON  : 致能标准输入, 使所有回应机能停用, 并不送出信号以叫用程序
	*/
 	newtio.c_lflag = ICANON;
 	/********************************************/

	/*
	* Clear Input Queue
	*/
	tcflush(fd, TCIFLUSH); //TCOFLUSH,TCIOFLUSH

	/*
	* Enforce Now
	*/
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
		perror("com set error");
		return -1;
	}

	printf("set done!\n");
	return 0;
}

int open_port(int fd, int comport) {
	//char* dev[] = {"/dev/ttyS0","/dev/ttyS1","/dev/ttyUSB0"};
	//long vdisable;
	if (comport == 1) {
		fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd) {
			perror("Can't Open Serial Port");
			return (-1);
		}
		else
			printf("open ttyS0 ......\n");
	}
	else if (comport == 2) {
		fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd) {
			perror("Can't Open Serial Port");
			return (-1);
		}
		else
			printf("open ttyS1 ......\n");
	}
	else if (comport == 3) {
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
		if (-1 == fd) {
			perror("Can't Open Serial Port");
			return (-1);
		}
		else
			printf("open ttyUSB0 ......\n");
	}

	/*
	*	Block the serial port
	*/
	if (fcntl(fd, F_SETFL, 0) < 0)
		printf("fcntl failed!\n");
	else
		printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
	
	/*
	*	Test whether port is attached to terminal device
	*/
	if (isatty(STDIN_FILENO) == 0)
		printf("standard input is not a terminal device\n");
	else
		printf("isatty success!\n");
	
	printf("fd-open=%d\n", fd);
	return fd;
}

int main(void) {
	int fd = -1;
	int nread,nwrite;
	char sendbuff[] = "$123.42675,12344.23454\n";
	char readbuff[515];
	long cnt = 0;

	double lat,lon,lat_old=118.0,lon_old = 32.0;
	double lat_filt,lon_filt,lat_filt_old = 118.0,lon_filt_old=32.0;
	double HDOP = 3.0;
	int numSV = 0;

	time_t now;
	struct tm *timenow;
	struct timeval  tick;
	long sec;

	// save the raw data and filtered data
	ofstream file1,file2;
	file1.open ("data/raw_data.txt", ios::out | ios::app);
	if(!file1.is_open()){
		cerr<<"open file raw_data.txt failed"<<endl;
		return 1;
	}
	
	file2.open("data/filtered_data.txt",ios::out | ios::app);
	if(!file2.is_open()){
		std::cerr<<"open file filtered_data.txt failed"<<std::endl;
		return 1;
	}

	// set precision
	file1.setf(ios::fixed);
	file1.precision(7);
	file2.setf(ios::fixed);
	file2.precision(7);

	if ((fd = open_port(fd, 3)) < 0) {
		perror("open_port error");
		return 1;
	}
	if (set_opt(fd, 460800, 8, 'N', 1) < 0) {
		perror("set_opt error");
		return 1;
	}
	printf("Starting! ......\n\n");

	//Test serial port
	nwrite = write(fd, sendbuff, sizeof(sendbuff));        //写串口  
    if(nwrite < 0){  
      	perror("write error");
    }
    // gps init
    gps_init(noise);

   	while(1){
   		nread = read(fd, readbuff, sizeof(readbuff));     //读串口数据  
   		if(nread > 0){
      		//printf("msg: %slen: %d\n", readbuff,nread);    //输出读到的数据
      		if(gpsparser(readbuff,&lat,&lon,&HDOP,&numSV)){
      			
      			gps_update(lat,lon,timestep);
      			gps_read(&lat_filt,&lon_filt);
      			time(&now);
      			timenow = localtime(&now);
      			gettimeofday(&tick,NULL);
      			sec = tick.tv_usec/10000;
      			double dist = get_distance(lat_old,lon_old,lat,lon);
      			double dist1 = get_distance(lat_filt_old,lon_filt_old,lat_filt,lon_filt);
      			lat_old = lat, lon_old = lon;
      			lat_filt_old = lat_filt, lon_filt_old = lon_filt;
      			printf("%2d:%2d:%2d:%2ld\tlat:  %f\tlon:  %f\t\tbias: %f\tbias2: %f\n",\
      				timenow->tm_hour,timenow->tm_min,timenow->tm_sec,sec,lat,lon,dist,dist1);
      			printf("        \tlatf: %f\tlonf: %f\t\tHDOP:%f\tnumSV:%d\n\n", lat_filt,lon_filt,HDOP,numSV);
      			file1<<lat<<" "<<lon<<endl;
      			file2<<lat_filt<<" "<<lon_filt<<endl;
      			nread = 0;
      		}
      		else{
      			nread = 0;
      			printf("\nmsg: %s\n\n", readbuff);

      			++cnt;
      			if(cnt%10==0)	printf("parse failed %ld times\n",cnt);
      		}
      		//only test once
      		//break;
   		}
   	}

   	file1.close();
   	file2.close();
	close(fd);
	return 0;
}
