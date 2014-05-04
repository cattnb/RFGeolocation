#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <netdb.h>
#include <math.h>
#include <sys/un.h>
#include "i2c-dev.h"
#include "i2cbusses.h"

//delay 50ms for communication
#define DELAY 50000
#define h_addr h_addr_list[0]
#define portHost 8080
#define portSDR 7000
#define adr 0x07
#define compass_addr 0x21
#define SOCK_PATH "echo_socket"
//#define h_addr2 h_addr_list[1]

int location = 1;

//define hostname as the IP address of the host computer
char hostname[] = "192.168.2.2";

//define hostname as the IP address of the SDR
char hostname2[] = "192.168.10.2";

int s,s2,t;
struct sockaddr_un local, remote;
int len;
int break_case = 0;
int writedata = 0;
typedef struct {
	double x;
	double y;
	int hour;
	int min;
	int sec;
	int valid;
	double speed;
	double aproxHeading;
	double date;
} dataGPS;

int init_GPS(void);
dataGPS getGPS(int);

//socket for host
int sockfd;
struct sockaddr_in serv_addr;
struct hostent *server;

//socket for SDR
int sockfd2;
struct sockaddr_in serv_addr2;
struct hostent *server2;
int sendData[16];

//file for i2c
int n, file;
char *filename = "/dev/i2c-1";
int gpsrequest = 0;
int compass_calibrate = 0;


int init_GPS(void){
        int serial_file;
	serial_file = open( "/dev/ttyO4", O_RDWR);
	if(serial_file == -1)
		fprintf(stderr, "Failed to open UART1: %m\n");
        return serial_file;
}

dataGPS getGPS(int serial_file){
	dataGPS temp;
	temp.x = 0;
	temp.y = 0;
	double time = 0;
	char str[10] = {0};
	temp.hour = 0;
	temp.min = 0;
	temp.sec = 0;
	temp.valid = 1;
	char latpos = '0';
	char longpos = '0';
	char isValid = 'V';
	temp.speed=0;
	temp.aproxHeading=0;
	temp.date = 0;

	int matches = -99;


	//printf("Looping\n");	
	char start[20] = {0};
	char data[500] = {0};
	char newdata[200] = {0};
	if(read(serial_file,data,sizeof(data))< 0)
		fprintf(stderr,"Failed to read GPS: %m\n");

	//printf("\n\n data: %s\n",data);
	for(int i=0;i<(sizeof(data)-50);i++){
		if(data[i] == '$' && data[i+3] == 'R'){
			//printf("Found RMC\n");
			for(int k = 0; k<100; k++){
				newdata[k] = data[i+k];
			}
			break;
		}
	}
	//printf("\n\n");
	//printf("newdata: %s enddata",newdata);
	//printf("\n\n");
	while(1){
		if(strncmp(newdata, "$GPRMC",6)==0) {
			if((matches = sscanf(newdata,"%6s,%lf,%c,%lf,%c,%lf,%c,%lf,%lf,%lf", start, &time, &isValid, &temp.y, &latpos, &temp.x, &longpos,&temp.speed,&temp.aproxHeading,&temp.date)) == 0){
				printf("Yeah, that last thing was bogus");
			}
			break;
		}
		else{
			if(read(serial_file,data,sizeof(data))< 0)
				fprintf(stderr,"Failed to read GPS: %m\n");

			//printf("\n\n data: %s\n",data);
			for(int i=0;i<(sizeof(data)-50);i++){
				if(data[i] == '$' && data[i+3] == 'R'){
					//printf("Found RMC\n");
					for(int k = 0; k<100; k++){
						newdata[k] = data[i+k];
					}
					break;
				}
			}
			
		}
		//printf("%6s, %lf, %c, %lf, %c, %lf, %c, %lf, %lf, %lf\n",start, &temp.time, &isValid, &temp.y, &latpos, &temp.x, &longpos,&temp.speed,&temp.aproxHeading,&temp.date);
	}
	//printf("\ntime: %lf\n\n",time -50000);
	//
	
	sprintf(str,"%lf",time);
	char hour[2] = {str[0],str[1]};
	char min[2] = {str[2],str[3]};
	char sec[2] = {str[4],str[5]};
	temp.hour = atoi(hour)-5;
	temp.min = atoi(min);
	temp.sec = atoi(sec);
	
	
	if(temp.x == 0)
		temp.valid = 0;
	else {
		if(latpos == 'S')
			temp.y = -temp.y;
		if(longpos == 'W')
			temp.x = -temp.x;
	}
	temp.x = (int)temp.x/100 + ((int)temp.x%100 + temp.x - (int)temp.x)/60.0;
	temp.y = (int)temp.y/100 + ((int)temp.y%100 + temp.y - (int)temp.y)/60.0;
	return temp;
}



void setup(){
	int a = -1;
	/*Communication with Host Computer*/
	if((sockfd=socket(AF_INET,SOCK_STREAM,0))<0){
		printf("Failed to create socket.\n");
	}
	server = gethostbyname(hostname);
	//printf("hostend of HOST: name = %s, aliases = %s, addrtype = %i, length = %i, addr_list = %s\n",server.h_name, server.h_aliases, server.h_addrtype, server.h_length, server.h_addr_list);
	if(server==NULL){
		printf("ERROR, no such host\n");
		exit(0);
	}	
	bzero((char*) &serv_addr, sizeof(serv_addr));	
	serv_addr.sin_family = AF_INET;	
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,server->h_length);
	serv_addr.sin_port = htons(portHost);	
	while(a<0){
		a = connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr));
		/*
		if(a < 0){
			//printf("Failed to connect to Host\n");
		}
		*/
	}
	if(location >0){
		printf("Connecting to SDR...\n");
		//Communication with SDR
		if ((s = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
			perror("socket");
			exit(1);
		}

		local.sun_family = AF_UNIX;
		strcpy(local.sun_path, SOCK_PATH);
		unlink(local.sun_path);
		len = strlen(local.sun_path) + sizeof(local.sun_family);
		if (bind(s, (struct sockaddr *)&local, len) == -1) {
			perror("bind");
			exit(1);
		}

		if (listen(s, 5) == -1) {
			perror("listen");
			exit(1);
		}
		t = sizeof(remote);
		if ((s2 = accept(s, (struct sockaddr *)&remote, &t)) == -1) {
		    perror("accept");
		    exit(1);
		}
	}
	/*Communication with Motor Controller (I2C)*/

	if ((file = open(filename,O_RDWR)) < 0) {
        	printf("Failed to open the bus.\n");
        	exit(1);
    	}
	if (ioctl(file, I2C_SLAVE, adr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
	init_compass(file);
	//calibrate_compass(file);
}

void init_compass(int fd){
	char buf[10] = {0};
        buf[0] = 'G';
        buf[1] = 0x74;
        buf[2] = 0b01100010;
	if (ioctl(file, I2C_SLAVE, compass_addr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
        if(write(fd,buf,3)!=3)
		fprintf(stderr, "Failed to set to continous mode: %m\n");
	if (ioctl(file, I2C_SLAVE, adr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
}


void calibrate_compass(int fd){
	char buf[10] = {0};
	int lmspeed = 0;
	int rmspeed = 0;
	__u8 motorData[27] = {0x0F, 4, ((lmspeed>>8)&0xFF),(lmspeed&0xFF),0,((rmspeed>>8)&0xFF),(rmspeed&0xFF),0,0,0,0,0,0,0,0,0,0,0,0,0,50,0,50,2,188,7,0};
	buf[0] = 'C';
	if (ioctl(file, I2C_SLAVE, compass_addr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
	if(write(file,buf,1)!=1)
		fprintf(stderr, "Failed to start calibration: %m\n");
	printf("Calibration started\n\r");
	if (ioctl(file, I2C_SLAVE, adr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
	for(int i = 5;i<250;i=i+5){
		lmspeed = i;
		rmspeed = -i;
		motorData[2] = ((lmspeed>>8)&0xFF);
		motorData[3] = lmspeed&0xFF;
		motorData[5] = ((rmspeed>>8)&0xFF);
		motorData[6] = rmspeed&0xFF;
		//printf("\n");
		if(write(file,motorData, 27)<0){
  			printf("Failed to write to I2C\n");
			printf("errno = %i\n",errno);
  		}	
	}
	int count = 0;
	while(count <= 1000){
		usleep(1000);
		count++;
		if(write(file,motorData, 27)<0){
  			//printf("Failed to write to I2C\n");
			//printf("errno = %i\n",errno);
  		}
		//printf("%i\n",count);
	}
	//printf("done looping\n");
	for(int i = 250;i<=0;i=i-5){
		lmspeed = i;
		rmspeed = -i;
		if(write(file,motorData, 27)<0){
  			//printf("Failed to write to I2C\n");
			//printf("errno = %i\n",errno);
  		}	
	}

	buf[0] = 'E';
	if (ioctl(file, I2C_SLAVE, compass_addr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
	if(write(file,buf,1)!=1)
		fprintf(stderr, "Failed to exit calibration: %m\n");
	printf("Calibration complete\n\r");
	if (ioctl(file, I2C_SLAVE, adr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
}
float getHeading(int fd){
        char data[10] = {0};
	if (ioctl(file, I2C_SLAVE, compass_addr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
        if(read(file,data,2)!=2)
                fprintf(stderr,"Failed to read compass: %m\n");
        float result = ((data[0] << 8) + data[1])/10.0;
	if (ioctl(file, I2C_SLAVE, adr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
        return result;
}

void print_ip(int ip)
{
    unsigned char bytes[4];
    bytes[0] = ip & 0xFF;
    bytes[1] = (ip >> 8) & 0xFF;
    bytes[2] = (ip >> 16) & 0xFF;
    bytes[3] = (ip >> 24) & 0xFF;	
    printf("%d.%d.%d.%d\n", bytes[3], bytes[2], bytes[1], bytes[0]);        
}
void loop(){

	//Send socket server data to motor controller
		//printf("Sending....\n");
		MasterSend();
		//usleep(DELAY);
		//send motor controller data to socket server
		MasterReceive();
		//usleep(DELAY);
}

void MasterReceive()
{
	unsigned char RSS[24] = {0};
  	/*
	*/

	/*
	if(read(sockfd2,RSS,1)<0){
		printf("Failed to read from SDR\n");
	}
	printf("RSS = %s\n",RSS);
	
	/*
	printf("Data from I2C = ");
	for(int i = 0;i<sizeof(bufferRead);i++){
		printf("%i ",bufferRead[i]);
	}
	printf("\n");
	*/
	/*
	for(int i=0;i<7;i++){
		printf("%2.4f ",sendData[i]);	
	}
	printf("\n");
	*/
  	//write Motor Controller data to socket server
	//f(writedata==1){
		/*
		printf("Data to server = ");
		for(int i = 0;i<sizeof(sendData);i++){
			printf("%i ",sendData[i]);
		}
		printf("\n");
		*/
		sendData[13] = location;
	  	if(write(sockfd,sendData,sizeof(sendData))<0){
	  			printf("Failed to write to socket\n");
				break_case = -1;
	  	}
	//}
	writedata = 0;
	
   
}

void MasterSend(){
	//printf("Sending....\n");
	int a;
	unsigned char dataRequest = 'R';
	__u8 buffer[30];
	__u8 temp_buffer[27] = {0x0F,4,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0,0, 0,0, 0,0, 0,50,0, 50,0x02, 0xBC,7,0};
	//read socket server data
  	if(a=read(sockfd, buffer, sizeof(buffer))<0){
  		printf("Failed to read from socket\n");
		printf("errno = %i\n",errno);
		break_case = -1;
  	}
	/*
	printf("Data from server = ");
	for(int i = 0;i<sizeof(buffer);i++){
		printf("%i ",buffer[i]);
	}
	printf("\n");
	*/
	
	//write request to SDR
	/*
	if(write(sockfd2,dataRequest,sizeof(dataRequest))<0){
		printf("Failed to write to SDR\n");
		return;
	}
	*/
	if(buffer[29] == location){
		writedata = 1;
		for(int i=0;i<sizeof(temp_buffer);i++){
			temp_buffer[i] = buffer[i];
		}
		gpsrequest = (int) buffer[27];
		compass_calibrate = (int) buffer[28];
	}
	if(write(file,temp_buffer, 27)<0){
	  		//printf("Failed to write to I2C\n");
			//printf("errno = %i\n",errno);
	}
	/*printf("Data to i2c ");
	for(int i = 0;i<sizeof(temp_buffer);i++){
		printf("%i ",temp_buffer[i]);
	}
	printf("\n");*/
	//else data is for another sensor
 
}


void main(){
	//set up I2C and socket communication
	setup();
	int serial_file = init_GPS();
	char SDRdata[14];
	char newSDRdata[8];
	float RSS;
	printf("Setup complete\n");
	__u8 bufferRead[24] = {0};
	gpsrequest = 0;
	//int count = 0;
 	while(1){
		if(gpsrequest){
			//usleep(20000);
			if(location>0){
				write(s2,"1",1);
				usleep(5000);
				read(s2,SDRdata,14);
				/*
				for(int i=0;i<14;i++){
					newSDRdata[i] = SDRdata[i];
				}
				*/
				//RSS = strtof(SDRdata,NULL);
				//printf("RSS string: %s RSS float: %3.8f RSS float: %3.8f\n",SDRdata,atof(SDRdata),strtof(SDRdata,NULL));
				//strtol(SDRdata,&RSS,10);
				sendData[14] = (int)atof(SDRdata);
				sendData[15] = (int) 10000000 * (-atof(SDRdata) + sendData[14]);
				printf("RSS = %i.%i\n",sendData[14],sendData[15]);
			}
			float heading = getHeading(file);
			//printf("Heading: %3.1f\n",heading);
			sendData[11] = (int) heading;
			sendData[12] = (int) 1000*(heading - sendData[11]);
			//printf("11: %i, 12: %i\n",sendData[11],sendData[12]);
			dataGPS gps = getGPS(serial_file);
			if(gps.valid==1){
				printf("GPS Has Fix\n");
				dataGPS gps = getGPS(serial_file);
				sendData[0] = (int) gps.y;
				sendData[1] = (int) 100000*(gps.y - sendData[0]);
				sendData[2] = (int) gps.x;
				sendData[3] = (int) 100000*(gps.x - sendData[2]);
				
				sendData[4] = gps.hour + 1;
				sendData[5] = gps.min;
				sendData[6] = gps.sec;
				sendData[7] = (int) gps.speed;
				sendData[8] = (int) 10000*(gps.speed - sendData[7]);
				sendData[9] = (int) gps.aproxHeading;
				sendData[10] = (int) 10000*(gps.aproxHeading - sendData[9]);
				//printf("Y: %2.8lf X: %2.8lf Time: %i:%i:%i Speed: %2.2lf Heading: %3.2lf\n\r",gps.y, gps.x, gps.hour,gps.min,gps.sec,gps.speed,gps.aproxHeading);
				//sleep(1);
			}
			/*
			if(read(file, bufferRead, sizeof(bufferRead)<0)){
				printf("Failed to read from I2C\n");
				sendData[11] = 0;
			}
			else sendData[11] = (int) bufferRead[2]<<8 || bufferRead[3];
			*/
			gpsrequest = 0;
		}
		if(compass_calibrate){
			calibrate_compass(file);
		}
		//printf("Looping.... count = %i\n",count);
		//count++;
		loop();
		if(break_case<0) break;
		
	}
	close(sockfd);
}
