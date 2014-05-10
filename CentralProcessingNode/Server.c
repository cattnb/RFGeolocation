#include <sys/socket.h>
#include <sys/uio.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stddef.h>
#include <sys/un.h>
#include<pthread.h> //for threading , link with lpthread
#include <signal.h>
#include <linux/joystick.h>

#define _REENTRANT
#define STARTBYTE 0x0F
#define I2CADDRESS 0x07
#define I2CBUS 1
#define DELAY 50000
#define transmitter 0
#define sensor1 1
#define sensor2 2


int sv1[6]={0,0,0,0,0,0};                 // servo positions: 0 = Not Used
//int lmspeed1=0,rmspeed1=0;                                 // left and right motor speed from -255 to +255 (negative value = reverse)
unsigned int lmbrake,rmbrake;                                // left and right motor brake (non zero value = brake)
unsigned int pwmf = 4;
unsigned int devibrate=50;                                   // time delay after impact to prevent false re-triggering due to chassis vibration
int sensitivity=50;                                  // threshold of acceleration / deceleration required to register as an impact
int lowbat=700;                                      // adjust to suit your battery: 550 = 5.50V
unsigned int i2caddr=7;                                      // default I2C address of T'REX is 7. If this is changed, the T'REX will automatically store new address in EEPROM
unsigned int i2cfreq=0;                                      // I2C clock frequency. Default is 0=100kHz. Set to 1 for 400kHz
float voltage = 0;

int receive[16] = {0};

pthread_mutex_t lock;
int service_count =0;
struct js_event xbox;
int xbox1=-1, xbox2 = -1,xbox3 = -3; 
int sockfd, newsockfd,portno,clilen;
struct sockaddr_in serv_addr, cli_addr;
void xboxSetup(){
	xbox1 = open ("/dev/input/js1", O_RDONLY | O_NONBLOCK);
	xbox2 = open ("/dev/input/js2", O_RDONLY | O_NONBLOCK);
	xbox3 = open ("/dev/input/js3", O_RDONLY | O_NONBLOCK);
	if(xbox1<0){
		printf("Xbox Receiver Not Connected\n");
	}
	while(xbox1<0){
		xbox1 = open ("/dev/input/js1", O_RDONLY | O_NONBLOCK);
	}
	
	/*
	int axes=0, buttons=0;
  	char name[128];
	ioctl(fd, JSIOCGAXES, &axes);
  	ioctl(fd, JSIOCGBUTTONS, &buttons);
  	ioctl(fd, JSIOCGNAME(sizeof(name)), &name);
 	 printf("%s\n  %d Axes %d Buttons\n", name, axes, buttons);
	*/
}

void socketSetup(){
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	if(sockfd<0){
		error("ERROR opening socket\n");
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));
	portno =8080;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY; //could be IP of beaglebone
	serv_addr.sin_port = htons(portno);
	if(bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))<0){
		error("ERROR on binding.\n");
		exit(1);
	}
	printf("Waiting for Clients to connect\n");
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if(newsockfd<0){
		error("ERROR on accept\n");
	}
	else{
		//printf("%s succesfully joined\n",cli_addr.sa_data);
	}
}

void setup(){
	xboxSetup();
	socketSetup();
}

void MasterSend(int sock, unsigned int sbyte, unsigned int pfreq, int lspeed, unsigned int lbrake, int rspeed, unsigned int rbrake, int sv_0, int sv_1, int sv_2, int sv_3, int sv_4, int sv_5, unsigned int dev,int sens,int lowbat, unsigned int i2caddr,unsigned int i2cfreq, int data_request, int compass_calibrate){
	int n;
	
	__u8 send_data[30]={sbyte,pfreq,((lspeed>>8)&0xFF), (lspeed&0xFF), lbrake, ((rspeed>>8)&0xFF), (rspeed&0xFF), rbrake, ((sv_0>>8)&0xFF), (sv_0&0xFF), ((sv_1>>8)&0xFF), (sv_1&0xFF),((sv_2>>8)&0xFF), (sv_2&0xFF),((sv_3>>8)&0xFF), (sv_3&0xFF),((sv_4>>8)&0xFF), (sv_4&0xFF),((sv_5>>8)&0xFF), (sv_5&0xFF),dev,((sens>>8)&0xFF), (sens&0xFF),((lowbat>>8)&0xFF), (lowbat&0xFF),i2caddr,i2cfreq,data_request,compass_calibrate};
	
	n = write(sock,send_data,sizeof(send_data));
	if(n<=0) {
		error("ERROR reading from socket\n");
	}
	return;
}
void MasterReceive(int sock){
	int n;
	if(n = read(sock,receive,sizeof(receive))<=0){
		error("ERROR writing to socket\n");
		}
	/*
	for(int i =0;i<7;i++){
		printf("%2.4f ",receive[i]);
	}
	printf("\n\n");
	*/
  	return;
}

void print_ip(int ip){
	unsigned char bytes[4];
	bytes[0] = ip & 0xFF;
	bytes[1] = (ip >> 8) & 0xFF;
	bytes[2] = (ip >> 16) & 0xFF;
	bytes[3] = (ip >> 24) & 0xFF;	
	printf("%d.%d.%d.%d\n", bytes[0], bytes[1], bytes[2], bytes[3]);        
}

void *connection_handler(void *socket_desc){
	int fd = -1;	
	int lmspeed = 0;
	int rmspeed = 0;
	int data_request = 0;
	int compass_calibrate = 0;
	char device[];
	int location_string = {0};
	//Get the socket descriptor
	int sock = *(int*)socket_desc;
	//int read_size;
	//char *message , client_message[2000];
	printf("New connection! Sock = %i\n",sock);
	if(service_count==0){
		fd = xbox1;
		device = "Transmitter";
		//location = sensor1;
	}
	else if(service_count == 1){
		fd = xbox2;
		device = "Receiver 1";
		//location = transmitter;
	}
	else if(service_count == 2){
		fd = xbox3;
		device = "Receiver 2";
		//location = sensor2;
	}
	else printf("service_count error");
	pthread_mutex_lock(&lock);
	service_count++;
	pthread_mutex_unlock(&lock);


	while(1){

		if(read(fd,&xbox,sizeof(xbox))>=0){
			//printf("Reading controller 1\n");
			//turn right with right trigger
			if(xbox.type==2 && xbox.number==2){
				if(lmspeed>=0){
					lmspeed = 0.003815*(xbox.value)+125;
				}
				else if(lmspeed<0){
					lmspeed = -(0.003815*(xbox.value)+125);
				}
				else lmspeed = 0;
				rmspeed = -lmspeed;
			}
			//turn left with left trigger
			if(xbox.type==2 && xbox.number==5){
				if(rmspeed>=0){
					rmspeed = 0.003815*(xbox.value)+125;
				}
				else if(rmspeed<0){
					rmspeed = -(0.003815*(xbox.value)+125);
				}
				else rmspeed = 0;
				lmspeed = -rmspeed;
			}
			//move forward or backward with left analog stick
			if(xbox.type==2 && xbox.number==1){
				if(xbox.value<-10000){
					float b = -112.004;
					float m = -0.0112;
					lmspeed = m*xbox.value+b;
					rmspeed = lmspeed;
				}
				else if(xbox.value>10000){
					float b = -112.004;
					float m = 0.0112;
					lmspeed = -(m*xbox.value+b);
					rmspeed = lmspeed;
				}
				else{
					lmspeed = 0;
					rmspeed = 0;
				}
			}
			//retreive data with a button
			if(xbox.type==1 && xbox.number==0 && xbox.value ==1){
				printf("Retreiving Data\n");
				int gpslatint[31] = {0};
				int gpslatdec[31] = {0};
				int gpslongint[31] = {0};
				int gpslongdec[31] = {0};
				int headingint[31] = {0};
				int headingdec[31] = {0};
				int RSSint[31] = {0};
				int RSSdec[31] = {0};
				float gpslat = 0;
				float gpslong = 0;
				float heading = 0;
				float RSS = 0;
				data_request = 1;
				for(int i=0;i<31;i++){
					MasterSend(sock,STARTBYTE,pwmf,lmspeed,lmbrake,rmspeed,rmbrake,sv1[0],sv1[1],sv1[2],sv1[3],sv1[4],sv1[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq,data_request,compass_calibrate);
					sleep(0.5);
					MasterReceive(sock);
					gpslatint[i] = receive[0];
					gpslatdec[i] = receive[1];
					gpslongint[i] = receive[2];
					gpslongdec[i] = receive[3];
					headingint[i] = receive[11];
					headingdec[i] = receive[12];
					RSSint[i] = receive[14];
					RSSdec[i] = receive[15];
					//printf("RSS = %i.%i\n",RSSint[i],RSSdec[i]);
				}
				for(int i=1;i<31;i++){
					//printf("heading: %i.%i\n",receive[11],receive[12]);
					gpslat = gpslatint[i]+(float) gpslatdec[i]/10000000 + gpslat;
					gpslong = gpslongint[i]+(float) gpslongdec[i]/10000000 + gpslong;
					heading = headingint[i]+(float) headingdec[i]/100000 + heading;
					RSS = RSSint[i]+(float) RSSdec[i]/10000000 + RSS;
				}
				printf("%s Average lat: %3.6f, Average long: %3.6f, Average heading: %3.2f Average RSS: %3.10f Time: %i:%i:%i\n",device,gpslat/30,gpslong/30,heading/30,RSS/30,receive[4],receive[5],receive[6]);
				//printf("Data from Sensor %i Y: %i.%i X: %i.%i Time: %i:%i:%i Speed: %i.%i Heading: %i.%i Heading: %i.%i RSS = %i.%i\n\r",receive[13],receive[0], receive[1], receive[2],receive[3],receive[4],receive[5],receive[6],receive[7], receive[8], receive[9],receive[10],receive[11],receive[12], receive[14],receive[15]);
				data_request = 0;
			}
			//calibrate with b button
			if(xbox.type==1 && xbox.number==1 && xbox.value ==1){
				compass_calibrate = 1;
				MasterSend(sock,STARTBYTE,pwmf,lmspeed,lmbrake,rmspeed,rmbrake,sv1[0],sv1[1],sv1[2],sv1[3],sv1[4],sv1[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq,data_request,compass_calibrate);
				MasterReceive(sock);
				compass_calibrate = 0;
				
			}			
		}
		
		MasterSend(sock,STARTBYTE,pwmf,lmspeed,lmbrake,rmspeed,rmbrake,sv1[0],sv1[1],sv1[2],sv1[3],sv1[4],sv1[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq,data_request,compass_calibrate);
	
		MasterReceive(sock);
			
	}
	//shutdown(sock,SHUT_WR);
	//close(sock);
	printf("Closing socket\n");
	pthread_exit(0);
	free(sock);	
	fflush(stdout);
} 







void main(){
	xboxSetup();
	int socket_desc , client_sock , c;
	struct sockaddr_in server , client;
	int sockfd;
	struct sockaddr_in serv_addr;
	//struct hostent *server;
	int portHost = 30000;

	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	sockfd = socket(AF_INET, SOCK_STREAM,0);
	if (socket_desc == -1){
		printf("Could not create socket");
	}
	if (sockfd == -1){
		printf("Could not create socket");
	}
	
	puts("Socket created");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons( 8080 );



	/*
	bzero((char*) &serv_addr, sizeof(serv_addr));	
	serv_addr.sin_family = AF_INET;	
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,server->h_length);
	serv_addr.sin_port = htons(portHost);	
	if(a = connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr))>0){
		printf("Connected to MATLAB\n");
	}
	else printf("Not using MATLAB\n");
	*/


	//Bind
	if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0){
		//print the error message
		perror("bind failed. Error");
		exit(1);
	}
	puts("bind done");

	//Listen
	listen(socket_desc , 5);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");
	c = sizeof(struct sockaddr_in);
	pthread_t thread_id;

	while( (client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)) ){
		printf("Connection accepted to ");
		//print_ip(client.sin_addr);
		 
		if( pthread_create( &thread_id , NULL ,  connection_handler , (void*) &client_sock) < 0){
		    perror("could not create thread");
		    exit(1);
		}
		//Now join the thread , so that we dont terminate before the thread
		//pthread_join( thread_id , NULL);
		puts("Handler assigned");
	}

	if (client_sock < 0){
		perror("accept failed");
		exit(1);
	}	
}
