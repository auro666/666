#ifndef _TRANSPORT2_H_
#define _TRANSPORT2_H_

#include <iostream>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h> 
#include <unistd.h>
#include <signal.h>
#include <time.h>

typedef struct {
	int width, height;
	time_t stamp;
	unsigned char data[500 * 500];
} PointCloud;

bool switch_pub=false;
int quit=0;
void signalHandler(int signum)
{	
		quit=1;
};

char NORMAL[]= "\033[0m";
char BLACK[]= "\033[0;30m";
char RED[]= "\033[0;31m";
char GREEN[]= "\033[0;32m";
char BROWN[]= "\033[0;33m";
char BLUE[]= "\033[0;34m";
char MAGENTA[]= "\033[0;35m";
char CYAN[]= "\033[0;36m";
char LIGHTGRAY[]= "\033[0;37m";
char YELLOW[]= "\033[0;33m";
char WHITE[]= "\033[37;01m";



template<typename T>
class Publisher{
private:
	/* Weather to publish the data or not */
	bool publish_on;
	/* The port number over which the data is to be transfered */
	int portno;
	/* A dummy variable used to catch errors*/
	int n;
	/* The socket Id of the current connection */
	int sockfd;
	/* The variable to define the protcol of transfer and also the port of transfer */
	struct sockaddr_in serv_addr;
	/* This will contain the address of the machine in network binary format */
	struct hostent *server;
	/* The pointer to the thread */
	pthread_t thread;
	/* A mutex lock to chnage data without memory voilation */
	pthread_mutex_t data_lock,on_lock;
	/* The wil contain the data that will be transmitted. */
	T data;
	/* @Brief - Function that is binded to the thread that keeps on           */
	/*          publishing data.                                              */
	void* publish_data(void *arg);
public:
	
	/* @Brief - Constructor to create a port for publishing data. This sets the*/
	/*         port address as local address and with the given port number.   */
	/* @params[in] - port : The port number where to publish data.             */
	/* @params[in] - size : The size of the data to be published.             */
	Publisher(int port);
	
	/* @Brief - Function to publish the data in the port                      */
	/*          This takes in any type of data and publishes it to the port.  */
	/* @params[in] - data : The pointer to data that is to be published       */
	int publish(T data_t);
	
	/* @Brief - Funciton to close all the ports and free all the allocated    */
	/*          space                                                         */
	void finish();
	/* @Brief - Funciton to return the port number that is currently          */
	/*          being used.                                                   */
	int port_ID();
};

template<typename T>
class Subscriber{
private:
	/* The pointer to the thread */
	pthread_t thread;
	/* A mutex lock to chnage data without memory voilation */
	pthread_mutex_t data_lock;
	/* The data recieved from the given port */
	T data;
	/* The socket Id of the current connection */
	int sockfd;
	/* The socket Id of connection being revieved */
	int newsockfd;
	/* The port number over which the data is to be transfered */
	int portno;
	int clilen;
	/* The variable to define the protcol of transfer of publisher and also the port of transfer. */
	struct sockaddr_in serv_addr;
	struct sockaddr_in cli_addr;
	/* A dummy variable used to catch errors*/
	int  n;
	/* The child process PID */
	pid_t pid;
//	void* thread_connect(void *arg);
public:	
	/* @Brief - Constructor to start subscribing from a port. This sets the    */
	/*         port address as local address and with the given port number.   */
	/* @params[in] - port : The port number from where to get data.            */
	typedef void (*data_callback)(T __DATA__);
	Subscriber(int port,data_callback user_function);
	data_callback _temp_f;
	
	
	/* @Brief - Funciton to close all the ports and free all the allocated    */
	/*          space                                                         */
	void finish();
	
	/* @Brief - Funtion to return the current data to the user                */
	/*                                                                        */
	/* @params[in] - data_t : A Variable of any type in which                 */
	/* the data is returned                                                   */
	//void getdata(void* data_t);
	//void* getdata();
	void* get_data_thread(void *arg){
		while (true)
		{
			usleep(10);
			newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*)&clilen);
			if (newsockfd < 0){ 
				perror("\033[0;31mERROR: \033[0m on accept");
			}
			read(newsockfd,&data,sizeof(T));
			_temp_f(data);
//			printf("%s DEBUG ( Line %d ) : Here! %s\n", YELLOW,__LINE__,NORMAL);
			close(newsockfd);
		}
	};
	
};




/*********************************************************************************************/

/* @Brief - Funciton to close all the ports and free all the allocated    */
/*          space                                                         */
template<typename T>
void Publisher<T>::finish(){
	close(sockfd);	
	pthread_cancel(thread);
	pthread_mutex_destroy(&data_lock);
};

/* @Brief - Function to create a port for publishing data. This sets the  */
/*         port address as local address and with the given port number.  */
/* @params[in] - port : The port number where to publish data.            */
/* @params[in] - size : The size of the data to be published.             */
template<typename T>
Publisher<T>::Publisher(int port){
//	signal(SIGINT, signalHandler);
	quit=0;
	publish_on=false;
	/* Initialize mutex lock */
	if (pthread_mutex_init(&data_lock, NULL) != 0){
		printf("\n mutex init failed\n");
		exit(1);
	}
	if (pthread_mutex_init(&on_lock, NULL) != 0){
		printf("\n mutex init failed\n");
		exit(1);
	}
	/* Save the port number */
	portno = port;
	/* Set the server address to local address */
	server = gethostbyname("127.0.0.1");
	if (server == NULL) {
		fprintf(stderr,"ERROR, no such host\n");
		exit(1);
	}
	/* Creating the sock address */
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);

	typedef  void* (Publisher<T>::*PublisherPtr)(void*);
	typedef  void* (*PthreadPtr)(void*);
	
	PublisherPtr   t = &Publisher::publish_data;
	PthreadPtr   p = *(PthreadPtr*)&t;
	n = pthread_create(&thread, NULL,p, this); //DO YOURSELF
	if (n != 0)
      printf("\nCan't create thread!");
}

/* @Brief - Function to publish the data in the port                      */
/*          This takes in any type of data and publishes it to the port.  */
/* @params[in] - data : The pointer to data that is to be published       */
template<typename T>
int Publisher<T>::publish(T data_t){
	pthread_mutex_lock(&on_lock);
		publish_on=true;
	pthread_mutex_unlock(&on_lock);
	pthread_mutex_lock(&data_lock);
		memcpy(&data,&data_t,sizeof(T));
	pthread_mutex_unlock(&data_lock);
}

/* @Brief - Function that is binded to the thread that keeps on           */
/*          publishing data.                                              */
template<typename T>
void* Publisher<T>::publish_data(void *arg){
	while(true){
		if(switch_pub==false){
			pthread_mutex_lock(&on_lock);
			switch_pub=publish_on;
			pthread_mutex_unlock(&on_lock);
			usleep(10);
			continue;
		}
		while((sockfd = socket(AF_INET, SOCK_STREAM, 0))<0){
			usleep(10);
			perror("ERROR opening socket");
		}
		//printf("%sDEBUG (Line %d)\n%s",YELLOW,__LINE__,NORMAL);
		int iSetOption=1;
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&iSetOption, sizeof(iSetOption));
		while( connect(sockfd,(const sockaddr*)&serv_addr,sizeof(serv_addr)) < 0){
			//printf("%sERROR on Connect!\n%s",RED,NORMAL);
         usleep(10);
		}
		/* Send message to the server */
		usleep(1000);
		pthread_mutex_lock(&data_lock);
    	n = write(sockfd,&data,sizeof(T));
    	pthread_mutex_unlock(&data_lock);
		usleep(10);
		close(sockfd);	
	}
}

/* @Brief - Funciton to return the port number that is currently          */
/*          being used.                                                   */
template<typename T>
int Publisher<T>::port_ID(){
	return portno;
}

//Subscriber::Subscriber(int port, int size){
template<typename T>
Subscriber<T>::Subscriber(int port,data_callback user_function){
	//size_data=size;
	//signal(SIGINT, signalHandler);
	portno=port;
	_temp_f=user_function;
	if (pthread_mutex_init(&data_lock, NULL) != 0){
		printf("\n mutex init failed\n");
		exit(1);
	}
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
	
	
	
	int iSetOption = 1;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) 
	{
		perror("ERROR opening socket");
		exit(0);
	}
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&iSetOption, sizeof(iSetOption));
	if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
	{
		perror("ERROR on binding");
		exit(0);
	}
	clilen = sizeof(cli_addr);
	
	listen(sockfd,5);
	typedef  void* (Subscriber<T>::*SubscriberPtr)(void*);
	typedef  void* (*PthreadPtr)(void*);
	
	SubscriberPtr   t = &Subscriber<T>::get_data_thread;
	PthreadPtr   p = *(PthreadPtr*)&t;
		n = pthread_create(&thread, NULL,p, this); //DO YOURSELF
}

template<typename T>
void Subscriber<T>::finish(){
	close(sockfd);	
	pthread_cancel(thread);
	pthread_mutex_destroy(&data_lock);

}
/*
void* Subscriber::getdata(){
	//printf("%sDEBUG (Line %d)\n%s",YELLOW,__LINE__,NORMAL);
	listen(sockfd,1);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*)&clilen);
	if (newsockfd < 0){ 
          perror("\033[0;31mERROR: \033[0m on accept");
	}
	read(newsockfd,data,size_data);
    close(newsockfd);
    return data;
}*/




/*****************************************************************************************/


#endif


