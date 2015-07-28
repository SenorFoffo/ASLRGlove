#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include<string.h>    //strlen
#include<string>  //string
#include<sstream>
#include<iostream>
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

using namespace std;

//TCP functions
bool tcp_conn(string address , int port);
bool tcp_send_data(string data);
struct sockaddr_in server;
//Globals
//static struct bno055_t *bno055;
int sock = -1; //for the tcp socket

int main(int argc, char** argv)
{

	//TCP Client variables

	string address = "";
	int port = 1000;


	if (argc > 1)
    {
    	address = argv[1];
    	if (argc > 2)
    		port = atoi(argv[2]);
  	}
  	else
  	{
  		printf("Error when IP input... Exiting...");
		return 1;
  	}
    bool connect = false;
	connect = tcp_conn(address, port);
	if (connect == false)
	{
		printf("Error when connecting... Exiting...");
		return 1;
	}

	char c;
	char buffer[1024];
	while (1)
	{

		scanf(" %c", &c);

		stringstream ss;
		ss.str("");
		ss << c;

		tcp_send_data(ss.str());

	}

	
	return 0;
}


bool tcp_conn(string address , int port)
{
	//create socket if it is not already created
	if(sock == -1)
	{
		//Create socket
		sock = socket(AF_INET , SOCK_STREAM , 0);
		if (sock == -1)
		{
			perror("Could not create socket");
		}

		printf("Socket created\n");
	}
	else    {      }

	//setup address structure
	if(inet_addr(address.c_str()) == -1)
	{
		struct hostent *he;
		struct in_addr **addr_list;

		//resolve the hostname, its not an ip address
		if ( (he = gethostbyname( address.c_str() ) ) == NULL)
		{
			//gethostbyname failed
			herror("gethostbyname");
			printf("Failed to resolve hostname\n");

			return false;
		}

		//Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
		addr_list = (struct in_addr **) he->h_addr_list;

		for(int i = 0; addr_list[i] != NULL; i++)
		{
			//strcpy(ip , inet_ntoa(*addr_list[i]) );
			server.sin_addr = *addr_list[i];

			//printf("%s resolved to %x\n", address, inet_ntoa(*addr_list[i]));
			printf("Connection established\n");

			break;
		}
	}

	//plain ip address
	else
	{
		server.sin_addr.s_addr = inet_addr( address.c_str() );
	}

	server.sin_family = AF_INET;
	server.sin_port = htons( port );

	//Connect to remote server
	if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		perror("connect failed. Error");
		return 1;
	}

	printf("Connected\n");
	return true;
}

bool tcp_send_data(string data)
{
	//Send some data
	int msg = send(sock , data.c_str() , strlen( data.c_str() ) , 0);
	if( msg < 0)
	{
		printf("Send failed\n");
		perror("Send failed : ");
		return false;
	}

	return true;
}

