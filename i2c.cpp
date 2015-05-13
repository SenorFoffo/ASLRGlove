#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "mraa.hpp"
#include <iostream>

using namespace std;

#define I2C_ADDR 0x15
#define MAX_BUFFER_LENGTH 6

sig_atomic_t volatile isrunning = 1;
void sig_handler(int signum);

int main()
{
    signal(SIGINT, &sig_handler);

    uint8_t buff[MAX_BUFFER_LENGTH];
    
    mraa::I2c* i2c;
    i2c = new mraa::I2c(0);

    i2c->address(I2C_ADDR);
    
    int n = 0;
    while (isrunning)
    {
        int len = i2c->read(buff,1);
        if(len>0)
        	fprintf(stdout, "Received: %x\n", buff[0]);
        else
            fprintf(stdout, "no data\n");
    	
    	n++;
        if(n >= 10)
            isrunning = 0;

        sleep(1);
    }
    
    delete i2c;
    fprintf(stdout, "Done\n");
    return MRAA_SUCCESS;
}

void sig_handler(int signum)
{
    if(signum == SIGINT)
        isrunning = 0;
}
