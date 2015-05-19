#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <mraa/i2c.h>

#define I2C_ADDR 0x15

sig_atomic_t volatile isrunning = 1;
void sig_handler(int signum);

int main()
{
	signal(SIGINT, &sig_handler);
    uint8_t * buff;
    mraa_init();
    mraa_i2c_context i2c;
    i2c = mraa_i2c_init(1);

    int n = 0;
    mraa_i2c_address(i2c, I2C_ADDR);
    while (isrunning)
    {
        int len = mraa_i2c_read(i2c,buff,1);
        if(len>0)
        	fprintf(stdout, "Received: %x\n", buff[0]);
        else
            fprintf(stdout, "no data\n");
    	
    	n++;
        if(n >= 10)
            isrunning = 0;

        sleep(1);
    }
    mraa_i2c_stop (i2c);
    fprintf(stdout, "\nDone\n");

    return MRAA_SUCCESS;
}

void sig_handler(int signum)
{
    if(signum == SIGINT)
        isrunning = 0;
}
