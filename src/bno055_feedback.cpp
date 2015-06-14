Skip to content
This repository  
Pull requests
Issues
Gist
 @HassanAmr
 Unwatch 4
  Star 0
  Fork 0
SenorFoffo/ASLRGlove
 branch: master ASLRGlove/src/bno055_feedback.cpp
@andrelutzaandrelutza 4 hours ago Update bno055_feedback.cpp
2 contributors @HassanAmr @andrelutza
RawBlameHistory     1098 lines (927 sloc)  40.17 kB
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <thread>

#include<string.h>    //strlen
#include<string>  //string
#include <sstream>
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

#include "mraa.hpp"
#include "math.h"

using namespace std;

//Definitions

/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long long int u64;/**< used for unsigned 64bit */

/********************************************************/
/**\name	I2C ADDRESS DEFINITION FOR BNO055           */
/********************************************************/
/* bno055 I2C Address */
#define BNO055_I2C_ADDR1                0x28
#define BNO055_I2C_ADDR2                0x29

/***************************************************/
/**\name	CONSTANT DEFINITIONS                   */
/***************************************************/
#define         BNO055_ZERO_U8X           ((u8)0)
#define         BNO055_ONE_U8X           ((u8)1)
#define         BNO055_TWO_U8X			  ((u8)2)
#define         BNO055_FOUR_U8X           ((u8)4)
#define         BNO055_FIVE_U8X           ((u8)5)
#define         BNO055_SIX_U8X            ((u8)6)
#define         BNO055_SEVEN_U8X          ((u8)7)
#define         BNO055_ELEVEN_U8X         ((u8)11)
#define         BNO055_SIXTEEN_U8X        ((u8)16)
#define			BNO055_EIGHT_U8X		  ((u8)8)
#define			BNO055_TWENTY_U8X         ((u8)20)
#define			BNO055_EIGHTEEN_U8X       ((u8)18)


/*< This refers BNO055 return type as s8 */
#define BNO055_RETURN_FUNCTION_TYPE        s8


/* Power mode*/
#define POWER_MODE_NORMAL	0X00
#define POWER_MODE_LOWPOWER	0X01
#define POWER_MODE_SUSPEND	0X02

/* Operation mode settings*/
#define OPERATION_MODE_CONFIG			0x00
#define OPERATION_MODE_ACCONLY			0x01
#define OPERATION_MODE_MAGONLY			0x02
#define OPERATION_MODE_GYRONLY			0x03
#define OPERATION_MODE_ACCMAG			0x04
#define OPERATION_MODE_ACCGYRO			0x05
#define OPERATION_MODE_MAGGYRO			0x06
#define OPERATION_MODE_AMG				0x07
#define OPERATION_MODE_IMUPLUS			0x08
#define OPERATION_MODE_COMPASS			0x09
#define OPERATION_MODE_M4G				0x0A
#define OPERATION_MODE_NDOF_FMC_OFF		0x0B
#define OPERATION_MODE_NDOF				0x0C

/*  BNO055 API error codes */
#define E_NULL_PTR                  ((s8)-127)
#define E_BNO055_OUT_OF_RANGE       ((s8)-2)
#define	SUCCESS						((u8)0)
#define	ERROR                       ((s8)-1)

/* Page ID */
#define PAGE_ZERO		0x00
#define PAGE_ONE		0x01

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR				0x3D
#define BNO055_PWR_MODE_ADDR				0x3E

/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR				    0X07
/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 0x00
#define BNO055_ACCEL_REV_ID_ADDR			0x01
#define BNO055_MAG_REV_ID_ADDR              0x02
#define BNO055_GYRO_REV_ID_ADDR             0x03
#define BNO055_SW_REV_ID_LSB_ADDR			0x04
#define BNO055_SW_REV_ID_MSB_ADDR			0x05
#define BNO055_BL_REV_ID_ADDR				0x06

/* Chip ID */
#define BNO055_CHIP_ID__POS             0
#define BNO055_CHIP_ID__MSK             0xFF
#define BNO055_CHIP_ID__LEN             8
#define BNO055_CHIP_ID__REG             BNO055_CHIP_ID_ADDR

/*Operation Mode data register*/
#define BNO055_OPERATION_MODE__POS			0
#define BNO055_OPERATION_MODE__MSK			0x0F
#define BNO055_OPERATION_MODE__LEN			4
#define BNO055_OPERATION_MODE__REG			BNO055_OPR_MODE_ADDR

/* Power Mode register*/
#define BNO055_POWER_MODE__POS             0
#define BNO055_POWER_MODE__MSK             0x03
#define BNO055_POWER_MODE__LEN             2
#define BNO055_POWER_MODE__REG             BNO055_PWR_MODE_ADDR

/*Page id*/
#define BNO055_PAGE_ID__POS             0
#define BNO055_PAGE_ID__MSK             0xFF
#define BNO055_PAGE_ID__LEN             8
#define BNO055_PAGE_ID__REG             BNO055_PAGE_ID_ADDR

/* Mag revision id*/
#define BNO055_MAG_REV_ID__POS             0
#define BNO055_MAG_REV_ID__MSK             0xFF
#define BNO055_MAG_REV_ID__LEN             8
#define BNO055_MAG_REV_ID__REG             BNO055_MAG_REV_ID_ADDR

/*Software revision id LSB*/
#define BNO055_SW_REV_ID_LSB__POS             0
#define BNO055_SW_REV_ID_LSB__MSK             0xFF
#define BNO055_SW_REV_ID_LSB__LEN             8
#define BNO055_SW_REV_ID_LSB__REG             BNO055_SW_REV_ID_LSB_ADDR

/* BOOTLODER revision id*/
#define BNO055_BL_REV_ID__POS             0
#define BNO055_BL_REV_ID__MSK             0xFF
#define BNO055_BL_REV_ID__LEN             8
#define BNO055_BL_REV_ID__REG             BNO055_BL_REV_ID_ADDR


#define INDEX_ZERO		0
#define INDEX_ONE		1
#define INDEX_TWO		2
#define INDEX_THREE		3
#define INDEX_FOUR		4
#define INDEX_FIVE		5
#define INDEX_SIX		6
#define INDEX_SEVEN		7
#define INDEX_EIGHT		8

#define ARRAY_SIZE_TWO		2
#define ARRAY_SIZE_THREE	3
#define ARRAY_SIZE_SIX		6
#define ARRAY_SIZE_FIVE		5
#define ARRAY_SIZE_EIGHT	8

#define         BNO055_SHIFT_8_POSITION	   ((u8)8)

/*Mag division factor*/
#define MAG_DIV_UT	16.0

#define	I2C_BUFFER_LEN 8

#define	BNO055_SIX_HUNDRES_U8X	600

/**\name GET AND SET BITSLICE FUNCTIONS    */
/*************************************************/
#define BNO055_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)


#define BNO055_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR			0X0E
#define BNO055_MAG_DATA_X_MSB_ADDR			0X0F
#define BNO055_MAG_DATA_Y_LSB_ADDR			0X10
#define BNO055_MAG_DATA_Y_MSB_ADDR			0X11
#define BNO055_MAG_DATA_Z_LSB_ADDR			0X12
#define BNO055_MAG_DATA_Z_MSB_ADDR			0X13

/*Mag status register*/
#define BNO055_CALIB_STAT_ADDR				0X35

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR				0X5B
#define MAG_OFFSET_X_MSB_ADDR				0X5C
#define MAG_OFFSET_Y_LSB_ADDR				0X5D
#define MAG_OFFSET_Y_MSB_ADDR				0X5E
#define MAG_OFFSET_Z_LSB_ADDR				0X5F
#define MAG_OFFSET_Z_MSB_ADDR				0X60

/* Mag data X-LSB register*/
#define BNO055_MAG_DATA_X_LSB_VALUEX__POS             0
#define BNO055_MAG_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_MAG_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_MAG_DATA_X_LSB_VALUEX__REG             \
BNO055_MAG_DATA_X_LSB_ADDR

/* Mag data X-MSB register*/
#define BNO055_MAG_DATA_X_MSB_VALUEX__POS             0
#define BNO055_MAG_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_MAG_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_MAG_DATA_X_MSB_VALUEX__REG             BNO055_MAG_DATA_X_MSB_ADDR

/* Mag data Y-LSB register*/
#define BNO055_MAG_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_MAG_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_MAG_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_MAG_DATA_Y_LSB_VALUEY__REG             BNO055_MAG_DATA_Y_LSB_ADDR

/* Mag data Y-MSB register*/
#define BNO055_MAG_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_MAG_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_MAG_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_MAG_DATA_Y_MSB_VALUEY__REG             BNO055_MAG_DATA_Y_MSB_ADDR

/* Mag data Z-LSB register*/
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__REG             BNO055_MAG_DATA_Z_LSB_ADDR

/* Mag data Z-MSB register*/
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__REG             BNO055_MAG_DATA_Z_MSB_ADDR

/*Mag_Calib status register*/
#define BNO055_MAG_CALIB_STAT__POS             0
#define BNO055_MAG_CALIB_STAT__MSK             0X03
#define BNO055_MAG_CALIB_STAT__LEN             2
#define BNO055_MAG_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

//*Mag Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB__POS		0
#define BNO055_MAG_OFFSET_X_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_X_LSB__LEN		8
#define BNO055_MAG_OFFSET_X_LSB__REG		MAG_OFFSET_X_LSB_ADDR

#define BNO055_MAG_OFFSET_X_MSB__POS		0
#define BNO055_MAG_OFFSET_X_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_X_MSB__LEN		8
#define BNO055_MAG_OFFSET_X_MSB__REG		MAG_OFFSET_X_MSB_ADDR

#define BNO055_MAG_OFFSET_Y_LSB__POS		0
#define BNO055_MAG_OFFSET_Y_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Y_LSB__LEN		8
#define BNO055_MAG_OFFSET_Y_LSB__REG		MAG_OFFSET_Y_LSB_ADDR

#define BNO055_MAG_OFFSET_Y_MSB__POS		0
#define BNO055_MAG_OFFSET_Y_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Y_MSB__LEN		8
#define BNO055_MAG_OFFSET_Y_MSB__REG		MAG_OFFSET_Y_MSB_ADDR

#define BNO055_MAG_OFFSET_Z_LSB__POS		0
#define BNO055_MAG_OFFSET_Z_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Z_LSB__LEN		8
#define BNO055_MAG_OFFSET_Z_LSB__REG		MAG_OFFSET_Z_LSB_ADDR

#define BNO055_MAG_OFFSET_Z_MSB__POS		0
#define BNO055_MAG_OFFSET_Z_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Z_MSB__LEN		8
#define BNO055_MAG_OFFSET_Z_MSB__REG		MAG_OFFSET_Z_MSB_ADDR

/* Radius registers*/
#define	MAG_RADIUS_LSB_ADDR					0X69
#define	MAG_RADIUS_MSB_ADDR					0X6A

/* Radius register definition*/
#define BNO055_MAG_RADIUS_LSB__POS		0
#define BNO055_MAG_RADIUS_LSB__MSK		0XFF
#define BNO055_MAG_RADIUS_LSB__LEN		8
#define BNO055_MAG_RADIUS_LSB__REG		MAG_RADIUS_LSB_ADDR

#define BNO055_MAG_RADIUS_MSB__POS		0
#define BNO055_MAG_RADIUS_MSB__MSK		0XFF
#define BNO055_MAG_RADIUS_MSB__LEN		8
#define BNO055_MAG_RADIUS_MSB__REG		MAG_RADIUS_MSB_ADDR

//end of Definitions

//Funcions
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);


BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(u8 v_power_mode_u8,struct bno055_t *bno055);
BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(u8 v_operation_mode_u8,struct bno055_t *bno055);
BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(u8 *v_operation_mode_u8,struct bno055_t *bno055);
BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(u8 v_page_id_u8,struct bno055_t *bno055);
BNO055_RETURN_FUNCTION_TYPE bno055_write_register(u8 v_addr_u8, u8 *p_data_u8, u8 v_len_u8,struct bno055_t *bno055);
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_calib_stat(u8 *v_mag_calib_u8, struct bno055_t *bno055);
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset(struct bno055_mag_offset_t  *mag_offset, struct bno055_t *bno055);
//End of functions

void LoopBreaker();
void sig_handler(int signo);

//TCP functions
bool tcp_conn(int port);
bool tcp_send_data(string data);
struct sockaddr_in serv_addr, cli_addr;

struct bno055_t {
    u8 chip_id;/**< chip_id of bno055 */
    u16 sw_rev_id;/**< software revision id of bno055 */
    u8 page_id;/**< page_id of bno055 */
    u8 accel_rev_id;/**< accel revision id of bno055 */
    u8 mag_rev_id;/**< mag revision id of bno055 */
    u8 gyro_rev_id;/**< gyro revision id of bno055 */
    u8 bl_rev_id;/**< boot loader revision id of bno055 */
    u8 dev_addr;/**< i2c device address of bno055 */
    //BNO055_WR_FUNC_PTR;/**< bus write function pointer */
    //BNO055_RD_FUNC_PTR;/**<bus read function pointer */
    //void (*delay_msec)(BNO055_MDELAY_DATA_TYPE);/**< delay function pointer */
};

struct bno055_mag_offset_t {
    s16 x;/**< Mag offset x data */
    s16 y;/**< Mag offset y data */
    s16 z;/**< Mag offset z data */
    s16 r;/**< Mag radius x data */
};

//Globals
//static struct bno055_t *bno055;
bno055_t sensor1, sensor2;
int running = 0;
bool breakLoop = false;
uint8_t rx_tx_buf[ARRAY_SIZE_SIX];
mraa::I2c* i2c;
std::thread br (LoopBreaker);
//for the tcp socket
int sockfd = -1;
int newsockfd = -1;

int main(int argc, char** argv)
{

    s32 comres1 = ERROR;
    s32 comres2 = ERROR;

    u8 power_mode = BNO055_ZERO_U8X;

    bno055_mag_offset_t offsetData1;
    bno055_mag_offset_t offsetData2;

    /*********read raw mag data***********/
    /* variable used to read the mag x data from the first sensor*/
    s16 mag_datax1  = BNO055_ZERO_U8X;
    /* variable used to read the mag y data from the first sensor*/
    s16 mag_datay1  = BNO055_ZERO_U8X;
    /* variable used to read the mag z data from the first sensor*/
    s16 mag_dataz1  = BNO055_ZERO_U8X;
    /* variable used to read the mag x data from the second sensor*/
    s16 mag_datax2  = BNO055_ZERO_U8X;
    /* variable used to read the mag y data from the second sensor*/
    s16 mag_datay2  = BNO055_ZERO_U8X;
    /* variable used to read the mag z data from the second sensor*/
    s16 mag_dataz2  = BNO055_ZERO_U8X;


	//TCP Client variables

    string address = "";
    int port = 1000;


    if (argc > 1)
    {
    	port = atoi(argv[1]);
    		
  	}
  	else
  	{
  		port = 1000;
  	}
    bool connect = false;*/
    connect = tcp_conn(port);
    if (connect == false)
    	breakLoop=true;

    signal(SIGINT, sig_handler);

    unsigned char mag_calib_status1 = 0;
    unsigned char mag_calib_status2 = 0;


        /*-----------------------------------------------------------------------*
     ************************* START INITIALIZATION ***********************
     *-------------------------------------------------------------------------*/


    i2c = new mraa::I2c(0);

    //bno055.bus_write = BNO055_I2C_bus_write;
    //bno055.bus_read = BNO055_I2C_bus_read;
    //bno055.delay_msec = BNO055_delay_msek;
    sensor1.dev_addr = BNO055_I2C_ADDR1;
    sensor2.dev_addr = BNO055_I2C_ADDR2;


    /*printf("%x\n\n",sensor1.dev_addr);
    printf("%x\n\n",sensor2.dev_addr);*/

    comres1 = bno055_init(&sensor1);
    comres2 = bno055_init(&sensor2);

    power_mode = POWER_MODE_NORMAL; /* set the power mode as NORMAL*/
    comres1 += bno055_set_power_mode(power_mode,&sensor1);
    comres1 += bno055_set_operation_mode(OPERATION_MODE_MAGONLY,&sensor1);

    comres2 += bno055_set_power_mode(power_mode,&sensor2);
    comres2 += bno055_set_operation_mode(OPERATION_MODE_MAGONLY,&sensor2);

    /*---------------------------------------------------------------------*
     ************************* END INITIALIZATION **********************
     *---------------------------------------------------------------------*/
    //printf("xOffset1: %x, yOffset1: %x, zOffset1: %x\n",offsetData1.x,offsetData1.y,offsetData1.z);
    //printf("xOffset2: %x, yOffset2: %x, zOffset2: %x\n\n",offsetData2.x,offsetData2.y,offsetData2.z);


    //bno055_read_mag_offset(&offsetData1, &sensor1);
    //bno055_read_mag_offset(&offsetData1, &sensor2);

    //printf("xOffset1: %x, yOffset1: %x, zOffset1: %x\n",offsetData1.x,offsetData1.y,offsetData1.z);
    //printf("xOffset2: %x, yOffset2: %x, zOffset2: %x\n\n",offsetData2.x,offsetData2.y,offsetData2.z);

    double x1,y1,z1,x2,y2,z2;
    double x1_offset, x2_offset, y1_offset, y2_offset, z1_offset, z2_offset;
    double x1_new, x2_new, y1_new, y2_new, z1_new, z2_new;


	//read data first once, then have it as an offset (-ve) for the next readings
	BNO055_I2C_bus_read(sensor1.dev_addr,BNO055_MAG_DATA_X_LSB_VALUEX__REG,rx_tx_buf, BNO055_SIX_U8X);
        /* Data X*/
    rx_tx_buf[INDEX_ZERO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ZERO],BNO055_MAG_DATA_X_LSB_VALUEX);
    rx_tx_buf[INDEX_ONE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ONE], BNO055_MAG_DATA_X_MSB_VALUEX);
    mag_datax1 = ((((s32)((s8)rx_tx_buf[INDEX_ONE])) << BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_ZERO]));
    x1 = (double)(mag_datax1/MAG_DIV_UT);
    /* Data Y*/
    rx_tx_buf[INDEX_TWO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_TWO],BNO055_MAG_DATA_Y_LSB_VALUEY);
    rx_tx_buf[INDEX_THREE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_THREE], BNO055_MAG_DATA_Y_MSB_VALUEY);
    mag_datay1 = ((((s32)((s8)rx_tx_buf[INDEX_THREE])) <<BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_TWO]));
    y1 = (double)(mag_datay1/MAG_DIV_UT);
    /* Data Z*/
    rx_tx_buf[INDEX_FOUR] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FOUR],BNO055_MAG_DATA_Z_LSB_VALUEZ);
    rx_tx_buf[INDEX_FIVE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FIVE],BNO055_MAG_DATA_Z_MSB_VALUEZ);
    mag_dataz1 = ((((s32)((s8)rx_tx_buf[INDEX_FIVE])) << BNO055_SHIFT_8_POSITION)| (rx_tx_buf[INDEX_FOUR]));
    z1 = (double)(mag_dataz1/MAG_DIV_UT);

    /*Read the six byte value of mag xyz from second sesnor*/
    BNO055_I2C_bus_read(sensor2.dev_addr,BNO055_MAG_DATA_X_LSB_VALUEX__REG,rx_tx_buf, BNO055_SIX_U8X);
    /* Data X*/
    rx_tx_buf[INDEX_ZERO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ZERO],BNO055_MAG_DATA_X_LSB_VALUEX);
    rx_tx_buf[INDEX_ONE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ONE], BNO055_MAG_DATA_X_MSB_VALUEX);
    mag_datax2 = ((((s32)((s8)rx_tx_buf[INDEX_ONE])) << BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_ZERO]));
    x2 = (double)(mag_datax2/MAG_DIV_UT);
    /* Data Y*/
    rx_tx_buf[INDEX_TWO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_TWO],BNO055_MAG_DATA_Y_LSB_VALUEY);
    rx_tx_buf[INDEX_THREE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_THREE], BNO055_MAG_DATA_Y_MSB_VALUEY);
    mag_datay2 = ((((s32)((s8)rx_tx_buf[INDEX_THREE])) <<BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_TWO]));
    y2 = (double)(mag_datay2/MAG_DIV_UT);
    /* Data Z*/
    rx_tx_buf[INDEX_FOUR] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FOUR],BNO055_MAG_DATA_Z_LSB_VALUEZ);
    rx_tx_buf[INDEX_FIVE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FIVE],BNO055_MAG_DATA_Z_MSB_VALUEZ);
    mag_dataz2 = ((((s32)((s8)rx_tx_buf[INDEX_FIVE])) << BNO055_SHIFT_8_POSITION)| (rx_tx_buf[INDEX_FOUR]));
    z2 = (double)(mag_dataz2/MAG_DIV_UT);

	x1_offset = x1;
	y1_offset = y1;
	z1_offset = z1;
	x2_offset = x2;
	y2_offset = y2;
	z2_offset = z2;


	int seq_counter = 0;
	int seq_num = 1;

	FILE * pFile;

	pFile = fopen ("data.txt","w");

	//string feedback_data = "";
	stringstream feedback_data;
	printf("Seq %d:\n", seq_num);
	fprintf (pFile, "Seq %d:\n", seq_num);
	int feedbackcounter = 0;//please remove when resolved
    while (1)
    {
        if(running == -1)
        {
            printf("Something went on the board wrong\n");
            break;
        }

        //bno055_get_mag_calib_stat(&mag_calib_status1, &sensor1);

        //bno055_get_mag_calib_stat(&mag_calib_status2, &sensor2);

        //printf("%x\t%x\n", mag_calib_status1, mag_calib_status2);

        /*Read the six byte value of mag xyz from first sensor*/
        BNO055_I2C_bus_read(sensor1.dev_addr,BNO055_MAG_DATA_X_LSB_VALUEX__REG,rx_tx_buf, BNO055_SIX_U8X);
        /* Data X*/
        rx_tx_buf[INDEX_ZERO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ZERO],BNO055_MAG_DATA_X_LSB_VALUEX);
        rx_tx_buf[INDEX_ONE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ONE], BNO055_MAG_DATA_X_MSB_VALUEX);
        mag_datax1 = ((((s32)((s8)rx_tx_buf[INDEX_ONE])) << BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_ZERO]));
        x1 = (double)(mag_datax1/MAG_DIV_UT);
        /* Data Y*/
        rx_tx_buf[INDEX_TWO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_TWO],BNO055_MAG_DATA_Y_LSB_VALUEY);
        rx_tx_buf[INDEX_THREE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_THREE], BNO055_MAG_DATA_Y_MSB_VALUEY);
        mag_datay1 = ((((s32)((s8)rx_tx_buf[INDEX_THREE])) <<BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_TWO]));
        y1 = (double)(mag_datay1/MAG_DIV_UT);
        /* Data Z*/
        rx_tx_buf[INDEX_FOUR] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FOUR],BNO055_MAG_DATA_Z_LSB_VALUEZ);
        rx_tx_buf[INDEX_FIVE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FIVE],BNO055_MAG_DATA_Z_MSB_VALUEZ);
        mag_dataz1 = ((((s32)((s8)rx_tx_buf[INDEX_FIVE])) << BNO055_SHIFT_8_POSITION)| (rx_tx_buf[INDEX_FOUR]));
        z1 = (double)(mag_dataz1/MAG_DIV_UT);

        /*Read the six byte value of mag xyz from second sesnor*/
        BNO055_I2C_bus_read(sensor2.dev_addr,BNO055_MAG_DATA_X_LSB_VALUEX__REG,rx_tx_buf, BNO055_SIX_U8X);
        /* Data X*/
        rx_tx_buf[INDEX_ZERO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ZERO],BNO055_MAG_DATA_X_LSB_VALUEX);
        rx_tx_buf[INDEX_ONE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_ONE], BNO055_MAG_DATA_X_MSB_VALUEX);
        mag_datax2 = ((((s32)((s8)rx_tx_buf[INDEX_ONE])) << BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_ZERO]));
        x2 = (double)(mag_datax2/MAG_DIV_UT);
        /* Data Y*/
        rx_tx_buf[INDEX_TWO] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_TWO],BNO055_MAG_DATA_Y_LSB_VALUEY);
        rx_tx_buf[INDEX_THREE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_THREE], BNO055_MAG_DATA_Y_MSB_VALUEY);
        mag_datay2 = ((((s32)((s8)rx_tx_buf[INDEX_THREE])) <<BNO055_SHIFT_8_POSITION) |(rx_tx_buf[INDEX_TWO]));
        y2 = (double)(mag_datay2/MAG_DIV_UT);
        /* Data Z*/
        rx_tx_buf[INDEX_FOUR] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FOUR],BNO055_MAG_DATA_Z_LSB_VALUEZ);
        rx_tx_buf[INDEX_FIVE] = BNO055_GET_BITSLICE(rx_tx_buf[INDEX_FIVE],BNO055_MAG_DATA_Z_MSB_VALUEZ);
        mag_dataz2 = ((((s32)((s8)rx_tx_buf[INDEX_FIVE])) << BNO055_SHIFT_8_POSITION)| (rx_tx_buf[INDEX_FOUR]));
        z2 = (double)(mag_dataz2/MAG_DIV_UT);


		x1_new = x1 - x1_offset;
		y1_new = y1 - y1_offset;
		z1_new = z1 - z1_offset;
		x2_new = x2 - x2_offset;
		y2_new = y2 - y2_offset;
		z2_new = z2 - z2_offset;



		//feedback_data = "";
		feedback_data.str() = "";
		/*if (feedbackcounter == 1000)
		{*/
			feedback_data << x1_new << " " << y1_new << " " << z1_new  << " " << x2_new << " " << y2_new << " " << z2_new << "\n";
		/*	feedbackcounter = 0;
		}
		feedbackcounter++;*/
		//tcp_send_data(feedback_data.str());
			double dummy = 124;
		printf("%f %f %f %f %f %f %f\n",dummy,x1_new, y1_new, z1_new, x2_new, y2_new, z2_new);
     	//printf("%f %f %f\n",x1_new, y1_new, z1_new);
       	//printf("%f %f %f\n\n",x2_new, y2_new, z2_new);

        fprintf (pFile, "%f\t%f\t%f\t\n",x1_new, y1_new, z1_new);
        fprintf (pFile, "%f\t%f\t%f\t\n\n",x2_new, y2_new, z2_new);

        seq_counter++;
        if (seq_counter == 1000)
        {
        	seq_num++;
        	seq_counter = 0;
        	printf("Seq %d:\n", seq_num);
        	fprintf (pFile, "Seq %d:\n", seq_num);
        }
        if (breakLoop)
        {
        	printf("Exit command received\n");
        	break;
        }
    }

    fclose (pFile);

    /*-----------------------------------------------------------------------*
     ************************* START DE-INITIALIZATION ***********************
     *-------------------------------------------------------------------------*/
    /*	For de - initializing the BNO sensor it is required to the operation mode
     of the sensor as SUSPEND
     Suspend mode can set from the register
     Page - page0
     register - 0x3E
     bit positions - 0 and 1*/
    power_mode = POWER_MODE_SUSPEND; /* set the power mode as SUSPEND*/
    comres1 += bno055_set_power_mode(power_mode,&sensor1);
    comres2 += bno055_set_power_mode(power_mode,&sensor2);

    delete i2c;

    /*---------------------------------------------------------------------*
     ************************* END DE-INITIALIZATION **********************
     *---------------------------------------------------------------------*/

    return MRAA_SUCCESS;
    return 0;
}


void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing nicely\n");
        running = -1;
    }
}

void LoopBreaker()
{
	char c;
	while (1)
	{
		scanf("%c", &c);
		if (c == 's')
		{
			breakLoop = true;
			br.detach();
			break;
		}
	}
}


BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055)
{
    /* Variable used to return value of
     communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = BNO055_ZERO_U8X;
    u8 v_page_zero_u8 = PAGE_ZERO;
    /* Array holding the Software revision id
     */
    u8 a_SW_ID_u8[ARRAY_SIZE_TWO] = {

        BNO055_ZERO_U8X, BNO055_ZERO_U8X};
    /* Write the default page as zero*/
    com_rslt = BNO055_I2C_bus_write(bno055->dev_addr,BNO055_PAGE_ID__REG, &v_page_zero_u8, BNO055_ONE_U8X);
    /* Read the chip id of the sensor from page
     zero 0x00 register*/
    com_rslt += BNO055_I2C_bus_read(bno055->dev_addr,BNO055_CHIP_ID__REG, &v_data_u8, BNO055_ONE_U8X);
    bno055->chip_id = v_data_u8;
    /* Read the accel revision id from page
     zero 0x01 register*/
    //com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,BNO055_ACCEL_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
    //p_bno055->accel_rev_id = v_data_u8;
    /* Read the mag revision id from page
     zero 0x02 register*/
    com_rslt += BNO055_I2C_bus_read(bno055->dev_addr, BNO055_MAG_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
    bno055->mag_rev_id = v_data_u8;
    /* Read the gyro revision id from page
     zero 0x02 register*/
    //com_rslt += p_bno055->BNO055_BUS_READ_FUNC
    //(p_bno055->dev_addr,
    // BNO055_GYRO_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
    //p_bno055->gyro_rev_id = v_data_u8;
    /* Read the boot loader revision from page
     zero 0x06 register*/
    com_rslt += BNO055_I2C_bus_read(bno055->dev_addr,BNO055_BL_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
    bno055->bl_rev_id = v_data_u8;
    /* Read the software revision id from page
     zero 0x04 and 0x05 register( 2 bytes of data)*/
    com_rslt += BNO055_I2C_bus_read(bno055->dev_addr, BNO055_SW_REV_ID_LSB__REG, a_SW_ID_u8, BNO055_TWO_U8X);
    a_SW_ID_u8[INDEX_ZERO] = BNO055_GET_BITSLICE(a_SW_ID_u8[INDEX_ZERO],BNO055_SW_REV_ID_LSB);
    bno055->sw_rev_id = (u16)((((u32)((u8)a_SW_ID_u8[INDEX_ONE])) << BNO055_SHIFT_8_POSITION) | (a_SW_ID_u8[INDEX_ZERO]));
    /* Read the page id from the register 0x07*/
    com_rslt += BNO055_I2C_bus_read(bno055->dev_addr,BNO055_PAGE_ID__REG, &v_data_u8, BNO055_ONE_U8X);
    bno055->page_id = v_data_u8;

    return com_rslt;
}


BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(u8 v_power_mode_u8, struct bno055_t *bno055)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8r = BNO055_ZERO_U8X;
    u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
    s8 v_stat_s8 = ERROR;
    /* Check the struct p_bno055 is empty */

    /* The write operation effective only if the operation
     mode is in config mode, this part of code is checking the
     current operation mode and set the config mode */
    v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8, bno055);
    if (v_stat_s8 == SUCCESS)
    {
        if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
            v_stat_s8 += bno055_set_operation_mode(OPERATION_MODE_CONFIG, bno055);
        if (v_stat_s8 == SUCCESS)
        {
            /* Write the value of power mode */
            com_rslt = BNO055_I2C_bus_read(bno055->dev_addr,BNO055_POWER_MODE__REG, &v_data_u8r, BNO055_ONE_U8X);
            if (com_rslt == SUCCESS) {
                v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,BNO055_POWER_MODE, v_power_mode_u8);
                com_rslt += BNO055_I2C_bus_write(bno055->dev_addr,BNO055_POWER_MODE__REG,&v_data_u8r, BNO055_ONE_U8X);
            }
        }
        else
        {
            com_rslt = ERROR;
        }
    }
    else
    {
        com_rslt = ERROR;
    }
    if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
    /* set the operation mode
     of previous operation mode*/
    com_rslt += bno055_set_operation_mode(v_prev_opmode_u8, bno055);
    return com_rslt;
}


BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(u8 v_operation_mode_u8, struct bno055_t *bno055)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8r = BNO055_ZERO_U8X;
    u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
    s8 v_stat_s8 = ERROR;
    /* Check the struct p_bno055 is empty */

    /* The write operation effective only if the operation
     mode is in config mode, this part of code is checking the
     current operation mode and set the config mode */
    v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8,bno055);
    if (v_stat_s8 == SUCCESS)
    {
        /* If the previous operation mode is config it is
         directly write the operation mode */
        if (v_prev_opmode_u8 == OPERATION_MODE_CONFIG)
        {
            com_rslt = BNO055_I2C_bus_read(bno055->dev_addr, BNO055_OPERATION_MODE__REG,&v_data_u8r,BNO055_ONE_U8X);
            if (com_rslt == SUCCESS)
            {
                v_data_u8r =
                BNO055_SET_BITSLICE(v_data_u8r, BNO055_OPERATION_MODE,v_operation_mode_u8);
                com_rslt += BNO055_I2C_bus_write(bno055->dev_addr,BNO055_OPERATION_MODE__REG,&v_data_u8r, BNO055_ONE_U8X);
                /* Config mode to other
                 operation mode switching
                 required delay of 600ms*/
                BNO055_delay_msek(BNO055_SIX_HUNDRES_U8X);
            }
        }
        else
        {
            /* If the previous operation
             mode is not config it is
             write the config mode */
            com_rslt = BNO055_I2C_bus_read(bno055->dev_addr, BNO055_OPERATION_MODE__REG, &v_data_u8r,BNO055_ONE_U8X);
            if (com_rslt == SUCCESS)
            {
                v_data_u8r =
                BNO055_SET_BITSLICE(v_data_u8r, BNO055_OPERATION_MODE, OPERATION_MODE_CONFIG);
                com_rslt += bno055_write_register(BNO055_OPERATION_MODE__REG,&v_data_u8r, BNO055_ONE_U8X, bno055);
                /* other mode to config mode switching
                 required delay of 20ms*/
                BNO055_delay_msek(BNO055_TWENTY_U8X);
            }
            /* Write the operation mode */
            if (v_operation_mode_u8 != OPERATION_MODE_CONFIG)
            {
                com_rslt = BNO055_I2C_bus_read(bno055->dev_addr,BNO055_OPERATION_MODE__REG,&v_data_u8r, BNO055_ONE_U8X);
                if (com_rslt == SUCCESS)
                {
                    v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,BNO055_OPERATION_MODE,v_operation_mode_u8);
                    com_rslt +=BNO055_I2C_bus_write(bno055->dev_addr, BNO055_OPERATION_MODE__REG, &v_data_u8r,BNO055_ONE_U8X);
                    /* Config mode to other
                     operation mode switching
                     required delay of 600ms*/
                    BNO055_delay_msek(BNO055_SIX_HUNDRES_U8X);
                }
            }
        }
    }
    else
    {
        com_rslt = ERROR;
    }

    return com_rslt;
}


BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(u8 *v_operation_mode_u8, struct bno055_t *bno055)
{
    /* Variable used to return value of
     communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8r = BNO055_ZERO_U8X;
    s8 v_stat_s8 = ERROR;
    /*condition check for page, operation mode is
     available in the page zero*/
    if (bno055->page_id != PAGE_ZERO)
    /* Write the page zero*/
        v_stat_s8 = bno055_write_page_id(PAGE_ZERO, bno055);
    if ((v_stat_s8 == SUCCESS) || (bno055->page_id == PAGE_ZERO))
    {
        /* Read the value of operation mode*/
        com_rslt = BNO055_I2C_bus_read(bno055->dev_addr,BNO055_OPERATION_MODE__REG, &v_data_u8r, BNO055_ONE_U8X);
        *v_operation_mode_u8 = BNO055_GET_BITSLICE(v_data_u8r,BNO055_OPERATION_MODE);
    }
    else
    {
        com_rslt = ERROR;
    }
    return com_rslt;
}

BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(u8 v_page_id_u8,struct bno055_t *bno055)
{
    /* Variable used to return value of
     communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8r = BNO055_ZERO_U8X;

    /* Read the current page*/
    com_rslt = BNO055_I2C_bus_read(bno055->dev_addr,BNO055_PAGE_ID__REG, &v_data_u8r, BNO055_ONE_U8X);
    /* Check condition for communication success*/
    if (com_rslt == SUCCESS) {
        v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r, BNO055_PAGE_ID, v_page_id_u8);
        /* Write the page id*/
        com_rslt += BNO055_I2C_bus_write(bno055->dev_addr,BNO055_PAGE_ID__REG,&v_data_u8r, BNO055_ONE_U8X);
            if (com_rslt == SUCCESS)
                bno055->page_id = v_page_id_u8;
    }
    else
    {
        com_rslt = ERROR;
    }

    return com_rslt;
}

BNO055_RETURN_FUNCTION_TYPE bno055_write_register(u8 v_addr_u8, u8 *p_data_u8, u8 v_len_u8,struct bno055_t *bno055)
{
    /* Variable used to return value of
     communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    /* Check the struct p_bno055 is empty */
    /* Write the values of respective given register */
    com_rslt = BNO055_I2C_bus_write(bno055->dev_addr, v_addr_u8, p_data_u8, v_len_u8);

    return com_rslt;
}

/*!
 *	@brief This API used to read
 *	mag calibration status from register from 0x35 bit 0 and 1
 *
 *	@param v_mag_calib_u8 : The value of mag calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_calib_stat(u8 *v_mag_calib_u8, struct bno055_t *bno055)
{
    /* Variable used to return value of
     communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8r = BNO055_ZERO_U8X;
    s8 v_stat_s8 = ERROR;
    /* Check the struct p_bno055 is empty */
    /*condition check for page, mag calib
         available in the page zero*/
    if (bno055->page_id != PAGE_ZERO)
    /* Write the page zero*/
    v_stat_s8 = bno055_write_page_id(PAGE_ZERO, bno055);
    if ((v_stat_s8 == SUCCESS) ||(bno055->page_id == PAGE_ZERO))
    {
        /* Read the mag calib v_stat_s8 */
        com_rslt = BNO055_I2C_bus_read(bno055->dev_addr,BNO055_MAG_CALIB_STAT__REG,&v_data_u8r, BNO055_ONE_U8X);
        *v_mag_calib_u8 =BNO055_GET_BITSLICE(v_data_u8r,BNO055_MAG_CALIB_STAT);
    }
    else
    {
        com_rslt = ERROR;
    }

    return com_rslt;
}

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset(struct bno055_mag_offset_t  *mag_offset, struct bno055_t *bno055)
{
    /* Variable used to return value of
     communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    /* Array holding the mag offset values
     v_data_u8[INDEX_ZERO] - offset x->LSB
     v_data_u8[INDEX_ONE] - offset x->MSB
     v_data_u8[INDEX_TWO] - offset y->LSB
     v_data_u8[INDEX_THREE] - offset y->MSB
     v_data_u8[INDEX_FOUR] - offset z->LSB
     v_data_u8[INDEX_FIVE] - offset z->MSB
     */
    u8 v_data_u8[ARRAY_SIZE_SIX] = {
        BNO055_ZERO_U8X, BNO055_ZERO_U8X,
        BNO055_ZERO_U8X, BNO055_ZERO_U8X,
        BNO055_ZERO_U8X, BNO055_ZERO_U8X};
    s8 v_stat_s8 = ERROR;
    /* Check the struct p_bno055 is empty */

    /*condition check for page, mag offset is
        available in the page zero*/
    if (bno055->page_id != PAGE_ZERO)
    /* Write the page zero*/
        v_stat_s8 = bno055_write_page_id(PAGE_ZERO, bno055);
        if ((v_stat_s8 == SUCCESS) ||(bno055->page_id == PAGE_ZERO))
        {
            /* Read mag offset value it the six bytes of data */
            com_rslt = BNO055_I2C_bus_read(bno055->dev_addr, BNO055_MAG_OFFSET_X_LSB__REG,v_data_u8, BNO055_SIX_U8X);
            if (com_rslt == SUCCESS) {
                /* Read mag x offset value*/
                v_data_u8[INDEX_ZERO] = BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],BNO055_MAG_OFFSET_X_LSB);
                v_data_u8[INDEX_ONE] = BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],BNO055_MAG_OFFSET_X_MSB);
                mag_offset->x = (s16)((((s32)(s8)(v_data_u8[INDEX_ONE])) <<(BNO055_SHIFT_8_POSITION)) |(v_data_u8[INDEX_ZERO]));

                /* Read mag y offset value*/
                v_data_u8[INDEX_TWO] = BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],BNO055_MAG_OFFSET_Y_LSB);
                v_data_u8[INDEX_THREE] =BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],BNO055_MAG_OFFSET_Y_MSB);
                mag_offset->y = (s16)((((s32)(s8)(v_data_u8[INDEX_THREE])) <<(BNO055_SHIFT_8_POSITION))| (v_data_u8[INDEX_TWO]));

                /* Read mag z offset value*/
                v_data_u8[INDEX_FOUR] = BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR], BNO055_MAG_OFFSET_Z_LSB);
                v_data_u8[INDEX_FIVE] = BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE], BNO055_MAG_OFFSET_Z_MSB);
                mag_offset->z = (s16)((((s32)(s8)(v_data_u8[INDEX_FIVE])) << (BNO055_SHIFT_8_POSITION)) | (v_data_u8[INDEX_FOUR]));

                /* Read mag radius value
                 it the two bytes of data */
                com_rslt += BNO055_I2C_bus_read(bno055->dev_addr, BNO055_MAG_RADIUS_LSB__REG, v_data_u8, BNO055_TWO_U8X);
                if (com_rslt == SUCCESS) {
                    /* Array holding the mag radius values
                     v_data_u8[INDEX_ZERO] - radius->LSB
                     v_data_u8[INDEX_ONE] - radius->MSB
                     */
                    v_data_u8[INDEX_ZERO] = BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],BNO055_MAG_RADIUS_LSB);
                    v_data_u8[INDEX_ONE] = BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE], BNO055_MAG_RADIUS_MSB);
                    mag_offset->r = (s16)((((s32)(s8)(v_data_u8[INDEX_ONE])) << (BNO055_SHIFT_8_POSITION)) | (v_data_u8[INDEX_ZERO]));
                }
                else
                {
                    com_rslt = ERROR;
                }
            }
            else
            {
                com_rslt = ERROR;
            }
        }

    return com_rslt;
}

bool tcp_conn(int port)
{
	* First call to socket() function */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
   
	if (sockfd < 0)
	{
		perror("ERROR opening socket");
		printf("ERROR opening socket");
		return false;
	}
   
	/* Initialize socket structure */
	bzero((char *) &serv_addr, sizeof(serv_addr));
   
   	serv_addr.sin_family = AF_INET;
   	serv_addr.sin_addr.s_addr = INADDR_ANY;
   	serv_addr.sin_port = htons(port);
   
   	/* Now bind the host address using bind() call.*/
   	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      	perror("ERROR on binding");
      	printf("ERROR on binding");
      	return false;
    }
      
   	/* Now start listening for the clients, here process will
   	* go in sleep mode and will wait for the incoming connection
   	*/
   
   	listen(sockfd,5);
   	clilen = sizeof(cli_addr);
   
   	/* Accept actual connection from the client */
   	newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
   	if (newsockfd < 0)
    {
      	perror("ERROR on accept");
      	printf("ERROR on accept");
      	return false;
    }
    
    printf("connection established");
    return true;
}

bool tcp_send_data(string data)
{
    //Send some data
    if( send(newsockfd , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
    	printf("Send failed\n");
        perror("Send failed : ");
        return false;
    }
    return true;
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

    uint8_t rx_tx_buf[ARRAY_SIZE_SIX];
    i2c->address(dev_addr);
    rx_tx_buf[0] = reg_addr;
    rx_tx_buf[1] = *reg_data;
    mraa_result_t status = i2c->write(rx_tx_buf, 2);

    //if (status == MRAA_SUCCESS)
        return SUCCESS;
    //else
    //    return (s8)iError;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    uint8_t reg = reg_addr;
    int size = cnt;
    i2c->address(dev_addr);
    i2c->writeByte(reg);

    i2c->address(dev_addr);
    i2c->read(rx_tx_buf, size);

    //if (status == MRAA_SUCCESS)
    return SUCCESS;
    //else
    //    return (s8)iError;
}

void BNO055_delay_msek(u32 msek)
{
    int ms = msek;
    /*Here you can write your own delay routine*/
    usleep(1000*ms);
}
Status API Training Shop Blog About Help
© 2015 GitHub, Inc. Terms Privacy Security Contact
