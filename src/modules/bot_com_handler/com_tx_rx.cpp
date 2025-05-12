#include "com_tx_rx.hpp"
#include <board_config.h>


int com4_init(int com_num)
{
	char com_str[24] = "/dev/ttyS4";
	//char char_num[8];
	int serial_fd = 0;

     //int serial_fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY  | O_NONBLOCK);
     //int serial_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY );
     //int serial_fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY );  //TEL3
     //int serial_fd = open("/dev/ttyS3", O_RDWR | O_NOCTTY ); //UART&I2C
     //int serial_fd = open("/dev/ttyS4", O_RDWR | O_NOCTTY );  //TEL2
     //int serial_fd = open("/dev/ttyS6", O_RDWR | O_NOCTTY );  //TEL1
     //int serial_fd = open("/dev/ttyS7", O_RDWR | O_NOCTTY );  //GPS2
    //memset(char_num, 0x00, sizeof(char_num));
    //sprintf(char_num, "%d", com_num) ;
    //strcat(com_str, char_num);

    PX4_INFO("open com: %s", com_str);
    serial_fd = open(com_str, O_RDWR | O_NOCTTY );

	if (serial_fd < 0) {
		PX4_INFO("failed to open port");
		return false;
	}
	PX4_INFO("Open the %d",serial_fd);

	return serial_fd;
}


int set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed;
	struct termios uart_config;
	int termios_state;

	switch (baud)
	{
		case 9600:   speed = B9600;   break;
		case 19200:  speed = B19200;  break;
		case 38400:  speed = B38400;  break;
		case 57600:  speed = B57600;  break;
		case 115200: speed = B115200; break;
		default:
			PX4_INFO("ERR: baudrate: %d", baud);
		return -EINVAL;
	}

    if ((termios_state = tcgetattr(fd, &uart_config)) < 0)
	{
	   //PX4_INFO("cary test tcgetattr <0 ");
	   return false;
	}

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR; //


	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
	{
		//PX4_INFO("cary test cfsetispeed < 0");
		return false;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0)
	{
		//PX4_INFO("cary test tcsetattr < 0");
		return false;
	}

	return true;
}

//crc check
uint16_t Handle_Get_Checksum(const char *in_buf, int in_len)
{
	uint8_t i;
	uint16_t temp_num = 0x0000;

	for(i=0; i<in_len; i++)
	{
		temp_num += in_buf[i];
	}

	temp_num = ~temp_num +1;
	return temp_num;
}
/*********************************************************
*
*
*********************************************************/
int hex_to_decimal(char hex)
{
	int decimal = 0;

	decimal = (hex >> 4)*16 + (hex&0x0F);
	return decimal;
}

/*********************************************************
*data: 55 05 01 02 01 00 FF FC AA
*	   55 05 02 04 64 64 64 64 FE 6A AA
*********************************************************/
int Handle_Read_55(int in_serial_fd, char *in_buf)
{
	int ret, i;
	int num = 0;
	int data_len = 0;
	int check_len = 0;
	char check_buf[8];
	char data_buf[32];
	char data;
	uint16_t ret_crc = 0;

	//1: read header
	memset(data_buf, 0x00, sizeof(data_buf));

	for(i=0; i<3; i++)
	{
		ret = read(in_serial_fd, &data, 1);
		if(ret != 1)
		{
			//PX4_INFO("0:(ret: %d) != 1", ret);
			return 0;
		}
		data_buf[i] = data;
	}

	//2:read data content
	data_len = hex_to_decimal(data_buf[2]) + 3;
	if(data_len > 10)
	{
		PX4_INFO("1: data_len(%d) > 10 , error ", data_len);
		return 0;
	}
	//ret = read(in_serial_fd, &data_buf[3], data_len);
	for(i=0; i<data_len; i++)
	{
		ret = read(in_serial_fd, &data, 1);
		if(ret != 1)
		{
			return 0;
		}
		data_buf[3+i] = data;
		num++;
	}
/*
    for(i=0; i<data_len+3; i++)
    {
		PX4_INFO("%d: 0x%02x ", i, data_buf[i]);
	}
	*/
	//if(ret != data_len)
	if(num != data_len)
	{
		PX4_INFO("2: ret(%d) ! = data_len(%d), error!",ret, data_len);
		return 0;
	}

	//3:0xAA: check end flag
	if(data_buf[data_len+3-1] != 0xAA)
	{
		PX4_INFO("3: data_buf[data_len+3-1](0x%02x) != 0xAA, error!", data_buf[data_len+3-1]);
		return 0;
	}

	//4:check data
	memset(check_buf, 0x00, sizeof(check_buf));
	check_len = hex_to_decimal(data_buf[2]) + 2;
	if(check_len > 7)
	{
		PX4_INFO("4: crc data length check_len(%d) > 7, error!", check_len);
		return 0;
	}
	for(i=0; i<check_len; i++)
	{
		check_buf[i] = data_buf[i+1];
	}

	ret_crc = Handle_Get_Checksum(check_buf, check_len);
	if((data_buf[data_len+3-3] != (ret_crc >> 8)) & (data_buf[data_len+3-2] != (ret_crc & 0xFF)))
	{
		PX4_INFO("5: crc error!");
		return 0;
	}
	else
	{
		memcpy(in_buf, data_buf, data_len+3);
	}

	return 	data_len+3;
}

/***********************************************************
*data:DD 04 00 0E 0E 43 0E 43 0E 44 0E 40 0E 43 0E 41 0E 45 FD BD 77
*     DD 05 00 15 4A 42 44 2D 53 50 31 35 53 30 32 30 41 2D 50 37 53 2D 34 30 41 FA E6 77
*     DD 03 00 1B 09 FB 00 00 07 44 0B E1 00 05 2F 4B 00 00 00 00 00 00 23 3D 03 07 02 0B AB 0B A9 FB 55 77
***********************************************************/
int Handle_Read_DD(int in_serial_fd, char *in_buf)
{
	int ret, i;
	int num = 0;
	int data_len = 0;
	int check_len = 0;
	char check_buf[32];
	char data_buf[60];
	char data;
	uint16_t ret_crc = 0;

	//1: read header
	memset(data_buf, 0x00, sizeof(data_buf));
	for(i=0; i<3; i++)
	{
		ret = read(in_serial_fd, &data, 1);
		if(ret != 1)
		{
			//PX4_INFO("(ret: %d) != 1", ret);
			return 0;
		}
		data_buf[i] = data;
	}
	if((data_buf[0] != 0x03)&(data_buf[0] != 0x04)&(data_buf[0] != 0x05))
	{
		//PX4_INFO("(0: data_buf[0]: 0x%x) != 0x03 04 05", data_buf[0]);
		return 0;
	}

	//2:read data content
	data_len = (hex_to_decimal(data_buf[1])*256)  + hex_to_decimal(data_buf[2]) + 3;
	//PX4_INFO("data_len = %d", data_len);
	if(data_len > 55)
	{
		//PX4_INFO("1: data_len(%d) > 35 , error!", data_len);
		return 0;
	}
	//ret = read(in_serial_fd, &data_buf[3], data_len);
	for(i=0; i<data_len; i++)
	{
		ret = read(in_serial_fd, &data, 1);
		if(ret != 1)
		{
			return 0;
		}
		data_buf[3+i] = data;
		num++;
	}
    for(i=0; i<data_len+3; i++)
	{
		//PX4_INFO("%d: 0x%02x ", i, data_buf_dd[i]);
		in_buf[i] = data_buf[i];
	}
	if(num != data_len)
	{
		//PX4_INFO("2: ret(%d) != data_len(%d), error!", ret, data_len);
		return 0;
	}

	//3:0x77: check end flag
	if(data_buf[data_len+3-1] != 0x77)
	{
		//PX4_INFO("3: (data_buf[data_len+3-1]: 0x%x) != 0x77, error!", data_buf[data_len+3-1]);
		return 0;
	}

	//4:check data
	memset(check_buf, 0x00, sizeof(check_buf));
	check_len = (hex_to_decimal(data_buf[1])*256)  + hex_to_decimal(data_buf[2]) + 2;
	if(check_len > 55)
	{
		//PX4_INFO("4: check_len(%d) > 32, error!", check_len);
		return 0;
	}
	//PX4_INFO("check_len = %d", check_len);
	for(i=0; i<check_len; i++)
	{
		check_buf[i] = data_buf[i+1];
	}

	ret_crc = Handle_Get_Checksum(check_buf, check_len);
	if((data_buf[data_len+3-3] != (ret_crc >> 8)) & (data_buf[data_len+3-2] != (ret_crc & 0xFF)))
	{
		//PX4_INFO("5: CRC error, return 0");
		return 0;
	}
	// else
	// {
	// 	memcpy(in_buf, data_buf, data_len+3);
	// }

	return 	data_len+3;
}



