#include "RS485.hpp"

// #include <board_config.h>

// USART2_CTS_TELEM3    PD3

int serial485_fd = 0;
bool serial_on_off_flag = FALSE;

// #define GPIO_USART2_CTS_T          /* PD3 */
// (GPIO_OUTPUT|GPIO_PULLDOWN|GPIO_PORTD|GPIO_PIN3)
#define GPIO_USART2_CTS_T /* PD3 */                                   \
  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
   GPIO_PORTD | GPIO_PIN3)

// #define GPIO_USART2_CTS_T          /* PD3
// */(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN3)

void RS485WriteON() {
  px4_arch_configgpio(GPIO_USART2_CTS_T);
  px4_arch_gpiowrite(GPIO_USART2_CTS_T, 1);
}

void RS485ReadON() {
  px4_arch_configgpio(GPIO_USART2_CTS_T);
  px4_arch_gpiowrite(GPIO_USART2_CTS_T, 0);
}

int RS485InitTest() {
  serial485_fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY);  // TEL3
  if (serial485_fd < 0) {
    printf("Failed to open port");
    return false;
  }
  serial_on_off_flag = TRUE;
  return serial485_fd;
}

int RS485SetUartBaudRate(const int fd, unsigned int baud) {
  int speed;

  switch (baud) {
    case 9600:
      speed = B9600;
      break;
    case 19200:
      speed = B19200;
      break;
    case 38400:
      speed = B38400;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    default:
      printf("ERR: baudrate: %d", baud);
      return -EINVAL;
  }

  struct termios uart_config;
  int termios_state;

  if ((termios_state = tcgetattr(fd, &uart_config)) < 0) {
    printf("RS485 test tcgetattr <0 ");
    return false;
  }

  /* clear ONLCR flag (which appends a CR for every LF) */
  uart_config.c_oflag &= ~ONLCR;  // ??NL?????CR(???)-NL?????

  if (cfsetispeed(&uart_config, speed) < 0 ||
      cfsetospeed(&uart_config, speed) < 0) {
    printf("RS485 test cfsetispeed < 0");
    return false;
  }

  if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
    printf("RS485 test tcsetattr < 0");
    return false;
  }

  return true;
}

void Bot4DGetSteeringPos(int16_t *pos) {
  uint8_t data[8] = {0xFF, 0xFF, 0x0, 0x04, 0x02, 0x38, 0x02, 0x0};
  int16_t id[4] = {0x01, 0x02, 0x03, 0x04};
  int16_t data_feedback[8] = {0,0,0,0,0,0,0,0};


  int uart = 0;
  uart = RS485InitTest();

  if (false == RS485SetUartBaudRate(uart, 115200)) {
    printf("[JXF]Set_uart_baudrate is failed");
  }

  for (int i = 0; i < 4; i++) {
    data[2] = id[i];
    data[7] = ~(data[2] + data[3] + data[4] + data[5] + data[6]);
    write(uart, &data, 8);

    read(uart, &data_feedback, 8);

    pos[i] = data_feedback[5] | data_feedback[6] << 8;
  }
  close(uart);

}

/*****************************************************************************
*舵机参数说明：
*	飞特舵机位置输入范围：0-4095
*	绝对角度控制，360 度对应角度分辨率是 4096 位，也就是转一圈 0-360 度，对应 0-4095；
* 	如果只转半圈即 180 度，就是 4096/2=2048。2048 便是 180 度的位置，即中位。
*   舵机定位的时候以中位置：2048为基准点，所以电机只能按照中位的位置左右旋转90°
*输入参数说明:
* 	angle：输入舵机转角范围-8000 ~ 8000
******************************************************************************/
int Bot4DSteeringAsync(uint8_t *id, uint16_t *angle, uint16_t *time, uint16_t *velocity, int size) {
  uint8_t data[size][13];
  //int uart = 0;
  uint16_t location = 0;

  for (int i = 0; i < size; i++) {
    data[i][0] = 0xFF;
    data[i][1] = 0xFF;

    data[i][2] = id[i];

    data[i][3] = 0x09; //长度
    data[i][4] = 0x04; //指令04:异步写
    data[i][5] = 0x2A; //目标位置L

    location = angle[i];

    data[i][6] = location;  //位置数据
    data[i][7] = location >> 8;
    data[i][8] = time[i];   //时间数据
    data[i][9] = time[i] >> 8;
    data[i][10] = velocity[i]; //速度数据
    data[i][11] = velocity[i] >> 8;

    uint8_t x = 0x0;
    for (int ii = 2; ii < 12; ii++) {
      x += data[i][ii];
    }
    data[i][12] = ~(x);
  }

  if(serial_on_off_flag == FALSE)
  {
  	RS485InitTest();
  	if (false == RS485SetUartBaudRate(serial485_fd, 115200)) {
    	printf("[JXF]Set_uart_baudrate is failed");
  	}
  }
  RS485WriteON();
  for (int i = 0; i < size; i++) {
    write(serial485_fd, &data[i], 13);
  }
  //usleep(10);
  //FE：广播id，每个舵机都可以收到，0x05:执行异步写
  uint8_t action[6] = {0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA};
  write(serial485_fd, &action, 6);
  return 0;
}

int Bot3DSteeringAsync(uint8_t id, uint16_t angle, uint16_t time, uint16_t velocity) {
  uint8_t data[13];

  uint16_t location = 0;

    data[0] = 0xFF;
    data[1] = 0xFF;

    data[2] = id;

    data[3] = 0x09; //长度
    data[4] = 0x04; //指令04:异步写
    data[5] = 0x2A; //目标位置L

    location = angle;

    data[6] = location;  //位置数据
    data[7] = location >> 8;
    data[8] = time;   //时间数据
    data[9] = time >> 8;
    data[10] = velocity; //速度数据
    data[11] = velocity >> 8;

    uint8_t x = 0x0;
    for (int ii = 2; ii < 12; ii++) {
      x += data[ii];
    }
    data[12] = ~(x);

  if(serial_on_off_flag == FALSE)
  {
  	RS485InitTest();
  	if (false == RS485SetUartBaudRate(serial485_fd, 115200)) {
    	printf("[JXF]Set_uart_baudrate is failed");
  	}
  }
  RS485WriteON();

  write(serial485_fd, &data, 13);

  //FE：广播id，每个舵机都可以收到，0x05:执行异步写
  uint8_t action[6] = {0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA};
  write(serial485_fd, &action, 6);

  return 0;
}


//angle输入范围：-8000 ~ 8000
void Bot4DSteeringToDegree(uint16_t *angle) {
  uint8_t id[4] = {0x01, 0x02, 0x03, 0x04};

  uint16_t time[4] = {0, 0, 0, 0};
  uint16_t velocity[4] = {0x3E8, 0x3E8, 0x3E8, 0x3E8};

  Bot4DSteeringAsync(id, angle, time, velocity, 4);
}

void Bot3DSteeringToDegree(uint16_t angle) {
  uint8_t id = 0x01;
  uint16_t time = 0;
  // uint16_t velocity = 0x3E88;
  uint16_t velocity = 0x3E8;
  Bot3DSteeringAsync(id, angle, time, velocity);
}

//4轮回正，归零
void Bot4DSteeringToZero() {
  uint8_t id[4] = {0x1, 0x2, 0x3, 0x4};
  uint16_t angle_1[4] = {2048, 2048, 2048, 2048};
  uint16_t time[4] = {0, 0, 0, 0}; //时间值
  uint16_t sd[4] = {0x3E8, 0x3E8, 0x3E8, 0x3E8}; //速度值：1000每秒

  Bot4DSteeringAsync(id, angle_1, time, sd, 4);
}
//旋转180度
void Bot4DSteeringTo180() {
  uint8_t id[4] = {0x1, 0x2, 0x3, 0x4};
  //横移是从零度即2047转到90度，舵机安装方向不同，转向的角度不同
  uint16_t angle_1[4] = {1024, 3071, 3071, 1024};
  uint16_t time[4] = {0, 0, 0, 0};
  uint16_t sd[4] = {0x3E8, 0x3E8, 0x3E8, 0x3E8};

  Bot4DSteeringAsync(id, angle_1, time, sd, 4);
}
