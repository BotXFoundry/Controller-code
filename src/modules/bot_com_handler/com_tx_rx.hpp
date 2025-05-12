/*
 * 串口读取函数
 * rw_uart.c
 */
#pragma once

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
//#include <systemlib/systemlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

//#include <uORB/topics/follow_camera.h>
#include <px4_platform_common/time.h>

#include <sys/ioctl.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <poll.h>
#include <math.h>
#include <stdlib.h>
#include <strings.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>

#if defined(__PX4_NUTTX)
#include <nuttx/serial/serial.h>
#else
#include <linux/serial.h>
#endif


int com_read(int com_num);
int com4_init(int com_num);
int Handle_Read_DD(int in_serial_fd, char *in_buf);
int Handle_Read_55(int in_serial_fd, char *in_buf);
int set_uart_baudrate(const int fd, unsigned int baud);

