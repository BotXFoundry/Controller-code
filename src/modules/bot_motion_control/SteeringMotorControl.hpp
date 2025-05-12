/*
 * 串口读取函数
 * rw_uart.c
 */
#pragma once

#include <drivers/drv_hrt.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <systemlib/err.h>
#include <termios.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <px4_platform_common/time.h>
#include <sys/ioctl.h>

#include <math.h>
#include <nuttx/serial/serial.h>
#include <poll.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include  <SocketCAN.hpp>

#include "BotMotionControl.hpp"

#include <stdlib.h>
#include <strings.h>
#include <uORB/uORB.h>


void RS485WriteON();
void RS485ReadON();
int RS485InitTest();

int RS485SetUartBaudRate(const int fd, unsigned int baud);

void Bot4DGetSteeringPos(int16_t *pos);

int Bot4DSteeringAsync(uint8_t *id, uint16_t *angle, uint16_t *time, uint16_t *velocity, int size);  // 异步写：同时控制多个舵机，参数：id，角度，时间，速度

// 移动到某个角度，一组角度（最大为4）
void Bot4DSteeringToDegree(uint16_t *angle);

// 回正
void Bot4DSteeringToZero();

// 横移
void Bot4DSteeringTo180();

// 转向对应角度
void Bot4DSteeringToSameAngle(int16_t angle);

void Bot3DSteeringToDegree(uint16_t angle);

int Bot3DSteeringAsync(uint8_t id, uint16_t angle, uint16_t time, uint16_t velocity);
