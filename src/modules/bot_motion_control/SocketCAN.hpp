
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

int SocketCanSend(uint32_t can_id, uint8_t *message);  // can1发送数据

int SocketCanBatchSend(int *can_id, uint8_t *message,
                       int size);  // can1批量发送函数
