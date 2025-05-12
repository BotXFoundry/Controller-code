#include "SocketCAN.hpp"

#define CANID_DELIM '#'
#define DATA_SEPARATOR '.'
#define MAXSOCK 2 /* max. number of CAN interfaces given on the cmdline */

#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */

int idx;


int SocketCanSend(uint32_t in_can_id, uint8_t *message) {

  int s; /* can raw socket */
  int required_mtu;

  // int enable_canfd = 1;
  struct sockaddr_can addr;
  struct canfd_frame frame;
  struct canfd_frame *cf;
  struct ifreq ifr;

  cf = &frame;

  required_mtu = sizeof(struct can_frame);

  /* prepare CAN frame */
  memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

  cf->can_id = in_can_id;
  cf->len = 8;
  cf->flags = 0;  // 0-Classic CAN; 1-FD-CAN

  for (int i = 0; i < cf->len; i++) {
    cf->data[i] = message[i];  // TODO:处理CAN data的赋值
  }

  /* open socket */
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    return 1;
  }

  strncpy(ifr.ifr_name, "can1", IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex) {
    return 1;
  }

  memset(&addr, 0, sizeof(addr));  // 初始化addr
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    return 1;
  }

  /* send frame */
  if (write(s, &frame, required_mtu) != required_mtu) {
    return 1;
  }
  close(s);

  return 0;
}


// 批量发送
int SocketCanBatchSend(int *in_can_id, uint8_t *message, int size) {
  int s; /* can raw socket */
  int required_mtu;

  struct sockaddr_can addr;
  struct canfd_frame frame;
  struct canfd_frame *cf;
  struct ifreq ifr;

  cf = &frame;

  required_mtu = sizeof(struct can_frame);

  // /* prepare CAN frame */
  memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

  // // cf->can_id = CAN_ID;
  cf->len = 8;
  cf->flags = 0;  // 0-Classic CAN; 1-FD-CAN

  // /* open socket */
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    return 1;
  }

  strncpy(ifr.ifr_name, "can1", IFNAMSIZ - 1);

  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex) {
    return 1;
  }

  memset(&addr, 0, sizeof(addr));  // 初始化addr

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    return 1;
  }

  /* send frame */

  int i_id = 0;
  int i_data = 0;

  for (int t = 0; t < size; t++) {
    cf->can_id = in_can_id[i_id];
    i_id++;
    for (int a = 0; a < cf->len; a++) {
      cf->data[a] = message[i_data];  // TODO 这里赋值可能存在问题
      i_data++;
    }

    if (write(s, &frame, required_mtu) != required_mtu) {
      return 1;
    }
  }

  close(s);

  memset(in_can_id, 0, sizeof(*in_can_id));
  memset(message, 0, sizeof(*message));
  // PX4_INFO("TEST SocketCanBatchSend end ...");
  return 0;
}
