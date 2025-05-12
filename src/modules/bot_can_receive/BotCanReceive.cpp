#include "BotCanReceive.hpp"


bool BotCanReceive::init() {
  // alternatively, Run on fixed interval
  ScheduleOnInterval(5000);  // 200000 us interval, 2 Hz rate
  return true;
}

void BotCanReceive::Run() {


  if (should_exit()) {
    ScheduleClear();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);
  perf_count(_loop_interval_perf);

  CanReceiveAndPublishMessage();

  perf_end(_loop_perf);
}

void BotCanReceive::InitCanReceiveFlag(){
  bot_can_message_receive_pub.can_data_receive_001 = 0;
  bot_can_message_receive_pub.can_data_receive_100 = 0;
  bot_can_message_receive_pub.can_data_receive_102 = 0;
  bot_can_message_receive_pub.can_data_receive_104 = 0;
  bot_can_message_receive_pub.can_data_receive_106 = 0;
  bot_can_message_receive_pub.can_data_receive_114 = 0;
  bot_can_message_receive_pub.can_data_receive_116 = 0;
  bot_can_message_receive_pub.can_data_receive_117 = 0;
  bot_can_message_receive_pub.can_data_receive_120 = 0;
  bot_can_message_receive_pub.can_data_receive_130 = 0;
  bot_can_message_receive_pub.can_data_receive_132 = 0;
  bot_can_message_receive_pub.can_data_receive_134 = 0;
  bot_can_message_receive_pub.can_data_receive_136 = 0;
  bot_can_message_receive_pub.can_data_receive_140 = 0;
  bot_can_message_receive_pub.can_data_receive_150 = 0;
  bot_can_message_receive_pub.can_data_receive_152 = 0;
  bot_can_message_receive_pub.can_data_receive_154 = 0;
  bot_can_message_receive_pub.can_data_receive_190 = 0;
  bot_can_message_receive_pub.can_data_receive_192 = 0;
  bot_can_message_receive_pub.can_data_receive_194 = 0;
  bot_can_message_receive_pub.can_data_receive_196 = 0;
  bot_can_message_receive_pub.can_data_receive_200 = 0;
  bot_can_message_receive_pub.can_data_receive_202 = 0;
  bot_can_message_receive_pub.can_data_receive_204 = 0;
  bot_can_message_receive_pub.can_data_receive_206 = 0;
  bot_can_message_receive_pub.can_data_receive_220 = 0;
  bot_can_message_receive_pub.can_data_receive_231 = 0;
  bot_can_message_receive_pub.can_data_receive_281 = 0;
  bot_can_message_receive_pub.can_data_receive_282 = 0;
  bot_can_message_receive_pub.can_data_receive_283 = 0;
  bot_can_message_receive_pub.can_data_receive_284 = 0;
  bot_can_message_receive_pub.can_data_receive_381 = 0;
  bot_can_message_receive_pub.can_data_receive_382 = 0;
  bot_can_message_receive_pub.can_data_receive_383 = 0;
  bot_can_message_receive_pub.can_data_receive_384 = 0;
  bot_can_message_receive_pub.can_data_receive_581 = 0;
  bot_can_message_receive_pub.can_data_receive_582 = 0;
  bot_can_message_receive_pub.can_data_receive_583 = 0;
  bot_can_message_receive_pub.can_data_receive_584 = 0;
  bot_can_message_receive_pub.can_data_receive_701 = 0;
  bot_can_message_receive_pub.can_data_receive_702 = 0;
  bot_can_message_receive_pub.can_data_receive_703 = 0;
  bot_can_message_receive_pub.can_data_receive_704 = 0;
}

int BotCanReceive::CanReceiveAndPublishMessage() {
  int running = 1;
  int nbytes;
  int s;
  const int canfd_on = 1;
  struct sockaddr_can can_addr;
  struct canfd_frame frame;
  struct can_frame frame_read;
  struct canfd_frame *cf;
  struct ifreq ifr_one;
  // const char *ptr;
  fd_set rdfs;
  int ret;

  int i;

  // struct can_filter rfilter[1];

  // 注意：timeval结构体中的tv_usec指的是毫秒Microseconds，不是微妙
  struct timeval timeout, timeout_config = {0, 0}, *timeout_current = NULL;

  timeout_config.tv_usec = 5000;
  timeout_config.tv_sec = timeout_config.tv_usec / 1000;
  timeout_config.tv_usec = (timeout_config.tv_usec % 1000) * 1000;
  timeout_current = &timeout;

  memset(&frame_read, 0, sizeof(frame_read)); /* init CAN FD frame, e.g. LEN = 0 */

  cf = &frame;
  memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    PX4_INFO("dump_socket open");
    PX4_INFO("errno= %d", errno);
    perror("socket");
    return 1;
  }

  memset(&ifr_one.ifr_name, 0, sizeof(ifr_one.ifr_name));
  strcpy(ifr_one.ifr_name, "can0");

  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = 0; /* any can interface */

  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	int flags = fcntl(s, F_GETFL, 0);
	if(flags < 0)
	{
		PX4_INFO(" fcntl get flags error!");
		return -1;
	}
	if(fcntl(s, F_SETFL, flags | O_NONBLOCK) < 0)
	{
		PX4_INFO("##110: fcntl set flags error!");
		return -1;
	}

  if (bind(s, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0) {
    PX4_INFO("dump_socket bind error");
    perror("bind");
    return 1;
  }

  while (running) {

    FD_ZERO(&rdfs);
    FD_SET(s, &rdfs);

    *timeout_current = timeout_config;
    if ((ret = select(s + 1, &rdfs, NULL, NULL, timeout_current)) <= 0) {
	  PX4_INFO("select ret <= 0");
      continue;
    }

    if (FD_ISSET(s, &rdfs)) {
      nbytes = read(s, &frame_read, sizeof(frame_read));
      if (nbytes < 0) {
        continue;
      }

      uint32_t frame_read_can_id = frame_read.can_id;
      switch (frame_read_can_id) {
          case 0x186:
            bot_can_message_pub.can_id_186 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_186[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_186 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_186 ++;
            if(bot_can_message_pub.can_data_receive_num_186 > 5000){
              bot_can_message_pub.can_data_receive_num_186 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_186 = 1;
            break;
          case 0x187:
            bot_can_message_pub.can_id_187 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_187[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_187 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_187 ++;
            if(bot_can_message_pub.can_data_receive_num_187 > 5000){
              bot_can_message_pub.can_data_receive_num_187 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_187 = 1;
            break;
          case 0x188:
            bot_can_message_pub.can_id_188 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_188[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_188 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_188 ++;
            if(bot_can_message_pub.can_data_receive_num_188 > 5000){
              bot_can_message_pub.can_data_receive_num_188 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_188 = 1;
            break;
          case 0x189:
            bot_can_message_pub.can_id_189 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_189[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_189 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_189 ++;
            if(bot_can_message_pub.can_data_receive_num_189 > 5000){
              bot_can_message_pub.can_data_receive_num_189 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_189 = 1;
            break;
          case 0x100:  // 控制模式设定
            bot_can_message_pub.can_id_100 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_100[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_100 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_100 ++;
            if(bot_can_message_pub.can_data_receive_num_100 > 5000){
              bot_can_message_pub.can_data_receive_num_100 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_100 = 1;
            break;
          case 0x102:  // 本体运动模型设置
            bot_can_message_pub.can_id_102 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_102[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_102 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_102 ++;
            if(bot_can_message_pub.can_data_receive_num_102 > 5000){
              bot_can_message_pub.can_data_receive_num_102 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_102 = 1;
            break;
          case 0x104:  // 查询本体线速度、角速度、转角
            bot_can_message_pub.can_id_104 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_104[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_104 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_104 ++;
            if(bot_can_message_pub.can_data_receive_num_104 > 5000){
              bot_can_message_pub.can_data_receive_num_104 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_104 = 1;
            break;
          case 0x106:  // 查询传感器状态反馈，如防撞条、急停
            bot_can_message_pub.can_id_106 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_106[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_106 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_106 ++;
            if(bot_can_message_pub.can_data_receive_num_106 > 5000){
              bot_can_message_pub.can_data_receive_num_106 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_106 = 1;
            break;
          case 0x114:  // 运动指令控制
            bot_can_message_pub.can_id_114 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_114[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_114 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_114 ++;
            if(bot_can_message_pub.can_data_receive_num_114 > 5000){
              bot_can_message_pub.can_data_receive_num_114 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_114 = 1;
            // PX4_INFO("Bot_can_receive 114 114 114 114 114 can_id=%ld",bot_can_message_pub.can_id_114);
            break;
          case 0x116: //加速时间设置
            bot_can_message_pub.can_id_116 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_116[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_116 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_116 ++;
            if(bot_can_message_pub.can_data_receive_num_116 > 5000){
              bot_can_message_pub.can_data_receive_num_116 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_116 = 1;
            break;
          case 0x117: //速度环KP KI值设置
            bot_can_message_pub.can_id_117 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_117[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_117 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_117 ++;
            if(bot_can_message_pub.can_data_receive_num_117 > 5000){
              bot_can_message_pub.can_data_receive_num_117 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_117 = 1;
            break;
          case 0x120:  // 主控及伺服驱动器系统软硬件版本反馈
            bot_can_message_pub.can_id_120 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_120[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_120 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_120 ++;
            if(bot_can_message_pub.can_data_receive_num_120 > 5000){
              bot_can_message_pub.can_data_receive_num_120 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_120 = 1;
            break;

            // 电机电压、电流、转速、温度数据反馈
          case 0x130:  // 第1电机电压、电流、转速、温度数据反馈
            bot_can_message_pub.can_id_130 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_130[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_130 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_130 ++;
            if(bot_can_message_pub.can_data_receive_num_130 > 5000){
              bot_can_message_pub.can_data_receive_num_130 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_130 = 1;
            break;
          case 0x132:  // 第2电机电压、电流、转速、温度数据反馈
            bot_can_message_pub.can_id_132 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_132[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_132 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_132 ++;
            if(bot_can_message_pub.can_data_receive_num_132 > 5000){
              bot_can_message_pub.can_data_receive_num_132 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_132 = 1;
            break;
          case 0x134:  // 第3电机电压、电流、转速、温度数据反馈
            bot_can_message_pub.can_id_134 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_134[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_134 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_134 ++;
            if(bot_can_message_pub.can_data_receive_num_134 > 5000){
              bot_can_message_pub.can_data_receive_num_134 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_134 = 1;
            break;
          case 0x136:  // 第4电机电压、电流、转速、温度数据反馈
            bot_can_message_pub.can_id_136 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_136[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_136 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_136 ++;
            if(bot_can_message_pub.can_data_receive_num_136 > 5000){
              bot_can_message_pub.can_data_receive_num_136 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_136 = 1;
            break;

          case 0x140:  // 灯光控制
            bot_can_message_pub.can_id_140 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_140[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_140 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_140 ++;
            if(bot_can_message_pub.can_data_receive_num_140 > 5000){
              bot_can_message_pub.can_data_receive_num_140 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_140 = 1;
            break;

          case 0x150:  // 行驶轮线速度反馈
            bot_can_message_pub.can_id_150 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_150[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_150 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_150 ++;
            if(bot_can_message_pub.can_data_receive_num_150 > 5000){
              bot_can_message_pub.can_data_receive_num_150 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_150 = 1;
            break;
          case 0x152:  // 行驶轮转向角反馈
            bot_can_message_pub.can_id_152 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_152[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_152 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_152 ++;
            if(bot_can_message_pub.can_data_receive_num_152 > 5000){
              bot_can_message_pub.can_data_receive_num_152 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_152 = 1;
            break;
          case 0x154:  // 行驶轮电机编码器脉冲值反馈
            bot_can_message_pub.can_id_154 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_154[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_154 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_154 ++;
            if(bot_can_message_pub.can_data_receive_num_154 > 5000){
              bot_can_message_pub.can_data_receive_num_154 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_154 = 1;
            break;

            // 里程计数据反馈
          case 0x190:  // 第1轮里程计数据反馈
            bot_can_message_pub.can_id_190 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_190[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_190 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_190 ++;
            if(bot_can_message_pub.can_data_receive_num_190 > 5000){
              bot_can_message_pub.can_data_receive_num_190 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_190 = 1;
            break;
          case 0x192:  // 第2轮里程计数据反馈
            bot_can_message_pub.can_id_192 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_192[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_192 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_192 ++;
            if(bot_can_message_pub.can_data_receive_num_192 > 5000){
              bot_can_message_pub.can_data_receive_num_192 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_192 = 1;
            break;
          case 0x194:  // 第3轮里程计数据反馈
            bot_can_message_pub.can_id_194 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_194[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_194 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_194 ++;
            if(bot_can_message_pub.can_data_receive_num_194 > 5000){
              bot_can_message_pub.can_data_receive_num_194 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_194 = 1;
            break;
          case 0x196:  // 第4轮里程计数据反馈
            bot_can_message_pub.can_id_196 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_196[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_196 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_196 ++;
            if(bot_can_message_pub.can_data_receive_num_196 > 5000){
              bot_can_message_pub.can_data_receive_num_196 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_196 = 1;
            break;

            // 电机驱动器状态反馈
          case 0x200:  // 第1电机驱动器状态反馈
            bot_can_message_pub.can_id_200 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_200[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_200 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_200 ++;
            if(bot_can_message_pub.can_data_receive_num_200 > 5000){
              bot_can_message_pub.can_data_receive_num_200 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_200 = 1;
            break;
          case 0x202:  // 第2电机驱动器状态反馈
            bot_can_message_pub.can_id_202 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_202[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_202 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_202 ++;
            if(bot_can_message_pub.can_data_receive_num_202 > 5000){
              bot_can_message_pub.can_data_receive_num_202 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_202 = 1;
            break;
          case 0x204:  // 第3电机驱动器状态反馈
            bot_can_message_pub.can_id_204 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_204[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_204 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_204 ++;
            if(bot_can_message_pub.can_data_receive_num_204 > 5000){
              bot_can_message_pub.can_data_receive_num_204 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_204 = 1;
            break;
          case 0x206:  // 第4电机驱动器状态反馈
            bot_can_message_pub.can_id_206 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_206[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_206 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_206 ++;
            if(bot_can_message_pub.can_data_receive_num_206 > 5000){
              bot_can_message_pub.can_data_receive_num_206 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_206 = 1;
            break;

          case 0x220:  // 电池BMS状态数据反馈
            bot_can_message_pub.can_id_220 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_220[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_220 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_220 ++;
            if(bot_can_message_pub.can_data_receive_num_220 > 5000){
              bot_can_message_pub.can_data_receive_num_220 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_220 = 1;
            break;

          case 0x231: // 传感器设置
            bot_can_message_pub.can_id_231 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_231[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_231 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_231 ++;
            if(bot_can_message_pub.can_data_receive_num_231 > 5000){
              bot_can_message_pub.can_data_receive_num_231 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_231 = 1;
            break;

          case 0x281:  // 第1电机温度、电流、电压
            bot_can_message_pub.can_id_281 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_281[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_281 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_281 ++;
            if(bot_can_message_pub.can_data_receive_num_281 > 5000){
              bot_can_message_pub.can_data_receive_num_281 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_281 = 1;
            break;
          case 0x282:  // 第2电机温度、电流、电压
            bot_can_message_pub.can_id_282 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_282[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_282 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_282 ++;
            if(bot_can_message_pub.can_data_receive_num_282 > 5000){
              bot_can_message_pub.can_data_receive_num_282 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_282 = 1;
            break;
          case 0x283:  // 第3电机温度、电流、电压
            bot_can_message_pub.can_id_283 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_283[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_283 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_283 ++;
            if(bot_can_message_pub.can_data_receive_num_283 > 5000){
              bot_can_message_pub.can_data_receive_num_283 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_283 = 1;
            break;
          case 0x284:  // 第4电机温度、电流、电压
            bot_can_message_pub.can_id_284 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_284[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_284 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_284 ++;
            if(bot_can_message_pub.can_data_receive_num_284 > 5000){
              bot_can_message_pub.can_data_receive_num_284 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_284 = 1;
            break;

          case 0x381:  // 第1电机心跳计数
            bot_can_message_pub.can_id_381 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_381[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_381 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_381 ++;
            if(bot_can_message_pub.can_data_receive_num_381 > 5000){
              bot_can_message_pub.can_data_receive_num_381 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_381 = 1;
            break;
          case 0x382:  // 第2电机心跳计数
            bot_can_message_pub.can_id_382 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_382[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_382 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_382 ++;
            if(bot_can_message_pub.can_data_receive_num_382 > 5000){
              bot_can_message_pub.can_data_receive_num_382 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_382 = 1;
            break;
          case 0x383:  // 第3电机心跳计数
            bot_can_message_pub.can_id_383 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_383[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_383 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_383 ++;
            if(bot_can_message_pub.can_data_receive_num_383 > 5000){
              bot_can_message_pub.can_data_receive_num_383 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_383 = 1;
            break;
          case 0x384:  // 第4电机心跳计数
            bot_can_message_pub.can_id_384 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_384[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_384 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_384 ++;
            if(bot_can_message_pub.can_data_receive_num_384 > 5000){
              bot_can_message_pub.can_data_receive_num_384 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_384 = 1;
            break;

          case 0x581:  // 第1电机驱动器SDO指令执行反馈
            bot_can_message_pub.can_id_581 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_581[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_581 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_581 ++;
            if(bot_can_message_pub.can_data_receive_num_581 > 5000){
              bot_can_message_pub.can_data_receive_num_581 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_581 = 1;
            break;
          case 0x582:  // 第2电机驱动器SDO指令执行反馈
            bot_can_message_pub.can_id_582 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_582[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_582 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_582 ++;
            if(bot_can_message_pub.can_data_receive_num_582 > 5000){
              bot_can_message_pub.can_data_receive_num_582 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_582 = 1;
            break;
          case 0x583:  // 第3电机驱动器SDO指令执行反馈
            bot_can_message_pub.can_id_583 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_583[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_583 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_583 ++;
            if(bot_can_message_pub.can_data_receive_num_583 > 5000){
              bot_can_message_pub.can_data_receive_num_583 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_583 = 1;
            break;
          case 0x584:  // 第4电机驱动器SDO指令执行反馈
            bot_can_message_pub.can_id_584 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_584[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_584 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_584 ++;
            if(bot_can_message_pub.can_data_receive_num_584 > 5000){
              bot_can_message_pub.can_data_receive_num_584 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_584 = 1;
            break;
          case 0x701:  // 第1电机NMT上线报文
            bot_can_message_pub.can_id_701 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_701[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_701 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_701 ++;
            if(bot_can_message_pub.can_data_receive_num_701 > 5000){
              bot_can_message_pub.can_data_receive_num_701 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_701 = 1;
            break;
          case 0x702:  // 第2电机NMT上线报文
            bot_can_message_pub.can_id_702 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_702[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_702 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_702 ++;
            if(bot_can_message_pub.can_data_receive_num_702 > 5000){
              bot_can_message_pub.can_data_receive_num_702 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_702 = 1;
            break;
          case 0x703:  // 第3电机NMT上线报文
            bot_can_message_pub.can_id_703 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_703[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_703 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_703 ++;
            if(bot_can_message_pub.can_data_receive_num_703 > 5000){
              bot_can_message_pub.can_data_receive_num_703 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_703 = 1;
            break;
          case 0x704:  // 第4电机NMT上线报文
            bot_can_message_pub.can_id_704 = frame_read.can_id;
            for (i = 0; i < 8; i++) {
               bot_can_message_pub.can_data_704[i] = frame_read.data[i];
            }
            bot_can_message_pub.can_data_length_704 = frame_read.can_dlc;
            bot_can_message_pub.can_data_receive_num_704 ++;
            if(bot_can_message_pub.can_data_receive_num_704 > 5000){
              bot_can_message_pub.can_data_receive_num_704 = 0;
            }
            bot_can_message_receive_pub.can_data_receive_704 = 1;
            break;

          default:
            break;
      }

      _bot_can_message_pub.publish(bot_can_message_pub);
      _bot_can_message_receive_pub.publish(bot_can_message_receive_pub);

      can_flag ++;
      if(can_flag > 5){
        InitCanReceiveFlag();
        can_flag = 0;
      }

    }
  }

  close(s);
  return 0;
}


BotCanReceive::BotCanReceive()
    : ModuleParams(nullptr),
      ScheduledWorkItem("bot_can_receive", px4::wq_configurations::bot_can_receive) {}

BotCanReceive::~BotCanReceive() {
  perf_free(_loop_perf);
  perf_free(_loop_interval_perf);
}

int BotCanReceive::task_spawn(int argc, char *argv[]) {
  BotCanReceive *instance = new BotCanReceive();
  if (instance) {
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}

int BotCanReceive::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int BotCanReceive::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("bot_can_receive", "BotCanReceive");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int bot_can_receive_main(int argc, char *argv[]) {
  PX4_INFO("bot can receive main start !!!!!!!!!!!!!");
  return BotCanReceive::main(argc, argv);
}
