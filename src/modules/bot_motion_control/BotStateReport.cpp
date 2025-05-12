#include "BotMotionControl.hpp"
#include "SocketCAN.hpp"
#include <uORB/topics/bot_run_control.h>
#include <uORB/topics/bot_sensor_event.h>

extern int SocketCanSend(uint32_t can_id, uint8_t *message);
int16_t odometer_encode_last_node1 = 0;
int16_t odometer_encode_last_node2 = 0;
int16_t odometer_encode_last_node3 = 0;
int16_t odometer_encode_last_node4 = 0;
uint16_t odometer_encode_check = 0;

// uint16_t Test_200HZ_flag = 0;
void BotMotionControl::BotStateBatchReportByCan()
{
  message_size_id = 0; //can 计数清零
  message_size_data = 0;
  bool batch_report = true;

  HandleImuMessage_261_2(sensor_combined_sub,batch_report); //261: 0x105  262: 0x106
  //气压计以及温度数据的发布
  HandleBaroMessage_263(sensor_baro_sub_0, sensor_baro_sub_1,batch_report); //263:0x107
  // HandleBaroAvrMessage_264(sensor_baro_sub_0, sensor_baro_sub_1,batch_report); //平均值  264: 0x108  //暂不提供，用户可以根据263的数据自己计算
  //磁传感器数据及温度
  HandlerMagMessage_265(sensor_mag_sub,batch_report); //265:0x109

  // 控制模式反馈帧
  HandleCanMessage_101(batch_report);

  // 本体运动模型，反馈周期：20ms
  // HandleCanMessage_103(batch_report); //暂不提供，因为在烧录时已经固定了底盘的运动模型

  // 反馈本体线速度、角速度反馈
  HandleCanMessage_105(batch_report);

  // 传感器状态反馈，包含防撞条、急停、防跌落
  HandleCanMessage_107(batch_report);

  // 传感器状态反馈，超声波数据
  HandleCanMessage_108(batch_report);


  // 电机电压、电流、转速、温度数据反馈，暂不对上位机提供
  // HandleCanMessage_130X(NODE_1, batch_report);//131(0x83)
  // HandleCanMessage_130X(NODE_2, batch_report);//133(0x85)
  // if (RUN_BODY_TYPE >= 3) {  //135 0x87
  //   HandleCanMessage_130X(NODE_3, batch_report);
  // }
  // if (RUN_BODY_TYPE >= 4) {  //137 0x89
  //   HandleCanMessage_130X(NODE_4, batch_report);
  // }

  // HandleCanMessage_130(NODE_1, batch_report);//131(0x83)
  // HandleCanMessage_132(NODE_2, batch_report);//133(0x85)
  // if (RUN_BODY_TYPE >= 3) {  //135 0x87
  //   HandleCanMessage_134(NODE_3, batch_report);
  // }
  // if (RUN_BODY_TYPE >= 4) {  //137 0x89
  //   HandleCanMessage_136(NODE_4, batch_report);
  // }

  // 行驶轮线速度反馈
  HandleCanMessage_150(batch_report);  // 151

  // 行驶轮转向角反馈
  HandleCanMessage_152(batch_report);  //153

  // 行驶轮电机编码器脉冲值反馈
  // HandleCanMessage_154(batch_report); // 155  暂不提供，因为已经在19x中提供；并且脉冲值累积起来会比较大，该指令适用int16存储，可能不够，值会出现错误，并且脉冲值在19x中会定期发送，所以可以不使用请求的方式

  // 里程计数据反馈
  HandleCanMessage_190(NODE_1, batch_report);  //191
  HandleCanMessage_192(NODE_2, batch_report); //193
  if (RUN_BODY_TYPE >= 3) {  //195
    HandleCanMessage_194(NODE_3, batch_report);
  }
  if (RUN_BODY_TYPE >= 4) {  //197
    HandleCanMessage_196(NODE_4, batch_report);
  }

  HandleCanMessage_220();

  SocketCanBatchSend(can_id, can_data, message_size_id);
}

void BotMotionControl::BotStateBatchReportByCanFD(){

}

// /zt send_imu 加速度计以及陀螺仪数据
void BotMotionControl::HandleImuMessage_261_2(const sensor_combined_s& data,bool batch_report)
{
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  uint32_t can_id_t_acce = 0x261;

  //加速度计使用8个字节传输，X、Y分别扩大30000倍后，使用3个字节传输；Z数据扩大1500倍后，使用2个字节传输
  int16_t acc_x = static_cast<int16_t>(data.accelerometer_m_s2[0] * 30000);
  int16_t acc_y = static_cast<int16_t>(data.accelerometer_m_s2[1] * 30000);
  int16_t acc_z = static_cast<int16_t>(data.accelerometer_m_s2[2] * 1500);

  can_message[0] = static_cast<uint8_t>((acc_x >> 16) & 0x0000FF);
  can_message[1] = static_cast<uint8_t>((acc_x >> 8) & 0x0000FF);
  can_message[2] = static_cast<uint8_t>(acc_x & 0x0000FF);
  can_message[3] = static_cast<uint8_t>((acc_y >> 16) & 0x0000FF);
  can_message[4] = static_cast<uint8_t>((acc_y >> 8) & 0x0000FF);
  can_message[5] = static_cast<uint8_t>(acc_y & 0x0000FF);
  can_message[6] = static_cast<uint8_t>((acc_z >> 8) & 0x00FF);
  can_message[7] = static_cast<uint8_t>(acc_z & 0x00FF);

   if (batch_report) {
    AssembleCanData(can_id_t_acce, can_message);
  } else {
    SocketCanSend(can_id_t_acce, can_message);
  }

  uint8_t can_message_gyro[8] = {0,0,0,0,0,0,0,0};
  uint32_t can_id_t_gyro = 0x262;
  // 陀螺仪数据使用8个字节传输，X、Y分别扩大30000倍后，使用3个字节传输；Z数据扩大1500倍后，使用2个字节传输
  int16_t gyro_x = static_cast<int16_t>(data.gyro_rad[0] * 30000);
  int16_t gyro_y = static_cast<int16_t>(data.gyro_rad[1] * 30000);
  int16_t gyro_z = static_cast<int16_t>(data.gyro_rad[2] * 1500);

  can_message_gyro[0] = static_cast<uint8_t>((gyro_x >> 16) & 0x0000FF);
  can_message_gyro[1] = static_cast<uint8_t>((gyro_x >> 8) & 0x0000FF);
  can_message_gyro[2] = static_cast<uint8_t>(gyro_x & 0x0000FF);
  can_message_gyro[3] = static_cast<uint8_t>((gyro_y >> 16) & 0x0000FF);
  can_message_gyro[4] = static_cast<uint8_t>((gyro_y >> 8) & 0x0000FF);
  can_message_gyro[5] = static_cast<uint8_t>(gyro_y & 0x0000FF);
  can_message_gyro[6] = static_cast<uint8_t>((gyro_z >> 8) & 0x00FF);
  can_message_gyro[7] = static_cast<uint8_t>(gyro_z & 0x00FF);

  // if(is_log_time){
  //   PX4_INFO("TEST data.accelerometer_m_s2[0]=%lf,accelerometer_m_s2[1]=%lf,accelerometer_m_s2[2]=%lf",(double)(data.accelerometer_m_s2[0]),(double)(data.accelerometer_m_s2[1]),(double)(data.accelerometer_m_s2[2]));
  //   PX4_INFO("TEST gyro_rad[0] = %lf,gyro_rad[1] = %lf,gyro_rad[2] = %lf",(double)data.gyro_rad[0],(double)data.gyro_rad[1],(double)data.gyro_rad[2]);
  // }
  if (batch_report) {
    AssembleCanData(can_id_t_gyro, can_message_gyro);
  } else {
    SocketCanSend(can_id_t_gyro, can_message_gyro);
  }
}

//发布气压计以及温度信息
void BotMotionControl::HandleBaroMessage_263(const sensor_baro_s &data0, const sensor_baro_s &data1,bool batch_report)
{
    uint8_t can_message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t can_id_t = 0x263; // 自定义的 CAN ID

    // 气压计0的数据
    int16_t pressure0 = static_cast<int16_t>(data0.pressure / 100); // 假设需要将压力缩小100倍
    int16_t temperature0 = static_cast<int16_t>(data0.temperature * 100); // 假设需要将温度放大100倍

    can_message[0] = static_cast<uint8_t>(pressure0 >> 8);
    can_message[1] = static_cast<uint8_t>(pressure0 & 0xFF);
    can_message[2] = static_cast<uint8_t>(temperature0 >> 8);
    can_message[3] = static_cast<uint8_t>(temperature0 & 0xFF);

    // 气压计1的数据
    int16_t pressure1 = static_cast<int16_t>(data1.pressure / 100); // 假设需要将压力缩小100倍
    int16_t temperature1 = static_cast<int16_t>(data1.temperature * 100); // 假设需要将温度放大100倍

    can_message[4] = static_cast<uint8_t>(pressure1 >> 8);
    can_message[5] = static_cast<uint8_t>(pressure1 & 0xFF);
    can_message[6] = static_cast<uint8_t>(temperature1 >> 8);
    can_message[7] = static_cast<uint8_t>(temperature1 & 0xFF);

    // if(is_log_time){
    //   PX4_INFO("TEST data0.pressure = %lf,data0.temperature = %lf",(double)data0.pressure,(double)data0.temperature);
    //   PX4_INFO("TEST data1.pressure = %lf,data1.temperature = %lf",(double)data1.pressure,(double)data1.temperature);
    // }

    // 发送 CAN 消息
    if (batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }
}

void BotMotionControl::HandleBaroAvrMessage_264(const sensor_baro_s &data0, const sensor_baro_s &data1,bool batch_report)
{
    uint8_t can_message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t can_id_t = 0x264;

    float average_pressure = (data0.pressure + data1.pressure) / 2.0f;
    float average_temperature = (data0.temperature + data1.temperature) / 2.0f;
    // 将气压和温度数据进行缩放和量化
    int16_t pressure = static_cast<int16_t>(average_pressure / 100); // 假设需要将压力缩小100倍
    int16_t temperature = static_cast<int16_t>(average_temperature * 100); // 假设需要将温度放大100倍

    // 将数据打包到 CAN 消息中
    can_message[0] = static_cast<uint8_t>(pressure >> 8);
    can_message[1] = static_cast<uint8_t>(pressure & 0xFF);
    can_message[2] = static_cast<uint8_t>(temperature >> 8);
    can_message[3] = static_cast<uint8_t>(temperature & 0xFF);

    // if(is_log_time){
    //   PX4_INFO("TEST pressure = %d",pressure);
    //   PX4_INFO("TEST temperature = %d",temperature);
    // }

    // 发送 CAN 消息
    if (batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }
}

//发布磁传感器数据
void BotMotionControl::HandlerMagMessage_265(const sensor_mag_s &data,bool batch_report)
{
    uint8_t can_message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t can_id_t = 0x265; // 自定义的 CAN ID

    // 将磁场数据从 Gauss 转换为 milliGauss，并将温度数据放大100倍
    int16_t mag_x = static_cast<int16_t>(data.x * 1000);
    int16_t mag_y = static_cast<int16_t>(data.y * 1000);
    int16_t mag_z = static_cast<int16_t>(data.z * 1000);
    int16_t temperature = static_cast<int16_t>(data.temperature * 100); // 将温度放大100倍  TODOTODO

    // // 将数据打包到 CAN 消息中
    can_message[0] = static_cast<uint8_t>(mag_x >> 8);
    can_message[1] = static_cast<uint8_t>(mag_x & 0xFF);
    can_message[2] = static_cast<uint8_t>(mag_y >> 8);
    can_message[3] = static_cast<uint8_t>(mag_y & 0xFF);
    can_message[4] = static_cast<uint8_t>(mag_z >> 8);
    can_message[5] = static_cast<uint8_t>(mag_z & 0xFF);
    can_message[6] = static_cast<uint8_t>(temperature >> 8);
    can_message[7] = static_cast<uint8_t>(temperature & 0xFF);

    // if(is_log_time){
    //    PX4_INFO("TEST data.x = %lf,data.y = %lf,data.z = %lf",(double)data.x,(double)data.y,(double)data.y);
    // }

    // 发送 CAN 消息
    if (batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }
}

/***********************************************
*给电机发送运行命令
************************************************/
void  BotMotionControl::BotSendCmdToMotor(int in_flag)
{
  message_size_id = 0; //can 计数清零
  message_size_data = 0;
  bool cmd_flag = true;

  int speed_L = 0;  //左轮速度
  int speed_R = 0;  //右轮速度
  if(in_flag == 1) //急停防撞条未触发
  {
	speed_L = bot_run_control_sub.target_speed_rpm[0];
	speed_R = bot_run_control_sub.target_speed_rpm[1];
  }
  else //急停防撞条触发
  {
	speed_L = 0;
	speed_R = 0;
  }

  memset(can_data, 0x00, CAN_DATA_LENGTH);
  //左轮速度判断
  if((fabs(speed_L) >= 0) && (fabs(speed_L) <=160))//要求速度大于0，小于160转
  {
    HandleCanMessage_601(cmd_flag, speed_L);
  }
  else
  {
	  PX4_INFO("speed_L = %d, > 160, is lost", speed_L);
	  return;
  }
  //右轮速度判断
  if((fabs(speed_R) >= 0) && (fabs(speed_R) <= 160))
  {
    HandleCanMessage_602(cmd_flag, speed_R);
  }
  else
  {
	   PX4_INFO("speed_R = %d, > 160, is lost", speed_R);
	   return;
  }

}

void BotMotionControl::HandleCanMessge_iq(int node_id,int32_t iq){ //TODO 测试，为了提高频率暂时注释掉
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  uint32_t can_id_t = 0x600 + node_id;

  can_message[0] = 0x2B;
  can_message[1] = 0x67;
  can_message[2] = 0x60;
  can_message[3] = 0x00;

  can_message[4] = (uint8_t)(iq & 0x00FF);
  can_message[5] = (uint8_t)((iq >> 8) & 0x00FF);
	can_message[6] = 0x00;
  can_message[7] = 0x00;

  SocketCanSend(can_id_t, can_message);

  }

void BotMotionControl::HandleCanBatchMessge_iq(uint32_t iq_can_id,int32_t node_iq1,int32_t node_iq2){
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  uint32_t can_id_t = iq_can_id;

  can_message[0] = 0x2B;
  can_message[1] = 0x67;
  can_message[2] = 0x60;
  can_message[3] = 0x00;

  can_message[4] = (uint8_t)(node_iq1 & 0x00FF);
  can_message[5] = (uint8_t)((node_iq1 >> 8) & 0x00FF);
	can_message[6] = (uint8_t)(node_iq2 & 0x00FF);
  can_message[7] = (uint8_t)((node_iq2 >> 8) & 0x00FF);

  SocketCanSend(can_id_t, can_message);

}

void BotMotionControl::HandleCanBatchMessge_speed(uint32_t iq_can_id,int32_t node_1_speed,int32_t node_2_speed){
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  uint32_t can_id_t = iq_can_id;

  can_message[0] = 0x23;
  can_message[1] = 0xFF;
  can_message[2] = 0x60;
  can_message[3] = 0x00;

  can_message[4] = (uint8_t)(node_1_speed & 0x00FF);
  can_message[5] = (uint8_t)((node_1_speed >> 8) & 0x00FF);
	can_message[6] = (uint8_t)(node_2_speed & 0x00FF);
  can_message[7] = (uint8_t)((node_2_speed >> 8) & 0x00FF);

  SocketCanSend(can_id_t, can_message);

}


void BotMotionControl::AssembleCanData(uint32_t in_can_id, uint8_t* in_canData) {
  can_id[message_size_id] = in_can_id;
  message_size_id++;
  for (int i = 0; i < 8; i++) {
    can_data[message_size_data] = in_canData[i];
    message_size_data++;
  }
}

void BotMotionControl::SendBizCanMessage(uint32_t in_can_id, uint8_t* data) {
  // SocketCanSend(in_can_id, data);
}

// 控制模式设定
void BotMotionControl::HandleCanMessage_100(uint8_t* data) {
  // 控制模式值，0x00：待机模式； 0x01：Can指令模式； 0x02：遥控模式
  // 默认为待机模式
  uint8_t control_model = data[0];
    // 运动模式
  uint8_t motion_mode_t = data[1];
  // 是否设置手刹
  set_handle_brake = data[2];
  // 设置的手刹电流值
  set_handle_brake_value = (data[3] << 8) | data[4];

  // 判断数值是否合规
  if (control_model != bot_run_control_s::BOT_CONTROL_MODE_STANDBY &&
      control_model != bot_run_control_s::BOT_CONTROL_MODE_CAN &&
      control_model != bot_run_control_s::BOT_CONTROL_MODE_RC)
  {
    control_model = bot_run_control_s::BOT_CONTROL_MODE_STANDBY;
    PX4_ERR("ERROR: control_mode setup error.");
  }

  // 此命令为设置控制模式命令，无论待机还是can模式，为了安全考虑，都先将线速度、角速度均置为0；可以保持当前转向角度及运动模式
  bot_run_control_sub.control_mode = control_model;
  bot_run_control_sub.motion_mode = motion_mode_t;
  bot_run_control_sub.linear_velocity = 0;
  bot_run_control_sub.angular_velocity = 0;
  bot_run_control_sub.body_angle = 0;
  bot_run_control_sub.is_can_control = 1;
  bot_run_control_sub.handle_brake = set_handle_brake;

  // 反馈CAN
  HandleCanMessage_101(false);
}

// 控制模式反馈帧，canID:0x101，反馈周期：20ms
void BotMotionControl::HandleCanMessage_101(bool in_batch_report) {
  uint8_t can_message[8] = {0};
  // 控制模式
  can_message[0] = bot_run_control_sub.control_mode;
  // 运动模式
  can_message[1] = bot_run_control_sub.motion_mode;
  // 是否启用手刹
  can_message[2] = bot_run_control_sub.handle_brake;
    // 车体当前状态 TODO目前只设置了急停和防撞状态，需要再完善
  can_message[3] = bot_run_control_sub.bot_status;

  // if(is_log_time){
  //   PX4_INFO("TEST 101 bot_run_control_sub.control_mode=%d",bot_run_control_sub.control_mode);
  //   PX4_INFO("TEST 101 control_mode === %d,bot_status === %d",can_message[0],can_message[1]);
  // }

  uint32_t can_id_t = 0x101;
  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 本体运动模型设置
void BotMotionControl::HandleCanMessage_102(uint8_t data) {
  // 本体运动模型: 0x02:两轮差速；0x03:阿克曼；0x04:四轮四驱
  // 默认为待机模式
  uint8_t robot_model_t;

  robot_model_t = data;
  // 判断数值是否合规
  if ((robot_model_t != bot_run_control_s::BOT_ROBOT_MODEL_2WD) &
      (robot_model_t != bot_run_control_s::BOT_ROBOT_MODEL_4WD) &
      (robot_model_t != bot_run_control_s::BOT_ROBOT_MODEL_ACK)) {
    PX4_INFO("ERROR: robot_model setup error.");
  }
  // 此命令为设置控制模式命令，无论待机还是can模式，为了安全考虑，都先将线速度、角速度均置为0；可以保持当前转向角度及运动模式
  bot_run_control_sub.robot_model = robot_model_t;
  bot_run_control_sub.linear_velocity = 0;
  bot_run_control_sub.angular_velocity = 0;
  // 反馈CAN
  HandleCanMessage_103(false);
}

// 本体运动模型，反馈周期：20ms
void BotMotionControl::HandleCanMessage_103(bool in_batch_report) {
  uint8_t can_message[8] = {0};
  // 本体运动模型: 0x02:两轮差速；0x03:阿克曼；0x04:四轮四驱
  can_message[0] = bot_run_control_sub.robot_model;
  //运动模式,0x00 驻车档；0x01 前进；0x02 后退；0x03 斜移模式；0x04 自旋模式；0x05 横移模式；0x06 转向；0x07 空档
  can_message[1] = bot_run_control_sub.motion_mode;

  uint32_t can_id_t = 0x103;
  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
     SocketCanSend(can_id_t, can_message);
  }
}

// 查询本体线速度、角速度、转角
void BotMotionControl::HandleCanMessage_104() { HandleCanMessage_105(false); }

// 反馈本体线速度、角速度、转角反馈
void BotMotionControl::HandleCanMessage_105(bool in_batch_report) {
  uint8_t can_message[8] = {0};
  int16_t body_speed_report = bot_run_control_sub.body_speed_report;
  // 第0~1字节为本体报告线速度
  can_message[0] = (uint8_t)((body_speed_report >> 8) & 0x00FF);
  can_message[1] = (uint8_t)(body_speed_report & 0x00FF);

  int16_t angular_velocity_t = bot_run_control_sub.body_angular_velocity_report;
  // 第2~3字节为本体报告角速度
  can_message[2] = (uint8_t)((angular_velocity_t >> 8) & 0x00FF);
  can_message[3] = (uint8_t)(angular_velocity_t & 0x00FF);

  int8_t bot_status = bot_run_control_sub.bot_status;
  // 第6字节为本体状态
  can_message[6] = bot_status;

  int8_t over_speed = bot_run_control_sub.over_speed;
  // 第7字节为是否超速
  can_message[7] = over_speed;

  // if(is_log_time){
  //   PX4_INFO("TEST 105 body_speed_report=%d,angular_velocity_t=%d,bot_status=%d,over_speed=%d",body_speed_report,angular_velocity_t,bot_status,over_speed);
  // }

  uint32_t can_id_t = 0x105;
  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
     SocketCanSend(can_id_t, can_message);
  }
}

// 查询传感器状态反馈，如防撞条、急停
void BotMotionControl::HandleCanMessage_106() { HandleCanMessage_107(false); }

// 传感器状态反馈，包含防撞条、急停、防跌落传感器等数据
void BotMotionControl::HandleCanMessage_107(bool in_batch_report) {
  uint8_t can_message[8] = {0};

  uint8_t touch_sensor1_flag = bot_sensor_event_sub.touch_sensor1_flag;  // 接触传感器状态
  uint8_t touch_sensor2_flag = bot_sensor_event_sub.touch_sensor2_flag;  // 接触传感器状态
  uint8_t emergency_button_flag = bot_sensor_event_sub.emergency_button_flag;  // 接触传感器状态 急停
  uint8_t fall_distance_flag  = bot_sensor_event_sub.fall_distance_status; //防跌落传感器数据  防跌落整体状态

  can_message[0] = ((touch_sensor1_flag & 0xFF) << 3) + ((touch_sensor2_flag & 0xFF) << 2) + ((emergency_button_flag & 0xFF) << 1) + ((fall_distance_flag & 0xFF));
  can_message[1] = bot_sensor_com_data_sub.fall_distance1;
  can_message[2] = bot_sensor_com_data_sub.fall_distance2;
  can_message[3] = bot_sensor_com_data_sub.fall_distance3;
  can_message[4] = bot_sensor_com_data_sub.fall_distance4;

  uint32_t can_id_t = 0x107;
  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
     SocketCanSend(can_id_t, can_message);
  }
}

// 传感器状态反馈，包含防撞条、急停、防跌落传感器等数据
void BotMotionControl::HandleCanMessage_108(bool in_batch_report) {
  uint8_t can_message[8] = {0};

  uint16_t ultrasonic1 = bot_distance_sensor_sub.distance1;
  uint16_t ultrasonic2 = bot_distance_sensor_sub.distance2;
  uint16_t ultrasonic3 = bot_distance_sensor_sub.distance3;
  uint16_t ultrasonic4 = bot_distance_sensor_sub.distance4;

  can_message[0] = (uint8_t)((ultrasonic1 >> 8) & 0x00FF);
  can_message[1] = (uint8_t)(ultrasonic1 & 0x00FF);

  can_message[2] = (uint8_t)((ultrasonic2 >> 8) & 0x00FF);
  can_message[3] = (uint8_t)(ultrasonic2 & 0x00FF);

  can_message[4] = (uint8_t)((ultrasonic3 >> 8) & 0x00FF);
  can_message[5] = (uint8_t)(ultrasonic3 & 0x00FF);

  can_message[6] = (uint8_t)((ultrasonic4 >> 8) & 0x00FF);
  can_message[7] = (uint8_t)(ultrasonic4 & 0x00FF);

  uint32_t can_id_t = 0x108;
  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
     SocketCanSend(can_id_t, can_message);
  }
}


// 运动指令控制
void BotMotionControl::HandleCanMessage_114(uint8_t data[]) {
  // 车体线速度高八位
  uint8_t linear_velocity_h = data[0];
  // 车体线速度低八位
  uint8_t linear_velocity_l = data[1];
  // 车体角速度高八位
  uint8_t angular_velocity_h = data[2];
  // 车体角速度高八位
  uint8_t angular_velocity_l = data[3];

  bot_run_control_sub.linear_velocity = (linear_velocity_h << 8) | linear_velocity_l;
  bot_run_control_sub.angular_velocity = (angular_velocity_h << 8) | angular_velocity_l;
}

// 加减速时间设置
void BotMotionControl::HandleCanMessage_116(uint8_t data[]) {
  // 车体线速度加速到最大值时间高八位
  uint8_t acc_L_h = data[0];
  // 车体线速度加速到最大值时间低八位
  uint8_t acc_L_l = data[1];
  // 车体线速度从最大值减速到0时间高八位
  uint8_t dec_L_h = data[2];
  // 车体线速度从最大值减速到0时间低八位
  uint8_t dec_L_l = data[3];

  // 车体角速度加速到最大值时间高八位
  uint8_t acc_A_h = data[4];
  // 车体角速度加速到最大值时间低八位
  uint8_t acc_A_l = data[5];
  // 车体角速度从最大值减速到0时间高八位
  uint8_t dec_A_h = data[6];
  // 车体角速度从最大值减速到0时间低八位
  uint8_t dec_A_l = data[7];

  bot_run_control_sub.acc_l_time = (acc_L_h << 8) | acc_L_l;
  bot_run_control_sub.dec_l_time = (dec_L_h << 8) | dec_L_l;
  bot_run_control_sub.acc_a_time = (acc_A_h << 8) | acc_A_l;
  bot_run_control_sub.dec_a_time = (dec_A_h << 8) | dec_A_l;
}

  // 速度环KP KI设置
  void BotMotionControl::HandleCanMessage_117(uint8_t data[]){
  //  速度环KP设置值高八位
  uint8_t speed_kp_h = data[0];
  // 速度环KP设置值低八位
  uint8_t speed_kp_l = data[1];
  // 速度环KI设置值高八位
  uint8_t speed_ki_h = data[2];
  // 速度环KI设置值高八位
  uint8_t speed_ki_l = data[3];
  // // 速度环KD设置值高八位
  // uint8_t speed_kd_h = data[2];
  // // 速度环KD设置值高八位
  // uint8_t speed_kd_l = data[3];

  bot_run_control_sub.speed_kp = (speed_kp_h << 8) | speed_kp_l;
  bot_run_control_sub.speed_ki = (speed_ki_h << 8) | speed_ki_l;

  //为了实时响应，设置PID后，立即发送给驱动器
  HandleCanMessage_618(false);
  }

   void BotMotionControl::HandleCanMessage_618(bool batch_report){
    uint16_t drive_speed_kp = bot_run_control_sub.speed_kp;
    uint16_t drive_speed_ki = bot_run_control_sub.speed_ki;

    uint8_t can_message[8] = {0};

    can_message[0] = 0x23;
    can_message[1] = 0x18;
    can_message[2] = 0x60;
    can_message[3] = 0x00;

    can_message[4] = (uint8_t)(drive_speed_kp & 0x00FF);
    can_message[5] = (uint8_t)((drive_speed_kp >> 8) & 0x00FF);
    can_message[6] = (uint8_t)(drive_speed_ki & 0x00FF);
    can_message[7] = (uint8_t)((drive_speed_ki >> 8) & 0x00FF);

    uint32_t can_id_t = 0x618;
    if (batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
    SocketCanSend(can_id_t, can_message);
    }

   }

// 主控及伺服驱动器系统软硬件版本反馈
void BotMotionControl::HandleCanMessage_120(bool in_batch_report) {
  uint8_t can_message[8] = {0};
  int16_t hardware_version = bot_motor_control_sub.hardware_version[1];
  // 第0~1字节为驱动器硬件版本
  can_message[0] = (uint8_t)(hardware_version & 0x00FF);
  can_message[1] = (uint8_t)((hardware_version >> 8) & 0x00FF);

  int16_t software_version = bot_motor_control_sub.software_version[1];
  // 第2~3字节为驱动器软件版本
  can_message[2] = (uint8_t)(software_version & 0x00FF);
  can_message[3] = (uint8_t)((software_version >> 8) & 0x00FF);

  hardware_version = HARDWARE_VERSION;
  // 第4~5字节为主控硬件版本
  can_message[4] = (uint8_t)(hardware_version & 0x00FF);
  can_message[5] = (uint8_t)((hardware_version >> 8) & 0x00FF);

  software_version = SOFTWARE_VERSION;
  // 第6~7字节为电机温度
  can_message[6] = (uint8_t)(software_version & 0x00FF);
  can_message[7] = (uint8_t)((software_version >> 8) & 0x00FF);

  uint32_t can_id_t = 0x121;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
   SocketCanSend(can_id_t, can_message);
  }
}

// 电机电流、电压、温度、转速数据反馈
void BotMotionControl::HandleCanMessage_130X(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0};
  int16_t current = bot_motor_control_sub.motor_voltage[node_id];
  // 第0~1字节为驱动器电流
  can_message[0] = (uint8_t)(current & 0x00FF);
  can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

  int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
  // 第2~3字节为驱动器电压
  can_message[2] = (uint8_t)(voltage & 0x00FF);
  can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

  int16_t motor_temperature = bot_motor_control_sub.motor_temperature[node_id];
  // 第4~5字节为电机温度
  can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
  can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

  int16_t speed_rpm = bot_motor_control_sub.speed_rpm[node_id];
  // 第6~7字节为电机转速
  can_message[6] = (uint8_t)(speed_rpm & 0x00FF);
  can_message[7] = (uint8_t)((speed_rpm >> 8) & 0x00FF);

  uint32_t can_id_t = 0x130 + (node_id - 1) * 2 + 1;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 电机电流、电压、温度、转速数据反馈
void BotMotionControl::HandleCanMessage_130(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0};
  int16_t current = bot_motor_control_sub.motor_voltage[node_id];
  // 第0~1字节为驱动器电流
  can_message[0] = (uint8_t)(current & 0x00FF);
  can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

  int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
  // 第2~3字节为驱动器电压
  can_message[2] = (uint8_t)(voltage & 0x00FF);
  can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

  int16_t motor_temperature = bot_motor_control_sub.motor_temperature[node_id];
  // 第4~5字节为电机温度
  can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
  can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

  int16_t speed_rpm = bot_motor_control_sub.speed_rpm[node_id];
  // 第6~7字节为电机转速
  can_message[6] = (uint8_t)(speed_rpm & 0x00FF);
  can_message[7] = (uint8_t)((speed_rpm >> 8) & 0x00FF);

   uint32_t can_id_t = 0x131;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 电机电流、电压、温度、转速数据反馈
void BotMotionControl::HandleCanMessage_132(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0};
  int16_t current = bot_motor_control_sub.motor_voltage[node_id];
  // 第0~1字节为驱动器电流
  can_message[0] = (uint8_t)(current & 0x00FF);
  can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

  int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
  // 第2~3字节为驱动器电压
  can_message[2] = (uint8_t)(voltage & 0x00FF);
  can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

  int16_t motor_temperature = bot_motor_control_sub.motor_temperature[node_id];
  // 第4~5字节为电机温度
  can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
  can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

  int16_t speed_rpm = bot_motor_control_sub.speed_rpm[node_id];
  // 第6~7字节为电机转速
  can_message[6] = (uint8_t)(speed_rpm & 0x00FF);
  can_message[7] = (uint8_t)((speed_rpm >> 8) & 0x00FF);

  uint32_t can_id_t = 0x130 + (node_id - 1) * 2 + 1;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 电机电流、电压、温度、转速数据反馈
void BotMotionControl::HandleCanMessage_134(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0};
  int16_t current = bot_motor_control_sub.motor_voltage[node_id];
  // 第0~1字节为驱动器电流
  can_message[0] = (uint8_t)(current & 0x00FF);
  can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

  int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
  // 第2~3字节为驱动器电压
  can_message[2] = (uint8_t)(voltage & 0x00FF);
  can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

  int16_t motor_temperature = bot_motor_control_sub.motor_temperature[node_id];
  // 第4~5字节为电机温度
  can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
  can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

  int16_t speed_rpm = bot_motor_control_sub.speed_rpm[node_id];
  // 第6~7字节为电机转速
  can_message[6] = (uint8_t)(speed_rpm & 0x00FF);
  can_message[7] = (uint8_t)((speed_rpm >> 8) & 0x00FF);

  uint32_t can_id_t = 0x130 + (node_id - 1) * 2 + 1;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 电机电流、电压、温度、转速数据反馈
void BotMotionControl::HandleCanMessage_136(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0};
  int16_t current = bot_motor_control_sub.motor_voltage[node_id];
  // 第0~1字节为驱动器电流
  can_message[0] = (uint8_t)(current & 0x00FF);
  can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

  int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
  // 第2~3字节为驱动器电压
  can_message[2] = (uint8_t)(voltage & 0x00FF);
  can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

  int16_t motor_temperature = bot_motor_control_sub.motor_temperature[node_id];
  // 第4~5字节为电机温度
  can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
  can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

  int16_t speed_rpm = bot_motor_control_sub.speed_rpm[node_id];
  // 第6~7字节为电机转速
  can_message[6] = (uint8_t)(speed_rpm & 0x00FF);
  can_message[7] = (uint8_t)((speed_rpm >> 8) & 0x00FF);

  uint32_t can_id_t = 0x130 + (node_id - 1) * 2 + 1;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 灯光控制
void BotMotionControl::HandleCanMessage_140(uint8_t* data) {}

// 行驶轮线速度反馈
void BotMotionControl::HandleCanMessage_150(bool in_batch_report) {
  uint8_t can_message[8] = {0};
  int16_t axis_linear_velocity = 0;
  // int16_t wheel_speed1 = 0;
  // int16_t wheel_speed2 = 0;
  // int16_t wheel_speed3 = 0;
  // int16_t wheel_speed4 = 0;
  int node_id = 0;
  if (1 <= AXIS_COUNT) {
    node_id = 1;
    axis_linear_velocity = bot_run_control_sub.wheel_speed_mms[node_id];
    // wheel_speed1 = axis_linear_velocity;
    // 第0~1字节为第1电机节点信息
    can_message[0] = (uint8_t)((axis_linear_velocity >> 8) & 0x00FF);
    can_message[1] = (uint8_t)(axis_linear_velocity & 0x00FF);
  }

  if (2 <= AXIS_COUNT) {
    node_id = 2;
    axis_linear_velocity = bot_run_control_sub.wheel_speed_mms[node_id];
    // wheel_speed2 = axis_linear_velocity;
    // 第2~3字节为第2电机节点信息
    can_message[2] = (uint8_t)((axis_linear_velocity >> 8) & 0x00FF);
    can_message[3] = (uint8_t)(axis_linear_velocity & 0x00FF);
  }

  if (3 <= AXIS_COUNT) {
    node_id = 3;
    axis_linear_velocity = bot_run_control_sub.wheel_speed_mms[node_id];
    // wheel_speed3 = axis_linear_velocity;
    // 第4~5字节为第3电机节点信息
    can_message[4] = (uint8_t)((axis_linear_velocity >> 8) & 0x00FF);
    can_message[5] = (uint8_t)(axis_linear_velocity & 0x00FF);
  }

  if (4 <= AXIS_COUNT) {
    node_id = 4;
    axis_linear_velocity = bot_run_control_sub.wheel_speed_mms[node_id];
    // wheel_speed4 = axis_linear_velocity;
    // 第6~7字节为第4电机节点信息
    can_message[6] = (uint8_t)((axis_linear_velocity >> 8) & 0x00FF);
    can_message[7] = (uint8_t)(axis_linear_velocity & 0x00FF);
  }

  uint32_t can_id_t = 0x151;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 行驶轮转向角反馈
void BotMotionControl::HandleCanMessage_152(bool in_batch_report) {
  uint8_t can_message[8] = {0};
  int16_t axis_angle = 0;
  // int16_t wheel_pos1 = 0;
  // int16_t wheel_pos2 = 0;
  // int16_t wheel_pos3 = 0;
  // int16_t wheel_pos4 = 0;
  int node_id = 0;
  if (1 <= AXIS_COUNT) {
    node_id = 1;
    axis_angle = bot_run_control_sub.wheel_pos[node_id];
    // wheel_pos1 = axis_angle;
    // 第0~1字节为第1电机节点信息
    can_message[0] = (uint8_t)((axis_angle >> 8) & 0x00FF);
    can_message[1] = (uint8_t)(axis_angle & 0x00FF);
  }

  if (2 <= AXIS_COUNT) {
    node_id = 2;
    axis_angle = bot_run_control_sub.wheel_pos[node_id];
    // wheel_pos2 = axis_angle;
    // 第2~3字节为第2电机节点信息
    can_message[2] = (uint8_t)((axis_angle >> 8) & 0x00FF);
    can_message[3] = (uint8_t)(axis_angle & 0x00FF);
  }

  if (3 <= AXIS_COUNT) {
    node_id = 3;
    axis_angle = bot_run_control_sub.wheel_pos[node_id];
    // wheel_pos3 = axis_angle;
    // 第4~5字节为第3电机节点信息
    can_message[4] = (uint8_t)((axis_angle >> 8) & 0x00FF);
    can_message[5] = (uint8_t)(axis_angle & 0x00FF);
  }

  if (4 <= AXIS_COUNT) {
    node_id = 4;
    axis_angle = bot_run_control_sub.wheel_pos[node_id];
    // wheel_pos4 = axis_angle;
    // 第6~7字节为第4电机节点信息
    can_message[6] = (uint8_t)((axis_angle >> 8) & 0x00FF);
    can_message[7] = (uint8_t)(axis_angle & 0x00FF);
  }

  uint32_t can_id_t = 0x153;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 行驶轮电机编码器脉冲值反馈
void BotMotionControl::HandleCanMessage_154(bool in_batch_report) {
  uint8_t can_message[8] = {0};
  int16_t encoder_count = 0;
  int node_id = 0;
  if (1 <= AXIS_COUNT) {
    node_id = 1;
    encoder_count = bot_motor_control_sub.encoder_count[node_id];
    // 第0~1字节为第1电机节点信息
    can_message[0] = (uint8_t)(encoder_count & 0x00FF);
    can_message[1] = (uint8_t)((encoder_count >> 8) & 0x00FF);
  }

  if (2 <= AXIS_COUNT) {
    node_id = 2;
    encoder_count = bot_motor_control_sub.encoder_count[node_id];
    // 第2~3字节为第2电机节点信息
    can_message[2] = (uint8_t)(encoder_count & 0x00FF);
    can_message[3] = (uint8_t)((encoder_count >> 8) & 0x00FF);
  }

  if (3 <= AXIS_COUNT) {
    node_id = 3;
    encoder_count = bot_motor_control_sub.encoder_count[node_id];
    // 第4~5字节为第3电机节点信息
    can_message[4] = (uint8_t)(encoder_count & 0x00FF);
    can_message[5] = (uint8_t)((encoder_count >> 8) & 0x00FF);
  }

  if (4 <= AXIS_COUNT) {
    node_id = 4;
    encoder_count = bot_motor_control_sub.encoder_count[node_id];
    // 第6~7字节为第4电机节点信息
    can_message[6] = (uint8_t)(encoder_count & 0x00FF);
    can_message[7] = (uint8_t)((encoder_count >> 8) & 0x00FF);
  }

  uint32_t can_id_t = 0x155;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 里程计数据反馈
void BotMotionControl::HandleCanMessage_190(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  int32_t odometer_encode = bot_motor_control_sub.odometer_encode[node_id];
  // 第0~3字节为里程计总脉冲数
  can_message[0] = (uint8_t)((odometer_encode >> 24) & 0x000000FF);
  can_message[1] = (uint8_t)((odometer_encode >> 16) & 0x000000FF);
  can_message[2] = (uint8_t)((odometer_encode >> 8) & 0x000000FF);
  can_message[3] = (uint8_t)(odometer_encode & 0x000000FF);

  // int32_t odometer_meter = bot_motor_control_sub.odometer_meter[node_id];
  // 第4~7字节为里程计米制单位累计（0.001米）
  // can_message[4] = (uint8_t)(odometer_meter & 0x000000FF);
  // can_message[5] = (uint8_t)((odometer_meter >> 8) & 0x000000FF);
  // can_message[6] = (uint8_t)((odometer_meter >> 16) & 0x000000FF);
  // can_message[7] = (uint8_t)((odometer_meter >> 24) & 0x000000FF);

  uint32_t can_id_t = 0x191;

  in_batch_report = false;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 里程计数据反馈
void BotMotionControl::HandleCanMessage_192(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  int32_t odometer_encode = bot_motor_control_sub.odometer_encode[node_id];

  // 第0~3字节为里程计总脉冲数
  can_message[0] = (uint8_t)((odometer_encode >> 24) & 0x000000FF);
  can_message[1] = (uint8_t)((odometer_encode >> 16) & 0x000000FF);
  can_message[2] = (uint8_t)((odometer_encode >> 8) & 0x000000FF);
  can_message[3] = (uint8_t)(odometer_encode & 0x000000FF);

  // int32_t odometer_meter = bot_motor_control_sub.odometer_meter[node_id];
  // // 第4~7字节为里程计米制单位累计（0.001米）
  // can_message[4] = (uint8_t)(odometer_meter & 0x000000FF);
  // can_message[5] = (uint8_t)((odometer_meter >> 8) & 0x000000FF);
  // can_message[6] = (uint8_t)((odometer_meter >> 16) & 0x000000FF);
  // can_message[7] = (uint8_t)((odometer_meter >> 24) & 0x000000FF);

  uint32_t can_id_t = 0x193;

  in_batch_report = false;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 里程计数据反馈
void BotMotionControl::HandleCanMessage_194(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  int32_t odometer_encode = bot_motor_control_sub.odometer_encode[node_id];
  // 第0~3字节为里程计总脉冲数
  can_message[0] = (uint8_t)((odometer_encode >> 24) & 0x000000FF);
  can_message[1] = (uint8_t)((odometer_encode >> 16) & 0x000000FF);
  can_message[2] = (uint8_t)((odometer_encode >> 8) & 0x000000FF);
  can_message[3] = (uint8_t)(odometer_encode & 0x000000FF);

  // int32_t odometer_meter = bot_motor_control_sub.odometer_meter[node_id];
  // // 第4~7字节为里程计米制单位累计（0.001米）
  // can_message[4] = (uint8_t)(odometer_meter & 0x000000FF);
  // can_message[5] = (uint8_t)((odometer_meter >> 8) & 0x000000FF);
  // can_message[6] = (uint8_t)((odometer_meter >> 16) & 0x000000FF);
  // can_message[7] = (uint8_t)((odometer_meter >> 24) & 0x000000FF);
  uint32_t can_id_t = 0x190 + (node_id - 1) * 2 + 1;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

// 里程计数据反馈
void BotMotionControl::HandleCanMessage_196(int node_id, bool in_batch_report) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  int32_t odometer_encode = bot_motor_control_sub.odometer_encode[node_id];
  // 第0~3字节为里程计总脉冲数
  can_message[0] = (uint8_t)((odometer_encode >> 24) & 0x000000FF);
  can_message[1] = (uint8_t)((odometer_encode >> 16) & 0x000000FF);
  can_message[2] = (uint8_t)((odometer_encode >> 8) & 0x000000FF);
  can_message[3] = (uint8_t)(odometer_encode & 0x000000FF);

  // int32_t odometer_meter = bot_motor_control_sub.odometer_meter[node_id];
  // // 第4~7字节为里程计米制单位累计（0.001米）
  // can_message[4] = (uint8_t)(odometer_meter & 0x000000FF);
  // can_message[5] = (uint8_t)((odometer_meter >> 8) & 0x000000FF);
  // can_message[6] = (uint8_t)((odometer_meter >> 16) & 0x000000FF);
  // can_message[7] = (uint8_t)((odometer_meter >> 24) & 0x000000FF);
  uint32_t can_id_t = 0x190 + (node_id - 1) * 2 + 1;

  if (in_batch_report) {
    AssembleCanData(can_id_t, can_message);
  } else {
    SocketCanSend(can_id_t, can_message);
  }
}

 // 电机驱动器状态反馈
void BotMotionControl::HandleCanMessage_200(int node_id, bool in_batch_report)
{
    if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
    uint8_t can_message[8] = {0};
    int16_t current = bot_motor_control_sub.motor_voltage[node_id];
    // 第0~1字节为驱动器电流
    can_message[0] = (uint8_t)(current & 0x00FF);
    can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

    int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
    // 第2~3字节为驱动器电压
    can_message[2] = (uint8_t)(voltage & 0x00FF);
    can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

    int16_t motor_temperature =
        bot_motor_control_sub.motor_temperature[node_id];
    // 第4~5字节为电机温度
    can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
    can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

    uint8_t node_state = bot_motor_control_sub.node_state[node_id];
    // 第6字节为节点状态
    can_message[6] = node_state;

    uint32_t can_id_t = 0x200 + (node_id - 1) * 2 + 1;
    if (in_batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }
}

 // 电机驱动器状态反馈
void BotMotionControl::HandleCanMessage_202(int node_id, bool in_batch_report)
{
    if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
    uint8_t can_message[8] = {0};
    int16_t current = bot_motor_control_sub.motor_voltage[node_id];
    // 第0~1字节为驱动器电流
    can_message[0] = (uint8_t)(current & 0x00FF);
    can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

    int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
    // 第2~3字节为驱动器电压
    can_message[2] = (uint8_t)(voltage & 0x00FF);
    can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

    int16_t motor_temperature =
        bot_motor_control_sub.motor_temperature[node_id];
    // 第4~5字节为电机温度
    can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
    can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

    uint8_t node_state = bot_motor_control_sub.node_state[node_id];
    // 第6字节为节点状态
    can_message[6] = node_state;

    uint32_t can_id_t = 0x200 + (node_id - 1) * 2 + 1;
    if (in_batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }
}

 // 电机驱动器状态反馈
void BotMotionControl::HandleCanMessage_204(int node_id, bool in_batch_report)
{
    if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
    uint8_t can_message[8] = {0};
    int16_t current = bot_motor_control_sub.motor_voltage[node_id];
    // 第0~1字节为驱动器电流
    can_message[0] = (uint8_t)(current & 0x00FF);
    can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

    int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
    // 第2~3字节为驱动器电压
    can_message[2] = (uint8_t)(voltage & 0x00FF);
    can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

    int16_t motor_temperature =
        bot_motor_control_sub.motor_temperature[node_id];
    // 第4~5字节为电机温度
    can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
    can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

    uint8_t node_state = bot_motor_control_sub.node_state[node_id];
    // 第6字节为节点状态
    can_message[6] = node_state;

    uint32_t can_id_t = 0x200 + (node_id - 1) * 2 + 1;
    if (in_batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }
}

 // 电机驱动器状态反馈
void BotMotionControl::HandleCanMessage_206(int node_id, bool in_batch_report)
{
    if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
    uint8_t can_message[8] = {0};
    int16_t current = bot_motor_control_sub.motor_voltage[node_id];
    // 第0~1字节为驱动器电流
    can_message[0] = (uint8_t)(current & 0x00FF);
    can_message[1] = (uint8_t)((current >> 8) & 0x00FF);

    int16_t voltage = bot_motor_control_sub.motor_voltage[node_id];
    // 第2~3字节为驱动器电压
    can_message[2] = (uint8_t)(voltage & 0x00FF);
    can_message[3] = (uint8_t)((voltage >> 8) & 0x00FF);

    int16_t motor_temperature =
        bot_motor_control_sub.motor_temperature[node_id];
    // 第4~5字节为电机温度
    can_message[4] = (uint8_t)(motor_temperature & 0x00FF);
    can_message[5] = (uint8_t)((motor_temperature >> 8) & 0x00FF);

    uint8_t node_state = bot_motor_control_sub.node_state[node_id];
    // 第6字节为节点状态
    can_message[6] = node_state;

    uint32_t can_id_t = 0x200 + (node_id - 1) * 2 + 1;
    if (in_batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }
}

    // 根据查询指令，反馈电池BMS状态数据
    void BotMotionControl::HandleCanMessage_220() {
    // byte[0]	总电压高位		signed int16	单位：10mV
    // byte[1]	总电压低位
    // byte[2]	电流高位		signed int16	单位：10mA，通过电流判断电池充放电状态，充电为正，放电为负
    // byte[3]	电流低位
    // byte[4]	剩余容量高位		signed int16	单位：10mAh
    // byte[5]	剩余容量低位
    // byte[6]	电池剩余容量百分比		signed int16
    // byte[7]	充电开关状态	bit0	signed int8	1为打开，0为关闭；
    //   放电开关状态	bit1	signed int8	1为打开，0为关闭；
    //   充电过温保护状态	bit2	signed int8	0未保护，1发生保护
    //   充电低温保护状态	bit3	signed int8	0未保护，1发生保护
    //   充电过流保护状态	bit4	signed int8	0未保护，1发生保护
    //   放电过温保护状态	bit5	signed int8	0未保护，1发生保护
    //   放电低温保护状态	bit6	signed int8	0未保护，1发生保护
    //   放电过流保护状态	bit7	signed int8	0未保护，1发生保护

    int16_t voltage_total = bot_sensor_com_data_sub.voltage_total; //总电压 单位10mV  2byte
		int16_t current = bot_sensor_com_data_sub.current;  //电流  单位10mA  2byte
		int16_t capacity = bot_sensor_com_data_sub.capacity;  //剩余容量  单位10mAh  2byte
    int8_t rsoc = bot_sensor_com_data_sub.rsoc; //剩余容量百分比 1byte、

		//保护状态
    int8_t mos_charge_status = bot_sensor_com_data_sub.mos_charge_status; //充电状态 1：打开；0：关闭
		int8_t mos_discharge_status = bot_sensor_com_data_sub.mos_discharge_status; //放电状态 1:打开  0：关闭
		int8_t over_temperature_charge = bot_sensor_com_data_sub.over_temperature_charge; //充电过温 0未保护，1发生保护
		int8_t low_temperature_charge = bot_sensor_com_data_sub.low_temperature_charge;  //充电低温 0未保护，1发生保护
    int8_t over_current_charge = bot_sensor_com_data_sub.over_current_charge ; //充电过流 0未保护，1发生保护
		int8_t over_temperature_discharge = bot_sensor_com_data_sub.over_temperature_discharge; //放电过温 0未保护，1发生保护
		int8_t low_temperature_discharge = bot_sensor_com_data_sub.low_temperature_discharge;  //放电低温 0未保护，1发生保护
		int8_t over_current_discharge = bot_sensor_com_data_sub.over_current_discharge; //放电过流 0未保护，1发生保护

    uint8_t can_message[8] = {0};
    can_message[0] = (voltage_total >> 8) & 0x00FF;
    can_message[1] = voltage_total & 0x00FF;
    can_message[2] = (current >> 8) & 0x00FF;
    can_message[3] = current & 0x00FF;
    can_message[4] = (capacity >> 8) & 0x00FF;
    can_message[5] = capacity & 0x00FF;
    can_message[6] = rsoc;
    can_message[7] = ((mos_charge_status & 0xFF) >> 7) + ((mos_discharge_status & 0xFF) >> 6) + ((over_temperature_charge & 0xFF) >> 5) + ((low_temperature_charge & 0xFF) >> 4) + ((over_current_charge & 0xFF) >> 3) + ((over_temperature_discharge & 0xFF) >> 2) + ((low_temperature_discharge & 0xFF) >> 1) + (over_current_discharge & 0xFF);

    uint32_t can_id_t = 0x221;
    bool in_batch_report = false;
    if (in_batch_report) {
      AssembleCanData(can_id_t, can_message);
    } else {
      SocketCanSend(can_id_t, can_message);
    }

  }

















