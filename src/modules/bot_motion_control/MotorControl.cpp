#include "BotMotionControl.hpp"

bool motor_start = false;
bool motor_init = false;

int8_t TX_ENABLE_ID = 0x00;

// 设置模式
void BotMotionControl::SetMotorRunMode(int16_t run_mode) {
  int max_axis_count = 4;
  if (AXIS_COUNT <= 4) max_axis_count = AXIS_COUNT;
  if (run_mode < 0 || run_mode > 3) return;  // 参数越界则不做处理
  for (int node_id = 1; node_id <= max_axis_count; node_id++) {
    // 运行模式，0：未定义，1：位置模式，2：速度模式,3:转矩模式
    //   默认：0
    UpdateSetRunMode(node_id, run_mode, true);
  }
}

// 使能也可以不设置，通过驱动器上电后自动使能，因为这里如果设置的话，在四轮中需要带两个驱动器，那同时需要设置驱动器的ID
void BotMotionControl::MotorControlEnable() {
  int max_axis_count = 4;
  if (AXIS_COUNT <= 4) max_axis_count = AXIS_COUNT;
  for (int node_id = 1; node_id <= max_axis_count; node_id++) {
    SendMotorCanMessage(node_id, TX_ENABLE_ID, 0, 0, 0);
  }
}

void util_truncate_number_abs(float *number, float max) {
  if (*number > max) {
    *number = max;
  } else if (*number < -max) {
    *number = -max;
  }
}


void BotMotionControl::BotMotorBatchSendSpeed(){
      //  /**适配中四轮 BX-07 及 四轮差速***/
      int32_t taget_speed_node_1 = bot_run_control_sub.target_speed_rpm[NODE_1];
      int32_t taget_speed_node_2 = bot_run_control_sub.target_speed_rpm[NODE_2];
      int32_t taget_speed_node_3 = bot_run_control_sub.target_speed_rpm[NODE_3];
      int32_t taget_speed_node_4 = bot_run_control_sub.target_speed_rpm[NODE_4];

       uint32_t can_31_id = 0x631;
       HandleCanBatchMessge_speed(can_31_id,taget_speed_node_3,taget_speed_node_1);
       uint32_t can_24_id = 0x624;
       HandleCanBatchMessge_speed(can_24_id,taget_speed_node_2,taget_speed_node_4);
}


void BotMotionControl::MotorSafeCheck()
{
  int unsafe = 0;
  emergency_button_status = bot_sensor_event_sub.emergency_button_flag;  // 急停状态，1:触发急停；0:未触发
  touch_sensor1_status = bot_sensor_event_sub.touch_sensor1_flag;  // 前防撞条，1:触发防撞；0:未触发
  touch_sensor2_status = bot_sensor_event_sub.touch_sensor2_flag;  // 后防撞条，1:触发防撞；0:未触发
  fall_distance_status = bot_sensor_event_sub.fall_distance_status; //防跌落，1:触发; 0:未触发
  ultrasonic_status = bot_sensor_event_sub.ultrasonic_flag; //1:在超声波安全范围内有障碍物，需要停车；0：在超声波安全范围内无障碍物  //超声波是否启用，由上位机控制，如果上位机设置关闭，则不启用，只向上位机传数据,在车体倒退时，如果超声波触发，是否起作用，看用户配置

  if(bot_run_control_sub.linear_velocity < 0){
    if(bot_sensor_event_sub.back_ultrasonic_is_close == 1){
      ultrasonic_status = 0;
    }
  }

  // power_button_status = bot_sensor_event_sub.power_flag;  // 电源按钮状态，1:开；0:关 ---20241019 电源开关不作为安全考量范围
  // charge_status = bot_sensor_event_sub.charge_flag; // 充电状态  ---20241019 充电状态不作为安全考量范围

  // if(is_log_time){
  //   PX4_INFO("TEST emergency_button_status=%d,touch_sensor1_status=%d,touch_sensor2_status=%d,fall_distance_status=%d,ultrasonic_status=%d",emergency_button_status,touch_sensor1_status,touch_sensor2_status,fall_distance_status,ultrasonic_status);
  //   PX4_INFO("TEST ultrasonic_status=%d",ultrasonic_status);
  // }

  unsafe = (touch_sensor1_status || touch_sensor2_status|| emergency_button_status || fall_distance_status || ultrasonic_status);

    if(unsafe)
    {
      motor_is_safe = false;
      motor_to_stop = true;
      bot_run_control_sub.bot_status = 0x01;
    }else{
      motor_is_safe = true;
      motor_to_stop = false;
      bot_run_control_sub.bot_status = 0x00;
    }
}

void BotMotionControl::BotMotorStop(){
  bot_run_control_sub.linear_velocity = 0;
  bot_run_control_sub.angular_velocity = 0;
  //将速度设置为0，发送给驱动器
  bot_run_control_sub.target_speed_rpm[NODE_1] = 0;
  bot_run_control_sub.target_speed_rpm[NODE_2] = 0;
  bot_run_control_sub.target_speed_rpm[NODE_3] = 0;
  bot_run_control_sub.target_speed_rpm[NODE_4] = 0;
  BotMotorBatchSendSpeed();
}

void BotMotionControl::BotMotorSetVToZero(){
  bot_run_control_sub.linear_velocity = 0;
  bot_run_control_sub.angular_velocity = 0;
  //将速度设置为0，发送给驱动器
  bot_run_control_sub.target_speed_rpm[NODE_1] = 0;
  bot_run_control_sub.target_speed_rpm[NODE_2] = 0;
  bot_run_control_sub.target_speed_rpm[NODE_3] = 0;
  bot_run_control_sub.target_speed_rpm[NODE_4] = 0;
}

void BotMotionControl::InitBrakeCurrent(){
  node1_brake_current = MOTOR_BRAKE_CURRENT;
  node2_brake_current = MOTOR_BRAKE_CURRENT;
  node3_brake_current = MOTOR_BRAKE_CURRENT;
  node4_brake_current = MOTOR_BRAKE_CURRENT;
  brake_current = MOTOR_BRAKE_CURRENT;
}

void BotMotionControl::BotHandleBrake(){

  int16_t handle_brake_current = MOTOR_BRAKE_CURRENT * 1000;

  if(bot_run_control_sub.is_can_control == 1){  //can模式
    handle_brake_current = set_handle_brake_value;
  }
  uint16_t brake_mode = 0x05;
  //发送刹车can指令
   UpdateSetBrakeMode(brake_mode,handle_brake_current);
}

void BotMotionControl::BotReleaseHandleBrake(){
   int16_t handle_brake_current = 0;
   //发送刹车can指令
   uint16_t brake_mode = 0x00;
   UpdateSetBrakeMode(brake_mode,handle_brake_current);
}

// 以下方法为设置或获取驱动器参数 目前有些没有实现
// 向驱动器发送业务内容为2字节的消息
bool BotMotionControl::SendMotorCanMessage(int node_id, uint8_t cmd,
                                           uint16_t index_code,
                                           uint8_t sub_index_code,
                                           uint16_t message)
{
  bool ret = true;
  // TODO:实现往伺服驱动器发送CAN消息的逻辑
  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  can_message[0] = cmd;
  can_message[1] = (uint8_t)(index_code & 0x000000FF);
  can_message[2] = (uint8_t)((index_code >> 8) & 0x0000FF);
  can_message[3] = sub_index_code;
  can_message[4] = (uint8_t)(message & 0x000000FF);
  can_message[5] = (uint8_t)((message >> 8) & 0x0000FF);
  can_message[6] = (uint8_t)((message >> 16) & 0x00FF);
  can_message[7] = (uint8_t)message >> 24;

  uint32_t can_id_t = 0x600 + node_id;

  SocketCanSend(can_id_t, can_message);
  return ret;
}

// 设置 通讯掉线保护时间 范围0~32767
bool BotMotionControl::UpdateDisConnectProtectionTime(int node_id,
                                                      uint16_t message,
                                                      bool is_setup_cmd) {
  bool ret = false;
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if ((message < 0) || (message > 32767)) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }

  uint16_t index_code = 0x2000;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.disconnect_protection_time[node_id] = message;
  return ret;
}

bool BotMotionControl::UpdateFeedbackPositionReset(int node_id,
                                                   uint16_t message,
                                                   bool is_setup_cmd) {
  bool ret = false;
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if ((message < 0) || (message > 32767)) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2005;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.feedback_position_reset[node_id] = message;
  return ret;
}

// 0x2006H 绝对位置模式时当前位置清零  0：无效
// 1：当前位置清零
bool BotMotionControl::UpdateAbsolutePositionModeResetToZero(
    int node_id, uint16_t message, bool is_setup_cmd) {
  bool ret = false;
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if ((message < 0) || (message > 1)) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2006;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.absolute_position_mode_reset_to_zero[node_id] = message;
  return ret;
}

// 限位停车方式 0:停止 1：急停 2：无效
bool BotMotionControl::UpdateRestrictedParkingMethods(int node_id,
                                                      uint16_t message,
                                                      bool is_setup_cmd) {
  bool ret = false;
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if ((message < 0) || (message > 2)) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2007;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.restricted_parking_methods[node_id] = message;
  return ret;
}

// 起始速度 单位r/min  范围1-300r/min 默认1
bool BotMotionControl::UpdateStartingSpeed(int node_id, uint16_t message,
                                           bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 300) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2008;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.starting_speed[node_id] = message;
  return ret;
}

// 0x200AH 电机最大转速 单位r/min  范围1-300r/min
bool BotMotionControl::UpdateMaximumSpeed(int node_id, uint16_t message,
                                          bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 300) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x200A;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.maximum_speed[node_id] = message;
  return ret;
}

// 编码器线数设置 0-4096 默认1024
bool BotMotionControl::UpdateLineEncoder(int node_id, uint16_t message,
                                         bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 4096) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x200B;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.line_encoder[node_id] = message;
  return ret;
}

// 电机极对数,15
bool BotMotionControl::UpdatePolePairs(int node_id, uint16_t message,
                                       bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 4 || message > 64) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x200C;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.pole_pairs[node_id] = message;
  return ret;
}

// 上电锁轴方式 0：不使能不锁轴，1：不使能锁轴
bool BotMotionControl::UpdateLockBearingMode(int node_id, uint16_t message,
                                             bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 1) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x200F;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.init_lock_axis_mode[node_id] = message;
  return ret;
}

// 电机与HALL的偏移角度 单位1度 范围-360度~360度
bool BotMotionControl::UpdateHallOffsetAngle(int node_id, int16_t message,
                                             bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < -360 || message > 360) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2011;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.hall_offset_angle[node_id] = message;
  return ret;
}

// 过载系数  范围0-300,单位%  默认200
bool BotMotionControl::UpdateOverloadFactor(int node_id, uint16_t message,
                                            bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 300) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2012;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.overload_factor[node_id] = message;
  return ret;
}

// 电机温度保护阈值 单位0.1度
// 范围0-1200(*0.1) 默认800
bool BotMotionControl::UpdateTemperatureProtectionThreshold(int node_id,
                                                            uint16_t message,
                                                            bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 1200) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2013;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.temperature_protection_threshold[node_id] = message;
  return ret;
}

// 额定电流  驱动器输出的额定电流 单位0.1A 范围0-150
bool BotMotionControl::UpdateLimitedCurrent(int node_id, uint16_t message,
                                            bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 150) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2014;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.limited_current[node_id] = message;
  return ret;
}

// 最大电流 驱动器输出的最大电流  单位0.1A 范围0-300
bool BotMotionControl::UpdateMaximumCurrent(int node_id, uint16_t message,
                                            bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 300) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2015;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.maximum_current[node_id] = message;
  return ret;
}

// 过载保护时间 驱动器过载保护时间 单位10ms 范围0-6553 缺省300
bool BotMotionControl::UpdateOverloadProtectionTime(int node_id,
                                                    uint16_t message,
                                                    bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 6553) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2016;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.overload_protection_time[node_id] = message;
  return ret;
}

//// 超差报警阈值 编码器超差阈值
//  单位*10counts 范围1-6553 缺省409
bool BotMotionControl::UpdateOverToleranceAlarmThreshold(int node_id,
                                                         uint16_t message,
                                                         bool is_setup_cmd) {
  bool ret = false;
 // if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 1 || message > 6553) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2017;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.encoder_diff_alarm_threshold[node_id] = message;
  return ret;
}

// 过度平滑系数 0-30000  缺省1000
bool BotMotionControl::UpdateSmoothingFactor(int node_id, uint16_t message,
                                             bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2018;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.smoothing_factor[node_id] = message;
  return ret;
}

// 电流环比例系数 0-30000   缺省600
bool BotMotionControl::UpdateCurrentLoopScaleFactor(int node_id,
                                                    uint16_t message,
                                                    bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2019;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.foc_current_kp[node_id] = message;
  return ret;
}

// 电流环积分增益 0-30000  缺省300
bool BotMotionControl::UpdateCurrentLoopIntegralGain(int node_id,
                                                     uint16_t message,
                                                     bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x201A;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.foc_current_ki[node_id] = message;
  return ret;
}

// 前馈输出平滑系数 0-30000 缺省100
bool BotMotionControl::UpdateFeedforwardSmoothingFactor(int node_id,
                                                        uint16_t message,
                                                        bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x201B;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.feed_forward_smoothing_factor[node_id] = message;
  return ret;
}

// 转矩输出平滑系数 0-30000 缺省100
bool BotMotionControl::UpdateTorqueOutputSmoothingFactor(int node_id,
                                                         uint16_t message,
                                                         bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x201C;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.duty_output_smoothing_factor[node_id] = message;
  return ret;
}

// 速度比例增益Kp 0-30000 缺省500
bool BotMotionControl::UpdateSpeedKp(int node_id, uint16_t message,
                                     bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x201D;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.s_pid_kp[node_id] = message;
  return ret;
}

// 速度积分增益Ki 0-30000 缺省100
bool BotMotionControl::UpdateSpeedKi(int node_id, uint16_t message,
                                     bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x201E;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.s_pid_ki[node_id] = message;
  return ret;
}

// 设置 速度微分增益Kd 0-30000 缺省1000
bool BotMotionControl::UpdateSpeedKd(int node_id, uint16_t message,
                                     bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x201F;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.s_pid_kd[node_id] = message;
  return ret;
}

// 位置比例增益Kp 0-30000 缺省50
bool BotMotionControl::UpdatePositionKp(int node_id, uint16_t message,
                                        bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2020;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.p_pid_kp[node_id] = message;
  return ret;
}

// 位置积分增益Kf 0-30000
bool BotMotionControl::UpdatePositionKi(int node_id, uint16_t message,
                                        bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2021;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.p_pid_ki[node_id] = message;
  return ret;
}

// 位置微分增益Kd 0-30000
bool BotMotionControl::UpdatePositionKd(int node_id, uint16_t message,
                                        bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x2022;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.p_pid_kd[node_id] = message;
  return ret;
}

// 软件版本   出厂默认
bool BotMotionControl::GetSoftwareVersion(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x2025;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.software_version[node_id] = message;
  return ret;
}

// 电机温度 单位0.1度 范围0-120度  默认800
bool BotMotionControl::GetMotorTemperature(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x2026;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.motor_temperature[node_id] = message;
  return ret;
}

// 霍尔输入状态 0-7 如果出现0或7  为霍尔出错 默认0
bool BotMotionControl::GetHallSector(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x2028;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.hall_sector[node_id] = message;
  return ret;
}

// 母线电压 单位0.01V  默认0
bool BotMotionControl::GetBusVoltage(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x2029;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.motor_voltage[node_id] = message;
  return ret;
}

// 报警PWM处理方式 0：关闭 1：开启
bool BotMotionControl::UpdatePWMAlarmMethod(int node_id, uint16_t message,
                                            bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 1) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x202A;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.pwm_alarm_method[node_id] = message;
  return ret;
}

// 过载处理方式 0：关闭 1：开启
bool BotMotionControl::UpdateOverloadHandlingMethod(int node_id,
                                                    uint16_t message,
                                                    bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 1) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x202B;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.overload_handling_method[node_id] = message;
  return ret;
}

// 驱动器最近一次故障码
bool BotMotionControl::GetLastFaultCode(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x2000;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.last_fault_code[node_id] = message;
  return ret;
}

//  控制字
bool BotMotionControl::UpdateControlWord(int node_id, uint16_t message,
                                         bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 16) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x603F;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.control_word[node_id] = message;
  return ret;
}

// 状态字
bool BotMotionControl::GetStatusWord(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x6040;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.status_word[node_id] = message;
  return ret;
}

// 快速停止代码 快速停止命令后驱动器处理方式 默认5
//   5:正常停止，维持快速停止状态，6：急减速停，维持快速停止状态，7：急停，维持快速停止状态
bool BotMotionControl::UpdateQuickStopCode(int node_id, uint16_t message,
                                           bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 7) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x605A;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.quick_stop_code[node_id] = message;
  return ret;
}

// 关闭操作代码 关闭命令后驱动器处理方式 0：无效
//  1：正常停止，转到Ready to switch on状态，默认1
bool BotMotionControl::UpdateCloseActionCode(int node_id, uint16_t message,
                                             bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 1) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x605B;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.close_action_code[node_id] = message;
  return ret;
}

// 禁用操作代码  禁用操作命令后驱动器处理方式 0：无效
//   1：正常停止，转到Switched On 状态 默认1
bool BotMotionControl::UpdateDisableActionCode(int node_id, uint16_t message,
                                               bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 1) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x605C;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.disable_action_code[node_id] = message;
  return ret;
}

// Halt控制寄存器  控制字Halt命令后驱动器处理方式
//  1：正常停止，维持Operation
//  Enabled状态，2：急速减停，维持Operation
//  Enabled状态，3：急停，维持Operation
//  Enabled状态，默认：1
bool BotMotionControl::UpdateHaltControlRegister(int node_id, uint16_t message,
                                                 bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 3) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x605D;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.halt_control_register[node_id] = message;
  return ret;
}

// 运行模式，0：未定义，1：位置模式，2：速度模式,3:转矩模式
//   默认：0
bool BotMotionControl::UpdateSetRunMode(int node_id, uint16_t message,
                                        bool is_setup_cmd) {
  bool ret = false;
  uint8_t cmd;
  if (is_setup_cmd) {
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x6060;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.run_mode[node_id] = message;
  return ret;
}

//解决can发送后在驱动器端接收丢包的问题，将发送合并到一个数据帧中 ----20241218
bool BotMotionControl::UpdateSetBrakeMode(uint16_t brake_mode,int16_t send_brake_current) {
  bool ret = false;
  // uint8_t cmd = 0x2B ;
  uint8_t cmd = 0x23 ;
  uint16_t index_code = 0x6068;
  uint8_t sub_index_code = 0x00;
  // uint16_t message = 0x05;

  uint8_t can_message[8] = {0,0,0,0,0,0,0,0};
  can_message[0] = cmd;
  can_message[1] = (uint8_t)(index_code & 0x000000FF);
  can_message[2] = (uint8_t)((index_code >> 8) & 0x0000FF);
  can_message[3] = sub_index_code;

  can_message[4] = (uint8_t)(brake_mode & 0x000000FF);
  can_message[5] = (uint8_t)((brake_mode >> 8) & 0x0000FF);
  //在发送刹车指令的同时，将统一的刹车电流发送给驱动器，将刹车电流扩大1000倍后发送
  can_message[6] = (uint8_t)(send_brake_current & 0x000000FF);;
  can_message[7] = (uint8_t)((send_brake_current >> 8) & 0x0000FF);

  uint32_t can_id_t = 0x615;
  SocketCanSend(can_id_t, can_message);

  for (int node_id = 1; node_id <= AXIS_COUNT; node_id++) {
      //设置手刹状态
    bot_motor_control_sub.run_mode[node_id] = brake_mode;
  }
  return ret;
}



// 运行模式状态
//  0：未定义，1：位置模式，2：速度模式,3:转矩模式，默认：0
bool BotMotionControl::GetRunningModeStatus(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x6061;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.run_mode[node_id] = message;
  return ret;
}

// 实际位置反馈  单位counts 默认：0
bool BotMotionControl::GetActualPosition(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x6064;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.actual_position[node_id] = message;
  return ret;
}

// 实际速度反馈 电机当前运行速度 单位0.1r/min 默认：0
bool BotMotionControl::GetActualSpeed(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x606C;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.speed_rpm[node_id] = message;
  return ret;
}

// 目标转矩 单位mA,范围-30000~30000 默认：0
bool BotMotionControl::UpdateTargetTorque(int node_id, int16_t message,
                                          bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < -30000 || message > 30000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x6071;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.target_torque[node_id] = message;
  return ret;
}

// 实时转矩反馈 单位0.1A,范围-300~300 默认：0
bool BotMotionControl::GetRealTimeTorque(int node_id) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd = 0x40;
  uint16_t index_code = 0x6077;
  uint8_t sub_index_code = 0x00;
  uint16_t message = 0;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.real_time_torque[node_id] = message;
  return ret;
}

// 设置 目标位置 位置模式运行总脉冲数范围：-1000000~1000000
// 此处存在Uint32往int32转换的兼容性问题
bool BotMotionControl::UpdateTargetPosition(int node_id, int32_t message,
                                            bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < -1000000 || message > 1000000) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x607A;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  //bot_motor_control_sub.target_position[node_id] = CMD.BizData32;
  bot_motor_control_sub.target_position[node_id] = message;
  return ret;
}

// 最大速度 位置模式时的最大速度 范围1-300r/min
// 默认：120r/min
bool BotMotionControl::UpdateMaxSpeed(int node_id, uint16_t message,
                                      bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 300) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x6081;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.max_speed_rpm[node_id] = message;
  return ret;
}

// 设置 位置模式启/停速度  范围1-300r/min
//  默认1r/min
bool BotMotionControl::UpdatePositionModeStartStopSpeed(int node_id,
                                                        uint16_t message,
                                                        bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 300) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x6082;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.position_mode_start_stop_speed[node_id] = message;
  return ret;
}

// 急停减速时间 范围0~32767ms 缺省10ms
bool BotMotionControl::UpdateEmergencyStopDecelerationTime(int node_id,
                                                           uint16_t message,
                                                           bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 32767) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x6085;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.emergency_stop_time[node_id] = message;
  return ret;
}

// 转矩斜率 电流/1000/second   单位mA/s  缺省300ms
bool BotMotionControl::UpdateTorqueGradient(int node_id, uint16_t message,
                                            bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    //if (message < 0 || message > 32767) return false;
    cmd = 0x2B;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x6087;
  uint8_t sub_index_code = 0x00;
  ret = SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.torque_gradient[node_id] = message;
  return ret;
}

// 目标速度  速度模式时的目标速度  范围:-300~300r/min
//  缺省0
// 此处存在Uint32往int32转换的兼容性问题
bool BotMotionControl::UpdateTargetSpeed(int node_id, int16_t message,
                                         bool is_setup_cmd) {
  bool ret = false;
  //if (node_id >= AXIS_COUNT && node_id <= 0) return ret;
  uint8_t cmd;
  if (is_setup_cmd) {
    // if (message < -300 || message > 300) return false;
    // cmd = 0x2B;//TODO test 先注释掉
    cmd = 0x23;
  } else {
    cmd = 0x40;
  }
  uint16_t index_code = 0x60FF;
  uint8_t sub_index_code = 0x00;
  SendMotorCanMessage(node_id, cmd, index_code, sub_index_code, message);
  bot_motor_control_sub.target_speed_rpm[node_id] = message;
  return ret;
}
