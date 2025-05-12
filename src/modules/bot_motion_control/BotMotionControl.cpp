#include "BotMotionControl.hpp"

bool BotMotionControl::init() {
  ScheduleOnInterval(1000_us);
  return true;
}


void BotMotionControl::Run() {

  BotMotionControl_printnum ++;

  //为了不影响整个module的执行频率，通过计数设置日志输出频率
  if(BotMotionControl_printnum > LOG_RATE){
    //  is_log_time = true;
  }

  if (should_exit()) {
    ScheduleClear();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);
  perf_count(_loop_interval_perf);

  dt_us = BotControlDiffTime_US();

  CustomControlRate();

  UpdateData();

  DispatchCanMessage();

  MotorSafeCheck();

  HandleCanMessageInterval();

if(motor_is_safe && bot_run_control_sub.handle_brake == 0){
  if(is_log_time){
    // PX4_INFO("TEST bot_run_control_sub.control_mode=%d",bot_run_control_sub.control_mode);
    // PX4_INFO("TEST bot_run_control_sub.motion_mode=%d",bot_run_control_sub.motion_mode);
    PX4_INFO("bot_run_control_sub.linear_velocity=%d,bot_run_control_sub.angular_velocity=%d",bot_run_control_sub.linear_velocity,bot_run_control_sub.angular_velocity);
    // PX4_INFO("bot_run_control_sub.angular_velocity=%d",bot_run_control_sub.angular_velocity);
    PX4_INFO("4");
  }


    if (RUN_BODY_TYPE == BODY_TYPE_4D) {  // 4轮驱动
      bot_run_control_sub.robot_model = 0x04;
      if(bot_run_control_sub.is_can_control == 1 && bot_run_control_sub.control_mode == 1){
        motor_to_stop = false;
        // 限速处理
        BotSpeedAccDecSmooth();
        // 运动控制
        Bot4DControl();
        // 底盘当前状态
        Bot4DComputerSpeedAngle();
      }else if(bot_run_control_sub.is_can_control == 2 && bot_run_control_sub.control_mode == 2){
        motor_to_stop = false;
        // 限速处理
        BotSpeedAccDecSmooth();
        // 运动控制
        Bot4DControl();
        // 底盘当前状态
        Bot4DComputerSpeedAngle();
      }
      else{  //如果是待机模式或者设置为其他值，指令不起作用，车体为驻车状态
        motor_to_stop = true;
      }
    }
}

  if(motor_to_stop){
    BotMotorStop();
  }

  if(bot_run_control_sub.handle_brake == 1){
    BotMotorSetVToZero();
    //使用电流手刹
    BotHandleBrake();
    set_handler_brake_status_last = bot_run_control_sub.handle_brake;
  }else if(set_handler_brake_status_last == 1 && bot_run_control_sub.handle_brake == 0){//只有切换过刹车模式，才发送停止刹车指令，防止持续发送
    BotReleaseHandleBrake();
    InitBrakeCurrent();
    set_handler_brake_status_last = bot_run_control_sub.handle_brake;
  }

  PublishData();

  if(is_rate_200HZ){
      BotStateBatchReportByCan();
  }

  // if(ISCANFD){
  //   if(is_rate_200HZ){
  //     BotStateBatchReportByCanFD();
  //   }
  // }

  if(BotMotionControl_printnum > LOG_RATE){
      BotMotionControl_printnum = 0;
      is_log_time = false;
  }

  perf_end(_loop_perf);
}

void BotMotionControl::BotSpeedAccDecSmooth(){
   // 车体线速度，单位mm/s
  int16_t vx = bot_run_control_sub.linear_velocity;
  // 车体角速度，单位0.01°/s
  int16_t rz = bot_run_control_sub.angular_velocity;

  if ((vx > 3000) || (vx < -3000))  // cmd over speed alarm
  {
    // 是否超速
    bot_run_control_sub.over_speed = 1;  // 0:未超速；1:超速   超速记录状态，目前可以反馈到上位机
  } else {
    bot_run_control_sub.over_speed = 0;  // 0:未超速；1:超速
  }

    if (vx > LIMIT_LINE_SPEED) {
      vx = LIMIT_LINE_SPEED;
    } else if (vx < -1 * LIMIT_LINE_SPEED) {
      vx = -1 * LIMIT_LINE_SPEED;  // 限速1500mm/s
    }

    if (rz > LIMIT_ANGLE_SPEED){
      rz = LIMIT_ANGLE_SPEED;
    } else if (rz < -1 * LIMIT_ANGLE_SPEED){
      rz = -1 * LIMIT_ANGLE_SPEED;  // 限角速度80°/s
    }

    //根据加速度或者减速度，计算速度值
    //加速度和减速度的值，控制器设置默认值为0-1m/s加速和减速为100ms，上位机可以设置加速度或者减速度，上位机设置后按照上位机的设定值来执行
    uint16_t l_acc_time = bot_run_control_sub.acc_l_time;
    uint16_t l_dec_time = bot_run_control_sub.dec_l_time;
    uint16_t a_acc_time = bot_run_control_sub.acc_a_time;
    uint16_t a_dec_time = bot_run_control_sub.dec_a_time;

    if(l_acc_time == 0){
      l_acc_time = ACC_L_TIME;
    }
    if(l_dec_time == 0){
      l_dec_time = DEC_L_TIME;
    }
    if(a_acc_time == 0){
      a_acc_time = ACC_A_TIME;
    }
    if(a_dec_time == 0){
      a_dec_time = DEC_A_TIME;
    }

    //计算加速度和减速度具体值
    float line_acc = LIMIT_LINE_SPEED / ((l_acc_time * 1000)/dt_us);
    float line_dec = LIMIT_LINE_SPEED / ((l_dec_time * 1000) /dt_us);
    float angle_acc = LIMIT_ANGLE_SPEED / ((a_acc_time * 1000)/dt_us);
    float angle_dec = LIMIT_ANGLE_SPEED / ((a_dec_time * 1000) /dt_us);

    if (velocity_smooth < vx) {
        if ((velocity_smooth + line_acc) < vx) {
            velocity_smooth += line_acc;
        } else {
            velocity_smooth = vx;
        }
      } else if (velocity_smooth > vx) {
        if ((velocity_smooth - line_dec) > vx) {
            velocity_smooth -= line_dec;
        } else {
            velocity_smooth = vx;
        }
    }

    if (angular_smooth < rz) {
        if ((angular_smooth + angle_acc) < rz) {
            angular_smooth += angle_acc;
        } else {
            angular_smooth = rz;
        }
      } else if (angular_smooth > rz) {
        if ((angular_smooth - angle_dec) > rz) {
            angular_smooth -= angle_dec;
        } else {
            angular_smooth = rz;
        }
    }
}


void BotMotionControl::PublishData()
{
	bot_run_control_sub.body_angle = body_angle;

  // 发布更新后的控制模式、运动模式和速度值
  _bot_run_control_pub.publish(bot_run_control_sub);

  // 发布更新后的电机状态
  _bot_motor_control_pub.publish(bot_motor_control_sub);

  _bot_sensor_event_pub.publish(bot_sensor_event_sub);

}

void BotMotionControl::UpdateData() {
  if (_bot_sensor_event_sub.updated()) {
    _bot_sensor_event_sub.copy(&bot_sensor_event_sub);
  }

  if (_bot_can_message_sub.updated()) {
    _bot_can_message_sub.copy(&bot_can_message_sub);
  }

  if (_bot_can_message_receive_sub.updated()) {
    _bot_can_message_receive_sub.copy(&bot_can_message_receive_sub);
  }

  if (_bot_motor_control_sub.updated()) {
    _bot_motor_control_sub.copy(&bot_motor_control_sub);
  }

  if (_bot_run_control_sub.updated()) {
    _bot_run_control_sub.copy(&bot_run_control_sub);
  }

  if (_sensor_combined_sub.updated()) {
    _sensor_combined_sub.copy(&sensor_combined_sub);
  }

  if (_sensor_baro_sub_0.updated()) {
    _sensor_baro_sub_0.copy(&sensor_baro_sub_0);
  }
  if (_sensor_baro_sub_1.updated()) {
    _sensor_baro_sub_1.copy(&sensor_baro_sub_1);
  }

  if (_sensor_mag_sub.updated()) {
    _sensor_mag_sub.copy(&sensor_mag_sub);
  }

  if (_bot_distance_sensor_sub.updated()) {
    _bot_distance_sensor_sub.copy(&bot_distance_sensor_sub);
  }

  if (_bot_sensor_com_data_sub.updated()) {
    _bot_sensor_com_data_sub.copy(&bot_sensor_com_data_sub);
  }

  control_mode = bot_run_control_sub.control_mode;
  robot_model = bot_run_control_sub.robot_model; // 0x00:未指定; 0x02:两轮差速；0x03:阿克曼；0x04:四轮四驱
  linear_velocity = bot_run_control_sub.linear_velocity;
  angular_velocity = bot_run_control_sub.angular_velocity;
}

hrt_abstime last_time_us = 0;
uint64_t BotMotionControl::BotControlDiffTime_US(){
     hrt_abstime now_time_us = hrt_absolute_time();
     uint64_t diff_time_us = 0;
     if(now_time_us >= last_time_us){
      diff_time_us = now_time_us - last_time_us;
     }
    else{
      diff_time_us = 0xFFFFFFFFFFFFFFFF - last_time_us + now_time_us;
     }
     last_time_us = now_time_us;
     return diff_time_us;
  }



void BotMotionControl::CustomControlRate(){

    //对于200HZ频率，执行一次时间应该是1000 000/200 = 5000us，5000us/dt_us就是200HZ的计数
    uint16_t rate_number_200HZ = (uint16_t)(5000 / dt_us);
    control_rate_200HZ ++;
    if(control_rate_200HZ >= rate_number_200HZ){  //不到200HZ，大概在160HZ
      is_rate_200HZ = 1;
      control_rate_200HZ = 0;
    }else{
      is_rate_200HZ = 0;
    }

    //对于50HZ频率，执行一次时间应该是1000 000/50 = 20000us，20000us/dt_us就是200HZ的计数
    uint16_t rate_number_50HZ = (uint16_t)(20000 / dt_us);
    control_rate_50HZ ++;
    if(control_rate_50HZ >= rate_number_50HZ){  //不到200HZ，大概在160HZ
      is_rate_50HZ = 1;
      control_rate_50HZ = 0;
    }else{
      is_rate_50HZ = 0;
    }

  //以下可以添加其他频率...


}


extern "C" __EXPORT int bot_motion_control_main(int argc, char* argv[]) {
  return BotMotionControl::main(argc, argv);
}

BotMotionControl::BotMotionControl()
    : ModuleParams(nullptr),
      ScheduledWorkItem("bot_motion_control", px4::wq_configurations::bot_motion_control) {}

BotMotionControl::~BotMotionControl() {
  perf_free(_loop_perf);
  perf_free(_loop_interval_perf);
}

int BotMotionControl::task_spawn(int argc, char* argv[]) {
  BotMotionControl* instance = new BotMotionControl();

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

int BotMotionControl::custom_command(int argc, char* argv[]) {
  return print_usage("unknown command");
}

int BotMotionControl::print_usage(const char* reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description


)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("bot_motion_control", "system");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}
