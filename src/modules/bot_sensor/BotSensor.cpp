#include "BotSensor.hpp"

#define GPIO_EMERGENCY_STOP /* PH11, FMU_CH3 */ \
  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTH | GPIO_PIN11)
#define GPIO_FRONT_TOUCH_SENSOR /* PH10, FMU_CH4 */ \
  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTH | GPIO_PIN10)
#define GPIO_END_TOUCH_SENSOR /* PH9, FMU_CH8 */ \
  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTH | GPIO_PIN9)

static constexpr int D_PowerButton     = 1;
static constexpr int D_EmergencyButton = 2;
static constexpr int D_TouchSensorOne  = 3;
static constexpr int D_TouchSensorTwo  = 4;
static constexpr int D_Change  = 5;
static constexpr int D_Ultrasonic      = 8;
static constexpr int D_Fall_Distance   = 9;

static int D_Num  = 9;

static constexpr uint8_t DEVICE_ID_POWER_BUTTON{1};   /* 电源按钮设备ID */
static constexpr uint8_t DEVICE_ID_BATTERY_CHARGE{2}; /* 电源按钮设备ID */
static constexpr uint8_t DEVICE_ID_EMERGENCY_BUTTON{3}; /* 电源按钮设备ID */
static constexpr uint8_t DEVICE_ID_TOUCH_SENSOR1{4}; /* 电源按钮设备ID */
static constexpr uint8_t DEVICE_ID_TOUCH_SENSOR2{5}; /* 电源按钮设备ID */
static constexpr uint8_t DEVICE_ID_ULTRASONIC{8}; /* 超声波状态 */
static constexpr uint8_t DEVICE_ID_FALL_DISTANCE{9}; /* 超声波状态 */

static constexpr uint8_t DEVICE_ON{1};  /* 设备开 */
static constexpr uint8_t DEVICE_OFF{0}; /* 设备关 */

bool BotSensor::init() {
  ScheduleOnInterval(2000);
  // 急停按钮
  px4_arch_configgpio(GPIO_EMERGENCY_STOP);
  // 前碰撞传感器
  px4_arch_configgpio(GPIO_FRONT_TOUCH_SENSOR);
  // 后碰撞传感器
  px4_arch_configgpio(GPIO_END_TOUCH_SENSOR);

  return true;
}

void BotSensor::Run() {
	BotSensor_printnum ++;

	if(BotSensor_printnum > LOG_RATE){
	  // is_log_time = true;
	}

  if (should_exit()) {
    ScheduleClear();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);
  perf_count(_loop_interval_perf);

  // 更新数据
  UpdateData();

  // 检测急停按钮状态, 3.3V返回1，0V返回0
  uint8_t emergency_button_flag = px4_arch_gpioread(GPIO_EMERGENCY_STOP); //值为1时，急停按下；0时，非急停状态
  EmergencyButtonRequest(DEVICE_ID_EMERGENCY_BUTTON, emergency_button_flag);

  // 检测防撞条1状态
  uint8_t touch_sensor1_flag = bot_sensor_com_data_sub.touch_sensor1_flag;
  TouchSensor1Request(DEVICE_ID_TOUCH_SENSOR1, touch_sensor1_flag);

  // 检测防撞条2状态
  uint8_t touch_sensor2_flag = bot_sensor_com_data_sub.touch_sensor2_flag;
  TouchSensor2Request(DEVICE_ID_TOUCH_SENSOR2, touch_sensor2_flag);

   //防跌落传感器数据
   uint8_t fall_distance_flag = get_Fall_Distance_Status();
   FallDistanceSensorReuqest(DEVICE_ID_FALL_DISTANCE, fall_distance_flag);

  // 检测超声波状态
  // 超声波数据完全提报给上位机，如果上位机不关闭超声波并且在阈值范围内，停车；否则，不触发
    uint8_t ultrasonic_flag = get_ultrasonic_status(); //1:在安全距离内，非安全状态，机器人停止；0:在安全距离外，为安全状态，机器人运行
    UltrasonicReuqest(DEVICE_ID_ULTRASONIC,ultrasonic_flag);

   PublishData();

  if(BotSensor_printnum > LOG_RATE){
		BotSensor_printnum = 0;
		is_log_time = false;
	}

  perf_end(_loop_perf);
}

void BotSensor::UpdateData() {
  if (_bot_sensor_event_sub.updated()) {
    _bot_sensor_event_sub.copy(&bot_sensor_event_sub);
  }

  if (_bot_distance_sensor_sub.updated()) {
    _bot_distance_sensor_sub.copy(&bot_distance_sensor_sub);
  }

  if (_bot_sensor_com_data_sub.updated()) {
    _bot_sensor_com_data_sub.copy(&bot_sensor_com_data_sub);
  }

}


/*
急停按钮触发
*/
void BotSensor::EmergencyButtonRequest(uint8_t device_id, uint8_t event_flag) {
  bot_sensor_event_sub.emergency_button_flag = event_flag;
  if (event_flag == DEVICE_ON)  // 急停按钮按下
  {
    PublishSensorEvent(device_id,
                       bot_sensor_event_s::BOT_EVENT_EMERGENCY_BUTTON_ON,
                       event_flag, D_EmergencyButton);
  }
  else if (event_flag == DEVICE_OFF)  // 急停按钮释放
  {
    if (_emergency_button_prev_state == DEVICE_ON) {  // 如果急停按钮由触发转为释放则触发事件，否则不做处理
      PublishSensorEvent(device_id,
                         bot_sensor_event_s::BOT_EVENT_EMERGENCY_BUTTON_OFF,
                         event_flag, D_EmergencyButton);
    }
  }

  _emergency_button_prev_state = event_flag;
}
/*
接触传感器（防撞条）触发1
*/
void BotSensor::TouchSensor1Request(uint8_t device_id, uint8_t event_flag) {
  if (event_flag == DEVICE_ON)  // 接触传感器（防撞条）1触发
  {
    bot_sensor_event_sub.touch_sensor1_flag = event_flag;
    PublishSensorEvent(device_id, bot_sensor_event_s::BOT_EVENT_TOUCH_SENSOR_ON,
                       event_flag, D_TouchSensorOne);
    _touch_sensor1_prev_state = event_flag;
  }
  else if (event_flag == DEVICE_OFF)  // 接触传感器（防撞条）1释放
  {
    //如果由触发转为释放则等待5s后触发解除事件，否则不做处理.默认值为5s，如果有上位机设置时间，使用上位机设置的时间
    uint16_t realease_time = TOUCH_SENSOR_RELEASE_TIME;
    if(bot_sensor_event_sub.touch_sensor_release_time > 0){
      realease_time = bot_sensor_event_sub.touch_sensor_release_time;
    }

    if(_touch_sensor1_prev_state == DEVICE_ON){
      if (_touch_sensor1_release_wait_flag == 0){
        _touch_sensor1_release_start = hrt_absolute_time();
        _touch_sensor1_release_wait_flag = 1;
      }

      const hrt_abstime now = hrt_absolute_time();
      uint64_t diff_time_us = 0;
      if(now >= _touch_sensor1_release_start){
        diff_time_us = now - _touch_sensor1_release_start;
      }
      else{
        diff_time_us = 0xFFFFFFFFFFFFFFFF - _touch_sensor1_release_start + now;
      }

      if(diff_time_us > (realease_time * 1000000)){
          _touch_sensor1_release_start = 0;
          _touch_sensor1_release_wait_flag = 0;
          bot_sensor_event_sub.touch_sensor1_flag = event_flag;
          PublishSensorEvent(device_id,
                             bot_sensor_event_s::BOT_EVENT_TOUCH_SENSOR_OFF,
                             event_flag,D_TouchSensorOne);
      }
    }
  }
}
/*
接触传感器（防撞条）触发2
*/
void BotSensor::TouchSensor2Request(uint8_t device_id, uint8_t event_flag) {
  if (event_flag == DEVICE_ON)  // 接触传感器（防撞条）1触发
  {
    bot_sensor_event_sub.touch_sensor2_flag = event_flag;
    PublishSensorEvent(device_id, bot_sensor_event_s::BOT_EVENT_TOUCH_SENSOR_ON,
                       event_flag, D_TouchSensorTwo);
    _touch_sensor2_prev_state = event_flag;
  }
  else if (event_flag == DEVICE_OFF)  // 接触传感器（防撞条）1释放
  {
    //如果由触发转为释放则等待10s后触发解除事件，否则不做处理.默认值为10s，如果有上位机设置时间，使用上位机设置的时间
    uint16_t realease_time = TOUCH_SENSOR_RELEASE_TIME;
    if(bot_sensor_event_sub.touch_sensor_release_time > 0){
      realease_time = bot_sensor_event_sub.touch_sensor_release_time;
    }

    if(_touch_sensor2_prev_state == DEVICE_ON){
      if (_touch_sensor2_release_wait_flag == 0){
        _touch_sensor2_release_start = hrt_absolute_time();
        _touch_sensor2_release_wait_flag = 1;
      }

      const hrt_abstime now = hrt_absolute_time();
      uint64_t diff_time_us = 0;
      if(now >= _touch_sensor1_release_start){
        diff_time_us = now - _touch_sensor1_release_start;
      }
      else{
        diff_time_us = 0xFFFFFFFFFFFFFFFF - _touch_sensor1_release_start + now;
      }

      if(diff_time_us > (realease_time * 1000000)){
          _touch_sensor2_release_start = 0;
          _touch_sensor2_release_wait_flag = 0;
          bot_sensor_event_sub.touch_sensor2_flag = event_flag;
          PublishSensorEvent(device_id,
                             bot_sensor_event_s::BOT_EVENT_TOUCH_SENSOR_OFF,
                             event_flag,D_TouchSensorTwo);
      }
    }
  }

}

void BotSensor::FallDistanceSensorReuqest(uint8_t device_id, uint8_t event_flag){
  bot_sensor_event_sub.fall_distance_status = event_flag;
  if (event_flag == DEVICE_ON)
  {
    PublishSensorEvent(device_id, bot_sensor_event_s::BOT_EVENT_FALL_DISTANCE_ALARM_ON,
                       event_flag, D_Fall_Distance);
  } else if (event_flag == DEVICE_OFF)
  {
    PublishSensorEvent(device_id,
                       bot_sensor_event_s::BOT_EVENT_FALL_DISTANCE_ALARM_OFF,
                       event_flag, D_Fall_Distance);
  }
}

/*
超声波 触发
*/
void BotSensor::UltrasonicReuqest(uint8_t device_id, uint8_t event_flag) {
  bot_sensor_event_sub.ultrasonic_flag = event_flag;
  if (event_flag == DEVICE_ON)  // 超声波检测到安全距离内有障碍物
  {
    PublishSensorEvent(device_id, bot_sensor_event_s::BOT_EVENT_TOUCH_SENSOR_ON,
                       event_flag, D_Ultrasonic);

  }
  else if (event_flag == DEVICE_OFF)  // 超声波检测到安全距离内有无障碍物
  {
    PublishSensorEvent(device_id,
                        bot_sensor_event_s::BOT_EVENT_TOUCH_SENSOR_OFF,
                              event_flag, D_Ultrasonic);

  }
}

void BotSensor::PublishSensorEvent(uint8_t device_id, uint8_t event_type,uint8_t event_flag, int in_device_type)
{
 if(in_device_type > D_Num)
	 return;

  bot_sensor_event_sub.device_id = device_id;
  bot_sensor_event_sub.event_type = event_type;
  bot_sensor_event_sub.event_flag = event_flag;
}

void BotSensor::PublishData(){
  bot_sensor_event_sub.timestamp = hrt_absolute_time();
  _bot_sensor_event_pub.publish(bot_sensor_event_sub);
}

extern "C" __EXPORT int bot_sensor_main(int argc, char *argv[]) {
  return BotSensor::main(argc, argv);
}
BotSensor::BotSensor()
    : ModuleParams(nullptr),
      ScheduledWorkItem("bot_sensor", px4::wq_configurations::bot_sensor) {}
BotSensor::~BotSensor() {
  // close_uart();
  perf_free(_loop_perf);
  perf_free(_loop_interval_perf);
}
int BotSensor::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("bot_sensor", "BotSensor");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

int BotSensor::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int BotSensor::task_spawn(int argc, char *argv[]) {
  BotSensor *instance = new BotSensor();
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
