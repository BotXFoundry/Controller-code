
#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <termios.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

#include <uORB/topics/bot_motor_control.h>
#include <uORB/topics/bot_run_control.h>
#include <uORB/topics/bot_sensor_event.h>
#include <uORB/topics/bot_distance_sensor.h>
#include <uORB/topics/bot_sensor_com_data.h>


#define LOG_RATE 333

class BotSensor : public ModuleBase<BotSensor>,
                  public ModuleParams,
                  public px4::ScheduledWorkItem {
 public:
  BotSensor();
  ~BotSensor() override;
  /** @see ModuleBase */
  static int task_spawn(int argc, char* argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char* argv[]);

  /** @see ModuleBase */
  static int print_usage(const char* reason = nullptr);

  bool init();

 private:
  void Run() override;
  void UpdateData();
  void PublishData();
  void EmergencyButtonRequest(uint8_t device_id, uint8_t event_flag);
  void TouchSensor1Request(uint8_t device_id, uint8_t event_flag);
  void TouchSensor2Request(uint8_t device_id, uint8_t event_flag);
  void PublishSensorEvent(uint8_t device_id, uint8_t event_type,uint8_t event_flag, int in_device_type);
  void UltrasonicReuqest(uint8_t device_id, uint8_t event_flag);
  void FallDistanceSensorReuqest(uint8_t device_id, uint8_t event_flag);

  int uart_fd;

  bool init_uart(const char *device, int baud_rate);
  void read_IO_Sensor_data();
  void close_uart();

  static constexpr uint16_t DISTANCE_FREE_MM = 300; //单位：mm
  static constexpr uint8_t FALL_DISTANCE_CM = 20; //单位：cm
  static constexpr uint8_t TOUCH_SENSOR_RELEASE_TIME = 5; //单位：s

  uint8_t get_ultrasonic_status();
  uint8_t get_Fall_Distance_Status();

  uint8_t read_uart_rate = 0; //读取超声波计数，将读取频率降低到5HZ

  int16_t BotSensor_printnum = 0; //记录日志频率
  bool is_log_time = false;

  uint8_t _power_button_prev_state{0};
  uint8_t _battery_charge_prev_state{0};
  uint8_t _emergency_button_prev_state{0};
  uint8_t _touch_sensor1_prev_state{0};
  uint8_t _touch_sensor2_prev_state{0};

  hrt_abstime _touch_sensor1_release_start{0};
  hrt_abstime _touch_sensor2_release_start{0};
  int _touch_sensor1_release_wait_flag{0};
  int _touch_sensor2_release_wait_flag{0};

  bot_sensor_event_s bot_sensor_event_sub;
  bot_sensor_com_data_s bot_sensor_com_data_sub;
  bot_distance_sensor_s bot_distance_sensor_sub;

  uORB::Subscription _bot_sensor_event_sub{ORB_ID(bot_sensor_event)};
  uORB::Subscription _bot_sensor_com_data_sub{ORB_ID(bot_sensor_com_data)};
  uORB::Subscription _bot_distance_sensor_sub{ORB_ID(bot_distance_sensor)};

  uORB::Publication<bot_sensor_event_s> _bot_sensor_event_pub{
      ORB_ID(bot_sensor_event)};


  perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
  perf_counter_t _loop_interval_perf{
      perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};
};
