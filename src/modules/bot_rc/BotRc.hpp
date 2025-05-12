#pragma once
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/bot_motor_control.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/bot_run_control.h>
#include <uORB/topics/bot_sensor_event.h>

#define BODY_TYPE_4D 4

#define RUN_BODY_TYPE BODY_TYPE_4D

#define RC_PI 3.14159265358979323846

//设置遥控控制线速度最大值
#define RC_LINEAR_VELOCITY 1500  //单位：0.001m/s
#define RC_BODY_ANGLE 8000
#define RC_ANGULAR_VELOCITY 8000  //单位：0.01/s


#define LOG_RATE 333

using namespace time_literals;

class BotRc : public ModuleBase<BotRc>,
              public ModuleParams,
              public px4::ScheduledWorkItem {
 public:
  BotRc();
  ~BotRc() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char* argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char* argv[]);

  /** @see ModuleBase */
  static int print_usage(const char* reason = nullptr);

  bool init();

 private:
  void Run() override;
  void ManualControl();
  void UpdateData();
  void ManualControl4D(void);
  void PublishData();

  int16_t BotRC_printnum = 0; //记录日志频率
  bool is_log_time = false;

  input_rc_s input_rc{};
  bot_run_control_s bot_run_control_sub{};
  rc_channels_s rc_channels{};
  bot_sensor_event_s bot_sensor_event_sub{};

  uORB::Subscription _input_rc_sub{
      ORB_ID(input_rc)};  // regular subscription for additional data
  uORB::Subscription _bot_run_control_sub{ORB_ID(bot_run_control)};
  uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
  uORB::Subscription _bot_sensor_event_sub{ORB_ID(bot_sensor_event)};

  uORB::Publication<bot_sensor_event_s> _bot_sensor_event_pub{
      ORB_ID(bot_sensor_event)};
  uORB::Publication<bot_run_control_s> _bot_run_control_pub{
      ORB_ID(bot_run_control)};

  perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
  perf_counter_t _loop_interval_perf{
      perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};
};
