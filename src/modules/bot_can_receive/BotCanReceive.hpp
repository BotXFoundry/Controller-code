#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <net/if.h>
#include <netpacket/can.h>
#include <nuttx/can.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <sys/select.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/bot_can_message.h>
#include <uORB/topics/bot_can_message_receive.h>

class BotCanReceive : public ModuleBase<BotCanReceive>,
                      public ModuleParams,
                      public px4::ScheduledWorkItem {
 public:
  BotCanReceive();
  ~BotCanReceive() override;
  /** @see ModuleBase */
  static int task_spawn(int argc, char* argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char* argv[]);

  /** @see ModuleBase */
  static int print_usage(const char* reason = nullptr);

  bool init();

 private:
  void Run() override;

  void InitCanReceiveFlag();
  int CanReceiveAndPublishMessage();

  uint32_t can_flag = 0;

  bot_can_message_s bot_can_message_pub{};
  bot_can_message_receive_s bot_can_message_receive_pub{};

  uORB::Publication<bot_can_message_s> _bot_can_message_pub{ORB_ID(bot_can_message)};
  uORB::Publication<bot_can_message_receive_s> _bot_can_message_receive_pub{ORB_ID(bot_can_message_receive)};

  // Performance (perf) counters
  perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
  perf_counter_t _loop_interval_perf{
      perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};
};
