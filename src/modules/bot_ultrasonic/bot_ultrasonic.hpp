#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

#include <uORB/topics/bot_sensor_event.h>
#include <uORB/topics/bot_distance_sensor.h>

using namespace time_literals;

class BotUltrasonic : public ModuleBase<BotUltrasonic>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    BotUltrasonic();
    ~BotUltrasonic() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();

    int print_status() override;

private:
    void Run() override;

    void UpdateData();

    // Initialize UART
    bool init_uart(const char *device,int baud_rate);

    // Set work mode to polling
    void set_work_mode();

    // Read sensor data
    void read_data();

    // Close UART
    void close_uart();

    // Calculate CRC for Modbus
    uint16_t calculate_crc(uint8_t *buffer, int length);

    // UART file descriptor
    int uart_fd;

    bot_sensor_event_s bot_sensor_event_sub;

    uORB::Subscription _bot_sensor_event_sub{ORB_ID(bot_sensor_event)};
    uORB::Publication<bot_sensor_event_s> _bot_sensor_event_pub{ORB_ID(bot_sensor_event)};

    bot_distance_sensor_s bot_distance_sensor_pub;
    uORB::Publication<bot_distance_sensor_s> _bot_distance_sensor_pub{ORB_ID(bot_distance_sensor)};

    // Performance (perf) counters
    perf_counter_t _loop_perf{ perf_alloc(PC_ELAPSED, MODULE_NAME": cycle") };
    perf_counter_t _loop_interval_perf{ perf_alloc(PC_INTERVAL, MODULE_NAME": interval") };

};
