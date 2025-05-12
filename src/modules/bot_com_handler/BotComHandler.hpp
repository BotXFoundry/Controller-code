#pragma once
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/topics/sensor_accel.h>
#include <px4_platform_common/px4_config.h>
#include <unistd.h> // write(), read(), close()
#include <termios.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <sys/select.h>
#include <termios.h>

#include "com_tx_rx.hpp"
#include <uORB/topics/bot_sensor_com_data.h>

using namespace time_literals;


class BotComHandler : public ModuleBase<BotComHandler>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BotComHandler();
	~BotComHandler() override;

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

	int ComHandler(int com_num);
	void Handle55(char read_data[]);
	void HandleDD(char read_data[]);
	void HandleTimeoutData();

	//防撞条数据，有两个防撞条，1:碰撞  0:未碰撞
	int8_t touch_sensor1 = 0;
	int8_t touch_sensor2 = 0;

	//防跌落数据，最多有4组防跌落，数据单位为cm，超过30cm，获取值为0x64；小于4cm，值为0x01
	//防跌落传感器获取的数据误差在2cm左右，大于30cm小于4cm的值都不太准确，所以设定固定值
	int8_t fall_sensor1 = 0;
	int8_t fall_sensor2 = 0;
	int8_t fall_sensor3 = 0;
	int8_t fall_sensor4 = 0;

	uint16_t handle55_num = 0;
	uint16_t handle55_01_num = 0;

	//电池状态数据
	int16_t voltage_total = 0; //总电压 单位10mV
	int16_t current = 0;  //电流  单位10mA
	int16_t capacity = 0;  //剩余容量  单位10mAh
	int16_t capacity_full_charge = 0; //实际满充容量  单位10mAh

	//保护状态
	int8_t over_voltage_unit = -1;  //单体过压 0未保护，1发生保护
	int8_t under_voltage_unit = -1;  //单体欠压 0未保护，1发生保护
	int8_t over_voltage_group = -1;  //整组过压 0未保护，1发生保护
	int8_t under_voltage_group = -1;  //整组欠压 0未保护，1发生保护
	int8_t over_temperature_charge = -1; //充电过温 0未保护，1发生保护
	int8_t low_temperature_charge = -1;  //充电低温 0未保护，1发生保护
	int8_t over_temperature_discharge = -1; //放电过温 0未保护，1发生保护
	int8_t low_temperature_discharge = -1;  //放电低温 0未保护，1发生保护
	int8_t over_current_charge = -1; //充电过流 0未保护，1发生保护
	int8_t over_current_discharge = -1; //放电过流 0未保护，1发生保护
	int8_t short_circuit_protection = -1; //短路保护 0未保护，1发生保护
	int8_t software_lock_mos = -1; //软件锁定MOS 0未保护，1发生保护

	int8_t rsoc = -1; //剩余容量百分比
	int8_t mos_charge_status = -1; //充电状态 1：打开；0：关闭
	int8_t mos_discharge_status = -1; //放电状态 1:打开  0：关闭

	bot_sensor_com_data_s bot_sensor_com_data_pub;

	uORB::Publication<bot_sensor_com_data_s> _bot_sensor_com_data_pub{ORB_ID(bot_sensor_com_data)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{ perf_alloc(PC_ELAPSED, MODULE_NAME": cycle") };
	perf_counter_t	_loop_interval_perf{ perf_alloc(PC_INTERVAL, MODULE_NAME": interval") };

};
