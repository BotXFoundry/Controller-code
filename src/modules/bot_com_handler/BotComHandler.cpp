/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *Time: 20241129 修改读取长度方式，实现接收数据的正确率
 *               发送端修改发送方式，防撞条、防跌落数据异常时，不发送电池数据，提高解析数据正确率
 ****************************************************************************/

#include "BotComHandler.hpp"

    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2---Using this one to RX/TX rs485
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */

int* ult_dist;
int obstacle_distance = 300 ;
int tty_num = 4; //open serial number


bool BotComHandler::init()
{
	 ScheduleOnInterval(2000_us);
	return true;
}

void BotComHandler::Run()
{

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	ComHandler(tty_num);

	perf_end(_loop_perf);
}

int BotComHandler::ComHandler(int com_num){
    char data;
	int uart_read =0;
	int ret = 0;
	int ret_timeout = 0;
	fd_set readfds;
	struct timeval com_timeout;

	char read_data[64];

    uart_read = com4_init(com_num);
    if (false == set_uart_baudrate(uart_read, 115200))
	{
		PX4_INFO("[JXF]set_uart_baudrate is failed");
		return -1;
	}

	com_timeout.tv_sec = 2; //set timeout 2s
	com_timeout.tv_usec = 0;

	while(1)
	{
	       FD_ZERO(&readfds);
	       FD_SET(uart_read, &readfds);

		ret_timeout = select(uart_read+1, &readfds, NULL, NULL, &com_timeout);
		if(ret_timeout == -1)
		{
			continue;
		}
		else if(ret_timeout == 0)
		{
			continue;
		}

	    memset(read_data, 0x00, sizeof(read_data));
		ret = 0;
		ret = read(uart_read, &data, 1);;
		if(ret == 0)
		{
			continue;
		}
		if(ret == 1)
		{
			if(data == 0x55)
			{
				read_data[0] = data;
				ret = Handle_Read_55(uart_read, &read_data[1]);
				if(ret == 0)
				{
					PX4_INFO("55: ret = 0, test handle read 55 error");
					continue;
				}
				else
				{
					Handle55(read_data);
				}
			}
			else if(data == 0xDD)
			{
				read_data[0] = data;
				ret = Handle_Read_DD(uart_read, &read_data[1]);
				if(ret == 0)
				{
					PX4_INFO("dd: read ret = 0, test DD error!");
					continue;
				}
				else
				{
					HandleDD(read_data);
				}
			}
			else
			{
				continue;
			}
		}
        _bot_sensor_com_data_pub.publish(bot_sensor_com_data_pub);

	}

    close(uart_read);
	return 0;

}

//串口没有数据时，对防撞条、红外测距进行初始设置
void BotComHandler::HandleTimeoutData()
{
	//防撞条_红外测距初始化
	bot_sensor_com_data_pub.touch_sensor1_flag = 0;
	bot_sensor_com_data_pub.touch_sensor2_flag = 0;
	bot_sensor_com_data_pub.fall_distance1 = 0;
	bot_sensor_com_data_pub.fall_distance2 = 0;
	bot_sensor_com_data_pub.fall_distance3 = 0;
	bot_sensor_com_data_pub.fall_distance4 = 0;

	//电池初始化
	bot_sensor_com_data_pub.voltage_total = -1; //总电压 单位10mV  2byte
	bot_sensor_com_data_pub.current = -1;  //电流  单位10mA  2byte
	bot_sensor_com_data_pub.capacity = -1;  //剩余容量  单位10mAh  2byte
	bot_sensor_com_data_pub.capacity_full_charge = -1; //实际满充容量，即电池的标称容量  单位10mAh 2byte
	//保护状态
	bot_sensor_com_data_pub.over_voltage_unit = -1;  //单体过压 0未保护，1发生保护
	bot_sensor_com_data_pub.under_voltage_unit = -1;  //单体欠压 0未保护，1发生保护
	bot_sensor_com_data_pub.over_voltage_group = -1;  //整组过压 0未保护，1发生保护
	bot_sensor_com_data_pub.under_voltage_group = -1;  //整组欠压 0未保护，1发生保护
	bot_sensor_com_data_pub.over_temperature_charge = -1; //充电过温 0未保护，1发生保护
	bot_sensor_com_data_pub.low_temperature_charge = -1;  //充电低温 0未保护，1发生保护
	bot_sensor_com_data_pub.over_temperature_discharge = -1; //放电过温 0未保护，1发生保护
	bot_sensor_com_data_pub.low_temperature_discharge = -1;  //放电低温 0未保护，1发生保护
	bot_sensor_com_data_pub.over_current_charge = -1; //充电过流 0未保护，1发生保护
	bot_sensor_com_data_pub.over_current_discharge = -1; //放电过流 0未保护，1发生保护
	bot_sensor_com_data_pub.short_circuit_protection = -1; //短路保护 0未保护，1发生保护
	bot_sensor_com_data_pub.software_lock_mos = -1; //软件锁定MOS 0未保护，1发生保护
	bot_sensor_com_data_pub.rsoc = -1; //剩余容量百分比 1byte
	bot_sensor_com_data_pub.mos_charge_status = -1; //充电状态 1：打开；0：关闭
	bot_sensor_com_data_pub.mos_discharge_status = -1; //放电状态 1:打开  0：关闭

	_bot_sensor_com_data_pub.publish(bot_sensor_com_data_pub);
	// PX4_INFO("touch_sensor1_flag = %d", bot_sensor_com_data_pub.touch_sensor1_flag);
        // PX4_INFO("touch_sensor2_flag = %d", bot_sensor_com_data_pub.touch_sensor2_flag);
}


//解析55的数据，包含防撞条、防跌落传感器信息
//read_data的数据示例：0x55 0x05 0x01 0x02 0x00 0x00 0xFF 0xFD 0xAA
//                   0x55 0x05 0x02 0x04 0x64 0x64 0x64 0x64 0xFE 0x6A 0xAA
//                   起始位 固定字节 类型 长度 数据内容 检验 结束位
void BotComHandler::Handle55(char read_data[])
{
        handle55_num ++;
	if(read_data[2] == 0x01)  //防撞条数据
	{
		handle55_01_num ++;
		touch_sensor1 = read_data[4];
		touch_sensor2 = read_data[5];
		handle55_num=0;
	}

	// 防撞条在触发的时候发送数据，为了能及时更新数据，在8次没有收到防撞条数据时，认为没有触发
	if(handle55_num >= 8){
		handle55_01_num = 0;
		touch_sensor1 = 0;
		touch_sensor2 = 0;
	}

	if(handle55_num < 8 && handle55_01_num == 0){
		touch_sensor1 = 0;
		touch_sensor2 = 0;
	}

	if(read_data[2] == 0x02)  //防跌落数据
	{
		fall_sensor1 = read_data[4];
		fall_sensor2 = read_data[5];
		fall_sensor3 = read_data[6];
		fall_sensor4 = read_data[7];
	}

	bot_sensor_com_data_pub.touch_sensor1_flag = touch_sensor1;
	bot_sensor_com_data_pub.touch_sensor2_flag = touch_sensor2;
	bot_sensor_com_data_pub.fall_distance1 = fall_sensor1;
	bot_sensor_com_data_pub.fall_distance2 = fall_sensor2;
	bot_sensor_com_data_pub.fall_distance3 = fall_sensor3;
	bot_sensor_com_data_pub.fall_distance4 = fall_sensor4;
}

// 解析电池BMS信息
/***********************************************************
*data:DD 04 00 0E 0E 43 0E 43 0E 44 0E 40 0E 43 0E 41 0E 45 FD BD 77
*     DD 05 00 15 4A 42 44 2D 53 50 31 35 53 30 32 30 41 2D 50 37 53 2D 34 30 41 FA E6 77
*     DD 03 00 1B 09 FB 00 00 07 44 0B E1 00 05 2F 4B 00 00 00 00 00 00 23 3D 03 07 02 0B AB 0B A9 FB 55 77  电池基本信息及状态
       0  1  2  3 |4 5|  6  7| 8  9|10 11|12 13|14 15|16 17|18 19|20|21|22|23|24|25 26 27 28 29 30 31 32 33
             1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29
***********************************************************/
void BotComHandler::HandleDD(char read_data[])
{
	if(read_data[1] == 0x03){
		//总电压 单位10mV
		voltage_total =  ((read_data[4] & 0xFF) << 8) + (read_data[5] & 0xFF);
		//电流  单位10mA
		current = ((read_data[6] & 0xFF) << 8) + (read_data[7] & 0xFF);
		//剩余容量  单位10mAh
		capacity = ((read_data[8] & 0xFF) << 8) + (read_data[9] & 0xFF);
		//实际满充容量  单位10mAh
		capacity_full_charge = ((read_data[10] & 0xFF) << 8) + (read_data[11] & 0xFF);
		//保护状态 0为未保护，1发生保护
		  // 提供：单体过压保护 单体欠压保护 整组过压保护 整组欠压保护 充电过温保护 充电低温保护 放电过温保护 放电低温保护
		  // 充电过流保护 放电过流保护 短路保护 软件锁定MOS
		over_voltage_unit = ((read_data[20] & 0xFF) >> 7) & 0x01;
		under_voltage_unit = ((read_data[20] & 0xFF) >> 6) & 0x01;
		over_voltage_group = ((read_data[20] & 0xFF) >> 5) & 0x01;
		under_voltage_group = ((read_data[20] & 0xFF) >> 4) & 0x01;
		over_temperature_charge = ((read_data[20] & 0xFF) >> 3) & 0x01;
		low_temperature_charge = ((read_data[20] & 0xFF) >> 2) & 0x01;
		over_temperature_discharge = ((read_data[20] & 0xFF) >> 1) & 0x01;
		low_temperature_discharge = ((read_data[20] & 0xFF) >> 0) & 0x01;
		over_current_charge = ((read_data[21] & 0xFF) >> 7) & 0x01;
		over_current_discharge = ((read_data[21] & 0xFF) >> 6) & 0x01;
		short_circuit_protection = ((read_data[21] & 0xFF) >> 5) & 0x01;
		software_lock_mos = ((read_data[21] & 0xFF) >> 3) & 0x01;
		// RSOC 剩余容量百分比
		rsoc = read_data[23] & 0xFF;
		// MOS指示状态，充电指示：bit0表示充电；bit1表示放电；MOS开关状态，1表示打开；0表示关闭
		mos_charge_status = ((read_data[24] & 0xFF) >> 7) & 0x01;
		mos_discharge_status = ((read_data[24] & 0xFF) >> 6) & 0x01;
	}


	        bot_sensor_com_data_pub.voltage_total = voltage_total; //总电压 单位10mV  2byte
		bot_sensor_com_data_pub.current = current;  //电流  单位10mA  2byte
		bot_sensor_com_data_pub.capacity = capacity;  //剩余容量  单位10mAh  2byte
		bot_sensor_com_data_pub.capacity_full_charge = capacity_full_charge; //实际满充容量，即电池的标称容量  单位10mAh 2byte
		//保护状态
		bot_sensor_com_data_pub.over_voltage_unit = over_voltage_unit;  //单体过压 0未保护，1发生保护
		bot_sensor_com_data_pub.under_voltage_unit = under_voltage_unit;  //单体欠压 0未保护，1发生保护
		bot_sensor_com_data_pub.over_voltage_group = over_voltage_group;  //整组过压 0未保护，1发生保护
		bot_sensor_com_data_pub.under_voltage_group = under_voltage_group;  //整组欠压 0未保护，1发生保护
		bot_sensor_com_data_pub.over_temperature_charge = over_temperature_charge; //充电过温 0未保护，1发生保护
		bot_sensor_com_data_pub.low_temperature_charge = low_temperature_charge;  //充电低温 0未保护，1发生保护
		bot_sensor_com_data_pub.over_temperature_discharge = over_temperature_discharge; //放电过温 0未保护，1发生保护
		bot_sensor_com_data_pub.low_temperature_discharge = low_temperature_discharge;  //放电低温 0未保护，1发生保护
		bot_sensor_com_data_pub.over_current_charge = over_current_charge; //充电过流 0未保护，1发生保护
		bot_sensor_com_data_pub.over_current_discharge = over_current_discharge; //放电过流 0未保护，1发生保护
		bot_sensor_com_data_pub.short_circuit_protection = short_circuit_protection; //短路保护 0未保护，1发生保护
		bot_sensor_com_data_pub.software_lock_mos = software_lock_mos; //软件锁定MOS 0未保护，1发生保护
		bot_sensor_com_data_pub.rsoc = rsoc; //剩余容量百分比 1byte
		bot_sensor_com_data_pub.mos_charge_status = mos_charge_status; //充电状态 1：打开；0：关闭
		bot_sensor_com_data_pub.mos_discharge_status = mos_discharge_status; //放电状态 1:打开  0：关闭
}

BotComHandler::BotComHandler() :
	ModuleParams(nullptr),
	ScheduledWorkItem("bot_com_handler", px4::wq_configurations::bot_com_handler)
{

}

BotComHandler::~BotComHandler()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

int BotComHandler::task_spawn(int argc, char *argv[])
{
	BotComHandler *instance = new BotComHandler();

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

int BotComHandler::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int BotComHandler::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BotComHandler::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("bot_com_handler", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("bot_com_handler start com_num");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int bot_com_handler_main(int argc, char *argv[])
{
	return BotComHandler::main(argc, argv);
}



























