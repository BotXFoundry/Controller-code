#include "BotMotionControl.hpp"
#include <uORB/topics/bot_can_message.h>
#include <uORB/topics/bot_motor_control.h>

extern bot_motor_control_s bot_motor_control_sub;
SDO SDO_CMD[5];

uint16_t can_receive_114_num = 0;
uint16_t can_receive_231_num = 0;

void BotMotionControl::DispatchCanMessage(){

    //判断186是否接收新的数据，有则处理数据
	if(bot_can_message_receive_sub.can_data_receive_186 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_186, bot_can_message_sub.can_data_186);
	}

      //判断187是否接收新的数据，有则处理数据
	if(bot_can_message_receive_sub.can_data_receive_187 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_187, bot_can_message_sub.can_data_187);
	}

      //判断184是否接收新的数据，有则处理数据
	if(bot_can_message_receive_sub.can_data_receive_188 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_188, bot_can_message_sub.can_data_188);
	}

      //判断184是否接收新的数据，有则处理数据
	if(bot_can_message_receive_sub.can_data_receive_189 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_189, bot_can_message_sub.can_data_189);
	}

  can_receive_100_count ++;
  if(bot_can_message_receive_sub.can_data_receive_100 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_100, bot_can_message_sub.can_data_100);
    can_receive_100_count = 0;
	}

	if(bot_can_message_receive_sub.can_data_receive_102 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_102, bot_can_message_sub.can_data_102);
	}

	if(bot_can_message_receive_sub.can_data_receive_104 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_104, bot_can_message_sub.can_data_104);
	}

	if(bot_can_message_receive_sub.can_data_receive_106 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_106, bot_can_message_sub.can_data_106);
	}

  can_receive_114_count ++;
  if(bot_can_message_receive_sub.can_data_receive_114 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_114, bot_can_message_sub.can_data_114);
    can_receive_114_num++;
    can_receive_114_count = 0;
	}

  if(bot_can_message_receive_sub.can_data_receive_116 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_116, bot_can_message_sub.can_data_116);
	}

  if(bot_can_message_receive_sub.can_data_receive_117 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_117, bot_can_message_sub.can_data_117);
	}

	if(bot_can_message_receive_sub.can_data_receive_120 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_120, bot_can_message_sub.can_data_120);
	}

	if(bot_can_message_receive_sub.can_data_receive_150 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_150, bot_can_message_sub.can_data_150);
	}

	if(bot_can_message_receive_sub.can_data_receive_152 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_152, bot_can_message_sub.can_data_152);
	}

	if(bot_can_message_receive_sub.can_data_receive_154 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_154, bot_can_message_sub.can_data_154);
	}

	if(bot_can_message_receive_sub.can_data_receive_190 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_190, bot_can_message_sub.can_data_190);
	}

	if(bot_can_message_receive_sub.can_data_receive_192 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_192, bot_can_message_sub.can_data_192);
	}

	if(bot_can_message_receive_sub.can_data_receive_194 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_194, bot_can_message_sub.can_data_194);
	}

	if(bot_can_message_receive_sub.can_data_receive_196 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_196, bot_can_message_sub.can_data_196);
	}

	if(bot_can_message_receive_sub.can_data_receive_200 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_200, bot_can_message_sub.can_data_200);
	}

	if(bot_can_message_receive_sub.can_data_receive_202 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_202, bot_can_message_sub.can_data_202);
	}

	if(bot_can_message_receive_sub.can_data_receive_204 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_204, bot_can_message_sub.can_data_204);
	}

	if(bot_can_message_receive_sub.can_data_receive_206 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_206, bot_can_message_sub.can_data_206);
	}

	if(bot_can_message_receive_sub.can_data_receive_220 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_220, bot_can_message_sub.can_data_220);
	}

  if(bot_can_message_receive_sub.can_data_receive_231 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_231, bot_can_message_sub.can_data_231);
	}

	if(bot_can_message_receive_sub.can_data_receive_281 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_281, bot_can_message_sub.can_data_281);
	}

	if(bot_can_message_receive_sub.can_data_receive_282 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_282, bot_can_message_sub.can_data_282);
	}

	if(bot_can_message_receive_sub.can_data_receive_283 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_283, bot_can_message_sub.can_data_283);
	}

	if(bot_can_message_receive_sub.can_data_receive_284 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_284, bot_can_message_sub.can_data_284);
	}

	if(bot_can_message_receive_sub.can_data_receive_381 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_381, bot_can_message_sub.can_data_381);
	}

	if(bot_can_message_receive_sub.can_data_receive_382 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_382, bot_can_message_sub.can_data_382);
	}

	if(bot_can_message_receive_sub.can_data_receive_383 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_383, bot_can_message_sub.can_data_383);
	}

	if(bot_can_message_receive_sub.can_data_receive_384 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_384, bot_can_message_sub.can_data_384);
	}

	if(bot_can_message_receive_sub.can_data_receive_581 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_581, bot_can_message_sub.can_data_581);
	}

	if(bot_can_message_receive_sub.can_data_receive_582 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_582, bot_can_message_sub.can_data_582);
	}

	if(bot_can_message_receive_sub.can_data_receive_583 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_583, bot_can_message_sub.can_data_583);
	}

	if(bot_can_message_receive_sub.can_data_receive_584 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_584, bot_can_message_sub.can_data_584);
	}

	if(bot_can_message_receive_sub.can_data_receive_701 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_701, bot_can_message_sub.can_data_701);
	}

	if(bot_can_message_receive_sub.can_data_receive_702 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_702, bot_can_message_sub.can_data_702);
	}

	if(bot_can_message_receive_sub.can_data_receive_703 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_703, bot_can_message_sub.can_data_703);
	}

	if(bot_can_message_receive_sub.can_data_receive_704 == 1)
	{
		HandleCanMessage(bot_can_message_sub.can_id_704, bot_can_message_sub.can_data_704);
	}
}

//此方法为了保护安全，对于速度指令等重要数据，在隔一段时间没有发送时，停止底盘运动，所以如果想持续控制底盘，需要按照50HZ的频率发送指令
void BotMotionControl::HandleCanMessageInterval(){
  if(bot_run_control_sub.is_can_control == 1 && can_receive_114_count > 1000){
      // bot_run_control_sub.motion_mode = 0; //对控制模式不做处理
      bot_run_control_sub.linear_velocity = 0;
      bot_run_control_sub.body_angle = 0;
      bot_run_control_sub.angular_velocity = 0;
  }

}

void BotMotionControl::HandleCanMessage(uint32_t can_id_in, uint8_t can_data_in[8])
{
	switch (can_id_in) {
    case 0x100:  // 控制模式设定
      HandleCanMessage_100(can_data_in);
      break;
    case 0x102:  // 本体运动模型设置
      HandleCanMessage_102(can_data_in[0]);  //此can不开放，在烧录时就内置好运行模式，不允许上位机设置
      break;
    case 0x104:  // 查询本体线速度、角速度、转角
      HandleCanMessage_104();
      break;
    case 0x106:  // 查询传感器状态反馈，如防撞条、急停
      HandleCanMessage_106();
      break;
    case 0x114:  // 运动指令控制
      HandleCanMessage_114(can_data_in);
      break;
    case 0x116:
      HandleCanMessage_116(can_data_in);
      break;
    case 0x117:
      HandleCanMessage_117(can_data_in);
      break;
    case 0x120:  // 主控及伺服驱动器系统软硬件版本反馈
      HandleCanMessage_120(false);
      break;
    case 0x140:  // 灯光控制
      HandleCanMessage_140(can_data_in);
      break;
    case 0x150:  // 行驶轮线速度反馈
      HandleCanMessage_150(false);
      break;
    case 0x152:  // 行驶轮转向角反馈
      HandleCanMessage_152(false);
      break;

      // 里程计数据反馈
    case 0x190:  // 第1轮里程计数据反馈
      HandleCanMessage_190(NODE_1, false);
      break;
    case 0x192:  // 第2轮里程计数据反馈
      HandleCanMessage_192(NODE_2, false);
      break;
    case 0x194:  // 第3轮里程计数据反馈
      HandleCanMessage_194(NODE_3, false);
      break;
    case 0x196:  // 第4轮里程计数据反馈
      HandleCanMessage_196(NODE_4, false);
      break;

      // 电机驱动器状态反馈
    case 0x200:  // 第1电机驱动器状态反馈
      HandleCanMessage_200(NODE_1, false);
      break;
    case 0x202:  // 第2电机驱动器状态反馈
      HandleCanMessage_202(NODE_2, false);
      break;
    case 0x204:  // 第3电机驱动器状态反馈
      HandleCanMessage_204(NODE_3, false);
      break;
    case 0x206:  // 第4电机驱动器状态反馈
      HandleCanMessage_206(NODE_4, false);
      break;

    // case 0x220:  // 根据220指令，反馈电池BMS状态数据
    //   HandleCanMessage_220(can_data_in);
    //   break;

    case 0x231:  //传感器设置
      HandleCanMessage_231(can_data_in);
      break;

    case 0x186:  // 第1、3电机轮里程计
      HandleMotorCanMessage_186(can_data_in);
      break;
    case 0x187:  // 第2、4电机轮里程计
      HandleMotorCanMessage_187(can_data_in);
      break;
    case 0x188:  // 第1、3电机速度
      HandleMotorCanMessage_188(can_data_in);
      break;
    case 0x189:  // 第2、4电机速度
      HandleMotorCanMessage_189(can_data_in);
      break;

    case 0x281:  // 第1电机温度、电流、电压
      HandleMotorCanMessage_281(NODE_1, can_data_in);
      break;
    case 0x282:  // 第2电机温度、电流、电压
      HandleMotorCanMessage_282(NODE_2, can_data_in);
      break;
    case 0x283:  // 第3电机温度、电流、电压
      HandleMotorCanMessage_283(NODE_3, can_data_in);
      break;
    case 0x284:  // 第4电机温度、电流、电压
      HandleMotorCanMessage_284(NODE_4, can_data_in);
      break;

    case 0x381:  // 第1电机心跳计数
      HandleMotorCanMessage_381(NODE_1, can_data_in);
      break;
    case 0x382:  // 第2电机心跳计数
      HandleMotorCanMessage_382(NODE_2, can_data_in);
      break;
    case 0x383:  // 第3电机心跳计数
      HandleMotorCanMessage_383(NODE_3, can_data_in);
      break;
    case 0x384:  // 第4电机心跳计数
      HandleMotorCanMessage_384(NODE_4, can_data_in);
      break;

    case 0x581:  // 第1电机驱动器SDO指令执行反馈
      HandleMotorCanMessage_58X(NODE_1, can_data_in);
      // HandleMotorCanMessage_581(NODE_1, can_data_in);
      break;
    case 0x582:  // 第2电机驱动器SDO指令执行反馈
      HandleMotorCanMessage_58X(NODE_2, can_data_in);
      // HandleMotorCanMessage_582(NODE_2, can_data_in);
      break;
    case 0x583:  // 第3电机驱动器SDO指令执行反馈
      HandleMotorCanMessage_58X(NODE_3, can_data_in);
      // HandleMotorCanMessage_583(NODE_3, can_data_in);
      break;
    case 0x584:  // 第4电机驱动器SDO指令执行反馈
      HandleMotorCanMessage_58X(NODE_4, can_data_in);
      // HandleMotorCanMessage_584(NODE_4, can_data_in);
      break;

// 70X数据暂不处理
    // case 0x701:  // 第1电机NMT上线报文
    //   // HandleMotorCanMessage_70X(NODE_1, can_data_in);
      // HandleMotorCanMessage_701(NODE_1, can_data_in);
    //   break;
    // case 0x702:  // 第2电机NMT上线报文
    //   // HandleMotorCanMessage_70X(NODE_2, can_data_in);
    //   HandleMotorCanMessage_702(NODE_2, can_data_in);
    //   break;
    // case 0x703:  // 第3电机NMT上线报文
    //   // HandleMotorCanMessage_70X(NODE_3, can_data_in);
    //   HandleMotorCanMessage_703(NODE_3, can_data_in);
    //   break;
    // case 0x704:  // 第4电机NMT上线报文
      // // HandleMotorCanMessage_70X(NODE_4, can_data_in);
      // HandleMotorCanMessage_704(NODE_4, can_data_in);
    //   break;

    default:
      break;
   }
}


 // 传感器设置
void BotMotionControl::HandleCanMessage_231(uint8_t* data){

  //防跌落阈值
  uint8_t fall_threshold = data[0];

  //防撞条解除时间
  uint16_t touch_sensor_relase_time = (uint16_t)(data[2] << 8 | data[1]); //单位：s

  //超声波触发距离
  uint16_t ultrasonic_trigger_distance = (uint16_t)(data[4] << 8 | data[3]); //单位：mm

  //是否关闭超声波
  uint8_t is_close_ultrasonic = data[5]; //0:不关闭；1:关闭

  uint8_t back_ultrasonic_is_close = data[6]; //0:不关闭；1：关闭

  //是否关闭防跌落
  uint8_t fall_is_close = data[7]; //0:不关闭；1:关闭

  bot_sensor_event_sub.fall_threshold = fall_threshold;
  bot_sensor_event_sub.touch_sensor_release_time = touch_sensor_relase_time; //单位：s
  bot_sensor_event_sub.ultrasonic_is_close = is_close_ultrasonic;
  bot_sensor_event_sub.ultrasonic_threshold = ultrasonic_trigger_distance;
  bot_sensor_event_sub.back_ultrasonic_is_close = back_ultrasonic_is_close;
  bot_sensor_event_sub.fall_sensor_is_close = fall_is_close;
}


// 驱动器设置或查询错误，转错误处理
void HandleMotorErrorReport(int node_id, SDO CMD) {}

void BotMotionControl::HandleMotorCanMessage_188(uint8_t* data) { //31

  // 第0~3字符为第3个轮子里程计总脉冲数
   bot_motor_control_sub.odometer_encode[3] =
      (uint32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);  // 里程计
  // 第4~7字节为第1个轮子里程计总脉冲数
  bot_motor_control_sub.odometer_encode[1] =
      (uint32_t)((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);  // 里程计

}

void BotMotionControl::HandleMotorCanMessage_186(uint8_t* data) { //31
      // 第0-1字节为电机转速
  bot_motor_control_sub.speed_rpm[3] = (int16_t)((data[1] << 8) | data[0]);
  bot_motor_control_sub.speed_rpm[1] = (int16_t)((data[5] << 8) | data[4]);

}

void BotMotionControl::HandleMotorCanMessage_189(uint8_t* data) { //31

  // 第0~3字符为第3个轮子里程计总脉冲数
   bot_motor_control_sub.odometer_encode[2] =
      (uint32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);  // 里程计
  // 第4~7字节为第1个轮子里程计总脉冲数
  bot_motor_control_sub.odometer_encode[4] =
      (uint32_t)((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);  // 里程计
}

void BotMotionControl::HandleMotorCanMessage_187(uint8_t* data) { //31

  bot_motor_control_sub.speed_rpm[2] = (int16_t)((data[1] << 8) | data[0]);
  bot_motor_control_sub.speed_rpm[4] = (int16_t)((data[5] << 8) | data[4]);

}


// 电机温度、电流、电压
void BotMotionControl::HandleMotorCanMessage_281(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  // 第2-3字节为电机温度
  bot_motor_control_sub.motor_temperature[node_id] = (int16_t)((data[3] << 8) | data[2]);

  // 第4-5字节为电机当前电流 刻度为0.01A
  bot_motor_control_sub.motor_current[node_id] = (int16_t)((data[5] << 8) | data[4]);

  // 第6~7字节为电压  刻度为0.01V
  bot_motor_control_sub.motor_voltage[node_id] = (int16_t)((data[7] << 8) | data[6]);
}

// 电机温度、电流、电压
void BotMotionControl::HandleMotorCanMessage_282(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;

  // 第2-3字节为电机温度
  bot_motor_control_sub.motor_temperature[node_id] = (int16_t)((data[3] << 8) | data[2]);

  // 第4-5字节为电机当前电流 刻度为0.01A
  bot_motor_control_sub.motor_current[node_id] = (int16_t)((data[5] << 8) | data[4]);

  // 第6~7字节为电压  刻度为0.01V
  bot_motor_control_sub.motor_voltage[node_id] = (int16_t)((data[7] << 8) | data[6]);
}

// 电机温度、电流、电压
void BotMotionControl::HandleMotorCanMessage_283(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;

  // 第2-3字节为电机温度
  bot_motor_control_sub.motor_temperature[node_id] = (int16_t)((data[3] << 8) | data[2]);

  // 第4-5字节为电机当前电流 刻度为0.01A
  bot_motor_control_sub.motor_current[node_id] = (int16_t)((data[5] << 8) | data[4]);

  // 第6~7字节为电压  刻度为0.01V
  bot_motor_control_sub.motor_voltage[node_id] = (int16_t)((data[7] << 8) | data[6]);
}

// 电机温度、电流、电压
void BotMotionControl::HandleMotorCanMessage_284(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;

  // 第2-3字节为电机温度
  bot_motor_control_sub.motor_temperature[node_id] = (int16_t)((data[3] << 8) | data[2]);

  // 第4-5字节为电机当前电流 刻度为0.01A
  bot_motor_control_sub.motor_current[node_id] = (int16_t)((data[5] << 8) | data[4]);

  // 第6~7字节为电压  刻度为0.01V
  bot_motor_control_sub.motor_voltage[node_id] = (int16_t)((data[7] << 8) | data[6]);
}


// 电机心跳计数
void BotMotionControl::HandleMotorCanMessage_381(int node_id, uint8_t* data) {}
// 电机心跳计数
void BotMotionControl::HandleMotorCanMessage_382(int node_id, uint8_t* data) {}
// 电机心跳计数
void BotMotionControl::HandleMotorCanMessage_383(int node_id, uint8_t* data) {}
// 电机心跳计数
void BotMotionControl::HandleMotorCanMessage_384(int node_id, uint8_t* data) {}
// 电机驱动器SDO指令执行反馈  目前没有完全实现
void BotMotionControl::HandleMotorCanMessage_58X(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;

  SDO CMD = SDO_CMD[node_id];

  CMD.CmdCode = data[0];       // SDO命令编码(SDO报文字节0)
  CMD.IndexCodeH = data[2];    // SDO索引编码高字节(SDO报文字节2)
  CMD.IndexCodeL = data[1];    // SDO索引编码低字节(SDO报文字节1)
  CMD.SubIndexCode = data[3];  // SDO子索引编码字节(SDO报文字节3)

  // 根据CanOpen规范，CAN数据位的第一个字节属于命令编码标识
  // 2F:设置(M->S请求) 1字节
  // 2B:设置(M->S请求) 2字节
  // 27:设置(M->S请求) 3字节
  // 23:设置(M->S请求) 4字节
  // 60:设置反馈(S->M确认)
  // 40:读取(M->S请求) 0字节
  // 4F:读取反馈(S->M应答) 1字节
  // 4B:读取反馈(S->M应答) 2字节
  // 47:读取反馈(S->M应答) 3字节
  // 43:读取反馈(S->M应答) 4字节
  // 80:报错(S->M应答) 4字节

  switch (CMD.CmdCode) {
    case 0x60:
      CMD.CmdDataLength = 0;  // 是设置反馈确认，命令Data区数据宽度为0
      CMD.AckDataLength = 0;
      CMD.IsSetupCmd = 1;  // 是设置命令
      break;
    case 0x4F:
      CMD.CmdDataLength = 1;  // 读取反馈，命令Data区数据宽度为1
      CMD.AckDataLength = 1;
      CMD.IsSetupCmd = 0;  // 读取反馈
      break;
    case 0x4B:
      CMD.CmdDataLength = 2;  // 读取反馈，命令Data区数据宽度为2
      CMD.AckDataLength = 2;
      CMD.IsSetupCmd = 0;  // 读取反馈
      break;
    case 0x47:
      CMD.CmdDataLength = 3;  // 读取反馈，命令Data区数据宽度为3
      CMD.AckDataLength = 3;
      CMD.IsSetupCmd = 0;  // 读取反馈
      break;
    case 0x43:
      CMD.CmdDataLength = 4;  // 读取反馈，命令Data区数据宽度为4
      CMD.AckDataLength = 4;
      CMD.IsSetupCmd = 0;  // 读取反馈
      break;
    case 0x80:  // 错误反馈
      CMD.HasError = 1;
      CMD.ErrorType = 1;      // 错误原因：SDO索引编码错误
      CMD.CmdDataLength = 4;  // 错误反馈，命令Data区数据宽度为4
      CMD.AckDataLength = 4;
      CMD.IsSetupCmd = 0;
      break;
    default:
      break;
  }

  // 根据命令对应的业务数据字节占位宽度合并拼接业务数据
  uint8_t temp4 = data[7];  // 32位业务数据第4字节(从低到高)
  uint8_t temp3 = data[6];  // 32位业务数据第3字节(从低到高)
  uint8_t temp2 = data[5];  // 32位业务数据第2字节(从低到高)
  uint8_t temp1 = data[4];  // 32位业务数据第1字节(从低到高)

  CMD.BizData8 = temp1;

  uint16_t tempL =
      (((uint16_t)temp2 & 0xFF) << 8) + (((uint16_t)(temp1)) & 0xFF);
  CMD.BizData16 = tempL;  // 如果命令对应业务数据为U16以内则取低两个字节

  if (CMD.CmdDataLength > 2) {
    uint16_t tempH =
        (((uint16_t)temp4 & 0xFF) << 8) + (((uint16_t)(temp3)) & 0xFF);
    uint32_t temp =
        ((((uint32_t)(tempH)) & 0xFFFF) << 16) + (((uint32_t)(tempL)) & 0xFFFF);
    CMD.BizData32 = temp;  // 如果命令对应业务数据超过U16则取四个字节
  }
  switch (CMD.IndexCodeH) {
    case 0x20:
      CMD.CmdDataLength = 2;  // 本码段命令Data区数据宽度为2
      CMD.AckDataLength = 2;
      CMD.AckSetCmdCode = 0x60;  // 设置类指令的执行反馈指令为0x60
      CMD.AckGetCmdCode = 0x4B;  // SDO两字节读取反馈指令编码0x4B
      switch (CMD.IndexCodeL) {
        case 0x00:  // 0x2000H 通讯掉线保护时间 范围0~32767
          HandleDisConnectProtectionTime(node_id, CMD);
          break;
        case 0x05:  // 0x2005H 反馈位置清零 ）0：无效 1：反馈位置清零不保存
          HandleFeedbackPositionReset(node_id, CMD);
          break;
        case 0x06:  // 0x2006H 绝对位置模式时当前位置清零  0：无效
                    // 1：当前位置清零
          HandleAbsolutePositionModeResetToZero(node_id, CMD);
          break;
        case 0x07:  // 0x2007H 限位停车方式 0:停止 1：急停 2：无效
          HandleRestrictedParkingMethods(node_id, CMD);
          break;
        case 0x08:  // 0x2008H 起始速度 单位r/min  范围1-300r/min 默认1
          HandleStartingSpeed(node_id, CMD);
          break;
        case 0x0A:  // 0x200AH 电机最大转速 单位r/min  范围1-300r/min
          HandleMaximumSpeed(node_id, CMD);
          break;
        case 0x0B:  // 0x200BH 编码器线数设置 0-4096 默认1024
          HandleLineEncoder(node_id, CMD);
          break;
        case 0x0C:  // 0x200CH 电机极对数
          HandlePolePairs(node_id, CMD);
          break;
        case 0x0F:  // 0x200FH 上电锁轴方式 0：不使能不锁轴，1：不使能锁轴
          HandleLockBearingMode(node_id, CMD);
          break;
        case 0x11:  // 0x2011H 电机与HALL的偏移角度 单位1度 范围-360度~360度
          HandleHallOffsetAngle(node_id, CMD);
          break;
        case 0x12:  // 0x2012H 过载系数  范围0-300,单位%  默认200
          HandleOverloadFactor(node_id, CMD);
          break;
        case 0x13:  // 0x2013H 电机温度保护阈值 单位0.1度  范围0-1200(*0.1)
                    // 默认800
          HandleTemperatureProtectionThreshold(node_id, CMD);
          break;
        case 0x14:  // 0x2014H 额定电流  驱动器输出的额定电流 单位0.1A 范围0-150
                    // 缺省150
          HandleLimitedCurrent(node_id, CMD);
          break;
        case 0x15:  // 0x2015H 最大电流 驱动器输出的最大电流  单位0.1A 范围0-300
                    // 缺省 300
          HandleMaximumCurrent(node_id, CMD);
          break;
        case 0x16:  // 0x2016H 过载保护时间 驱动器过载保护时间 单位10ms
                    // 范围0-6553 缺省300
          HandleOverloadProtectionTime(node_id, CMD);
          break;
        case 0x17:  // 0x2017H 超差报警阈值 编码器超差阈值  单位*10counts
                    // 范围1-6553 缺省409
          HandleOverToleranceAlarmThreshold(node_id, CMD);
          break;
        case 0x18:  // 0x2018H 过度平滑系数 0-30000  缺省1000
          HandleSmoothingFactor(node_id, CMD);
          break;
        case 0x19:  // 0x2019H 电流环比例系数 0-30000   缺省600
          HandleCurrentLoopScaleFactor(node_id, CMD);
          break;
        case 0x1A:  // 0x201AH 电流环积分增益 0-30000  缺省300
          HandleCurrentLoopIntegralGain(node_id, CMD);
          break;
        case 0x1C:  // 0x201CH 转矩输出平滑系数 0-30000 缺省100
          HandleTorqueOutputSmoothingFactor(node_id, CMD);
          break;
        case 0x1D:  // 0x201DH 速度比例增益Kp 0-30000 缺省500
          HandleSpeedKp(node_id, CMD);
          break;
        case 0x1E:  // 0x201EH 速度积分增益Ki 0-30000 缺省100
          HandleSpeedKi(node_id, CMD);
          break;
        case 0x1F:  // 0x201FH 速度前馈增益Kf 0-30000 缺省1000
          HandleSpeedKf(node_id, CMD);
          break;
        case 0x20:  // 0x2020H 位置比例增益Kp 0-30000 缺省50
          HandlePositionKp(node_id, CMD);
          break;
        case 0x21:  // 0x2021H 位置积分增益Ki 0-30000
          HandlePositionKi(node_id, CMD);
		  break;
        case 0x22:  // 0x2021H 位置微分增益Kd 0-30000
          HandlePositionKd(node_id, CMD);
          break;
        case 0x25:  // 0x2025H 软件版本   出厂默认
          HandleSoftwareVersion(node_id, CMD);
          break;
        case 0x26:  // 0x2026H 电机温度 单位0.1度 范围0-120度  默认800
          HandleMotorTemperature(node_id, CMD);
          break;
        case 0x28:  // 0x2028H 霍尔输入状态 0-7 如果出现0或7  为霍尔出错 默认0
          HandleHallSector(node_id, CMD);
          break;
        case 0x29:  // 0x2029H 母线电压 单位0.01V  默认0
          HandleBusVoltage(node_id, CMD);
          break;
        case 0x2A:  // 0x202AH 报警PWM处理方式 0：关闭 1：开启
          HandlePWMAlarmMethod(node_id, CMD);
          break;
        case 0x2B:  // 0x202BH  过载处理方式 0：关闭 1：开启
          HandleOverloadHandlingMethod(node_id, CMD);
		  break;
        default:  // SDO索引编码错误
          CMD.HasError = 1;
          CMD.ErrorType = 1;  // 错误原因：SDO索引编码错误
          break;
      }
      break;
    case 0x60:
      if (CMD.IsSetupCmd & ((CMD.CmdCode <= 0x77) & (CMD.CmdCode != 0x2B)) &
          ((CMD.CmdCode > 0x77) &
           (CMD.CmdCode !=
               0x23))) {  // 如果是设置命令但U16命令字码段不是0x2B以及U32命令字码段不是0x23则报告错误
        CMD.HasError = 1;
        CMD.ErrorType = 1;  // 错误原因：SDO索引编码错误
        CMD.AckDataLength = 4;
        CMD.AckData16 = 0;
        CMD.AckData32 = 0;
        //com_can_message_send_58X(node_id);  // 回复操作成功确认消息
        return;
      }
      if (CMD.CmdCode <= 0x77)  //????? 需要重新赋数据区域宽度值？
      {  // 如果是设置命令但U16命令字码段数据宽度2字节
        CMD.CmdDataLength = 2;  // 命令Data区数据宽度为2
        CMD.AckDataLength = 2;
      }
      if (CMD.CmdCode > 0x77) {  // 如果是设置命令但U32命令字码段数据宽度4字节
        CMD.CmdDataLength = 4;  // 命令Data区数据宽度为2
        CMD.AckDataLength = 4;
      }
      CMD.AckSetCmdCode = 0x60;  // 设置类指令的执行反馈指令为0x60
      CMD.AckGetCmdCode = 0x4B;  // SDO两字节读取反馈指令编码0x4B
      switch (CMD.IndexCodeL) {
        case 0x3F:  // 0x603F 驱动器最近一次故障码
          HandleLastFaultCode(node_id, CMD);
          break;
        case 0x40:  // 0x6040 控制字
          HandleControlWord(node_id, CMD);
          break;
        case 0x41:  // 0x6041 状态字
          HandleStatusWord(node_id, CMD);
          break;
        case 0x5A:  // 0x605A 快速停止代码 快速停止命令后驱动器处理方式 默认5
                    // 5:正常停止，维持快速停止状态，6：急减速停，维持快速停止状态，7：急停，维持快速停止状态
          HandleQuickStopCode(node_id, CMD);
          break;
        case 0x5B:  // 0x605B 关闭操作代码 关闭命令后驱动器处理方式 0：无效
                    // 1：正常停止，转到Ready to switch on状态，默认1
          HandleCloseActionCode(node_id, CMD);
          break;
        case 0x5C:  // 0x605C 禁用操作代码  禁用操作命令后驱动器处理方式 0：无效
                    // 1：正常停止，转到Switched On 状态 默认1
          HandleDisableActionCode(node_id, CMD);
          break;
        case 0x5D:  // 0x605D Halt控制寄存器  控制字Halt命令后驱动器处理方式
                    // 1：正常停止，维持Operation
                    // Enabled状态，2：急速减停，维持Operation
                    // Enabled状态，3：急停，维持Operation Enabled状态，默认：1
          HandleHaltControlRegister(node_id, CMD);
          break;
        case 0x60:  // 0x6060
                    // 运行模式，0：未定义，1：位置模式，2：速度模式,3:转矩模式
                    // 默认：0
          HandleSetRunMode(node_id, CMD);
          break;
        case 0x61:  // 0x6061  运行模式状态
                    // 0：未定义，1：位置模式，2：速度模式,3:转矩模式，默认：0
          HandleRunningModeStatus(node_id, CMD);
          break;
        case 0x65:  // 0x6071 目标转矩 单位mA,范围-30000~30000 默认：0
          HandleTargetTorque(node_id, CMD);
          break;
        case 0x66:  // 0x6077 实时转矩反馈 单位0.1A,范围-300~300 默认：0
          HandleRealTimeTorque(node_id, CMD);
          break;
        case 0x71:  // 0x6064 实际位置反馈  单位counts 默认：0
          HandleActualPosition(node_id, CMD);
          break;
        case 0x77:  // 0x606C 实际速度反馈 电机当前运行速度 单位0.1r/min 默认：0
          HandleActualSpeed(node_id, CMD);
          break;
        case 0x7A:  // 0x607A 目标位置
                    // 位置模式运行总脉冲数范围：-1000000~1000000  默认：0
          CMD.AckGetCmdCode = 0x43;  // 四个字节
          HandleTargetPosition(node_id, CMD);
          break;
        case 0x81:  // 0x6081 最大速度 位置模式时的最大速度 范围1-300r/min
                    // 默认：120r/min
          HandleMaximumSpeed(node_id, CMD);
          break;
        case 0x82:  // 0x6082 位置模式启/停速度  范围1-300r/min 默认1r/min
          HandlePositionModeStartStopSpeed(node_id, CMD);
          break;
        case 0x85:  // 0x6085 急停减速时间 范围0~32767ms 缺省10ms
          HandleEmergencyStopDecelerationTime(node_id, CMD);
          break;
        case 0x87:  // 0x6087 转矩斜率 电流/1000/second   单位mA/s  缺省300ms
          HandleTorqueGradient(node_id, CMD);
          break;
        case 0xFF:  // 0x60FF 目标速度  速度模式时的目标速度  范围:-300~300r/min
                    // 缺省0
          HandleTargetSpeed(node_id, CMD);
          break;
        default:  // SDO索引编码错误
          CMD.HasError = 1;
          CMD.ErrorType = 1;  // 错误原因：SDO索引编码错误
          break;
      }
      break;
    default:  // SDO索引编码错误
      CMD.HasError = 1;
      CMD.ErrorType = 1;  // 错误原因：SDO索引编码错误
      break;
  }

  // 应答编码
  bot_motor_control_sub.sdo_command[node_id] = data[0];
  // 错误类型
  bot_motor_control_sub.error_code[node_id] = data[4];
}

// 电机NMT上线报文
void BotMotionControl::HandleMotorCanMessage_701(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  //int max_axis_count = 4;
  //if (AXIS_COUNT <= 4) max_axis_count = AXIS_COUNT;
  bot_motor_control_sub.motor_device_ids[node_id] = (uint32_t)node_id;
  // 第0字节为电机状态
  bot_motor_control_sub.node_state[node_id] = data[0];

  //第4、5字节是电机的负载转矩
  int16_t motor_tl = (int16_t)((( data[5] & 0xFF) << 8) + ( data[4] & 0xFF));
  if(is_log_time){
    PX4_INFO("TEST motor_tl=%d",motor_tl);
  }
  bot_motor_control_sub.motor_tl[node_id] = motor_tl;

}

// 电机NMT上线报文
void BotMotionControl::HandleMotorCanMessage_702(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  //int max_axis_count = 4;
  //if (AXIS_COUNT <= 4) max_axis_count = AXIS_COUNT;
  bot_motor_control_sub.motor_device_ids[node_id] = (uint32_t)node_id;
  // 第0字节为电机状态
  bot_motor_control_sub.node_state[node_id] = data[0];
}

// 电机NMT上线报文
void BotMotionControl::HandleMotorCanMessage_703(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  //int max_axis_count = 4;
  //if (AXIS_COUNT <= 4) max_axis_count = AXIS_COUNT;
  bot_motor_control_sub.motor_device_ids[node_id] = (uint32_t)node_id;
  // 第0字节为电机状态
  bot_motor_control_sub.node_state[node_id] = data[0];
}

// 电机NMT上线报文
void BotMotionControl::HandleMotorCanMessage_704(int node_id, uint8_t* data) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  //int max_axis_count = 4;
  //if (AXIS_COUNT <= 4) max_axis_count = AXIS_COUNT;
  bot_motor_control_sub.motor_device_ids[node_id] = (uint32_t)node_id;
  // 第0字节为电机状态
  bot_motor_control_sub.node_state[node_id] = data[0];
}

// 设置 通讯掉线保护时间 范围0~32767
void HandleDisConnectProtectionTime(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.disconnect_protection_time[node_id] = CMD.BizData16;
}

void HandleFeedbackPositionReset(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.feedback_position_reset[node_id] = CMD.BizData16;
}

// 0x2006H 绝对位置模式时当前位置清零  0：无效
// 1：当前位置清零
void HandleAbsolutePositionModeResetToZero(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.absolute_position_mode_reset_to_zero[node_id] =
      CMD.BizData16;
}

// 限位停车方式 0:停止 1：急停 2：无效
void HandleRestrictedParkingMethods(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.restricted_parking_methods[node_id] = CMD.BizData16;
}

// 起始速度 单位r/min  范围1-300r/min 默认1
void HandleStartingSpeed(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.starting_speed[node_id] = CMD.BizData16;
}

// 0x200AH 电机最大转速 单位r/min  范围1-300r/min
void HandleMaximumSpeed(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.maximum_speed[node_id] = CMD.BizData16;
}

// 编码器线数设置 0-4096 默认1024
void HandleLineEncoder(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.line_encoder[node_id] = CMD.BizData16;
}

// 电机极对数,15
void HandlePolePairs(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.pole_pairs[node_id] = CMD.BizData16;
}

// 上电锁轴方式 0：不使能不锁轴，1：不使能锁轴
void HandleLockBearingMode(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.init_lock_axis_mode[node_id] = CMD.BizData16;
}

// 电机与HALL的偏移角度 单位1度 范围-360度~360度
void HandleHallOffsetAngle(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.hall_offset_angle[node_id] = CMD.BizData16;
}

// 过载系数  范围0-300,单位%  默认200
void HandleOverloadFactor(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.overload_factor[node_id] = CMD.BizData16;
}

// 电机温度保护阈值 单位0.1度
// 范围0-1200(*0.1) 默认800
void HandleTemperatureProtectionThreshold(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.temperature_protection_threshold[node_id] =
      CMD.BizData16;
}

// 额定电流  驱动器输出的额定电流 单位0.1A 范围0-150
void HandleLimitedCurrent(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.limited_current[node_id] = CMD.BizData16;
}

// 最大电流 驱动器输出的最大电流  单位0.1A 范围0-300
void HandleMaximumCurrent(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.maximum_current[node_id] = CMD.BizData16;
}

// 过载保护时间 驱动器过载保护时间 单位10ms 范围0-6553 缺省300
void HandleOverloadProtectionTime(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.overload_protection_time[node_id] = CMD.BizData16;
}

//// 超差报警阈值 编码器超差阈值
//  单位*10counts 范围1-6553 缺省409
void HandleOverToleranceAlarmThreshold(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.encoder_diff_alarm_threshold[node_id] = CMD.BizData16;
}

// 过度平滑系数 0-30000  缺省1000
void HandleSmoothingFactor(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.smoothing_factor[node_id] = CMD.BizData16;
}

// 电流环比例系数 0-30000   缺省600
void HandleCurrentLoopScaleFactor(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.foc_current_kp[node_id] = CMD.BizData16;
}

// 电流环积分增益 0-30000  缺省300
void HandleCurrentLoopIntegralGain(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.foc_current_ki[node_id] = CMD.BizData16;
}

// 前馈输出平滑系数 0-30000 缺省100
void HandleFeedforwardSmoothingFactor(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.feed_forward_smoothing_factor[node_id] = CMD.BizData16;
}

// 转矩输出平滑系数 0-30000 缺省100
void HandleTorqueOutputSmoothingFactor(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.duty_output_smoothing_factor[node_id] = CMD.BizData16;
}

// 速度比例增益Kp 0-30000 缺省500
void HandleSpeedKp(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.s_pid_kp[node_id] = CMD.BizData16;
}

// 速度积分增益Ki 0-30000 缺省100
void HandleSpeedKi(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.s_pid_ki[node_id] = CMD.BizData16;
}

// 设置 速度微分增益Kd 0-30000 缺省1000
void HandleSpeedKd(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.s_pid_kd[node_id] = CMD.BizData16;
}

// 位置比例增益Kp 0-30000 缺省50
void HandlePositionKp(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.p_pid_kp[node_id] = CMD.BizData16;
}

// 位置积分增益Kf 0-30000
void HandlePositionKi(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.p_pid_ki[node_id] = CMD.BizData16;
}

// 位置微分增益Kd 0-30000
void HandlePositionKd(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.p_pid_kd[node_id] = CMD.BizData16;
}

// 软件版本   出厂默认
void HandleSoftwareVersion(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.software_version[node_id] = CMD.BizData16;
}

// 电机温度 单位0.1度 范围0-120度  默认800
void HandleMotorTemperature(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.motor_temperature[node_id] = CMD.BizData16;
}

// 霍尔输入状态 0-7 如果出现0或7  为霍尔出错 默认0
void HandleHallSector(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.hall_sector[node_id] = CMD.BizData16;
}

// 母线电压 单位0.01V  默认0
void HandleBusVoltage(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.motor_voltage[node_id] = CMD.BizData16;
}

// 报警PWM处理方式 0：关闭 1：开启
void HandlePWMAlarmMethod(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.pwm_alarm_method[node_id] = CMD.BizData16;
}

// 过载处理方式 0：关闭 1：开启
void HandleOverloadHandlingMethod(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.overload_handling_method[node_id] = CMD.BizData16;
}

// 驱动器最近一次故障码
void HandleLastFaultCode(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.last_fault_code[node_id] = CMD.BizData16;
}

//  控制字
void HandleControlWord(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.control_word[node_id] = CMD.BizData16;
}

// 状态字
void HandleStatusWord(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.status_word[node_id] = CMD.BizData16;
}

// 快速停止代码 快速停止命令后驱动器处理方式 默认5
//   5:正常停止，维持快速停止状态，6：急减速停，维持快速停止状态，7：急停，维持快速停止状态
void HandleQuickStopCode(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.quick_stop_code[node_id] = CMD.BizData16;
}

// 关闭操作代码 关闭命令后驱动器处理方式 0：无效
//  1：正常停止，转到Ready to switch on状态，默认1
void HandleCloseActionCode(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.close_action_code[node_id] = CMD.BizData16;
}

// 禁用操作代码  禁用操作命令后驱动器处理方式 0：无效
//   1：正常停止，转到Switched On 状态 默认1
void HandleDisableActionCode(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.disable_action_code[node_id] = CMD.BizData16;
}

// Halt控制寄存器  控制字Halt命令后驱动器处理方式
//  1：正常停止，维持Operation
//  Enabled状态，2：急速减停，维持Operation
//  Enabled状态，3：急停，维持Operation
//  Enabled状态，默认：1
void HandleHaltControlRegister(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.halt_control_register[node_id] = CMD.BizData16;
}

// 运行模式，0：未定义，1：位置模式，2：速度模式,3:转矩模式
//   默认：0
void HandleSetRunMode(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.run_mode[node_id] = CMD.BizData16;
}

// 运行模式状态
//  0：未定义，1：位置模式，2：速度模式,3:转矩模式，默认：0
void HandleRunningModeStatus(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.run_mode[node_id] = CMD.BizData16;
}

// 实际位置反馈  单位counts 默认：0
void HandleActualPosition(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.actual_position[node_id] = CMD.BizData16;
}

// 实际速度反馈 电机当前运行速度 单位0.1r/min 默认：0
void HandleActualSpeed(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.speed_rpm[node_id] = CMD.BizData16;
}

// 目标转矩 单位mA,范围-30000~30000 默认：0
void HandleTargetTorque(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.target_torque[node_id] = CMD.BizData16;
}

// 实时转矩反馈 单位0.1A,范围-300~300 默认：0
void HandleRealTimeTorque(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.real_time_torque[node_id] = CMD.BizData16;
}

// 设置 目标位置 位置模式运行总脉冲数范围：-1000000~1000000
// 此处存在Uint32往int32转换的兼容性问题
void HandleTargetPosition(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.target_position[node_id] = CMD.BizData32;
}

// 最大速度 位置模式时的最大速度 范围1-300r/min
// 默认：120r/min
void HandleMaxSpeed(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.max_speed_rpm[node_id] = CMD.BizData16;
}

// 设置 位置模式启/停速度  范围1-300r/min
//  默认1r/min
void HandlePositionModeStartStopSpeed(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.position_mode_start_stop_speed[node_id] = CMD.BizData16;
}

// 急停减速时间 范围0~32767ms 缺省10ms
void HandleEmergencyStopDecelerationTime(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.emergency_stop_time[node_id] = CMD.BizData16;
}

// 转矩斜率 电流/1000/second   单位mA/s  缺省300ms
void HandleTorqueGradient(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.torque_gradient[node_id] = CMD.BizData16;
}

// 目标速度  速度模式时的目标速度  范围:-300~300r/min
//  缺省0
// 此处存在Uint32往int32转换的兼容性问题
void HandleTargetSpeed(int node_id, SDO CMD) {
  if ((node_id >= AXIS_COUNT) & (node_id <= 0)) return;
  if (CMD.IsSetupCmd == 1) return;  // 设置反馈无需进一步处理
  if (CMD.HasError == 1) {  // 驱动器设置或查询错误，转错误处理
    HandleMotorErrorReport(node_id, CMD);
    return;
  }
  bot_motor_control_sub.target_speed_rpm[node_id] = CMD.BizData16;
}



void BotMotionControl::HandleDisConnectProtectionTime(int node_id, SDO CMD){}

void BotMotionControl::HandleFeedbackPositionReset(int node_id, SDO CMD){}

void BotMotionControl::HandleAbsolutePositionModeResetToZero(int node_id, SDO CMD){}

void BotMotionControl::HandleRestrictedParkingMethods(int node_id, SDO CMD){}

void BotMotionControl::HandleStartingSpeed(int node_id, SDO CMD){}

void BotMotionControl::HandleMaximumSpeed(int node_id, SDO CMD){}

void BotMotionControl::HandleLineEncoder(int node_id, SDO CMD){}

void BotMotionControl::HandlePolePairs(int node_id, SDO CMD){}

void BotMotionControl::HandleLockBearingMode(int node_id, SDO CMD){}

void BotMotionControl::HandleHallOffsetAngle(int node_id, SDO CMD){}

void BotMotionControl::HandleOverloadFactor(int node_id, SDO CMD){}

void BotMotionControl::HandleTemperatureProtectionThreshold(int node_id, SDO CMD){}

void BotMotionControl::HandleLimitedCurrent(int node_id, SDO CMD){}

void BotMotionControl::HandleMaximumCurrent(int node_id, SDO CMD){}

void BotMotionControl::HandleOverloadProtectionTime(int node_id, SDO CMD){}

void BotMotionControl::HandleOverToleranceAlarmThreshold(int node_id, SDO CMD){}

void BotMotionControl::HandleSmoothingFactor(int node_id, SDO CMD){}

void BotMotionControl::HandleCurrentLoopScaleFactor(int node_id, SDO CMD){}

void BotMotionControl::HandleCurrentLoopIntegralGain(int node_id, SDO CMD){}

void BotMotionControl::HandleTorqueOutputSmoothingFactor(int node_id, SDO CMD){}

void BotMotionControl::HandleSpeedKp(int node_id, SDO CMD){}

void BotMotionControl::HandleSpeedKi(int node_id, SDO CMD){}

void BotMotionControl::HandleSpeedKf(int node_id, SDO CMD){}

void BotMotionControl::HandlePositionKp(int node_id, SDO CMD){}

void BotMotionControl::HandlePositionKi(int node_id, SDO CMD){}

void BotMotionControl::HandlePositionKd(int node_id, SDO CMD){}

void BotMotionControl::HandleSoftwareVersion(int node_id, SDO CMD){}

void BotMotionControl::HandleMotorTemperature(int node_id, SDO CMD){}

void BotMotionControl::HandleHallSector(int node_id, SDO CMD){}

void BotMotionControl::HandleBusVoltage(int node_id, SDO CMD){}

void BotMotionControl::HandlePWMAlarmMethod(int node_id, SDO CMD){}

void BotMotionControl::HandleOverloadHandlingMethod(int node_id, SDO CMD){}

void BotMotionControl::HandleLastFaultCode(int node_id, SDO CMD){}

void BotMotionControl::HandleTargetSpeed(int node_id, SDO CMD){}

void BotMotionControl::HandleControlWord(int node_id, SDO CMD){}

void BotMotionControl::HandleStatusWord(int node_id, SDO CMD){}

void BotMotionControl::HandleQuickStopCode(int node_id, SDO CMD){}

void BotMotionControl::HandleCloseActionCode(int node_id, SDO CMD){}

void BotMotionControl::HandleDisableActionCode(int node_id, SDO CMD){}

void BotMotionControl::HandleHaltControlRegister(int node_id, SDO CMD){}

void BotMotionControl::HandleSetRunMode(int node_id, SDO CMD){}

void BotMotionControl::HandleRunningModeStatus(int node_id, SDO CMD){}

void BotMotionControl::HandleTargetTorque(int node_id, SDO CMD){}

void BotMotionControl::HandleRealTimeTorque(int node_id, SDO CMD){}

void BotMotionControl::HandleActualPosition(int node_id, SDO CMD){}

void BotMotionControl::HandleTargetPosition(int node_id, SDO CMD){}

void BotMotionControl::HandleTorqueGradient(int node_id, SDO CMD){}

void BotMotionControl::HandleActualSpeed(int node_id, SDO CMD){}

void BotMotionControl::HandlePositionModeStartStopSpeed(int node_id, SDO CMD){}

void BotMotionControl::HandleEmergencyStopDecelerationTime(int node_id, SDO CMD){}

void BotMotionControl::HandleCanMessage_001(uint8_t* data){
    union{
      float f;
      uint8_t bytes[4];
    }rdata;
    for(int i=0;i<4;i++){
      rdata.bytes[i] = data[i];
    }
    if(is_log_time){
      PX4_INFO("TEST rdata.f == %lf",(double)rdata.f);
    }


}













