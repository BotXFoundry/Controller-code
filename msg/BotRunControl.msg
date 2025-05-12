uint64 timestamp

uint8 BOT_STATUS_OK = 0		#正常
uint8 BOT_STATUS_EMERGENCY_STOP = 1		#急停
uint8 BOT_STATUS_ERROR = 2		#系统异常

uint8 bot_status  ##车体状态，0x00：系统正常；0x01：急停；0x02：系统异常

uint8 BOT_CONTROL_MODE_STANDBY = 0		#待机
uint8 BOT_CONTROL_MODE_CAN = 1		#Can指令模式
uint8 BOT_CONTROL_MODE_RC = 2		#遥控模式
uint8 control_mode  ##控制模式，0x00：待机模式； 0x01：Can指令模式； 0x02：遥控模式  默认为待机模式
uint8 is_can_control
uint8 park_mode  ##停车方式，0x00 设置车体速度为0时，立即停车；0x01 设置给定车体速度为0后，缓慢停车；在遥控模式下，默认为0x00，立即停车


uint8 BOT_ROBOT_MODEL_NONE = 0		#未指定
uint8 BOT_ROBOT_MODEL_2WD = 2		#两轮差速
uint8 BOT_ROBOT_MODEL_4WD = 4		#四轮四驱
uint8 BOT_ROBOT_MODEL_ACK = 3		#阿克曼
uint8 BOT_ROBOT_MODEL_4WD_DIFF = 8       #四轮差速
uint8 robot_model ##本体运动模型: 0x02:两轮差速；0x03:阿克曼；0x04:四轮四驱；0x05:三轮；0x08:四轮差速

uint8 BOT_MOTION_MODE_PARK = 0		#驻车档
uint8 BOT_MOTION_MODE_FORWARD_REVERSE = 1		#前进/后退
##uint8 BOT_MOTION_MODE_REVERSE = 2		#后退
uint8 BOT_MOTION_MODE_ROLL = 2		#斜移
uint8 BOT_MOTION_MODE_ROTATE = 3		#自旋
uint8 BOT_MOTION_MODE_HORIZONTAL = 4		#横移
uint8 BOT_MOTION_MODE_TURN = 5		#转向
uint8 BOT_MOTION_MODE_NEUTRAL = 6		#空档
uint8 BOT_MOTION_MODE_HANDLE_BRAKE = 10		#手刹

uint8 motion_mode  ##档位，0x00 驻车档；0x01 前进/后退；0x02 斜移模式；0x03 自旋模式；0x04 横移模式；0x05 转向；0x06 空档；

uint8 handle_brake  #是否手刹

uint8 motion_mod_switch ##是否处于运动模式切换中，0:切换完成；1:切换中
uint8 over_speed  ##是否超速，0:未超速；1:超速

uint16 acc_l_time  #线速度从0加速到最大值时间，单位ms，默认值为100ms，如果上位机有设置，按照上位机设置值
uint16 dec_l_time  #线速度从最大值减速到0时间，单位ms，默认值为100ms，如果上位机有设置，按照上位机设置值
uint16 acc_a_time  #角速度从0加速到最大值时间，单位ms，默认值为100ms，如果上位机有设置，按照上位机设置值
uint16 dec_a_time  #角速度从最大值减速到0时间，单位ms，默认值为100ms，如果上位机有设置，按照上位机设置值

int16 linear_velocity  ##车体线速度  单位0.001m/s
int16 angular_velocity  ##车体角速度  单位0.01°/s
int16 body_angle  ##车体转角  单位0.01°

int16 body_speed_report
int16 body_angle_report
int16 body_angular_velocity_report

int16[5] wheel_speed_mms	#轴线速度
int16[5] wheel_pos	#轴转向角

int16[5] target_speed_rpm	#目标转速
int32[5] target_axis_angle	#电机轴目标转向角（适用于四轮驱动）

uint16 speed_kp
uint16 speed_ki
uint16 speed_kd

int16 linear_velocity_smooth
int16 angular_velocity_smooth





