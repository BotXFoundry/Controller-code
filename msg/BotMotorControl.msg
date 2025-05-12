uint64 timestamp		# time since system start (microseconds)

uint32[5] motor_device_ids	#电机节点ID
uint8[5] node_state 	#电机节点状态
int32[5] odometer_meter		#轮式里程计（0.001米）
uint16[5] encoder_count	#编码器脉冲数
int32[5] odometer_encode	#轮式里程计（编码器脉冲数）
uint16[5] sdo_command	#电机CAN SDO命令编码
uint16[5] error_code	#电机最新错误编码


uint16[5] disconnect_protection_time	#通讯掉线保护时间 范围0~32767
uint16[5] feedback_position_reset	#设置 反馈位置清零 ）0：无效 1：反馈位置清零不保存
uint16[5] absolute_position_mode_reset_to_zero	#设置 绝对位置模式时当前位置清零  0：无效 1：当前位置清零
uint16[5] restricted_parking_methods	#设置 限位停车方式 0:停止 1：急停 2：无效
uint16[5] starting_speed	#设置 起始速度 单位r/min  范围1-300r/min 默认1
uint16[5] maximum_speed	#设置 电机最大转速 单位r/min  范围1-300r/min
uint16[5] line_encoder	#设置 编码器线数设置 0-4096 默认1024
uint16[5] pole_pairs	#电机极对数
uint16[5] init_lock_axis_mode	#上电锁轴方式 0：不使能不锁轴，1：不使能锁轴
int16[5] hall_offset_angle	#电机与HALL的偏移角度 单位1度 范围 0度~360度
uint16[5] overload_factor	#过载系数  范围0-300,单位%  默认200
uint16[5] temperature_protection_threshold	#电机温度保护阈值 单位0.1度 范围0-1200(*0.1) 默认800
uint16[5] limited_current	#额定电流  驱动器输出的额定电流 单位0.1A 范围0-150 缺省150
uint16[5] maximum_current	#设置 最大电流 驱动器输出的最大电流  单位0.1A 范围0-300 缺省300
uint16[5] overload_protection_time	#设置 过载保护时间 驱动器过载保护时间 单位10ms 范围0-6553 缺省300
uint16[5] encoder_diff_alarm_threshold	#设置 超差报警阈值 编码器超差阈值 单位*10counts 范围1-6553 缺省409
uint16[5] smoothing_factor	#设置 过度平滑系数 0-30000  缺省1000
uint16[5] feed_forward_smoothing_factor	#前馈输出平滑系数 0-30000 缺省100
uint16[5] duty_output_smoothing_factor	#转矩输出平滑系数 0-30000 缺省100
int16[5] motor_temperature	#电机温度0.1度
int16[5] driver_temperature	#驱动器温度0.1度
uint16[5] hall_sector	#霍尔扇区
int16[5] motor_current	#电机当前电流0.01A）
int16[5] motor_voltage	#电机当前电压0.01V）
uint16[5] pwm_alarm_method	#报警PWM处理方式 0：关闭 1：开启
uint16[5] overload_handling_method	#过载处理方式 0：关闭 1：开启
uint16[5] last_fault_code	#驱动器最近一次故障码
uint16[5] control_word	#设置 控制字
uint16[5] status_word	#设置 状态字
uint16[5] quick_stop_code	#快速停止代码 快速停止命令后驱动器处理方式 默认5  5:正常停止，维持快速停止状态，6：急减速停，维持快速停止状态，7：急停，维持快速停止状态
uint16[5] close_action_code	#关闭操作代码 关闭命令后驱动器处理方式 0：无效  1：正常停止，转到Ready to switch on状态，默认1
uint16[5] disable_action_code	#设置 禁用操作代码  禁用操作命令后驱动器处理方式 0：无效 1：正常停止，转到Switched On 状态 默认1
uint16[5] halt_control_register	#设置 Halt控制寄存器  控制字Halt命令后驱动器处理方式 1：正常停止，维持Operation Enabled状态，2：急速减停，维持Operation Enabled状态，3：急停，维持Operation  Enabled状态，默认：1
uint16[5] run_mode	#运行模式
int32[5] actual_position	#实际位置
int32[5] speed_rpm	#电机转速（RPM)
int16[5] speed_erpm	#电机转速（RPM)电角度速度
int32[5] target_speed_rpm	#目标速度  速度模式时的目标速度  范围:-300~300r/min
int32[5] target_torque	#目标转矩 单位mA,范围-30000~30000 默认：0
int16[5] real_time_torque	#实时转矩反馈 单位0.1A,范围-300~300 默认：0
int32[5] target_position	#目标位置 位置模式运行总脉冲数范围：-1000000~1000000
int32[5] max_speed_rpm	#最大速度 位置模式时的最大速度 范围1-300r/min
uint16[5] position_mode_start_stop_speed	#设置 位置模式启/停速度  范围1-300r/min 默认1r/min
uint16[5] emergency_stop_time	#急停减速时间 范围0~32767ms 缺省10ms
uint16[5] torque_gradient	#转矩斜率 电流/1000/second   单位mA/s  缺省300ms
uint16[5] software_version	#软件版本号
uint16[5] hardware_version	#硬件件版本号
int16[5] motor_tl   #负载转矩

uint16[5] foc_current_kp	#
uint16[5] foc_current_ki	#
float32[5] s_pid_kp	#
float32[5] s_pid_ki	#
float32[5] s_pid_kd	#
uint16[5] p_pid_kp	#
uint16[5] p_pid_ki	#
uint16[5] p_pid_kd	#

uint16 s_pid_ramp_erpms_s = 10000
float32[5] m_speed_pid_set_rpm  #增量增长后的速度
float32[5] m_speed_prev_error
float32[5] m_speed_d_filter
float32 s_pid_kd_filter = 0.2
float32[5] m_speed_i_term
float32[5] m_iq_set

float32 L_CURRENT_MAX_SCALE = 1.0

#float32 L_CURRENT_MAX = 10.0   #与电机参数有关,6.5寸双出轴
#float32 L_CURRENT_MAX = 5.0   #与电机参数有关,6.5寸双出轴
#float32 L_CURRENT_MAX = 6.0   #与电机参数有关



