uint64 timestamp			# time since system start (microseconds)
uint8 device_id			# Input source

uint8 BOT_EVENT_POWER_OFF = 0		#电源按钮关
uint8 BOT_EVENT_POWER_ON = 1		#电源按钮开
uint8 BOT_EVENT_BATTERY_CHARGE_ON = 2		#开始电池充电
uint8 BOT_EVENT_BATTERY_CHARGE_OFF = 3		#结束电池充电
uint8 BOT_EVENT_TOUCH_SENSOR_ON = 4		#接触传感器(防碰撞传感器)开
uint8 BOT_EVENT_TOUCH_SENSOR_OFF = 5		#接触传感器(防碰撞传感器)关
uint8 BOT_EVENT_EMERGENCY_BUTTON_ON = 6		#急停按钮开
uint8 BOT_EVENT_EMERGENCY_BUTTON_OFF = 7	#急停按钮关
uint8 BOT_EVENT_ULTRASONIC = 8                  #超声波状态
uint8 BOT_EVENT_FALL_DISTANCE_ALARM_ON = 10          #防跌落报警
uint8 BOT_EVENT_FALL_DISTANCE_ALARM_OFF = 11        #防跌落报警关闭

uint8 event_type			# Event type
uint8 event_flag

#uint8 power_flag			# 电源按钮状态 不需要提供
#uint8 charge_flag			# 充电状态 放到电池状态中提供
uint8 touch_sensor1_flag			# 接触传感器状态  防撞条1
uint8 touch_sensor2_flag			# 接触传感器状态  防撞条2
uint8 emergency_button_flag			# 接触传感器状态  急停开关
uint8 ultrasonic_flag                  # 超声波状态 0:正常；1:有障碍物，非正常
uint8 fall_distance_status             # 防跌落状态 0:正常；1:触发，非正常

uint16 touch_sensor_release_time   #防撞条解除时间 单位：s

uint16 ultrasonic_threshold  #超声波触发阈值，检测到此范围内有物体停车 单位mm
uint8 ultrasonic_is_close   #是否关闭超声波，如果关闭，所有的将同时关闭
uint8 back_ultrasonic_is_close #倒退时是否关闭超声波，即如果前方有障碍物，关闭超声波可以后退，否则不能运动
uint8 fall_sensor_is_close #防跌落传感器是否关闭

uint8 fall_threshold  #防跌落传感器阈值，高于此值停车 单位cm


uint16 battery_voltage   #电池总电压
int16 battery_current   #电池电流，充电为正，放电为负
uint16 battery_soc    #电池容量
int8 battery_mos   #MOS指示状态，0表示关闭，在充电；1表示打开，在放电

uint8 EVENT_QUEUE_LENGTH = 5


