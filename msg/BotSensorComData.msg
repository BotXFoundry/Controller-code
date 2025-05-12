uint64 timestamp			# time since system start (microseconds)

uint8 fall_distance1  #防跌落传感器1测试的距离
uint8 fall_distance2  #防跌落传感器2测试的距离
uint8 fall_distance3  #防跌落传感器3测试的距离
uint8 fall_distance4  #防跌落传感器4测试的距离
#uint8 fall_threshold  #防跌落传感器阈值，高于此值停车

uint8 touch_sensor1_flag			# 接触传感器状态  防撞条1
uint8 touch_sensor2_flag			# 接触传感器状态  防撞条2


int16 voltage_total   #总电压 单位10mV
int16 current   #电流  单位10mA
int16 capacity   #剩余容量  单位10mAh
int16 capacity_full_charge  #实际满充容量  单位10mAh

#保护状态
int8 over_voltage_unit  #单体过压 0未保护，1发生保护
int8 under_voltage_unit #单体欠压 0未保护，1发生保护
int8 over_voltage_group  #整组过压 0未保护，1发生保护
int8 under_voltage_group   #整组欠压 0未保护，1发生保护
int8 over_temperature_charge  #充电过温 0未保护，1发生保护
int8 low_temperature_charge   #充电低温 0未保护，1发生保护
int8 over_temperature_discharge  #放电过温 0未保护，1发生保护
int8 low_temperature_discharge  #放电低温 0未保护，1发生保护
int8 over_current_charge  #充电过流 0未保护，1发生保护
int8 over_current_discharge  #放电过流 0未保护，1发生保护
int8 short_circuit_protection  #短路保护 0未保护，1发生保护
int8 software_lock_mos  #软件锁定MOS 0未保护，1发生保护

int16 rsoc #剩余容量百分比
int8 mos_charge_status #充电状态 1：打开；0：关闭
int8 mos_discharge_status #放电状态 1:打开  0：关闭
