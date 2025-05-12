#include "BotSensor.hpp"

  uint8_t BotSensor::get_ultrasonic_status(){

    uint16_t distance1 = bot_distance_sensor_sub.distance1;
    uint16_t distance2 = bot_distance_sensor_sub.distance2;
    uint16_t distance3 = bot_distance_sensor_sub.distance3;
    uint16_t distance4 = bot_distance_sensor_sub.distance4;

    // if(is_log_time){
    //     PX4_INFO("BotSensor distance1: %d,distance2: %d,distance3: %d,distance4: %d", distance1,distance2,distance3,distance4);
    //     PX4_INFO("bot_sensor_event_sub.ultrasonic_is_close=%d",bot_sensor_event_sub.ultrasonic_is_close);
    // }

    //如果超声波由上位机设置为关闭状态，不触发
    if(bot_sensor_event_sub.ultrasonic_is_close == 1){
      return 0;
    }


    //阈值设置默认值，如果上位机没有发送阈值数据，则使用默认值，如果上位发送阈值数据，使用上位机数据
    uint16_t ultrasonic_threshold_mm = DISTANCE_FREE_MM;
    if(bot_sensor_event_sub.ultrasonic_threshold > 0){
	     ultrasonic_threshold_mm = bot_sensor_event_sub.ultrasonic_threshold;
    }

    if (((distance1 > 0) && (distance1 < ultrasonic_threshold_mm)) || ((distance2 > 0) && (distance2 < ultrasonic_threshold_mm)) ||
		((distance3 > 0) && (distance3 < ultrasonic_threshold_mm)) || ((distance4 > 0) && (distance4 < ultrasonic_threshold_mm))) {
        return 1;
    }
    else {
        return 0;
    }

  }


