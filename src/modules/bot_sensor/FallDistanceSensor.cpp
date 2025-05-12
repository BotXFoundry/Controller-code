#include "BotSensor.hpp"

  uint8_t BotSensor::get_Fall_Distance_Status(){

    // uint16_t fall_distance1 = bot_sensor_com_data_sub.fall_distance1;
    uint16_t fall_distance2 = bot_sensor_com_data_sub.fall_distance2;
    uint16_t fall_distance3 = bot_sensor_com_data_sub.fall_distance3;
    uint16_t fall_distance4 = bot_sensor_com_data_sub.fall_distance4;

    //阈值设置默认值，如果上位机没有发送阈值数据，则使用默认值，如果上位发送阈值数据，使用上位机数据
    uint8_t fall_threshold_cm = FALL_DISTANCE_CM;
    if(bot_sensor_event_sub.fall_threshold > 0){
	     fall_threshold_cm = bot_sensor_event_sub.fall_threshold;
    }

    if(bot_sensor_event_sub.fall_sensor_is_close == 1){
      return 0;
    }

    // if(fall_distance1 >= fall_threshold_cm || fall_distance2 >= fall_threshold_cm || fall_distance3 >= fall_threshold_cm || fall_distance4 >= fall_threshold_cm){
  if(fall_distance2 >= fall_threshold_cm || fall_distance3 >= fall_threshold_cm || fall_distance4 >= fall_threshold_cm){
	return 1;
    }else{
	return 0;
    }

  }


