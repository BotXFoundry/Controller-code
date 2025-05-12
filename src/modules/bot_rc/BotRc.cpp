#include "BotRc.hpp"

uint16_t last_value0 = 0;

bool BotRc::init() {
  // 上面为由加速度传感器消息调度，这里实际用100ms的固定周期调度
  ScheduleOnInterval(3000_us);

  return true;
}

void BotRc::Run() {

  BotRC_printnum ++;

  if(BotRC_printnum > LOG_RATE){
    //  is_log_time = true;
  }

  if (should_exit()) {
    ScheduleClear();
    exit_and_cleanup();
    return;
  }
  perf_begin(_loop_perf);
  perf_count(_loop_interval_perf);


  UpdateData();

/* 因为目前控制器检测不到遥控器的状态，所以根据is_can_control标志设定的值结合控制模式确定是否发送指令
 * 这就要求在遥控器开机后，先拨动一下控制模式即SB键，确定是遥控器接管；或者在can接管时，遥控器即使没有关闭并且在遥控档位，也需要遥控器拨动一下才能遥控器接管
 */
  ManualControl();

  ManualControl4D();

  if(bot_run_control_sub.is_can_control == 2){
    PublishData();
  }

  if(BotRC_printnum > LOG_RATE){
      BotRC_printnum = 0;
      is_log_time = false;
  }
  perf_end(_loop_perf);
}

void BotRc::ManualControl(){
  if((input_rc.values[0] >= 1913) && (input_rc.values[0] <= 1953)) //上下20冗余
   {
	    input_rc.values[0] = 1933;
   }
   else if((input_rc.values[0] >= 1075) && (input_rc.values[0] <= 1115)) //上下20冗余
   {
	    input_rc.values[0] = 1095;
   }else if((input_rc.values[0] <= 1534) &&(input_rc.values[0] >= 1494))
   {
      input_rc.values[0] = 1514;
   }
  uint16_t value0 = input_rc.values[0];
  if(value0 != last_value0){
    bot_run_control_sub.is_can_control = 2;
  }
  last_value0 = value0;
}

void BotRc::UpdateData() {
  if (_input_rc_sub.updated()) {
    _input_rc_sub.copy(&input_rc);
  }
  if (_bot_run_control_sub.updated()) {
    _bot_run_control_sub.copy(&bot_run_control_sub);
  }
  if (_bot_sensor_event_sub.updated()) {
    _bot_sensor_event_sub.copy(&bot_sensor_event_sub);
  }
}


void BotRc::ManualControl4D() {

    int16_t rc_linear_velocity = 0;
    int16_t rc_angular_velocity = 0;
    int16_t rc_body_angle = 0;

    /************************************************************
    四轮底盘支持的控制模式比较多，需要使用较多通道。本代码中使用控制器与WFLY ET08A匹配，特对通道设置进行配置说明
    四轮支持驻车/空档、直行（前进/后退）、斜移、自旋、横移等模式，因为模式比较多，需要设置组合通道来区分
    针对WFLY ET08A遥控器，使用多个通道组合控制
    1、SB设置控制模式，拨到上侧为can模式，拨到中间为待机模式，拨到下侧为遥控模式
    2、SA拨到上侧，为驻车或空档模式
    //增加一路控制，区别开驻车和空档，当驻车时，轮子相互环抱状态
     2.1 SA拨到上侧，SD拨到下侧时，为空挡模式
     2.2 SA拨到上侧，SD拨到上侧时，为驻车模式
    3、SA拨到下侧，并且SD拨到下侧，为普通运动模式，即直行前进后退或者转弯，直行或者转弯由左右遥杆J1、J2来控制
    4、SA拨到下侧，并且SD拨到上侧，为特殊运行模式，即：斜移、自旋、横移
    5、SA拨到下侧，并且SD拨到上侧，SC拨到上侧时，为斜移模式；SC拨到中间时，为自旋模式；SC拨到下侧时，为横移模式
    6、SA拨到下侧，并且SD拨到下侧，SC拨到中间时，转向为阿克曼转向模式  --20241007
    7、SA拨到下侧，并且SD拨到下侧，SC拨到上侧时，为刹车模式  --20241210
    8、直行，前进或者后退、或者运行中转弯，使用左右遥杆J1、J2
    9、LD、RD及T1、T2、T3、T4微调按钮暂不使用


    通道设置：
    1、SB设置为1，取值索引为0
    2、SA设置为2，取值索引为1
    3、J4设置为7，取值索引为6，左摇杆横向  //因为通道4识别不出来，改为通道7
    4、J2设置为3，取值索引为2，右摇杆纵向
    5、SD设置为5，取值索引为4
    6、SC设置为6，取值索引为5

    WFLY ET08A没有速度档位通道，所以先按照预定义速度执行
    **************************************************************/

// if(is_log_time){
  // PX4_INFO("RCx_IN[1]: %d", input_rc.values[0]);
  // PX4_INFO("RC_IN[2]: %d", input_rc.values[1]);
  // PX4_INFO("RC_IN[3]: %d", input_rc.values[2]);
  // PX4_INFO("RC_IN[4]: %d", input_rc.values[3]);
  // PX4_INFO("RC_IN[5]: %d", input_rc.values[4]);
  // PX4_INFO("RC_IN[6]: %d", input_rc.values[5]);
  // }

  /****************************************************************
   * 对所有通道进行冗余设置
  *******************************************************************/
  if((input_rc.values[0] >= 1913) && (input_rc.values[0] <= 1953)){ //上下20冗余
     input_rc.values[0] = 1933;
   }else if((input_rc.values[0] >= 1075) && (input_rc.values[0] <= 1115)){ //上下20冗余
	  input_rc.values[0] = 1095;
   }else if((input_rc.values[0] <= 1534) &&(input_rc.values[0] >= 1494)){
     input_rc.values[0] = 1514;
   }

  if(input_rc.values[1] > 1075 && input_rc.values[1] < 1115){
    input_rc.values[1] = 1095;
  }else if(input_rc.values[1] > 1913 && input_rc.values[1] <1953){
    input_rc.values[1] = 1933;
  }

  if ((input_rc.values[2] <= 1534) &&(input_rc.values[2] >= 1494)){  // 为了避免操作误差，在比较值时取+-20的区间
	  input_rc.values[2] = 1514;
	}else if ((input_rc.values[2] <= 1953) &&(input_rc.values[2] >= 1913)){
	  input_rc.values[2] = 1933;
	}else if ((input_rc.values[2] <= 1115) &&(input_rc.values[2] >= 1075)){
	  input_rc.values[2] = 1095;
	}

  if ((input_rc.values[6] <= 1534) &&(input_rc.values[6] >= 1494)){  // 为了避免操作误差，在比较值时取+-20的区间
	  input_rc.values[6] = 1514;
	}else if ((input_rc.values[6] <= 1953) &&(input_rc.values[6] >= 1913)){
	  input_rc.values[6] = 1933;
	}else if ((input_rc.values[6] <= 1115) &&(input_rc.values[6] >= 1075)){
	  input_rc.values[6] = 1095;
	}

  if(input_rc.values[4] > 1075 && input_rc.values[4] < 1115){
    input_rc.values[4] = 1095;
  }else if(input_rc.values[4] > 1913 && input_rc.values[4] <1953){
    input_rc.values[4] = 1933;
  }

  if ((input_rc.values[5] <= 1534) &&(input_rc.values[5] >= 1494)){  // 为了避免操作误差，在比较值时取+-20的区间
	  input_rc.values[5] = 1514;
	}else if ((input_rc.values[5] <= 1953) &&(input_rc.values[5] >= 1913)){
	  input_rc.values[5] = 1933;
	}else if ((input_rc.values[5] <= 1115) &&(input_rc.values[5] >= 1075)){
	  input_rc.values[5] = 1095;
	}

  /****************************************************************
   * SB 为控制模式切换，三个档位，拨杆向下时为遥控器控制模式，拨杆向上时切换为CAN指令控制模式，在中间时为待机模式；（通道 1）
   * 通道1：上拨：1095, 中间：1514， 下拨：1933
   ****************************************************************/
  if (input_rc.values[0] == 1933)
  {
    bot_run_control_sub.control_mode = bot_run_control_s::BOT_CONTROL_MODE_RC;
  }
  else if(input_rc.values[0] == 1095)
  {
    // 通过遥控器切换到can模式时，车体设置为驻车档
    bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_PARK;
    bot_run_control_sub.control_mode = bot_run_control_s::BOT_CONTROL_MODE_CAN;
  }
  else
  {
    bot_run_control_sub.control_mode = bot_run_control_s::BOT_STATUS_OK;
  }

  /*****************************************************************
   * can控制模式下，各数值获取
  *********************************************************************/

  if (bot_run_control_sub.control_mode ==
      bot_run_control_s::BOT_CONTROL_MODE_RC)  // 遥控器控制模式,开始遥控器控制
  {
  // 遥控速度为WFLY遥控器右侧旋摇杆(J2)上下控制，J2 前后为油门控制，摇杆的前后运动控制底盘前进和后退(通道3)
	//通道3： 中间位置：1514  最上端：1933  最下端：1095
    rc_linear_velocity = ((double)((input_rc.values[2] - 1514) * RC_LINEAR_VELOCITY) / 419);  //正数, 1514-1095=419,1094为了保证V1最小值时，调节速度按钮正常使用
  //J1 左右为方向控制摇杆，摇杆的左右运动控制底盘的左右转向（通道 4）
  //通道4：最左端：1095, 中间：1514， 最右端：1933
      rc_angular_velocity = ((double)(input_rc.values[6] - 1514)/419) * RC_ANGULAR_VELOCITY;
      //斜移模式下，将J1摇杆横向控制设置为角度，限幅180度，统一单位0.01度，值为18000
      rc_body_angle = ((double)((input_rc.values[6] - 1514) * RC_BODY_ANGLE) / 419);

    //驻车、空档、斜移、自旋、横移
    //SA拨到上侧，SD在下侧，为空档模式 0x07
    if(input_rc.values[1] == 1095 && input_rc.values[4] == 1933){
      bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_NEUTRAL;
      rc_linear_velocity = 0;
      rc_angular_velocity = 0;
      rc_body_angle = 0;
    }

    //SA拨到上侧，SD拨到上侧，为驻车模式 0x00
    if(input_rc.values[1] == 1095 && input_rc.values[4] == 1095){
      bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_PARK;
      rc_linear_velocity = 0;
      rc_angular_velocity = 0;
      rc_body_angle = 0;
    }

    //斜移
    if(input_rc.values[1] == 1933 && input_rc.values[4] == 1095 && input_rc.values[5] == 1095){
      bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_ROLL;
      // rc_angular_velocity = 0;
    }

    //自旋模式
    if(input_rc.values[1] == 1933 && input_rc.values[4] == 1095 && input_rc.values[5] == 1514){
      bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_ROTATE;
    }

    //横移
    if(input_rc.values[1] == 1933 && input_rc.values[4] == 1095 && input_rc.values[5] == 1933){
      bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_HORIZONTAL;
    }

    //直行或运行中转向
    if(input_rc.values[1] == 1933 && input_rc.values[4] == 1933 && input_rc.values[5] == 1933){
      if (rc_angular_velocity == 0) //直行，前进或者后退，没有转向
      {
        bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_FORWARD_REVERSE;
      }
      if(rc_angular_velocity != 0)//运行中转向
      {
        bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_TURN;
      }
    }

    //转向使用阿克曼模式
    if(input_rc.values[1] == 1933 && input_rc.values[4] == 1933 && input_rc.values[5] == 1514){
      bot_run_control_sub.robot_model = 0x03;//阿克曼转向模式
    }else{
       bot_run_control_sub.robot_model = 0x04;//全转向模式
    }

    //手刹模式
    if(input_rc.values[1] == 1933 && input_rc.values[4] == 1933 && input_rc.values[5] == 1095){
      // bot_run_control_sub.motion_mode = bot_run_control_s::BOT_MOTION_MODE_HANDLE_BRAKE;
      bot_run_control_sub.handle_brake = 1;
    }else{
      bot_run_control_sub.handle_brake = 0;
    }
  }

    // 增加失控保护
    if (rc_channels.signal_lost){
      rc_linear_velocity = 0;
      rc_angular_velocity = 0;
      rc_body_angle = 0;
    }

      bot_run_control_sub.linear_velocity = rc_linear_velocity;
      bot_run_control_sub.angular_velocity = rc_angular_velocity;
      bot_run_control_sub.body_angle = rc_body_angle;
}

void BotRc::PublishData() {
  _bot_sensor_event_pub.publish(bot_sensor_event_sub);
  _bot_run_control_pub.publish(bot_run_control_sub);

}

extern "C" __EXPORT int bot_rc_main(int argc, char* argv[]) {
  return BotRc::main(argc, argv);
}

BotRc::BotRc()
    : ModuleParams(nullptr),
      // ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}
       ScheduledWorkItem("bot_rc", px4::wq_configurations::bot_rc) {}

BotRc::~BotRc() {
  perf_free(_loop_perf);
  perf_free(_loop_interval_perf);
}

int BotRc::task_spawn(int argc, char* argv[]) {
  BotRc* instance = new BotRc();

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

int BotRc::custom_command(int argc, char* argv[]) {
  return print_usage("unknown command");
}

int BotRc::print_usage(const char* reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description


)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("bot_rc", "system");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}
