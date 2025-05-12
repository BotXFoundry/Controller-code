#include "BotMotionControl.hpp"

void BotMotionControl::Bot4DControl() {

  motion_mode = bot_run_control_sub.motion_mode;

  // 为了减少通过485与转向舵机的通信，设置切换标志，状态有变化时给舵机发送指令
  int motion_mode_is_change = 0;
  if(motion_mode != motion_mode_last) //没有切换档位
  {
    motion_mode_is_change = 1;
  }

  // 0x07 空档
  if (motion_mode == bot_run_control_s::BOT_MOTION_MODE_NEUTRAL) {
    bot_run_control_sub.motion_mod_switch = 1;
    // 空档时，速度为0，方向回正，电机使能防止溜车
    velocity_smooth = 0;  // 设置线速度为0
    angular_smooth = 0;   // 设置角速度为0
    body_angle = 0;       // 设置车体转角为0
    // 前行速度为0
    Bot4DRunForward(0);
    // 轮子回正
    if(motion_mode_is_change){
        Bot4DSteeringToZero();
        steer_angle1 = 2048;
        steer_angle2 = 2048;
        steer_angle3 = 2048;
        steer_angle4 = 2048;
    }
    bot_run_control_sub.motion_mod_switch = 0;
  }
  // 0x00 驻车档
  else if(motion_mode == bot_run_control_s::BOT_MOTION_MODE_PARK){
     bot_run_control_sub.motion_mod_switch = 1;
     //驻车时，速度为0，轮子环抱
     velocity_smooth = 0;  // 设置线速度为0
     angular_smooth = 0;   // 设置角速度为0
     body_angle = 0;       // 设置车体转角为0
      // 前行速度为0
     Bot4DRunForward(0);
     // 轮子环抱
    if(motion_mode_is_change){
       Bot4DsteeringPark();//轮子相互八字的转向
    }
    bot_run_control_sub.motion_mod_switch = 0;
  }
  // 0x01前进、后退，直线前进，没有转向
  else if (motion_mode == bot_run_control_s::BOT_MOTION_MODE_FORWARD_REVERSE) {
    bot_run_control_sub.motion_mod_switch = 1;
    // 设置角速度、转向角为0
    angular_smooth = 0;  // 设置角速度为0
    body_angle = 0;      // 设置车体转角为0
    // 轮子回正
    if(motion_mode_is_change){
        Bot4DSteeringToZero();
        steer_angle1 = 2048;
        steer_angle2 = 2048;
        steer_angle3 = 2048;
        steer_angle4 = 2048;
    }
    // 前行
    Bot4DRunForward(velocity_smooth);
    bot_run_control_sub.motion_mod_switch = 0;
  }
  // 0x02 斜移模式，先设置转向角进行转向，然后按照线速度直线前进
  else if (motion_mode == bot_run_control_s::BOT_MOTION_MODE_ROLL) {
    bot_run_control_sub.motion_mod_switch = 1;
    //轮子转向对应角度
    Bot4DSteeringToSameAngle(angular_smooth);
    //前行
    Bot4DRunForward(velocity_smooth);
    bot_run_control_sub.motion_mod_switch = 0;

  }
  // 0x03 自旋模式，原地旋转
  else if (motion_mode == bot_run_control_s::BOT_MOTION_MODE_ROTATE) {
    bot_run_control_sub.motion_mod_switch = 1;

    //将舵机转到对应角度
    if(motion_mode_is_change){
      Bot4DSteeringRotation();
    }
    //线速度
    Bot4DRunForwardRotation(velocity_smooth);

    bot_run_control_sub.motion_mod_switch = 0;
  }
  // 0x04 横移模式，设置轮子横向即旋转90度，然后按照线速度直线前进
  else if (motion_mode == bot_run_control_s::BOT_MOTION_MODE_HORIZONTAL) {
    bot_run_control_sub.motion_mod_switch = 1;
    angular_smooth = 0;  // 设置角速度为0
    body_angle = 0;      // 设置车体转角为0
    // 轮子到横移状态；直接移到横移状态，不需要回正
    if(motion_mode_is_change){
      Bot4DSteeringTo180();
      steer_angle1 = 1024;
      steer_angle2 = 3071;
      steer_angle3 = 3071;
      steer_angle4 = 1024;
    }
    // 前行
    Bot4DRunForwardHorizontal(velocity_smooth);
    bot_run_control_sub.motion_mod_switch = 0;
  }
  // 0x05 转向,主要为行进中转向
  else if (motion_mode == bot_run_control_s::BOT_MOTION_MODE_TURN) {
    bot_run_control_sub.motion_mod_switch = 1;
    Bot4DRun4WD(velocity_smooth, angular_smooth);
    bot_run_control_sub.motion_mod_switch = 0;
  }
  //0x10 刹车，直接发送刹车指令
  else if(motion_mode == bot_run_control_s::BOT_MOTION_MODE_HANDLE_BRAKE){
    motor_to_stop = true;
  }

  motion_mode_last = motion_mode;

}

void BotMotionControl::Bot4DVelocitySmooth(int16_t vx) {
  // speed smooth
  while (abs(vx) != abs(velocity_smooth))
  {
      if ((vx < 0) && (velocity_smooth > 0)) {
      velocity_smooth = velocity_smooth - Acc_L;
    } else if ((vx > 0) && (velocity_smooth < 0)) {
      velocity_smooth = velocity_smooth + Acc_L;
    } else if (abs(vx) > abs(velocity_smooth)) {
      if (vx >= 0){
        velocity_smooth = velocity_smooth + Acc_L;
        }
      else if (vx < 0){
        velocity_smooth = velocity_smooth - Acc_L;
        }
    } else if (abs(vx) < abs(velocity_smooth)) {
      if (velocity_smooth >= 0){
        velocity_smooth = velocity_smooth - Acc_L;
        }
      else if (velocity_smooth < 0){
        velocity_smooth = velocity_smooth + Acc_L;
        }
    } else if (abs(vx) == abs(velocity_smooth)) {
      velocity_smooth = vx;
    }

    if (abs(velocity_smooth - vx) < (2 * Acc_L)) {  //TODO 原逻辑为 if (abs(velocity_smooth - vx) <= (2 * Acc_L))，但加上=逻辑后，不运行，待查
      velocity_smooth = vx;
    }
  }
}

void BotMotionControl::Bot4DAngleSmooth(int16_t rz) {
  // angular smooth
  while (abs(rz) != abs(angular_smooth))
  {
      if ((rz < 0) && (angular_smooth > 0)) {
      angular_smooth = angular_smooth - Acc_A;
    } else if ((rz > 0) && (angular_smooth < 0)) {
      angular_smooth = angular_smooth + Acc_A;
    } else if (abs(rz) > abs(angular_smooth)) {
      if (rz >= 0)
        angular_smooth = angular_smooth + Acc_A;
      else if (rz < 0)
        angular_smooth = angular_smooth - Acc_A;
    } else if (abs(rz) < abs(angular_smooth)) {
      if (angular_smooth >= 0)
        angular_smooth = angular_smooth - Acc_A;
      else if (angular_smooth < 0)
        angular_smooth = angular_smooth + Acc_A;
    } else if (abs(rz) == abs(angular_smooth)) {
      angular_smooth = rz;
    }

    if (abs(angular_smooth - rz) <= (2 * Acc_A)) {
      angular_smooth = rz;
    }
  }
}

// 设置电机工作模式
void BotMotionControl::SetupRunMode() {
  int16_t motor_run_mode = 2;  // 速度模式
  SetMotorRunMode(motor_run_mode);
}

/*前进后退不转弯*/
// speed_x_sign,有符号数，线速度
// 变量：sign,前进速度符号，0为正，前进，1为负，后退
// speed_x，前进速度mm/s
void BotMotionControl::Bot4DRunForward(int16_t speed) {
//根据电机特点，设置速度，在设置线速度为正值时，往前走
  PublishMotorSpeed(NODE_1, -speed);
  PublishMotorSpeed(NODE_2, speed);
  PublishMotorSpeed(NODE_3, -speed);
  PublishMotorSpeed(NODE_4, speed);

//因为PX4发送601、602指令到驱动器时，会有丢包现象，导致电机运转不一致，为了解决此问题，将两个电机数据封装成一个数据帧发送，确保电机执行同步  20241118
  BotMotorBatchSendSpeed();
}

//横移
void BotMotionControl::Bot4DRunForwardHorizontal(int16_t speed){

  PublishMotorSpeed(NODE_1, speed);
  PublishMotorSpeed(NODE_2, speed);
  PublishMotorSpeed(NODE_3, -speed);
  PublishMotorSpeed(NODE_4, -speed);

  BotMotorBatchSendSpeed();
}

/*底盘原地转向*/
// 角速度转向
// angular_sign，有符号数，原地旋转角速度
// 变量：angular_sign,旋转角速度符号，正，逆时针，负，顺时针
// angular_16，旋转角速度mrad/s
// speed_turn,mm/s
void BotMotionControl::Bot4DRunRotation(int16_t angular_16) {
  int16_t speed_turn = 0;
  speed_turn = (int16_t)(angular_16 * TURN_WHEN_STOP_RADIUS_MM * PI) / 180;
  PublishMotorSpeed(NODE_1, speed_turn);
  PublishMotorSpeed(NODE_2, speed_turn);
  PublishMotorSpeed(NODE_3, speed_turn);
  PublishMotorSpeed(NODE_4, speed_turn);

  BotMotorBatchSendSpeed();
}

//原地转向线速度设置
void BotMotionControl::Bot4DRunForwardRotation(int16_t speed){
  PublishMotorSpeed(NODE_1, speed);
  PublishMotorSpeed(NODE_2, speed);
  PublishMotorSpeed(NODE_3, speed);
  PublishMotorSpeed(NODE_4, speed);

  BotMotorBatchSendSpeed();
}

//自旋模式旋转角度设置
 void BotMotionControl::Bot4DSteeringRotation(){
  uint16_t data[4] = {0,0,0,0};

  //自旋模式时，设置各轮子转向到45度，45度对应舵机计数为512
	data[0] = (FRONTL_ANGLE_ZERO - 512 + 1);
	data[1] = (FRONTR_ANGLE_ZERO + 512 + 1);
	data[2] = (BACKL_ANGLE_ZERO + 512 + 1);
	data[3] = (BACKR_ANGLE_ZERO - 512 + 1);

  Bot4DSteeringToDegree(data);

    steer_angle1 = data[0];
    steer_angle2 = data[1];
    steer_angle3 = data[2];
    steer_angle4 = data[3];
 }

 //驻车模式轮子相互外八字转向
void  BotMotionControl::Bot4DsteeringPark(){
  uint16_t data[4] = {0,0,0,0};
	data[0] = (FRONTL_ANGLE_ZERO + 512 + 1);
	data[1] = (FRONTR_ANGLE_ZERO - 512 + 1);
	data[2] = (BACKL_ANGLE_ZERO - 512 + 1);
	data[3] = (BACKR_ANGLE_ZERO + 512 + 1);
  Bot4DSteeringToDegree(data);

    steer_angle1 = data[0];
    steer_angle2 = data[1];
    steer_angle3 = data[2];
    steer_angle4 = data[3];

}

/*底盘行驶中转向功能函数*/
// speed1前左，speed2前右，speed2后左，speed2后右，
// 变量：speed_sign，行驶线速度符号，0为正，前进，1为负，后退
// angular_sign,拐弯角速度符号，0为正，右转（顺时针前进或逆时针后退），1为负，左转（逆时针前进或顺时针后退），本函数没用到，参考
// speed，行驶线速度，实际转弯速度，mm/s
// angular，旋转角速度mrad/s，本函数没用到，参考
void BotMotionControl::Bot4DRunTurning(int16_t speed1, int16_t speed2,
                                       int16_t speed3, int16_t speed4) {

  PublishMotorSpeed(NODE_1, speed1);
  PublishMotorSpeed(NODE_2, speed2);
  PublishMotorSpeed(NODE_3, speed3);
  PublishMotorSpeed(NODE_4, speed4);

  BotMotorBatchSendSpeed();
}

//旋转指定角度值
void BotMotionControl::Bot4DSteeringToSameAngle(int16_t angle_turn) {

  uint16_t data[4] = {0,0,0,0};
  double angle = 0;
  int16_t step_angle = 0;

  angle = (double)((angle_turn/100)*PI / 180);
	step_angle = (int16_t)(angle / PI *  2048);

	data[0] = (FRONTL_ANGLE_ZERO-step_angle + 1);
	data[1] = (FRONTR_ANGLE_ZERO-step_angle + 1);
	data[2] = (BACKL_ANGLE_ZERO-step_angle + 1);
	data[3] = (BACKR_ANGLE_ZERO-step_angle + 1);

  Bot4DSteeringToDegree(data);

    steer_angle1 = data[0];
    steer_angle2 = data[1];
    steer_angle3 = data[2];
    steer_angle4 = data[3];

}


// 两轮驱动差速阿克曼运动学模型函数
// speed：行驶速度，mm/s
// angular_16：底盘转向角速度(角度制)，方向控制转向方向，单位0.01°/s，使用时先除以100
// angle_turn：底盘转向角度，用户上位机CAN给或遥控器给，单位0.01°，使用时先除以100
// 变量：angle：底盘转向角弧度，用户给定，弧度
// angular_sign,旋转角速度符号，0为正，逆时针，1为负，顺时针
void BotMotionControl::Bot4DRunAckermannModel(int16_t speed, int16_t angular_16,
                                              int16_t angle_turn) {
  uint8_t angular_sign = 0;

  double angle_front_in = 0;   // 前内轮转角弧度
  double angle_front_out = 0;  // 前外轮转角弧度
  int16_t step_front_in = 0;   // 前内轮转角步长
  int16_t step_front_out = 0;  // 前外轮转角步长

  int16_t velo_front_in = 0;
  int16_t velo_front_out = 0;
  int16_t velo_back_in = 0;
  int16_t velo_back_out = 0;
  double buf_tan_angle =
      0;  // tan(θ)缓存，根据线速度和角速度计算，为正数，θ为底盘转向角
  double angular_radian = 0;  // 转向角速度弧度，单位rad/s
  uint8_t steer_id[4] = {1, 2, 3, 4};
  uint16_t steer_position_right[4] = {0};
  uint16_t steer_position_left[4] = {0};
  uint16_t velocity[4] = {500, 500, 500, 500};
  uint16_t time[4] = {0, 0, 0, 0};

  angular_radian = ((double)angular_16 / 100) * PI /
                   180;  // 转向角速度（弧度制）,提供角速度时使用
  if (angular_16 >= 0) {
    angular_sign = 0;
    if (speed > 0)
      buf_tan_angle = (angular_radian * B / 1000) / ((double)speed / 1000);
    else if (speed < 0)
      buf_tan_angle =
          -(angular_radian * B / 1000) /
          ((double)speed /
           1000);  // 转向角速度弧度，以角速度和线速度计算转向角度转向使用
    else
      buf_tan_angle = 0;  // 速度为0，实际不存在
  } else if (angular_16 < 0) {
    angular_sign = 1;
    if (speed > 0)
      buf_tan_angle = -(angular_radian * B / 1000) / ((double)speed / 1000);
    else if (speed < 0)
      buf_tan_angle =
          (angular_radian * B / 1000) /
          ((double)speed /
           1000);  // 转向角速度弧度，以角速度和线速度计算转向角度转向使用
    else
      buf_tan_angle = 0;  // 速度为0，实际不存在
  }

  if (buf_tan_angle > 1.7) buf_tan_angle = 1.7;  // 限制转向角小于59.6°
  // 前轮角度
  angle_front_in = atan(2 * B * buf_tan_angle /
                        (2 * B - A * buf_tan_angle));  // 使用角速度时
  angle_front_out = atan(2 * B * buf_tan_angle /
                         (2 * B + A * buf_tan_angle));  // 使用角速度时
  step_front_in =
      (int16_t)(angle_front_in / PI * 4095);  // 计算前内轮子转角步长
  step_front_out =
      (int16_t)(angle_front_out / PI * 4095);  // 计算前外轮子转角步长

  // 四轮速度
  velo_front_in =
      (int16_t)((sqrt(pow((B / buf_tan_angle - A / 2), 2) + pow(B, 2)) /
                 sqrt(pow((B / 2), 2) + pow((B / buf_tan_angle), 2))) *
                speed);
  velo_front_out =
      (int16_t)((sqrt(pow((B / buf_tan_angle + A / 2), 2) + pow(B, 2)) /
                 sqrt(pow((B / 2), 2) + pow((B / buf_tan_angle), 2))) *
                speed);
  velo_back_in =
      (int16_t)(((B / buf_tan_angle - A / 2) /
                 sqrt(pow((B / 2), 2) + pow((B / buf_tan_angle), 2))) *
                speed);
  velo_back_out =
      (int16_t)(((B / buf_tan_angle + A / 2) /
                 sqrt(pow((B / 2), 2) + pow((B / buf_tan_angle), 2))) *
                speed);

  if (angular_sign == 1)  // 负方向，右转方向
  {
    steer_position_right[0] = (FRONTL_ANGLE_ZERO - step_front_out + 1);
    steer_position_right[1] = (FRONTR_ANGLE_ZERO - step_front_in + 1);
    steer_position_right[2] = BACKL_ANGLE_ZERO;
    steer_position_right[3] = BACKR_ANGLE_ZERO;

    steer_angle1 = steer_position_right[0];
    steer_angle2 = steer_position_right[1];
    steer_angle3 = steer_position_right[2];
    steer_angle4 = steer_position_right[3];

    Bot4DSteeringAsync(steer_id, steer_position_right, time, velocity, 4);
    Bot4DRunTurning(velo_front_out, -velo_front_in, velo_back_out,
                    -velo_back_in);
  } else if (angular_sign == 0)  // 正方向，左转方向
  {
    steer_position_left[0] = (FRONTL_ANGLE_ZERO + step_front_in - 1);
    steer_position_left[1] = (FRONTR_ANGLE_ZERO + step_front_out - 1);
    steer_position_left[2] = BACKL_ANGLE_ZERO;
    steer_position_left[3] = BACKR_ANGLE_ZERO;

    steer_angle1 = steer_position_left[0];
    steer_angle2 = steer_position_left[1];
    steer_angle3 = steer_position_left[2];
    steer_angle4 = steer_position_left[3];

    Bot4DSteeringAsync(steer_id, steer_position_left, time, velocity, 4);
    Bot4DRunTurning(velo_front_in, -velo_front_out, velo_back_in,
                    -velo_back_out);
  }
}

// 四轮驱动四轮转向差速运动学模型函数
// speed：行驶速度，mm/s
// angular_16：底盘转向角速度(角度制)，方向控制转向方向，单位0.01°/s，使用时先除以100
// angle_turn：底盘转向角度，用户上位机CAN给或遥控器给，单位0.01°，使用时先除以100
// 变量：angle：底盘转向角弧度，用户给定，弧度
// angular_sign,旋转角速度符号，0为正，逆时针，1为负，顺时针
void BotMotionControl::Bot4DRun4WD(int16_t speed, int16_t angular_16) {

  if (angular_16 != 0)
  {

    uint8_t angular_sign = 0;

    double angle_front_in = 0;   // 前内轮转角弧度
    double angle_front_out = 0;  // 前外轮转角弧度
    int16_t step_front_in = 0;   // 前内轮转角步长
    int16_t step_front_out = 0;  // 前外轮转角步长
    int16_t step_back_in = 0;    // 后内轮子转角步长
    int16_t step_back_out = 0;   // 后外轮子转角步长

    int16_t velo_front_in = 0;
    int16_t velo_front_out = 0;
    int16_t velo_back_in = 0;
    int16_t velo_back_out = 0;
    double buf_tan_angle =
        0;  // tan(θ)缓存，根据线速度和角速度计算，为正数，θ为底盘转向角
    double angular_radian = 0;  // 转向角速度弧度，单位rad/s
    uint8_t steer_id[4] = {1, 2, 3, 4};
    uint16_t steer_position_right[4] = {0,0,0,0};
    uint16_t steer_position_left[4] = {0,0,0,0};
    uint16_t velocity[4] = {500, 500, 500, 500};
    uint16_t time[4] = {0, 0, 0, 0};

    angular_radian = ((double)angular_16 / 100) * PI / 180;  // 转向角速度（弧度制）,提供角速度时使用

    if (angular_16 >= 0) {
      angular_sign = 0;
      if (speed > 0)
        buf_tan_angle = (angular_radian * B / 1000) / (2 * (double)speed / 1000);
      else if (speed < 0)
        buf_tan_angle =
            -(angular_radian * B / 1000) /
            (2 * (double)speed /
            1000);  // 转向角速度弧度，以角速度和线速度计算转向角度转向使用
      else
        buf_tan_angle = 0;  // 速度为0，实际不存在
    } else if (angular_16 < 0) {
      angular_sign = 1;
      if (speed > 0)
        buf_tan_angle = -(angular_radian * B / 1000) / (2 * (double)speed / 1000);
      else if (speed < 0)
        buf_tan_angle =
            (angular_radian * B / 1000) /
            (2 * (double)speed /
            1000);  // 转向角速度弧度，以角速度和线速度计算转向角度转向使用
      else
        buf_tan_angle = 0;  // 速度为0，实际不存在
    }
    if (buf_tan_angle > 5.6) buf_tan_angle = 5.6;  // 限制转向角小于80°

    // 四轮角度
    angle_front_in =
        atan(B * buf_tan_angle / (2 * B - A * buf_tan_angle));  // 使用角速度时
    angle_front_out =
        atan(B * buf_tan_angle / (2 * B + A * buf_tan_angle));  // 使用角速度时

      step_front_in =
        (int16_t)((angle_front_in / PI) * 4095);  // 计算前内轮子转角步长
    step_front_out =
        (int16_t)((angle_front_out / PI) * 4095);  // 计算前外轮子转角步长
    step_back_in = step_front_in;                // 后内轮子转角步长
    step_back_out = step_front_out;              // 后外轮子转角步长

    // 四轮速度
    velo_front_in = (int16_t)((sqrt(pow(((B / 2) / buf_tan_angle - A / 2), 2) +
                                    pow((B / 2), 2)) /
                              ((B / 2) / buf_tan_angle)) *
                              speed);
    velo_front_out = (int16_t)((sqrt(pow(((B / 2) / buf_tan_angle + A / 2), 2) +
                                    pow((B / 2), 2)) /
                                ((B / 2) / buf_tan_angle)) *
                              speed);
    velo_back_in = velo_front_in;
    velo_back_out = velo_front_out;

    if (angular_sign == 1)  // 负方向，右转方向
    {
      //根据电机特点进行转向 原来和现在对应关系：1-4 2-3 3-2 4-1
      steer_position_right[0] = (BACKR_ANGLE_ZERO + step_back_in - 1);
      steer_position_right[1] = (BACKL_ANGLE_ZERO + step_back_out - 1);
      steer_position_right[2] = (FRONTR_ANGLE_ZERO - step_front_in + 1);
      steer_position_right[3] = (FRONTL_ANGLE_ZERO - step_front_out + 1);

      steer_angle1 = steer_position_right[0];
      steer_angle2 = steer_position_right[1];
      steer_angle3 = steer_position_right[2];
      steer_angle4 = steer_position_right[3];

      Bot4DSteeringAsync(steer_id, steer_position_right, time, velocity, 4);
      Bot4DRunTurning(-velo_back_in,velo_back_out, -velo_front_in,velo_front_out);

    } else if (angular_sign == 0)  // 正方向，左转方向
    {
      steer_position_left[0] = (BACKR_ANGLE_ZERO - step_back_out + 1);
      steer_position_left[1] = (BACKL_ANGLE_ZERO - step_back_in + 1);
      steer_position_left[2] = (FRONTR_ANGLE_ZERO + step_front_out - 1);
      steer_position_left[3] = (FRONTL_ANGLE_ZERO + step_front_in - 1);

      steer_angle1 = steer_position_left[0];
      steer_angle2 = steer_position_left[1];
      steer_angle3 = steer_position_left[2];
      steer_angle4 = steer_position_left[3];

      Bot4DSteeringAsync(steer_id, steer_position_left, time, velocity, 4);
      Bot4DRunTurning(-velo_back_out,velo_back_in,-velo_front_out,velo_front_in);

    }
  }else if(angular_16 == 0)
  {
          Bot4DSteeringToZero();
          steer_angle1 = 2048;
          steer_angle2 = 2048;
          steer_angle3 = 2048;
          steer_angle4 = 2048;
      // 前行
      Bot4DRunForward(speed);
  }

}

/*底盘横移功能函数*/
// speed1前左，speed2前右，speed2后左，speed2后右，
void BotMotionControl::Bot4DRunEastWestGoing(int16_t speed1, int16_t speed2,
                                             int16_t speed3, int16_t speed4) {
  int16_t speed_node1 = 0;  // 前左
  int16_t speed_node3 = 0;  // 前右
  int16_t speed_node2 = 0;  // 后左
  int16_t speed_node4 = 0;  // 后右
  speed_node1 = -(int16_t)speed1;
  speed_node3 = -(int16_t)speed2;
  speed_node2 = -(int16_t)speed3;
  speed_node4 = -(int16_t)speed4;

  PublishMotorSpeed(NODE_1, speed_node1);
  PublishMotorSpeed(NODE_2, speed_node3);
  PublishMotorSpeed(NODE_3, speed_node2);
  PublishMotorSpeed(NODE_4, speed_node4);
  BotMotorBatchSendSpeed();
}

// 横移底盘运动函数
// angle_turn：底盘转向角度，用户上位机CAN给或遥控器给，单位0.01°，使用时先除以100
void BotMotionControl::Bot4DRunEastWest(int16_t speed, int16_t angle_turn) {
  int16_t step_angle = 0;
  double angle = 0;
  uint8_t steer_id[4] = {1, 2, 3, 4};
  uint16_t steer_position[4] = {0};
  uint16_t velocity[4] = {500, 500, 500, 500};
  uint16_t time[4] = {0, 0, 0, 0};
  angle = (double)((angle_turn / 100) * PI / 180);
  step_angle = (int16_t)(angle / PI * 2048);  // 计算转角步长
  steer_position[0] = (FRONTL_ANGLE_ZERO - step_angle + 1);
  steer_position[1] = (FRONTR_ANGLE_ZERO - step_angle + 1);
  steer_position[2] = (BACKL_ANGLE_ZERO - step_angle + 1);
  steer_position[3] = (BACKR_ANGLE_ZERO - step_angle + 1);

    steer_angle1 = steer_position[0];
    steer_angle2 = steer_position[1];
    steer_angle3 = steer_position[2];
    steer_angle4 = steer_position[3];

  Bot4DSteeringAsync(steer_id, steer_position, time, velocity, 4);
  Bot4DRunEastWestGoing(speed, -speed, speed, -speed);
}

void  BotMotionControl::Bot4DComputerSpeedAngle() {

  int16_t body_speed_report = 0; //线速度
  int16_t body_angular_velocity_report = 0; //角速度
  // int16_t body_angle_report = 0; //转角度，单位：0.01度  不需要计算，只需要反馈各舵机的角度，按照0-360度反馈舵机值上报给上位机，由上位机的定位导航根据实际在地图中的位置判断姿态
  double angle_feed = 0;
  double angle_tan = 0;

  int16_t wheel_speed[5] = {0,0,0,0,0};
  int16_t wheel_angle[5] = {0,0,0,0,0};

   wheel_speed[1] = 0;
   wheel_angle[1] = 0;
   wheel_speed[3] = 0;
   wheel_angle[3] = 0;
   wheel_speed[4] = 0;
   wheel_angle[4] = 0;

  int16_t body_speed_buf = 0;  // speed buf

  int16_t wheel_pos[4] = {0,0,0,0};
  // Bot4DGetSteeringPos(wheel_pos);  // TODOTODOTODO
  // 临时方案
  wheel_pos[0] = steer_angle1;
  wheel_pos[1] = steer_angle2;
  wheel_pos[2] = steer_angle3;
  wheel_pos[3] = steer_angle4;

  // 各轮子转速
  int16_t wheel_speed_rpm[5] = {0,0,0,0,0};
  // TODO:此处要认真复核且保持全局电机ID跟运动模型一致，要不数据会整体错乱
   wheel_speed_rpm[1] = bot_motor_control_sub.speed_rpm[1];
   wheel_speed_rpm[2] = bot_motor_control_sub.speed_rpm[2];
   wheel_speed_rpm[3] = bot_motor_control_sub.speed_rpm[3];
   wheel_speed_rpm[4] = bot_motor_control_sub.speed_rpm[4];

  // get wheel speed
  double wheel_speed_mms[5] = {0,0,0,0,0};
   wheel_speed_mms[1] = (double)(wheel_speed_rpm[1] * 0.1 * PI *
                                         WHEEL_DIAMETER_MM / 60);  // mm/s
   wheel_speed_mms[2] = (double)(wheel_speed_rpm[2] * 0.1 * PI *
                                         WHEEL_DIAMETER_MM / 60);  // mm/s
   wheel_speed_mms[3] = (double)(wheel_speed_rpm[3] * 0.1 * PI *
                                         WHEEL_DIAMETER_MM / 60);  // mm/s
   wheel_speed_mms[4] = (double)(wheel_speed_rpm[4] * 0.1 * PI *
                                         WHEEL_DIAMETER_MM / 60);  // mm/s

  if (bot_run_control_sub.motion_mode ==
          bot_run_control_s::BOT_MOTION_MODE_PARK || // 0，驻车档
      bot_run_control_sub.motion_mode ==
          bot_run_control_s::BOT_MOTION_MODE_FORWARD_REVERSE || //1，前进/后退
      bot_run_control_sub.motion_mode ==
          bot_run_control_s::BOT_MOTION_MODE_HORIZONTAL || //4，横移
      bot_run_control_sub.motion_mode ==
          bot_run_control_s::BOT_MOTION_MODE_NEUTRAL) { //6，空档
    body_speed_buf = -(int16_t)((wheel_speed_rpm[1] + wheel_speed_rpm[3] -
                                 wheel_speed_rpm[4]) /
                                3.0 * WHEEL_DIAMETER * 0.1 / 60 * PI *
                                1000);  // mm/s,3 wheels
    if (((body_speed_buf >= 0) && (bot_run_control_sub.linear_velocity >= 0)) ||
        ((body_speed_buf < 0) && (bot_run_control_sub.linear_velocity < 0)))
      body_speed_report = body_speed_buf;
    body_angular_velocity_report = 0;
  }
  else if (bot_run_control_sub.motion_mode ==
             bot_run_control_s::BOT_MOTION_MODE_ROTATE)  // turn when stop 4，自旋
  {
    body_speed_report = 0;
    body_angular_velocity_report =
        (int16_t)((wheel_speed_rpm[1] + wheel_speed_rpm[3] +
                   wheel_speed_rpm[4]) /
                  3.0 * WHEEL_DIAMETER * 0.1 / 60 * PI /
                  TURN_WHEN_STOP_RADIUS_M * 1000);  // mrad/s,3wheels
  }
  else if (bot_run_control_sub.motion_mode ==
           bot_run_control_s::BOT_MOTION_MODE_TURN)  // turn when run 6，行进中转向
  {
    if (bot_run_control_sub.robot_model ==
        bot_run_control_s::BOT_ROBOT_MODEL_ACK)  // 2wheels ackerman
    {
      if ((wheel_pos[0] < FRONTL_ANGLE_ZERO) &&
          (wheel_pos[1] < FRONTR_ANGLE_ZERO))  // turn right,1wheel out,2wheel in
      {
        angle_feed = (FRONTL_ANGLE_ZERO - wheel_pos[0]) * PI /
                     2048;  // left front wheel/radian
        angle_tan = 2 * B * tan(angle_feed) /
                    (2 * B - A * tan(angle_feed));  // tan(angle),fl

        wheel_speed[1] = -(
            int16_t)(wheel_speed_mms[1] /
                     (sqrt(pow((B / angle_tan + A / 2), 2) + pow(B, 2)) /
                      sqrt(pow((B / 2), 2) + pow((B / angle_tan), 2))));  // fl
        wheel_angle[1] =
            -(int16_t)(angle_tan * 1000 * (double)wheel_speed[1] / B);

        wheel_speed[3] = -(
            int16_t)(wheel_speed_mms[3] /
                     ((B / angle_tan + A / 2) /
                      sqrt(pow((B / 2), 2) + pow((B / angle_tan), 2))));  // bl
        wheel_angle[3] =
            -(int16_t)(angle_tan * 1000 * (double)wheel_speed[3] / B);

        wheel_speed[4] =
            (int16_t)(wheel_speed_mms[4] /
                      ((B / angle_tan - A / 2) /
                       sqrt(pow((B / 2), 2) + pow((B / angle_tan), 2))));  // br
        wheel_angle[4] =
            -(int16_t)(angle_tan * 1000 * (double)wheel_speed[4] / B);

        body_speed_report = wheel_speed[1];
        if (abs(body_speed_report) < abs(wheel_speed[3]))
          body_speed_report = wheel_speed[3];
        if (abs(body_speed_report) < abs(wheel_speed[4]))
          body_speed_report = wheel_speed[4];

        body_angular_velocity_report = wheel_angle[1];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[3]))
          body_angular_velocity_report = wheel_angle[3];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[4]))
          body_angular_velocity_report = wheel_angle[4];
        if (body_angular_velocity_report > 0) {
          body_angular_velocity_report = -body_angular_velocity_report;
        }

      } else if ((wheel_pos[0] > FRONTL_ANGLE_ZERO) &&
                 (wheel_pos[1] >
                  FRONTR_ANGLE_ZERO))  // turn left,1wheel in,2wheel out
      {
        angle_feed = (wheel_pos[0] - FRONTL_ANGLE_ZERO) * PI /
                     2048;  // left front wheel/radian
        angle_tan = 2 * B * tan(angle_feed) /
                    (2 * B + A * tan(angle_feed));  // tan(angle)

        wheel_speed[1] =
            -(int16_t)(wheel_speed_mms[1] /
                       (sqrt(pow((B / angle_tan - A / 2), 2) + pow(B, 2)) /
                        sqrt(pow((B / 2), 2) + pow((B / angle_tan), 2))));
        wheel_angle[1] =
            (int16_t)(angle_tan * 1000 * (double)wheel_speed[1] / B);

        wheel_speed[3] =
            -(int16_t)(wheel_speed_mms[3] /
                       ((B / angle_tan - A / 2) /
                        sqrt(pow((B / 2), 2) + pow((B / angle_tan), 2))));
        wheel_angle[3] =
            (int16_t)(angle_tan * 1000 * (double)wheel_speed[3] / B);

        wheel_speed[4] =
            (int16_t)(wheel_speed_mms[4] /
                      ((B / angle_tan + A / 2) /
                       sqrt(pow((B / 2), 2) + pow((B / angle_tan), 2))));
        wheel_angle[4] =
            (int16_t)(angle_tan * 1000 * (double)wheel_speed[4] / B);

        body_speed_report = wheel_speed[1];
        if (abs(body_speed_report) < abs(wheel_speed[3]))
          body_speed_report = wheel_speed[3];
        if (abs(body_speed_report) < abs(wheel_speed[4]))
          body_speed_report = wheel_speed[4];

        body_angular_velocity_report = wheel_angle[1];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[3]))
          body_angular_velocity_report = wheel_angle[3];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[4]))
          body_angular_velocity_report = wheel_angle[4];
        if (body_angular_velocity_report < 0) {
          body_angular_velocity_report = -body_angular_velocity_report;
        }

      }
    }
  else if (bot_run_control_sub.robot_model ==
               bot_run_control_s::BOT_ROBOT_MODEL_4WD)  // 4wheels drive
    {
      if ((wheel_pos[0] < FRONTL_ANGLE_ZERO - 1) &&
          (wheel_pos[1] < FRONTR_ANGLE_ZERO - 1) &&
          (wheel_pos[2] > BACKL_ANGLE_ZERO + 1) &&
          (wheel_pos[3] >
           BACKR_ANGLE_ZERO + 1))  // turn right,1wheel out,2wheel in
      {
        // 1 wheel
        angle_feed = (FRONTL_ANGLE_ZERO - wheel_pos[0]) * PI /
                     2048;  // left front wheel/radian
        angle_tan =
            2 * B * tan(angle_feed) / (B - A * tan(angle_feed));  // tan(angle)
        if (angle_tan > 1.5) angle_tan = 1.5;
        body_speed_buf =
            -(int16_t)(wheel_speed_mms[1] /
                       (sqrt(pow(((B / 2) / angle_tan + A / 2), 2) +
                             pow((B / 2), 2)) /
                        ((B / 2) / angle_tan)));
        if (((body_speed_buf > 0) && (wheel_speed_mms[1] < 0)) ||
            ((body_speed_buf < 0) && (wheel_speed_mms[1] > 0))) {
          wheel_speed[1] = body_speed_buf;
        }
        wheel_angle[1] =
            -(int16_t)((angle_tan * 2.0 * (double)wheel_speed[1] / B) * 1000);

        // wheel 3
        angle_feed = (wheel_pos[2] - BACKL_ANGLE_ZERO) * PI /
                     2048;  // left back wheel/radian
        angle_tan =
            2 * B * tan(angle_feed) / (B - A * tan(angle_feed));  // tan(angle)
        if (angle_tan > 1.5) angle_tan = 1.5;
        body_speed_buf =
            -(int16_t)(wheel_speed_mms[3] /
                       (sqrt(pow(((B / 2) / angle_tan + A / 2), 2) +
                             pow((B / 2), 2)) /
                        ((B / 2) / angle_tan)));
        if (((body_speed_buf > 0) && (wheel_speed_mms[3] < 0)) ||
            ((body_speed_buf < 0) && (wheel_speed_mms[3] > 0))) {
          wheel_speed[3] = body_speed_buf;
        }
        wheel_angle[3] =
            -(int16_t)((angle_tan * 2.0 * (double)wheel_speed[3] / B) * 1000);

        // wheel 4
        angle_feed = (wheel_pos[3] - BACKR_ANGLE_ZERO) * PI /
                     2048;  // right back wheel/radian
        angle_tan =
            2 * B * tan(angle_feed) / (B + A * tan(angle_feed));  // tan(angle)
        if (angle_tan > 1.5) angle_tan = 1.5;
        body_speed_buf = (int16_t)(wheel_speed_mms[4] /
                                   (sqrt(pow(((B / 2) / angle_tan - A / 2), 2) +
                                         pow((B / 2), 2)) /
                                    ((B / 2) / angle_tan)));
        if (((body_speed_buf > 0) && (wheel_speed_mms[4] > 0)) ||
            ((body_speed_buf < 0) && (wheel_speed_mms[4] < 0))) {
          wheel_speed[4] = body_speed_buf;
        }
        wheel_angle[4] =
            -(int16_t)((angle_tan * 2.0 * (double)wheel_speed[4] / B) * 1000);
        body_speed_buf = wheel_speed[1];
        if (abs(body_speed_buf) < abs(wheel_speed[3]))
          body_speed_buf = wheel_speed[3];
        if (abs(body_speed_buf) < abs(wheel_speed[4]))
          body_speed_buf = wheel_speed[4];
        // if (((body_speed_buf > 0) && (Vx > 0)) || ((body_speed_buf <
        // 0) && (Vx < 0)))
        if (((body_speed_buf > 0) &&
             (bot_run_control_sub.linear_velocity > 0)) ||
            ((body_speed_buf < 0) && (bot_run_control_sub.linear_velocity < 0)))
          body_speed_report = body_speed_buf;

        body_angular_velocity_report = wheel_angle[1];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[3]))
          body_angular_velocity_report = wheel_angle[3];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[4]))
          body_angular_velocity_report = wheel_angle[4];
        if (body_angular_velocity_report > 0) {
          body_angular_velocity_report = -body_angular_velocity_report;
        }

      } else if ((wheel_pos[0] > FRONTL_ANGLE_ZERO) &&
                 (wheel_pos[1] > FRONTR_ANGLE_ZERO) &&
                 (wheel_pos[2] < BACKL_ANGLE_ZERO - 1) &&
                 (wheel_pos[3] <
                  BACKR_ANGLE_ZERO - 1))  // turn left,1wheel in,2wheel out
      {
        // wheel 1
        angle_feed = (wheel_pos[0] - FRONTL_ANGLE_ZERO) * PI /
                     2048;  // left front wheel/radian
        angle_tan =
            2 * B * tan(angle_feed) / (B + A * tan(angle_feed));  // tan(angle)
        if (angle_tan > 1.5) angle_tan = 1.5;
        body_speed_buf =
            -(int16_t)(wheel_speed_mms[1] /
                       (sqrt(pow(((B / 2) / angle_tan - A / 2), 2) +
                             pow((B / 2), 2)) /
                        ((B / 2) / angle_tan)));
        if (((body_speed_buf > 0) && (wheel_speed_mms[1] < 0)) ||
            ((body_speed_buf < 0) && (wheel_speed_mms[1] > 0))) {
          wheel_speed[1] = body_speed_buf;
        }
        wheel_angle[1] =
            (int16_t)((angle_tan * 2.0 * (double)wheel_speed[1] / B) * 1000);

        // wheel 3
        angle_feed = (BACKL_ANGLE_ZERO - wheel_pos[2]) * PI /
                     2048;  // left back wheel/radian
        angle_tan =
            2 * B * tan(angle_feed) / (B + A * tan(angle_feed));  // tan(angle)
        if (angle_tan > 1.5) angle_tan = 1.5;
        body_speed_buf =
            -(int16_t)(wheel_speed_mms[3] /
                       (sqrt(pow(((B / 2) / angle_tan - A / 2), 2) +
                             pow((B / 2), 2)) /
                        ((B / 2) / angle_tan)));
        if (((body_speed_buf > 0) && (wheel_speed_mms[3] < 0)) ||
            ((body_speed_buf < 0) && (wheel_speed_mms[3] > 0))) {
          wheel_speed[3] = body_speed_buf;
        }
        wheel_angle[3] =
            (int16_t)((angle_tan * 2.0 * (double)wheel_speed[3] / B) * 1000);

        // wheel 4
        angle_feed = (BACKR_ANGLE_ZERO - wheel_pos[3]) * PI /
                     2048;  // right back wheel/radian
        angle_tan =
            2 * B * tan(angle_feed) / (B - A * tan(angle_feed));  // tan(angle)
        if (angle_tan > 1.5) angle_tan = 1.5;
        body_speed_buf = (int16_t)(wheel_speed_mms[4] /
                                   (sqrt(pow(((B / 2) / angle_tan + A / 2), 2) +
                                         pow((B / 2), 2)) /
                                  ((B / 2) / angle_tan)));
        if (((body_speed_buf > 0) && (wheel_speed_mms[4] > 0)) ||
            ((body_speed_buf < 0) && (wheel_speed_mms[4] < 0))) {
          wheel_speed[4] = body_speed_buf;
        }
        wheel_angle[4] =
            (int16_t)((angle_tan * 2.0 * (double)wheel_speed[4] / B) * 1000);
        body_speed_buf = wheel_speed[1];
        if (abs(body_speed_buf) < abs(wheel_speed[3]))
          body_speed_buf = wheel_speed[3];
        if (abs(body_speed_buf) < abs(wheel_speed[4]))
          body_speed_buf = wheel_speed[4];
        // if (((body_speed_buf > 0) && (Vx > 0)) || ((body_speed_buf <
        // 0) && (Vx < 0)))
        if (((body_speed_buf > 0) &&
             (bot_run_control_sub.linear_velocity > 0)) ||
            ((body_speed_buf < 0) && (bot_run_control_sub.linear_velocity < 0)))
          body_speed_report = body_speed_buf;

        body_angular_velocity_report = wheel_angle[1];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[3]))
          body_angular_velocity_report = wheel_angle[3];
        if (abs(body_angular_velocity_report) < abs(wheel_angle[4]))
          body_angular_velocity_report = wheel_angle[4];
        if (body_angular_velocity_report < 0) {
          body_angular_velocity_report = -body_angular_velocity_report;
        }

      } else  // similar to line
      {
        body_speed_buf = -(int16_t)((wheel_speed_rpm[1] + wheel_speed_rpm[3] -
                                     wheel_speed_rpm[4]) /
                                    3.0 * WHEEL_DIAMETER * 0.1 / 60 * PI *
                                    1000);  // mm/s,3 wheels
        if (((body_speed_buf >= 0) &&
             (bot_run_control_sub.linear_velocity >= 0)) ||
            ((body_speed_buf < 0) && (bot_run_control_sub.linear_velocity < 0)))
          body_speed_report = body_speed_buf;
        body_angular_velocity_report = 0;
      }
    }
  }



  bot_run_control_sub.body_speed_report = body_speed_report;
  bot_run_control_sub.body_angular_velocity_report = body_angular_velocity_report;

  // if(is_log_time){
  //    PX4_INFO("TEST body_speed_report === %d",body_speed_report);
  //    PX4_INFO("TEST body_angle_report === %d",body_angular_velocity_report);
  // }

  bot_run_control_sub.wheel_speed_mms[1] = (int16_t)wheel_speed_mms[1];
  bot_run_control_sub.wheel_speed_mms[2] = (int16_t)wheel_speed_mms[2];
  bot_run_control_sub.wheel_speed_mms[3] = (int16_t)wheel_speed_mms[3];
  bot_run_control_sub.wheel_speed_mms[4] = (int16_t)wheel_speed_mms[4];

  bot_run_control_sub.wheel_pos[1] = wheel_pos[0];
  bot_run_control_sub.wheel_pos[2] = wheel_pos[1];
  bot_run_control_sub.wheel_pos[3] = wheel_pos[2];
  bot_run_control_sub.wheel_pos[4] = wheel_pos[3];

}

  void BotMotionControl::PublishMotorSpeed(int node_id, int16_t speed_meter_per_second){

   int16_t speed_rpm = (speed_meter_per_second * 60) /
                      (PI * WHEEL_DIAMETER_MM);  // mm/min除以mm/转
   if(node_id == NODE_1){
    bot_run_control_sub.target_speed_rpm[NODE_1] = speed_rpm;
   }
   if(node_id == NODE_2){
    bot_run_control_sub.target_speed_rpm[NODE_2] = speed_rpm;
   }
    if(node_id == NODE_3){
    bot_run_control_sub.target_speed_rpm[NODE_3] = speed_rpm;
   }
    if(node_id == NODE_4){
    bot_run_control_sub.target_speed_rpm[NODE_4] = speed_rpm;
   }

  }
