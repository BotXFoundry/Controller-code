#pragma once
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include "RS485.hpp"
#include "SocketCAN.hpp"

#include <uORB/topics/bot_can_message.h>
#include <uORB/topics/bot_can_message_receive.h>
#include <uORB/topics/bot_run_control.h>
#include <uORB/topics/bot_sensor_event.h>
#include <uORB/topics/bot_motor_control.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/bot_distance_sensor.h>
#include <uORB/topics/bot_sensor_com_data.h>

#define FOUR_WHEEL_55 1  //5.5寸四轮四驱 对应中四轮

// 配置是can模式还是canfd模式，支持两种模式
#define ISCAN 1
#define ISCANFD 0

#define BODY_TYPE_4D 4

#define NODE_1 1
#define NODE_2 2
#define NODE_3 3
#define NODE_4 4

#define SENSOR_DEVICE_COUNTS 6

#define PI 3.14159265358979323846


/********** 中四轮5.5寸双出轴配置 ***********/
#ifdef FOUR_WHEEL_55
#define A 400                  // mm 轮距
#define B 400                  // mm 轴距
#define WHEEL_DIAMETER_MM 139  // wheel diameter,mm
#define WHEEL_DIAMETER 0.139   // wheel diameter,m

#define Wheel_D 13.9        // 单位 cm
#define Wheel_Spacing 40  // 单位 cm 轮距

#define TURN_WHEN_STOP_RADIUS_M 0.41  // r_turn,m
#define TURN_WHEN_STOP_RADIUS_MM 410   // r_turn,mm

#define POLE_PAIRS 10 //电机极对数
#define L_CURRENT_MAX 5.0f //额定电流

#define MOTOR_BRAKE_CURRENT 0.5f //刹车电流初始值
#define MOTOR_BRAKE_CURRENT_MAX 1.0f  //刹车电流最大值
#define MOTOR_BRAKE_CURRENT_ACCL 0.0001f //刹车电流增加系数

#define LIMIT_LINE_SPEED 1500 //限制最大线速度1500mm/s
#define LIMIT_ANGLE_SPEED 8000 //限制最大角速度80°/s

#define AXIS_COUNT 4

#define RUN_BODY_TYPE BODY_TYPE_4D   //适配四轮

#define Track_Spacing  40     //单位 cm 轴距
#endif


#define ACC_L_TIME 100 //车体线速度加速时间，默认为100ms
#define DEC_L_TIME 200 //车体线速度减速时间，默认为100ms
#define ACC_A_TIME 100 //车体线速度加速时间，默认为100ms
#define DEC_A_TIME 200 //车体线速度减速时间，默认为100ms

#define HARDWARE_VERSION 3
#define SOFTWARE_VERSION 3

#define T 36  // 36mm/s/20ms,a=1.8

#define Acc_ALARM 44   // alarm stop/bump status线加速度mm/s2,2.2m/s2
#define Acc_L (1.0 * T)  // 线加速度，mm/s2,1.8m/s2
#define Acc_A 270.0    // 270*0.01degree/s/20ms,a=	135degree/s2=2.36rad/s2

//定义舵机0位置
#define FRONTL_ANGLE_ZERO 2048  // 左前修正回正角度
#define FRONTR_ANGLE_ZERO 2048  // 右前修正回正角度
#define BACKL_ANGLE_ZERO 2048   // 左后回正角度
#define BACKR_ANGLE_ZERO 2048   // 右后回正角度

#define VELO_STEERING 0   // 舵机高速度
#define ACCE_STEERING 50  // 舵机加速度

#define SPEED_FILTER_RATIO 0.2f  //速度滤波比例

#define CAN_DATA_LENGTH  240

#define LOG_RATE 200

#define UTIL_LP_FAST(value, sample, filter_constant) \
  (value -= (filter_constant) * ((value) - (sample)))

using namespace time_literals;

typedef struct {
  uint16_t NodeId;       // 节点ID（电机ID） 1：左 2:右
  uint8_t CmdCode;       // SDO命令编码(SDO报文字节0)
  uint8_t IndexCodeH;    // SDO索引编码高字节(SDO报文字节2)
  uint8_t IndexCodeL;    // SDO索引编码低字节(SDO报文字节1)
  uint8_t SubIndexCode;  // SDO子索引编码字节(SDO报文字节3)
  uint8_t BizData8;      // 8位业务字段
  uint16_t BizData16;    // 16位业务字段
  uint32_t BizData32;    // 32位业务字段
  uint8_t CmdDataLength;  // 根据CanOpen SDO规范命令对应的Data数据位长度
  uint8_t IsSetupCmd;  // 是否设置命令标志，不是设置就是读取命令
  uint8_t HasError;       // 命令执行过程中是否存在错误
  uint8_t ErrorType;      // 错误原因
  uint8_t AckSetCmdCode;  // 执行设置反馈SDO命令编码(SDO报文字节0)
  uint8_t AckGetCmdCode;  // 执行读取反馈SDO命令编码(SDO报文字节0)
  uint8_t AckDataLength;  // 根据CanOpen SDO规范命令对应的反馈Data数据位长度
  int16_t AckData16;        // 执行反馈16位业务字段
  int32_t AckData32;        // 执行反馈32位业务字段
  uint8_t AckErrorCmdCode;  // 执行反馈SDO命令编码(SDO报文字节0)
} SDO;

struct Bot2DCommand {
  double LinearVelocity;
  double AngularVelocity;

  double TargetSpeed_NodeL;
  double TargetSpeed_NodeR;

  double TargetRpm_NodeL;
  double TargetRpm_NodeR;

  int32_t TargetRpm_L;
  int32_t TargetRpm_R;
  double LinerVelocitySmooth;
  double AngularVelocitySmooth;
};

struct Bot4DDiffCommand {
  double LinearVelocity;
  double AngularVelocity;

  double TargetSpeed_NodeL;
  double TargetSpeed_NodeR;

  double TargetRpm_NodeL;
  double TargetRpm_NodeR;

  int32_t TargetRpm_L;
  int32_t TargetRpm_R;
  double LinerVelocitySmooth;
  double AngularVelocitySmooth;
};

class BotMotionControl : public ModuleBase<BotMotionControl>,
                         public ModuleParams,
                         public px4::ScheduledWorkItem {
 public:
  BotMotionControl();
  ~BotMotionControl() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char* argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char* argv[]);

  /** @see ModuleBase */
  static int print_usage(const char* reason = nullptr);

  bool init();

 private:
  void Run() override;
  void UpdateData();
  void PublishData();
  void CustomControlRate();

  // 以下是4WD实现代码
  void Bot4DControl();

  // 限制速度和角速度
  void Bot4DVelocitySmooth(int16_t vx);

  void Bot4DAngleSmooth(int16_t rz);

  uint64_t BotControlDiffTime_US();

  void BotMotorAccDecSpeedControl(int8_t node_id,int16_t now_speed,int16_t target_speed);

  // 设置电机工作模式
  void SetupRunMode();

   //设置手刹模式
  void BotMotorSetVToZero();

  void BotMotorStop();

  void BotHandleBrake();
  void BotReleaseHandleBrake();

  /*前进后退不转弯*/
  // speed_x_sign,有符号数，线速度
  // 变量：sign,前进速度符号，0为正，前进，1为负，后退
  // speed_x，前进速度mm/s
  void Bot4DRunForward(int16_t speed);

  //原地转向线速度设置
  void Bot4DRunForwardRotation(int16_t speed);

  //横移线速度设置
  void Bot4DRunForwardHorizontal(int16_t speed);

  //驻车的轮子相互外八字转向
  void Bot4DsteeringPark();

  /*底盘原地转向*/
  // 角速度转向
  // angular_sign，有符号数，原地旋转角速度
  // 变量：angular_sign,旋转角速度符号，正，逆时针，负，顺时针
  // angular_16，旋转角速度mrad/s
  // speed_turn,mm/s
  void Bot4DRunRotation(int16_t angular_16);

  /*底盘行驶中转向功能函数*/
  // speed1前左，speed2前右，speed2后左，speed2后右，
  // 变量：speed_sign，行驶线速度符号，0为正，前进，1为负，后退
  // angular_sign,拐弯角速度符号，0为正，右转（顺时针前进或逆时针后退），1为负，左转（逆时针前进或顺时针后退），本函数没用到，参考
  // speed，行驶线速度，实际转弯速度，mm/s
  // angular，旋转角速度mrad/s，本函数没用到，参考
  void Bot4DRunTurning(int16_t speed1, int16_t speed2, int16_t speed3,
                       int16_t speed4);

  //旋转至指定角度
  void Bot4DSteeringToSameAngle(int16_t angle_turn) ;


  // 两轮驱动差速阿克曼运动学模型函数
  // speed：行驶速度，mm/s
  // angular_16：底盘转向角速度(角度制)，方向控制转向方向，单位0.01°/s，使用时先除以100
  // angle_turn：底盘转向角度，用户上位机CAN给或遥控器给，单位0.01°，使用时先除以100
  // 变量：angle：底盘转向角弧度，用户给定，弧度
  // angular_sign,旋转角速度符号，0为正，逆时针，1为负，顺时针
  void Bot4DRunAckermannModel(int16_t speed, int16_t angular_16,
                              int16_t angle_turn);

  // 四轮驱动四轮转向差速运动学模型函数
  // speed：行驶速度，mm/s
  // angular_16：底盘转向角速度(角度制)，方向控制转向方向，单位0.01°/s，使用时先除以100
  // 变量：angle：底盘转向角弧度，用户给定，弧度
  // angular_sign,旋转角速度符号，0为正，逆时针，1为负，顺时针
  void Bot4DRun4WD(int16_t speed, int16_t angular_16);

  /*底盘横移功能函数*/
  // speed1前左，speed2前右，speed2后左，speed2后右，
  void Bot4DRunEastWestGoing(int16_t speed1, int16_t speed2, int16_t speed3,
                             int16_t speed4);

  // 横移底盘运动函数
  // angle_turn：底盘转向角度，用户上位机CAN给或遥控器给，单位0.01°，使用时先除以100
  void Bot4DRunEastWest(int16_t speed, int16_t angle_turn);

  void Bot4DComputerSpeedAngle();

  //自旋模式
  void Bot4DSteeringRotation();
  // 以上是4WD实现代码

  void BotMotorBatchSendTorque();
  void BotMotorBatchSendSpeed();

  void BotMotorBatchSetTorqueToZero();
  void ComputeBrakeCurrent();
  void InitBrakeCurrent();
  void BotSpeedAccDecSmooth();

  // 以下为接收消息处理接口
  void DispatchCanMessage();
  void HandleCanMessageInterval();
  void HandleCanMessage(uint32_t can_id_in, uint8_t can_data_in[8]);

  void HandleCanMessge_iq(int node_id,int32_t iq);

  void HandleCanBatchMessge_iq(uint32_t iq_can_id,int32_t node_iq1,int32_t node_iq2);

   void HandleCanBatchMessge_speed(uint32_t iq_can_id,int32_t node_iq1,int32_t node_iq2);

  void HandleImuMessage_261_2(const sensor_combined_s& data,bool batch_report);
  void HandlerMagMessage_265(const sensor_mag_s& data,bool batch_report);
  void HandleBaroMessage_263(const sensor_baro_s&data0, const sensor_baro_s&data1,bool batch_report);
  void HandleBaroAvrMessage_264(const sensor_baro_s&data0, const sensor_baro_s&data1,bool batch_report);

  // 驱动器设置或查询错误，转错误处理
  void HandleMotorErrorReport(int node_id, SDO CMD);

  void BotStateBatchReportByCan();
  void BotStateBatchReportByCanFD();

  void AssembleCanData(uint32_t in_can_id, uint8_t* in_canData) ;

  void SendBizCanMessage(uint32_t in_can_id, uint8_t* data) ;

  void BotSendCmdToMotor(int in_flag);
  void HandleCanMessage_601(bool in_cmd_flag, int in_speed_L);
  void HandleCanMessage_602(bool in_cmd_flag, int in_speed_R);

  void HandleCanMessage_001(uint8_t* data); //测试使用
  // 控制模式设定
  void HandleCanMessage_100(uint8_t* data);

  // 控制模式反馈帧
  void HandleCanMessage_101(bool batch_report);

  // 本体运动模型设置
  void HandleCanMessage_102(uint8_t data);

  // 本体运动模型，反馈周期：20ms
  void HandleCanMessage_103(bool batch_report);

  // 查询本体线速度、角速度、转角
  void HandleCanMessage_104();

  // 反馈本体线速度、角速度、转角反馈
  void HandleCanMessage_105(bool batch_report);

  // 查询传感器状态反馈，如防撞条、急停
  void HandleCanMessage_106();

  // 传感器状态反馈，包含防撞条、急停、防跌落传感器数据
  void HandleCanMessage_107(bool batch_report);

    // 传感器状态反馈，包含超声波传感器等数据
  void HandleCanMessage_108(bool batch_report);

  // 运动指令控制
  void HandleCanMessage_114(uint8_t data[]);

  // 加减速时间设置
  void HandleCanMessage_116(uint8_t data[]);

  // 速度环KP KI设置
  void HandleCanMessage_117(uint8_t data[]);
  // 设置驱动器速度环KP KI值
  void HandleCanMessage_618(bool batch_report);

  // 主控及伺服驱动器系统软硬件版本反馈
  void HandleCanMessage_120(bool batch_report);

  // 电机电压、电流、转速、温度数据反馈
  void HandleCanMessage_130X(int node_id, bool batch_report);
  void HandleCanMessage_130(int node_id, bool batch_report);
  void HandleCanMessage_132(int node_id, bool batch_report);
  void HandleCanMessage_134(int node_id, bool batch_report);
  void HandleCanMessage_136(int node_id, bool batch_report);

  // 灯光控制
  void HandleCanMessage_140(uint8_t* data);

  // 行驶轮线速度反馈
  void HandleCanMessage_150(bool batch_report);
  // 行驶轮转向角反馈
  void HandleCanMessage_152(bool batch_report);

  // 行驶轮电机编码器脉冲值反馈
  void HandleCanMessage_154(bool batch_report);

  // 里程计数据反馈
  void HandleCanMessage_190(int node_id, bool batch_report);
  void HandleCanMessage_192(int node_id, bool batch_report);
  void HandleCanMessage_194(int node_id, bool batch_report);
  void HandleCanMessage_196(int node_id, bool batch_report);

  // 电机驱动器状态反馈
  void HandleCanMessage_200(int node_id, bool batch_report);
  void HandleCanMessage_202(int node_id, bool batch_report);
  void HandleCanMessage_204(int node_id, bool batch_report);
  void HandleCanMessage_206(int node_id, bool batch_report);

  // 电池BMS状态数据反馈
  void HandleCanMessage_220();

  // 传感器设置
  void HandleCanMessage_231(uint8_t* data);

  // 里程计、速度、角度等
  void HandleMotorCanMessage_188(uint8_t* data);
  void HandleMotorCanMessage_186(uint8_t* data);
  void HandleMotorCanMessage_187(uint8_t* data);
  void HandleMotorCanMessage_189(uint8_t* data);

  // 电机温度、电流、电压
  void HandleMotorCanMessage_281(int node_id, uint8_t* data);
  void HandleMotorCanMessage_282(int node_id, uint8_t* data);
  void HandleMotorCanMessage_283(int node_id, uint8_t* data);
  void HandleMotorCanMessage_284(int node_id, uint8_t* data);

  // 电机心跳计数
  void HandleMotorCanMessage_381(int node_id, uint8_t* data);
  void HandleMotorCanMessage_382(int node_id, uint8_t* data);
  void HandleMotorCanMessage_383(int node_id, uint8_t* data);
  void HandleMotorCanMessage_384(int node_id, uint8_t* data);

  // 电机驱动器SDO指令执行反馈
  void HandleMotorCanMessage_58X(int node_id, uint8_t* data);
  void HandleMotorCanMessage_581(int node_id, uint8_t* data);
  void HandleMotorCanMessage_582(int node_id, uint8_t* data);
  void HandleMotorCanMessage_583(int node_id, uint8_t* data);
  void HandleMotorCanMessage_584(int node_id, uint8_t* data);

  // 电机NMT上线报文
  void HandleMotorCanMessage_701(int node_id, uint8_t* data);
  void HandleMotorCanMessage_702(int node_id, uint8_t* data);
  void HandleMotorCanMessage_703(int node_id, uint8_t* data);
  void HandleMotorCanMessage_704(int node_id, uint8_t* data);

  // 设置 通讯掉线保护时间 范围0~32767
  void HandleDisConnectProtectionTime(int node_id, SDO CMD);

  void HandleFeedbackPositionReset(int node_id, SDO CMD);

  // 0x2006H 绝对位置模式时当前位置清零  0：无效
  // 1：当前位置清零
  void HandleAbsolutePositionModeResetToZero(int node_id, SDO CMD);

  // 限位停车方式 0:停止 1：急停 2：无效
  void HandleRestrictedParkingMethods(int node_id, SDO CMD);

  // 起始速度 单位r/min  范围1-300r/min 默认1
  void HandleStartingSpeed(int node_id, SDO CMD);

  // 0x200AH 电机最大转速 单位r/min  范围1-300r/min
  void HandleMaximumSpeed(int node_id, SDO CMD);

  // 编码器线数设置 0-4096 默认1024
  void HandleLineEncoder(int node_id, SDO CMD);

  // 电机极对数,15
  void HandlePolePairs(int node_id, SDO CMD);

  // 上电锁轴方式 0：不使能不锁轴，1：不使能锁轴
  void HandleLockBearingMode(int node_id, SDO CMD);

  // 电机与HALL的偏移角度 单位1度 范围-360度~360度
  void HandleHallOffsetAngle(int node_id, SDO CMD);

  // 过载系数  范围0-300,单位%  默认200
  void HandleOverloadFactor(int node_id, SDO CMD);

  // 电机温度保护阈值 单位0.1度
  // 范围0-1200(*0.1) 默认800
  void HandleTemperatureProtectionThreshold(int node_id, SDO CMD);

  // 额定电流  驱动器输出的额定电流 单位0.1A 范围0-150
  void HandleLimitedCurrent(int node_id, SDO CMD);

  // 最大电流 驱动器输出的最大电流  单位0.1A 范围0-300
  void HandleMaximumCurrent(int node_id, SDO CMD);

  // 过载保护时间 驱动器过载保护时间 单位10ms 范围0-6553 缺省300
  void HandleOverloadProtectionTime(int node_id, SDO CMD);

  //// 超差报警阈值 编码器超差阈值
  //  单位*10counts 范围1-6553 缺省409
  void HandleOverToleranceAlarmThreshold(int node_id, SDO CMD);

  // 过度平滑系数 0-30000  缺省1000
  void HandleSmoothingFactor(int node_id, SDO CMD);
  // 电流环比例系数 0-30000   缺省600
  void HandleCurrentLoopScaleFactor(int node_id, SDO CMD);

  // 电流环积分增益 0-30000  缺省300
  void HandleCurrentLoopIntegralGain(int node_id, SDO CMD);

  // 前馈输出平滑系数 0-30000 缺省100
  void HandleFeedforwardSmoothingFactor(int node_id, SDO CMD);

  // 转矩输出平滑系数 0-30000 缺省100
  void HandleTorqueOutputSmoothingFactor(int node_id, SDO CMD);

  // 速度比例增益Kp 0-30000 缺省500
  void HandleSpeedKp(int node_id, SDO CMD);

  // 速度积分增益Ki 0-30000 缺省100
  void HandleSpeedKi(int node_id, SDO CMD);

  void HandleSpeedKf(int node_id, SDO CMD);

  // 设置 速度微分增益Kd 0-30000 缺省1000
  void HandleSpeedKd(int node_id, SDO CMD);

  // 位置比例增益Kp 0-30000 缺省50
  void HandlePositionKp(int node_id, SDO CMD);

  // 位置积分增益Kf 0-30000
  void HandlePositionKi(int node_id, SDO CMD);

  // 位置微分增益Kd 0-30000
  void HandlePositionKd(int node_id, SDO CMD);

  // 软件版本   出厂默认
  void HandleSoftwareVersion(int node_id, SDO CMD);

  // 电机温度 单位0.1度 范围0-120度  默认800
  void HandleMotorTemperature(int node_id, SDO CMD);

  // 霍尔输入状态 0-7 如果出现0或7  为霍尔出错 默认0
  void HandleHallSector(int node_id, SDO CMD);

  // 母线电压 单位0.01V  默认0
  void HandleBusVoltage(int node_id, SDO CMD);

  // 报警PWM处理方式 0：关闭 1：开启
  void HandlePWMAlarmMethod(int node_id, SDO CMD);

  // 过载处理方式 0：关闭 1：开启
  void HandleOverloadHandlingMethod(int node_id, SDO CMD);

  // 驱动器最近一次故障码
  void HandleLastFaultCode(int node_id, SDO CMD);

  //  控制字
  void HandleControlWord(int node_id, SDO CMD);

  // 状态字
  void HandleStatusWord(int node_id, SDO CMD);

  // 快速停止代码 快速停止命令后驱动器处理方式 默认5
  //   5:正常停止，维持快速停止状态，6：急减速停，维持快速停止状态，7：急停，维持快速停止状态
  void HandleQuickStopCode(int node_id, SDO CMD);

  // 关闭操作代码 关闭命令后驱动器处理方式 0：无效
  //  1：正常停止，转到Ready to switch on状态，默认1
  void HandleCloseActionCode(int node_id, SDO CMD);

  // 禁用操作代码  禁用操作命令后驱动器处理方式 0：无效
  //   1：正常停止，转到Switched On 状态 默认1
  void HandleDisableActionCode(int node_id, SDO CMD);

  // Halt控制寄存器  控制字Halt命令后驱动器处理方式
  //  1：正常停止，维持Operation
  //  Enabled状态，2：急速减停，维持Operation
  //  Enabled状态，3：急停，维持Operation
  //  Enabled状态，默认：1
  void HandleHaltControlRegister(int node_id, SDO CMD);

  // 运行模式，0：未定义，1：位置模式，2：速度模式,3:转矩模式
  //   默认：0
  void HandleSetRunMode(int node_id, SDO CMD);

  // 运行模式状态
  //  0：未定义，1：位置模式，2：速度模式,3:转矩模式，默认：0
  void HandleRunningModeStatus(int node_id, SDO CMD);

  // 实际位置反馈  单位counts 默认：0
  void HandleActualPosition(int node_id, SDO CMD);

  // 实际速度反馈 电机当前运行速度 单位0.1r/min 默认：0
  void HandleActualSpeed(int node_id, SDO CMD);

  // 目标转矩 单位mA,范围-30000~30000 默认：0
  void HandleTargetTorque(int node_id, SDO CMD);

  // 实时转矩反馈 单位0.1A,范围-300~300 默认：0
  void HandleRealTimeTorque(int node_id, SDO CMD);

  // 设置 目标位置 位置模式运行总脉冲数范围：-1000000~1000000
  // 此处存在Uint32往int32转换的兼容性问题
  void HandleTargetPosition(int node_id, SDO CMD);

  // 最大速度 位置模式时的最大速度 范围1-300r/min
  // 默认：120r/min
  void HandleMaxSpeed(int node_id, SDO CMD);

  // 设置 位置模式启/停速度  范围1-300r/min
  //  默认1r/min
  void HandlePositionModeStartStopSpeed(int node_id, SDO CMD);

  // 急停减速时间 范围0~32767ms 缺省10ms
  void HandleEmergencyStopDecelerationTime(int node_id, SDO CMD);

  // 转矩斜率 电流/1000/second   单位mA/s  缺省300ms
  void HandleTorqueGradient(int node_id, SDO CMD);

  // 目标速度  速度模式时的目标速度  范围:-300~300r/min
  //  缺省0
  // 此处存在Uint32往int32转换的兼容性问题
  void HandleTargetSpeed(int node_id, SDO CMD);

  //////////Motor CAN通信接口开始
  // 设置模式
  void SetMotorRunMode(int16_t run_mode);

  void MotorControlEnable();

  // 以下方法为设置或获取驱动器参数
  void PublishMotorSpeed(int node_id, int16_t speed_meter_per_second);

  // int MotorSpeedControl();
  void MotorSafeCheck();

  // 向驱动器发送业务内容为2字节的消息
  bool SendMotorCanMessage(int node_id, uint8_t cmd, uint16_t index_code,
                           uint8_t sub_index_code, uint16_t message);

  // 设置 通讯掉线保护时间 范围0~32767
  bool UpdateDisConnectProtectionTime(int node_id, uint16_t message,
                                      bool is_setup_cmd);

  bool UpdateFeedbackPositionReset(int node_id, uint16_t message,
                                   bool is_setup_cmd);

  // 0x2006H 绝对位置模式时当前位置清零  0：无效
  // 1：当前位置清零
  bool UpdateAbsolutePositionModeResetToZero(int node_id, uint16_t message,
                                             bool is_setup_cmd);

  // 限位停车方式 0:停止 1：急停 2：无效
  bool UpdateRestrictedParkingMethods(int node_id, uint16_t message,
                                      bool is_setup_cmd);

  // 起始速度 单位r/min  范围1-300r/min 默认1
  bool UpdateStartingSpeed(int node_id, uint16_t message, bool is_setup_cmd);

  // 0x200AH 电机最大转速 单位r/min  范围1-300r/min
  bool UpdateMaximumSpeed(int node_id, uint16_t message, bool is_setup_cmd);

  // 编码器线数设置 0-4096 默认1024
  bool UpdateLineEncoder(int node_id, uint16_t message, bool is_setup_cmd);

  // 电机极对数,15
  bool UpdatePolePairs(int node_id, uint16_t message, bool is_setup_cmd);

  // 上电锁轴方式 0：不使能不锁轴，1：不使能锁轴
  bool UpdateLockBearingMode(int node_id, uint16_t message, bool is_setup_cmd);

  // 电机与HALL的偏移角度 单位1度 范围-360度~360度
  bool UpdateHallOffsetAngle(int node_id, int16_t message, bool is_setup_cmd);

  // 过载系数  范围0-300,单位%  默认200
  bool UpdateOverloadFactor(int node_id, uint16_t message, bool is_setup_cmd);

  // 电机温度保护阈值 单位0.1度
  // 范围0-1200(*0.1) 默认800
  bool UpdateTemperatureProtectionThreshold(int node_id, uint16_t message,
                                            bool is_setup_cmd);

  // 额定电流  驱动器输出的额定电流 单位0.1A 范围0-150
  bool UpdateLimitedCurrent(int node_id, uint16_t message, bool is_setup_cmd);

  // 最大电流 驱动器输出的最大电流  单位0.1A 范围0-300
  bool UpdateMaximumCurrent(int node_id, uint16_t message, bool is_setup_cmd);

  // 过载保护时间 驱动器过载保护时间 单位10ms 范围0-6553 缺省300
  bool UpdateOverloadProtectionTime(int node_id, uint16_t message,
                                    bool is_setup_cmd);

  //// 超差报警阈值 编码器超差阈值
  //  单位*10counts 范围1-6553 缺省409
  bool UpdateOverToleranceAlarmThreshold(int node_id, uint16_t message,
                                         bool is_setup_cmd);

  // 过度平滑系数 0-30000  缺省1000
  bool UpdateSmoothingFactor(int node_id, uint16_t message, bool is_setup_cmd);

  // 电流环比例系数 0-30000   缺省600
  bool UpdateCurrentLoopScaleFactor(int node_id, uint16_t message,
                                    bool is_setup_cmd);

  // 电流环积分增益 0-30000  缺省300
  bool UpdateCurrentLoopIntegralGain(int node_id, uint16_t message,
                                     bool is_setup_cmd);

  // 前馈输出平滑系数 0-30000 缺省100
  bool UpdateFeedforwardSmoothingFactor(int node_id, uint16_t message,
                                        bool is_setup_cmd);

  // 转矩输出平滑系数 0-30000 缺省100
  bool UpdateTorqueOutputSmoothingFactor(int node_id, uint16_t message,
                                         bool is_setup_cmd);

  // 速度比例增益Kp 0-30000 缺省500
  bool UpdateSpeedKp(int node_id, uint16_t message, bool is_setup_cmd);

  // 速度积分增益Ki 0-30000 缺省100
  bool UpdateSpeedKi(int node_id, uint16_t message, bool is_setup_cmd);

  // 设置 速度微分增益Kd 0-30000 缺省1000
  bool UpdateSpeedKd(int node_id, uint16_t message, bool is_setup_cmd);

  // 位置比例增益Kp 0-30000 缺省50
  bool UpdatePositionKp(int node_id, uint16_t message, bool is_setup_cmd);

  // 位置积分增益Kf 0-30000
  bool UpdatePositionKi(int node_id, uint16_t message, bool is_setup_cmd);

  // 位置微分增益Kd 0-30000
  bool UpdatePositionKd(int node_id, uint16_t message, bool is_setup_cmd);

  // 软件版本   出厂默认
  bool GetSoftwareVersion(int node_id);

  // 电机温度 单位0.1度 范围0-120度  默认800
  bool GetMotorTemperature(int node_id);

  // 霍尔输入状态 0-7 如果出现0或7  为霍尔出错 默认0
  bool GetHallSector(int node_id);

  // 母线电压 单位0.01V  默认0
  bool GetBusVoltage(int node_id);

  // 报警PWM处理方式 0：关闭 1：开启
  bool UpdatePWMAlarmMethod(int node_id, uint16_t message, bool is_setup_cmd);

  // 过载处理方式 0：关闭 1：开启
  bool UpdateOverloadHandlingMethod(int node_id, uint16_t message,
                                    bool is_setup_cmd);

  // 驱动器最近一次故障码
  bool GetLastFaultCode(int node_id);

  //  控制字
  bool UpdateControlWord(int node_id, uint16_t message, bool is_setup_cmd);

  // 状态字
  bool GetStatusWord(int node_id);

  // 快速停止代码 快速停止命令后驱动器处理方式 默认5
  //   5:正常停止，维持快速停止状态，6：急减速停，维持快速停止状态，7：急停，维持快速停止状态
  bool UpdateQuickStopCode(int node_id, uint16_t message, bool is_setup_cmd);

  // 关闭操作代码 关闭命令后驱动器处理方式 0：无效
  //  1：正常停止，转到Ready to switch on状态，默认1
  bool UpdateCloseActionCode(int node_id, uint16_t message, bool is_setup_cmd);

  // 禁用操作代码  禁用操作命令后驱动器处理方式 0：无效
  //   1：正常停止，转到Switched On 状态 默认1
  bool UpdateDisableActionCode(int node_id, uint16_t message,
                               bool is_setup_cmd);

  // Halt控制寄存器  控制字Halt命令后驱动器处理方式
  //  1：正常停止，维持Operation
  //  Enabled状态，2：急速减停，维持Operation
  //  Enabled状态，3：急停，维持Operation
  //  Enabled状态，默认：1
  bool UpdateHaltControlRegister(int node_id, uint16_t message,
                                 bool is_setup_cmd);

  // 运行模式，0：未定义，1：位置模式，2：速度模式,3:转矩模式，5：手刹模式
  //   默认：0
  bool UpdateSetRunMode(int node_id, uint16_t message, bool is_setup_cmd);

  bool UpdateSetRunModeNew(int node_id, uint16_t message, bool is_setup_cmd);//为了解决can发送在驱动器端接收时丢包的问题，将数据合并到一个数据帧中

  // 运行模式状态
  //  0：未定义，1：位置模式，2：速度模式,3:转矩模式，默认：0
  bool GetRunningModeStatus(int node_id);

  bool UpdateSetBrakeMode(uint16_t brake_mode,int16_t handle_brake_current);

  // 实际位置反馈  单位counts 默认：0
  bool GetActualPosition(int node_id);

  // 实际速度反馈 电机当前运行速度 单位0.1r/min 默认：0
  bool GetActualSpeed(int node_id);

  // 目标转矩 单位mA,范围-30000~30000 默认：0
  bool UpdateTargetTorque(int node_id, int16_t message, bool is_setup_cmd);

  // 实时转矩反馈 单位0.1A,范围-300~300 默认：0
  bool GetRealTimeTorque(int node_id);

  // 设置 目标位置 位置模式运行总脉冲数范围：-1000000~1000000
  // 此处存在Uint32往int32转换的兼容性问题
  bool UpdateTargetPosition(int node_id, int32_t message, bool is_setup_cmd);

  // 最大速度 位置模式时的最大速度 范围1-300r/min
  // 默认：120r/min
  bool UpdateMaxSpeed(int node_id, uint16_t message, bool is_setup_cmd);

  // 设置 位置模式启/停速度  范围1-300r/min
  //  默认1r/min
  bool UpdatePositionModeStartStopSpeed(int node_id, uint16_t message,
                                        bool is_setup_cmd);

  // 急停减速时间 范围0~32767ms 缺省10ms
  bool UpdateEmergencyStopDecelerationTime(int node_id, uint16_t message,
                                           bool is_setup_cmd);

  // 转矩斜率 电流/1000/second   单位mA/s  缺省300ms
  bool UpdateTorqueGradient(int node_id, uint16_t message, bool is_setup_cmd);

  // 目标速度  速度模式时的目标速度  范围:-300~300r/min
  //  缺省0
  // 此处存在Uint32往int32转换的兼容性问题
  bool UpdateTargetSpeed(int node_id, int16_t message, bool is_setup_cmd);

  //////////Motor CAN通信接口结束

  int message_size_id = 0;
  int message_size_data = 0;
  int can_id[30];         // 发送id数组
  uint8_t can_data[CAN_DATA_LENGTH];  // 发送数据数组

  uint8_t power_button_status = 0;  // 电源按钮状态，1:开；0:关
  uint8_t emergency_button_status = 0;  // 急停状态，1:触发急停；0:未触发
  uint8_t touch_sensor1_status = 0;  // 前防撞条，1:触发防撞；0:未触发
  uint8_t touch_sensor2_status = 0;  // 后防撞条，1:触发防撞；0:未触发
  uint8_t charge_status = 0;
  uint8_t ultrasonic_status = 0; //1:在超声波安全范围内有障碍物，需要停车；0：在超声波安全范围内无障碍物 //超声波是否启用，由上位机控制，如果上位机设置关闭，则不启用，只向上位机传超声波数据
  uint8_t fall_distance_status = 0; //防跌落传感器状态，1为触发，0为不触发

  uint8_t control_mode = 0;  // 控制模式，0x00：待机模式； 0x01：Can指令模式；
                             // 0x02：遥控模式  默认为待机模式
  uint8_t motion_mode = 0;  // 运动模式，0x00 驻车档；0x01 前进/后退；0x02 斜移模式；0x03
          // 自旋模式；0x04 横移模式；0x06 转向；0x07 空档
  uint8_t motion_mode_last = 0xA0;   //因为转向舵机发送指令后不需要多次发送，所以记录上次档位状态，如果没有变化，就不再给舵机发指令，有变化时再发送
  uint8_t robot_model = 0;  // 运动模型，0x04：四轮四驱；0x03：阿克曼；0x08:四轮差速
  int16_t linear_velocity = 0;    // 车体线速度  单位0.001m/s
  int16_t angular_velocity = 0;  // 车体角速度  单位0.01°/s
  int16_t body_angle = 0;        // 车体转角  单位0.01°

  int16_t velocity_smooth = 0;  // smooth速度，单位mm/s
  int16_t angular_smooth = 0;   // smooth angular,单位0.01°/s

  float target_speed_last[5] = {0,0,0,0,0}; //记录上次目标速度，用于低通滤波计算
  float now_speed_last[5] = {0,0,0,0,0}; //记录上次运行速度，用于低通滤波计算

  uint64_t dt_us = 0;  //执行间隔时间
  uint16_t control_rate_200HZ = 0; //添加全局变量，用于在当前控制频率下二次控制执行频率
  uint8_t is_rate_200HZ = 0;
  uint16_t control_rate_50HZ = 0; //添加全局变量，用于在当前控制频率下二次控制执行频率
  uint8_t is_rate_50HZ = 0;

  int16_t BotMotionControl_printnum = 0; //记录日志频率
  bool is_log_time = false;

  //临时方案，根据设置舵机值反馈
  uint16_t steer_angle1 = 0;
  uint16_t steer_angle2 = 0;
  uint16_t steer_angle3 = 0;
  uint16_t steer_angle4 = 0;

  uint8_t set_handle_brake = 0; //是否启用手刹模式
  int16_t set_handle_brake_value = 0; // 设置的手刹电流值
  uint8_t set_handler_brake_status_last = 0;
  bool is_handle_brake = false;
  uint16_t handle_brake = 0x05;


  uint32_t can_test_flag = 0;
  uint16_t iq_test = 0;

  bool motor_is_safe = false; //根据传感器进行安全检查
  int16_t BotMotionSafeTimeOut = 0; //记录安全异常后计时处理

  int16_t node1_last_speed = 0;
  int16_t node2_last_speed = 0;
  int16_t node3_last_speed = 0;
  int16_t node4_last_speed = 0;
  float node1_brake_current = 0;
  float node2_brake_current = 0;
  float node3_brake_current = 0;
  float node4_brake_current = 0;

  float brake_current = 0;

  bool motor_to_stop = false; //是否要停车

  uint8_t TEST_is_turn = 0;


  //对于can的控制帧，记录接收到can数据的时间间隔，间隔20ms以上没有收到can数据，则不运行
  //根据不同的can帧分别判断，目前只处理100和114的can指令
  uint16_t can_receive_100_count = 0;
  uint16_t can_receive_114_count = 0;

  bot_run_control_s bot_run_control_sub{};
  bot_can_message_s bot_can_message_sub{};
  bot_can_message_receive_s bot_can_message_receive_sub{};
  bot_sensor_event_s bot_sensor_event_sub{};
  bot_motor_control_s bot_motor_control_sub{};
  sensor_combined_s sensor_combined_sub{};
  sensor_baro_s sensor_baro_sub_0{};
  sensor_baro_s sensor_baro_sub_1{};
  sensor_mag_s sensor_mag_sub{};
  bot_distance_sensor_s bot_distance_sensor_sub;
  bot_sensor_com_data_s bot_sensor_com_data_sub;


  // 订阅消息
  uORB::Subscription _bot_run_control_sub{ORB_ID(bot_run_control)};
  uORB::Subscription _bot_can_message_sub{ORB_ID(bot_can_message)};
  uORB::Subscription _bot_can_message_receive_sub{ORB_ID(bot_can_message_receive)};
  uORB::Subscription _bot_sensor_event_sub{ORB_ID(bot_sensor_event)};
  uORB::Subscription _bot_motor_control_sub{ORB_ID(bot_motor_control)};
  uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
  uORB::Subscription _sensor_baro_sub_0{ORB_ID(sensor_baro),0};
  uORB::Subscription _sensor_baro_sub_1{ORB_ID(sensor_baro),1};
  uORB::Subscription _sensor_mag_sub{ORB_ID(sensor_mag)};
  uORB::Subscription _bot_distance_sensor_sub{ORB_ID(bot_distance_sensor)};
   uORB::Subscription _bot_sensor_com_data_sub{ORB_ID(bot_sensor_com_data)};

  // 发布消息
  uORB::Publication<bot_run_control_s> _bot_run_control_pub{ORB_ID(bot_run_control)};
  uORB::Publication<bot_sensor_event_s> _bot_sensor_event_pub{ORB_ID(bot_sensor_event)};
  uORB::Publication<bot_motor_control_s> _bot_motor_control_pub{ORB_ID(bot_motor_control)};
  uORB::Publication<bot_can_message_s> _bot_can_message_pub{ORB_ID(bot_can_message)};


  // Performance (perf) counters--与系统性能管理相关的函数
  perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
  perf_counter_t _loop_interval_perf{
      perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};
};
