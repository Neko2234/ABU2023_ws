#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

// デバッグ用。LEDでデバッグする際はコメントを外す
// #define SPR_DEBUG
#define AIM_DEBUG
// #define BELT_DEBUG

/*初期状態は下の刃でリングを受け止めている状態を想定*/
#define SPR_MOTOR 8          // 分離のモーター番号
#define SPR_ENC_NUM 0        // 分離のIncエンコーダ番号
#define ONE_WAY_COUNT 700    //片道のエンコーダカウント
#define SPR_STOP_ENC 500000  // 分離で片道を行った後折り返すまでに止まっている時間(us)

#define AIM_ENC 0              // 照準Absエンコーダ
#define REC_POS_R 0            // 向かって右でリングを受け取るときの角度(rad)　**未調整**
#define REC_POS_L 0            // 向かって左でリングを受け取るときの角度(rad)　**未調整**
#define ROT_SHORT_TIME 500000  //手動照準の最小移動時間(us)
#define ROT_MIDDLE_TIME 1000000
#define ROT_LONG_TIME 2000000  //手動照準の最長移動時間

#define SHT_DUTY_BIUS 50  // 射出Dutyにかけるバイアス項

// DCモータ番号
#define SHOOT_MOTOR_LU 5  // 左上
#define SHOOT_MOTOR_LD 4  // 左下
#define SHOOT_MOTOR_RU 1  // 右上
#define SHOOT_MOTOR_RD 0  // 右下
#define BELT_MOTOR 6      // ベルト
#define AIM_MOTOR 7       // 照準

using namespace Cubic_controller;

bool emergency_stop = false;
//分離
int16_t spr_indicated_duty = 150;       //指定するDutyの絶対値。ROSメッセージから指定できる。
int16_t spr_duty = spr_indicated_duty;  //実際にCubicに指定する値
bool spr_is_go_separating = false;      //初期位置から折り返しまでの間は真
bool spr_is_come_separating = false;    //折り返しから初期位置までの間は真
bool spr_is_stopping = false;
unsigned long spr_stop_start_time = 0;
// 照準
int16_t aim_mode;  // 手動照準のモード
bool is_rotating = false;
double target = 0;  // 正面　**未調整**
unsigned long rot_start_time = 0;
int16_t rot_duty = 70;       // 手動で動かすときの照準のDuty　**未調整**
unsigned long rot_time = 0;  //移動する時間を入れる。
int16_t aim_duty = 0;
//射出
bool is_moving_belt = false;
int16_t belt_duty = 500;  // ベルトのDuty、コマンドから操作可能
int16_t shoot_duty = 0;   // 射出のDuty

ros::NodeHandle nh;
std_msgs::Float64 angle_diff;
std_msgs::Float64 angle;
std_msgs::Int16 duty;
ros::Publisher pub_angle_diff("angle_diff", &angle_diff);
ros::Publisher pub_angle("angle", &angle);
ros::Publisher pub_duty("duty", &duty);

// トピックのコールバック関数
// 分離
void cmdToggleShootCb(const std_msgs::Bool &spr_msg) {
  spr_is_go_separating = spr_msg.data;
}
void termSprCb(const std_msgs::Int16 &msg) {
  spr_indicated_duty = msg.data;
}
//照準
void cmdAngleCb(const std_msgs::Float64 &angle_msg) {
  target = angle_msg.data;
 nh.loginfo("recieved angle");  
}
void cmdToggleReceiveCb(const std_msgs::Bool &recieve_msg) {
  // tureのとき向かって右、falseのとき向かって左で受け取り体勢
  if (recieve_msg.data) target = REC_POS_R;
  else target = REC_POS_L;
}
void cmdAimCb(const std_msgs::Int16 &aim_msg) {
  aim_duty = aim_msg.data;
  nh.loginfo("duty updated");
}
void cmdToggleLidarCb(const std_msgs::Bool &lidar_msg) {
}
// 射出
void cmdAngleAdjustCb(const std_msgs::Int16 &angle_msg) {
  aim_mode = angle_msg.data;
  nh.loginfo("angle adjusted");
}
void cmdToggleBeltCb(const std_msgs::Bool &belt_msg) {
  is_moving_belt = belt_msg.data;
}
void termBeltDutyCb(const std_msgs::Int16 &duty_msg) {
  belt_duty = duty_msg.data;
}
void cmdShootingDutyCb(const std_msgs::Int16 &duty_msg) {
  shoot_duty = duty_msg.data;
}
// 緊急停止
void cmdEmergencyStopCb(const std_msgs::Bool &stop_msg) {
  nh.loginfo("receiving emergency");
  emergency_stop = stop_msg.data;
}

// トピックを受け取るためのサブスクライバーを作成
// 分離
ros::Subscriber<std_msgs::Bool> cmd_toggle_shoot_sub("cmd_toggle_shoot", &cmdToggleShootCb);
ros::Subscriber<std_msgs::Int16> term_spr_sub("term_spr", &termSprCb);
//照準
ros::Subscriber<std_msgs::Float64> cmd_angle_sub("cmd_angle", &cmdAngleCb);
ros::Subscriber<std_msgs::Bool> cmd_toggle_receive_sub("cmd_toggle_receive", &cmdToggleReceiveCb);
ros::Subscriber<std_msgs::Int16> cmd_aim_sub("cmd_aim", &cmdAimCb);
// 射出
ros::Subscriber<std_msgs::Int16> cmd_angle_adjust_sub("cmd_angle_adjust", &cmdAngleAdjustCb);
ros::Subscriber<std_msgs::Bool> cmd_toggle_belt_sub("cmd_toggle_belt", &cmdToggleBeltCb);
ros::Subscriber<std_msgs::Int16> term_belt_duty_sub("term_belt_duty", &termBeltDutyCb);
ros::Subscriber<std_msgs::Int16> cmd_shooting_duty_sub("cmd_shooting_duty", &cmdShootingDutyCb);
// 緊急停止
ros::Subscriber<std_msgs::Bool> cmd_emergency_stop_sub("cmd_emergency_stop", &cmdEmergencyStopCb);

void separate() {
  int32_t enc_count = abs(Inc_enc::get(SPR_ENC_NUM));
  unsigned long time_now = micros();
  // char buf[100];
  // sprintf(buf, "%d", enc_count);
  // nh.loginfo(buf);
#ifdef SPR_DEBUG
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);
#endif
  //両方とも真のときは停止してふたつとも偽とする
  if (spr_is_go_separating && !spr_is_come_separating) {
    if (enc_count < ONE_WAY_COUNT) {
#ifdef SPR_DEBUG
      digitalWrite(24, LOW);
#endif
      spr_duty = spr_indicated_duty;
      spr_stop_start_time = micros();
    } else {
      spr_is_go_separating = false;
      spr_is_stopping = true;
    }
  } else if (spr_is_come_separating && !spr_is_go_separating) {
    if (enc_count > 100) {
#ifdef SPR_DEBUG
      digitalWrite(23, LOW);
#endif
      spr_duty = -spr_indicated_duty;
    } else {
      spr_is_come_separating = false;
      spr_duty = 0;
    }
  } else if (spr_is_stopping && (time_now - spr_stop_start_time > SPR_STOP_ENC)) {
    spr_is_come_separating = true;
    spr_is_stopping = false;
  } else {
    spr_is_go_separating = false;
    spr_is_come_separating = false;
    spr_duty = 0;
  }
}

void publish() {
  pub_angle_diff.publish(&angle_diff);
  pub_angle.publish(&angle);
  pub_duty.publish(&duty);
}

void manual_set_position() {
  unsigned long now_time = micros();
  unsigned long dt = now_time - rot_start_time;

  if ((aim_mode != 0) && !is_rotating) {
    if (abs(aim_mode) == 1) rot_time = ROT_SHORT_TIME;
    else if (abs(aim_mode) == 2) rot_time = ROT_MIDDLE_TIME;
    else if (abs(aim_mode) == 3) rot_time = ROT_LONG_TIME;
    else return;

    if (aim_mode > 0) DC_motor::put(AIM_MOTOR, rot_duty);
    else if (aim_mode < 0) DC_motor::put(AIM_MOTOR, -rot_duty);

    is_rotating = true;
    rot_start_time = micros();
#ifdef AIM_DEBUG
    digitalWrite(23, LOW);
    digitalWrite(24, LOW);
#endif
  } else if ((dt > rot_time) && is_rotating) {
    DC_motor::put(AIM_MOTOR, 0);
    is_rotating = false;
#ifdef AIM_DEBUG
    digitalWrite(23, HIGH);
    digitalWrite(24, HIGH);
#endif
  }
}

void pid_set_position() {
  //p項で大きい範囲をある程度の精度で制御できるようになったらi項で最小単位分移動させて精度を高める
  const double capable_duty = 0.15;
  const double Kp = 2.8;
  const double Ki = 0.02;
  const double Kd = 0.0;
  static Position_PID positionPID(AIM_MOTOR, AIM_ENC, encoderType::abs, AMT22_CPR, capable_duty, Kp, Ki, Kd, target, true, false);
  positionPID.setTarget(target);
  positionPID.compute();
  angle_diff.data = abs(positionPID.getTarget() - positionPID.getCurrent());
  angle.data = positionPID.getCurrent();
  angle.data = radToDeg(angle.data);
  duty.data = DC_motor::get(AIM_MOTOR);
}

void setup() {
  // デバッグ用
  pinMode(23, OUTPUT);  // blue
  pinMode(24, OUTPUT);  // green
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);

  // すべてのモータ，エンコーダの初期化
  Cubic::begin(3.0);
  Inc_enc::reset();
  nh.getHardware()->setBaud(2000000);

  // ROSの通信を開始
  nh.initNode();

  //アドバタイズ
  nh.advertise(pub_angle_diff);
  nh.advertise(pub_angle);
  nh.advertise(pub_duty);

  //サブスクライバ
  nh.subscribe(cmd_toggle_shoot_sub);
  nh.subscribe(term_spr_sub);
  nh.subscribe(cmd_shooting_duty_sub);
  nh.subscribe(cmd_angle_sub);
  nh.subscribe(cmd_angle_adjust_sub);
  nh.subscribe(cmd_aim_sub);
  nh.subscribe(cmd_toggle_receive_sub);
  nh.subscribe(cmd_toggle_belt_sub);
  nh.subscribe(cmd_emergency_stop_sub);
  nh.subscribe(term_belt_duty_sub);
}

void loop() {
  nh.spinOnce();

#ifdef BELT_DEBUG
  if (is_moving_belt) digitalWrite(23, LOW);
  else digitalWrite(23, HIGH);
  if (emergency_stop) digitalWrite(24, HIGH);
  else digitalWrite(24, LOW);
#endif

  // 分離のDuty決定
  separate();
  // 照準を定める
  pid_set_position();
  // manual_set_position();

  // DC_motor::put(AIM_MOTOR, aim_duty);

  // publish();

  //dutyをセット
  if (is_moving_belt) {
    DC_motor::put(BELT_MOTOR, belt_duty);
  } else {
    DC_motor::put(BELT_MOTOR, 0);
  }
  DC_motor::put(SHOOT_MOTOR_LU, -(shoot_duty + SHT_DUTY_BIUS));
  DC_motor::put(SHOOT_MOTOR_LD, -shoot_duty);
  DC_motor::put(SHOOT_MOTOR_RU, (shoot_duty + SHT_DUTY_BIUS));
  DC_motor::put(SHOOT_MOTOR_RD, -shoot_duty);
  DC_motor::put(SPR_MOTOR, spr_duty);


  // 緊急停止信号が来たらすべて停止
  if (emergency_stop) {
    for (int i = 0; i < DC_MOTOR_NUM; i++) {
      DC_motor::put(i, 0);
    }
    spr_is_go_separating = false;
    spr_is_come_separating = false;
  }

  // データの送受信を行う
  Cubic::update();
}