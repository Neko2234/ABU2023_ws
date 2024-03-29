#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

/*初期状態は下の刃でリングを受け止めている状態を想定*/
#define SPR_MOTOR 8        // 分離のモーター番号
#define SPR_ENC_NUM 0      // 分離のエンコーダ番号
#define ONE_WAY_COUNT 400  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5
#define SPR_STOP_ENC 500000  // 分離で片道を行った後折り返すまでに止まっている時間(us)

#define ZERO 0.000000001      // floatにおける等価演算のまるめ
#define AIM_ENC 0             // 照準エンコーダ
#define TARGET_DIFF_THRESH 0.10471973  // 位置PIDか手動かの閾値(rad)この値は6度
#define ROT_TIME_FIVE 1000000    // 照準を5度動かすために必要な時間(us)

// DCモータ番号
#define SHOOT_MOTOR_LU 3  // 左上
#define SHOOT_MOTOR_LD 2  // 左下
#define SHOOT_MOTOR_RU 6  // 右上
#define SHOOT_MOTOR_RD 0  // 右下
#define BELT_MOTOR 1      // ベルト
#define AIM_MOTOR 7       // 昇降

using namespace Cubic_controller;

bool emergency_stop = false;
//分離
int16_t spr_indicated_duty = 70;        //指定するDutyの絶対値。ROSメッセージから指定できる。
int16_t spr_duty = spr_indicated_duty;  //実際にCubicに指定する値
bool spr_is_go_separating = false;      //初期位置から折り返しまでの間は真
bool spr_is_come_separating = false;    //折り返しから初期位置までの間は真
bool spr_is_stopping = false;
unsigned long spr_stop_start_time = 0;
// 照準
bool is_rotating = false;
double target = 0;  // 正面
double pre_target = 0;
unsigned long rot_start_time = 0;
int16_t rot_duty = 70;
//射出
bool is_moving_belt = false;
int16_t belt_duty = 300;
int16_t shoot_duty = 0;  // 射出のDuty

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
  pre_target = target;
  target = angle_msg.data;
}
void cmdToggleReceiveCb(const std_msgs::Bool &recieve_msg) {
}
void cmdToggleLidarCb(const std_msgs::Bool &lidar_msg) {
}
// 射出
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
  emergency_stop = stop_msg.data;
}

// トピックを受け取るためのサブスクライバーを作成
// 分離
ros::Subscriber<std_msgs::Bool> cmd_toggle_shoot_sub("cmd_toggle_shoot", &cmdToggleShootCb);
ros::Subscriber<std_msgs::Int16> term_spr_sub("term_spr", &termSprCb);
//照準
ros::Subscriber<std_msgs::Float64> cmd_angle_sub("cmd_angle", &cmdAngleCb);
ros::Subscriber<std_msgs::Bool> cmd_toggle_receive_sub("cmd_toggle_receive", &cmdToggleReceiveCb);
// 射出
ros::Subscriber<std_msgs::Bool> cmd_toggle_belt_sub("cmd_toggle_belt", &cmdToggleBeltCb);
ros::Subscriber<std_msgs::Int16> term_belt_duty_sub("term_belt_duty", &termBeltDutyCb);
ros::Subscriber<std_msgs::Int16> cmd_shooting_duty_sub("cmd_shooting_duty", &cmdShootingDutyCb);
// 緊急停止
ros::Subscriber<std_msgs::Bool> cmd_emergency_stop_sub("cmd_emergency_stop", &cmdEmergencyStopCb);

void spr_set_duty() {
  int32_t enc_count = abs(Inc_enc::get(SPR_ENC_NUM));
  unsigned long time_now = micros();

  // digitalWrite(23, HIGH);
  // digitalWrite(24, HIGH);

  //両方とも真のときは停止してふたつとも偽とする
  if (spr_is_go_separating && !spr_is_come_separating) {
    if (enc_count < ONE_WAY_COUNT) {
      // digitalWrite(24, LOW);
      spr_duty = spr_indicated_duty;
      spr_stop_start_time = micros();
    } else {
      spr_is_go_separating = false;
      spr_is_stopping = true;
    }
  } else if (spr_is_come_separating && !spr_is_go_separating) {
    if (enc_count > 100) {
      // digitalWrite(23, LOW);
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

void set_position() {
  static Position_PID positionPID(AIM_MOTOR, AIM_ENC, encoderType::abs, AMT22_CPR, 0.1, 2.8, 0.02, 0.0, target, true, false);
  angle_diff.data = abs(positionPID.getTarget() - positionPID.getCurrent());
  angle.data = positionPID.getCurrent();
  angle.data = radToDeg(angle.data);
  duty.data = DC_motor::get(AIM_MOTOR);

  unsigned long now_time = micros();
  unsigned long dt = now_time - rot_start_time;
  double target_diff = target - pre_target;

  if (abs(target_diff) > TARGET_DIFF_THRESH) {
    positionPID.setTarget(target);
    positionPID.compute();
    digitalWrite(23, HIGH);
    digitalWrite(24, HIGH);
  } else if (!is_rotating && (target_diff != 0)) {
    if (target_diff > 0) {
      DC_motor::put(AIM_MOTOR, rot_duty);
      digitalWrite(23, LOW);
    } else if (target_diff < 0) {
      DC_motor::put(AIM_MOTOR, -rot_duty);
      digitalWrite(24, LOW);
    }
    is_rotating = true;
    rot_start_time = micros();
  } else if (dt > ROT_TIME_FIVE) {  // 回転している時間が閾値を越えたら止める
    is_rotating = false;
    target_diff = 0;
    pre_target = target;
    DC_motor::put(AIM_MOTOR, 0);
    digitalWrite(23, HIGH);
    digitalWrite(24, HIGH);
  }
}

void setup() {
  // デバッグ用
  pinMode(23, OUTPUT); // blue
  pinMode(24, OUTPUT); // green
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
  nh.subscribe(cmd_toggle_receive_sub);
  nh.subscribe(cmd_toggle_belt_sub);
  nh.subscribe(cmd_emergency_stop_sub);
  nh.subscribe(term_belt_duty_sub);
}

void loop() {
  nh.spinOnce();
  // 緊急停止信号が来たらすべて停止
  if (emergency_stop) {
    for (int i = 0; i < DC_MOTOR_NUM; i++) {
      DC_motor::put(i, 0);
    }
    spr_is_go_separating = false;
    spr_is_come_separating = false;
  }

  // if (is_moving_belt) digitalWrite(23, LOW);
  // else digitalWrite(23, HIGH);
  // if (emergency_stop) digitalWrite(24, HIGH);
  // else digitalWrite(24, LOW);
  // 分離のDuty決定
  spr_set_duty();

  set_position();

  publish();
  // pub_angle_diff.publish(&angle_diff);
  // pub_angle.publish(&angle);
  // pub_duty.publish(&duty);

  //dutyをセット
  if (is_moving_belt) {
    DC_motor::put(BELT_MOTOR, belt_duty);
  } else {
    DC_motor::put(BELT_MOTOR, 0);
  }
  DC_motor::put(SHOOT_MOTOR_LU, -shoot_duty);
  DC_motor::put(SHOOT_MOTOR_LD, -shoot_duty);
  DC_motor::put(SHOOT_MOTOR_RU, shoot_duty);
  DC_motor::put(SHOOT_MOTOR_RD, -shoot_duty);
  DC_motor::put(SPR_MOTOR, spr_duty);

  // データの送受信を行う
  Cubic::update();
}