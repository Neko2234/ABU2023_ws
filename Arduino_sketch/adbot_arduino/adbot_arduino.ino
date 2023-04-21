#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

///初期状態は下の刃でリングを受け止めている状態を想定///
#define SPR_MOTOR 11       // 分離のモーター番号
#define SPR_ENC_NUM 0      // 分離のエンコーダ番号
#define ONE_WAY_COUNT 500  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5

// DCモータ番号
#define SHOOT_MOTOR_LU 0  // 左上
#define SHOOT_MOTOR_LD 1  // 左下
#define SHOOT_MOTOR_RU 2  // 右上
#define SHOOT_MOTOR_RD 3  // 右下

using namespace Cubic_controller;

//分離
int spr_indicated_duty = 250;       //指定するDutyの絶対値。ROSメッセージから指定できる。メインなら70、サブなら250
int spr_duty = spr_indicated_duty;  //実際にCubicに指定する値
bool spr_sign = false;
bool spr_pre_sign = false;
bool spr_is_go_separating = false;    //初期位置から折り返しまでの間は真
bool spr_is_come_separating = false;  //折り返しから初期位置までの間は真
//射出
double target = 2.25;    // 正面
int32_t shoot_duty = 0;  // 射出のDuty

void publish(void);
void spr_set_duty(void);

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
  spr_pre_sign = spr_sign;
  spr_sign = spr_msg.data;
}
void termSprCb(const std_msgs::Int16 &msg) {
  spr_indicated_duty = msg.data;
}
//照準
void cmdShootingDutyCb(const std_msgs::Int32 &duty_msg) {
  shoot_duty = duty_msg.data;
}
void cmdAngleCb(const std_msgs::Float64 &angle_msg) {
  target = degToRad(angle_msg.data) + 2.25;
  // target = cmd_angle.data + 2.25;
}
void cmdToggleReceiveCb(const std_msgs::Bool &recieve_msg) {
}
void cmdToggleBeltCb(const std_msgs::Bool &belt_msg) {
}
void cmdToggleLidarCb(const std_msgs::Bool &lidar_msg) {
}
void cmdEmergencyStopCb(const std_msgs::Bool &stop_msg) {
}

// トピックを受け取るためのサブスクライバーのコンポーネントを作成
// 分離
ros::Subscriber<std_msgs::Bool> cmd_toggle_shoot_sub("cmd_toggle_shoot", &cmdToggleShootCb);
ros::Subscriber<std_msgs::Int16> term_spr_sub("term_spr", &termSprCb);
//照準
ros::Subscriber<std_msgs::Int32> cmd_shooting_duty_sub("cmd_shooting_duty", &cmdShootingDutyCb);
ros::Subscriber<std_msgs::Float64> cmd_angle_sub("cmd_angle", &cmdAngleCb);
ros::Subscriber<std_msgs::Bool> cmd_toggle_receive_sub("cmd_toggle_receive", &cmdToggleReceiveCb);
ros::Subscriber<std_msgs::Bool> cmd_toggle_belt_sub("cmd_toggle_belt", &cmdToggleBeltCb);
ros::Subscriber<std_msgs::Bool> cmd_emergency_stop_sub("cmd_emergency_stop", &cmdEmergencyStopCb);

void setup() {
  //分離デバッグ用
  pinMode(24, OUTPUT);
  pinMode(23, OUTPUT);

  // すべてのモータ，エンコーダの初期化
  Cubic::begin(3.0);
  Inc_enc::reset();
  nh.getHardware()->setBaud(9600);

  // ROSの通信を開始
  nh.initNode();

  //サブスクライバ
  nh.subscribe(cmd_toggle_shoot_sub);
  nh.subscribe(term_spr_sub);
  nh.subscribe(cmd_shooting_duty_sub);
  nh.subscribe(cmd_angle_sub);
  nh.subscribe(cmd_toggle_receive_sub);
  nh.subscribe(cmd_toggle_belt_sub);
  nh.subscribe(cmd_emergency_stop_sub);

  // 分離デバッグ用
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);
}

void loop() {
  // 分離のDuty決定
  spr_set_duty();

  //dutyをセット
  DC_motor::put(SHOOT_MOTOR_LU, shoot_duty);
  DC_motor::put(SHOOT_MOTOR_LD, shoot_duty);
  DC_motor::put(SHOOT_MOTOR_RU, shoot_duty);
  DC_motor::put(SHOOT_MOTOR_RD, shoot_duty);
  DC_motor::put(SPR_MOTOR, spr_duty);
  set_position();

  // データの送受信を行う
  Cubic::update();
  // メッセージを送信
  publish();

  nh.spinOnce();
}

void set_position() {
  static Velocity_PID velocityPID(3, 0, encoderType::inc, 2048 * 4, 0.5, 0.5, 0.5, 0.1, 0.4, false, true);
  static Position_PID positionPID(3, 0, encoderType::abs, AMT22_CPR, 0.2, 0.25, 0.0, 0.0, target, true, true);
  static bool stopFlag = false;
  if (stopFlag) {
    // Serial.println("stopping...");
    for (int i = 0; i < 8; i++) {
      DC_motor::put(i, 0);
    }
  } else {
    // velocityPID.compute();
    positionPID.setTarget(target);
    positionPID.compute();
    duty.data = DC_motor::get(3);
  }

  angle_diff.data = abs(positionPID.getTarget() - positionPID.getCurrent());
  angle.data = positionPID.getCurrent() - 2.25;
  angle.data = radToDeg(angle.data);
}

void publish() {
  pub_angle_diff.publish(&angle_diff);
  pub_angle.publish(&angle);
  pub_duty.publish(&duty);
}

void spr_set_duty() {
  int32_t enc_count = abs(Inc_enc::get(SPR_ENC_NUM));

  // 立ち上がり(スイッチを押した瞬間)で分離実行を切り替え
  if (spr_sign && !spr_pre_sign) {
    spr_is_go_separating = true;
  }

  //両方とも真のときは停止してふたつとも偽とする
  if (spr_is_go_separating && !spr_is_come_separating) {
    if (enc_count < ONE_WAY_COUNT) {
      digitalWrite(24, LOW);
      spr_duty = spr_indicated_duty;
    } else {
      spr_is_go_separating = false;
      spr_is_come_separating = true;
    }
  } else if (spr_is_come_separating && !spr_is_go_separating) {
    if (enc_count > 50) {
      digitalWrite(23, LOW);
      spr_duty = -spr_indicated_duty;
    } else {
      spr_is_come_separating = false;
      spr_duty = 0;
    }
  } else {
    spr_is_go_separating = false;
    spr_is_come_separating = false;
    spr_duty = 0;
  }
}