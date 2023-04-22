#include "cubic_arduino.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// #include <adbot_msgs/SprMsg.h>

///初期状態は下の刃でリングを受け止めている状態を想定///
#define SPR_MOTOR 8        // 分離のモーター番号
#define SPR_ENC_NUM 0      // 分離のエンコーダ番号
#define ONE_WAY_COUNT 400  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5
#define STOP_TIME 500  // ミリ秒

// DCモータ番号
#define SHOOT_MOTOR_LU 3  // 左上
#define SHOOT_MOTOR_LD 2  // 左下
#define SHOOT_MOTOR_RU 1  // 右上
#define SHOOT_MOTOR_RD 0  // 右下
#define BELT_MOTOR 6      // ベルト

bool emergency_stop = false;
//分離
int32_t spr_indicated_duty = 70;        //指定するDutyの絶対値。ROSメッセージから指定できる。
int32_t spr_duty = spr_indicated_duty;  //実際にCubicに指定する値
bool spr_sign = false;
bool spr_pre_sign = false;
bool spr_is_go_separating = false;    //初期位置から折り返しまでの間は真
bool spr_is_come_separating = false;  //折り返しから初期位置までの間は真
bool spr_is_stopping = false;
unsigned long spr_stop_start_time = 0;
//射出
bool is_moving_belt = false;
int32_t belt_duty = 300;
int32_t shoot_duty = 0;  // 射出のDuty

void spr_set_duty(void);

ros::NodeHandle nh;

// トピックのコールバック関数
// 分離
void cmdToggleShootCb(const std_msgs::Bool &spr_msg) {
  spr_pre_sign = spr_sign;
  spr_sign = spr_msg.data;
}
void termSprCb(const std_msgs::Int32 &msg) {
  spr_indicated_duty = msg.data;
}
//照準
void cmdToggleLidarCb(const std_msgs::Bool &lidar_msg) {
}
void cmdAngleCb(const std_msgs::Float64 &angle_msg) {
}
void cmdToggleReceiveCb(const std_msgs::Bool &recieve_msg) {
}
// 射出
void cmdToggleBeltCb(const std_msgs::Bool &belt_msg) {
  is_moving_belt = belt_msg.data;
}
void termBeltDutyCb(const std_msgs::Int32 &duty_msg) {
  belt_duty = duty_msg.data;
}
void cmdShootingDutyCb(const std_msgs::Int32 &duty_msg) {
  shoot_duty = duty_msg.data;
}
// 緊急停止
void cmdEmergencyStopCb(const std_msgs::Bool &stop_msg) {
  emergency_stop = stop_msg.data;
}

// トピックを受け取るためのサブスクライバーを作成
// 分離
ros::Subscriber<std_msgs::Bool> cmd_toggle_shoot_sub("cmd_toggle_shoot", &cmdToggleShootCb);
ros::Subscriber<std_msgs::Int32> term_spr_sub("term_spr", &termSprCb);
//照準
ros::Subscriber<std_msgs::Float64> cmd_angle_sub("cmd_angle", &cmdAngleCb);
ros::Subscriber<std_msgs::Bool> cmd_toggle_receive_sub("cmd_toggle_receive", &cmdToggleReceiveCb);
// 射出
ros::Subscriber<std_msgs::Bool> cmd_toggle_belt_sub("cmd_toggle_belt", &cmdToggleBeltCb);
ros::Subscriber<std_msgs::Int32> term_belt_duty_sub("term_belt_duty", &termBeltDutyCb);
ros::Subscriber<std_msgs::Int32> cmd_shooting_duty_sub("cmd_shooting_duty", &cmdShootingDutyCb);
// 緊急停止
ros::Subscriber<std_msgs::Bool> cmd_emergency_stop_sub("cmd_emergency_stop", &cmdEmergencyStopCb);

void setup() {
  // 分離デバッグ用
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);

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
  nh.subscribe(term_belt_duty_sub);
}

void loop() {
  nh.spinOnce();
  // 緊急停止信号が来たらすべて停止
  if (emergency_stop) {
    for (int i; i < DC_MOTOR_NUM; i++) {
      DC_motor::put(i, 0);
    }
    spr_is_go_separating = false;
    spr_is_come_separating = false;
  }

  if (is_moving_belt) digitalWrite(23, LOW);
  else digitalWrite(23, HIGH);
  if (emergency_stop) digitalWrite(24, HIGH);
  else digitalWrite(24, LOW);
  // 分離のDuty決定
  spr_set_duty();

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

void spr_set_duty() {
  int32_t enc_count = abs(Inc_enc::get(SPR_ENC_NUM));
  unsigned long time_now = micros() / 1000;

  // 立ち上がり(スイッチを押した瞬間)で分離実行を切り替え
  if (spr_sign && !spr_pre_sign) {
    spr_is_go_separating = true;
  }
  // digitalWrite(23, HIGH);
  // digitalWrite(24, HIGH);

  //両方とも真のときは停止してふたつとも偽とする
  if (spr_is_go_separating && !spr_is_come_separating) {
    if (enc_count < ONE_WAY_COUNT) {
      // digitalWrite(24, LOW);
      spr_duty = spr_indicated_duty;
      spr_stop_start_time = micros() / 1000;
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
  } else if (spr_is_stopping && (time_now - spr_stop_start_time > STOP_TIME)) {
    spr_is_come_separating = true;
    spr_is_stopping = false;
  } else {
    spr_is_go_separating = false;
    spr_is_come_separating = false;
    spr_duty = 0;
  }
}