#include "cubic_arduino.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// #include <adbot_msgs/SprMsg.h>

///初期状態は下の刃でリングを受け止めている状態を想定///
#define SPR_MOTOR 8       // 分離のモーター番号
#define SPR_ENC_NUM 0      // 分離のエンコーダ番号
#define ONE_WAY_COUNT 400  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5
#define STOP_TIME 500  // ミリ秒

// DCモータ番号
#define SHOOT_MOTOR_LU 0  // 左上
#define SHOOT_MOTOR_LD 1  // 左下
#define SHOOT_MOTOR_RU 2  // 右上
#define SHOOT_MOTOR_RD 3  // 右下
#define BELT_MOTOR 6      // ベルト

//分離
int32_t spr_indicated_duty = 70;        //指定するDutyの絶対値。ROSメッセージから指定できる。
int32_t spr_duty = spr_indicated_duty;  //実際にCubicに指定する値
bool spr_sign = false;
bool spr_pre_sign = false;
bool spr_is_go_separating = false;    //初期位置から折り返しまでの間は真
bool spr_is_come_separating = false;  //折り返しから初期位置までの間は真
bool spr_is_stopping = false;
unsigned long spr_stop_start_time = 0;

void spr_set_duty(void);

ros::NodeHandle nh;

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
}
void cmdAngleCb(const std_msgs::Float64 &angle_msg) {
}
void cmdToggleReceiveCb(const std_msgs::Bool &recieve_msg) {
}
void cmdToggleBeltCb(const std_msgs::Bool &belt_msg) {
}
void cmdToggleLidarCb(const std_msgs::Bool &lidar_msg) {
}
void cmdEmergencyStopCb(const std_msgs::Bool &stop_msg) {
}

// トピックを受け取るためのサブスクライバーを作成
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
  pinMode(24, OUTPUT);
  pinMode(23, OUTPUT);
  // すべてのモータ，エンコーダの初期化
  Cubic::begin(3.0);
  Inc_enc::reset();
  nh.getHardware()->setBaud(9600);

  // ROSの通信を開始
  nh.initNode();
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
  nh.spinOnce();

  spr_set_duty();

  DC_motor::put(SPR_MOTOR, spr_duty);  //dutyをセット

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
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);

  //両方とも真のときは停止してふたつとも偽とする
  if (spr_is_go_separating && !spr_is_come_separating) {
    if (enc_count < ONE_WAY_COUNT) {
      digitalWrite(24, LOW);
      spr_duty = spr_indicated_duty;
      spr_stop_start_time = micros() / 1000;
    } else {
      spr_is_go_separating = false;
      spr_is_stopping = true;
    }
  } else if (spr_is_come_separating && !spr_is_go_separating) {
    if (enc_count > 100) {
      digitalWrite(23, LOW);
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