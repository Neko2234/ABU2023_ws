#include "cubic_arduino.h"
#include <ros.h>
#include <fabot_msgs/ArmMsg.h>

#define HAND_MOTOR 0  // 手のモーター番号
#define ARM_MOTOR 1   // 腕のモーター番号
#define STOP 0
#define DO_OPEN 1
#define DO_CLOSE 2
#define DO_UP 1
#define DO_DOWN 2

int duty = 500;
int hand_state = 0;
int arm_state = 0;

ros::NodeHandle nh;

// トピックのコールバック関数
void loadRingCallback(const fabot_msgs::ArmMsg &arm_msg) {
  // messageが0なら停止、1なら開く、2なら閉じる
  hand_state = arm_msg.hand;
  arm_state = arm_msg.arm;
}

// トピックを受け取るためのサブスクライバーを作成
ros::Subscriber<fabot_msgs::ArmMsg> sub("hand_state", &loadRingCallback);

void setup() {
  pinMode(22, OUTPUT);
  // すべてのモータ，エンコーダの初期化
  Cubic::begin();
  nh.getHardware()->setBaud(115200);

  // ROSの通信を開始
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(30);

  if (hand_state == STOP) {
    digitalWrite(23, HIGH);
    DC_motor::put(HAND_MOTOR, 0);
  } else if (hand_state == DO_OPEN) {
    digitalWrite(23, LOW);
    DC_motor::put(HAND_MOTOR, duty);
  } else if (hand_state == DO_CLOSE) {
    digitalWrite(23, LOW);
    DC_motor::put(HAND_MOTOR, -duty);
  }
  
  if (arm_state == STOP) {
    digitalWrite(23, HIGH);
    DC_motor::put(ARM_MOTOR, 0);
  } else if (arm_state == DO_UP) {
    digitalWrite(23, LOW);
    DC_motor::put(ARM_MOTOR, duty);
  } else if (arm_state == DO_DOWN) {
    digitalWrite(23, LOW);
    DC_motor::put(ARM_MOTOR, -duty);
  }

  // データの送受信を行う
  Cubic::update();
}