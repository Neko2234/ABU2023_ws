#include "cubic_arduino.h"
#include <ros.h>
#include <adbot_msgs/SprMsg.h>

#define SPR_MOTOR 8  // 分離のモーター番号
#define STOP 0
#define CW 1
#define CCW 2
#define ONE_WAY_TIME 3000  //ミリ秒
#define STOP_TIME 1000

int spr_duty = 800;
unsigned long time_prev = 0;
unsigned long time_now = 0;
unsigned long dt = 0;
unsigned long corrected_dt = 0;
bool separate_sign = false;
bool separate_pre_sign = false;
bool is_separating = true;
bool is_pausing = false;
unsigned long pause_start_time = 0;
unsigned long pause_end_time = 0;
unsigned long pause_time = 0;

ros::NodeHandle nh;

// トピックのコールバック関数
void separateRingCallback(const adbot_msgs::SprMsg &spr_msg) {
  separate_pre_sign = separate_sign;
  separate_sign = spr_msg.isOn;
  spr_duty = spr_msg.duty;
}

// トピックを受け取るためのサブスクライバーを作成
ros::Subscriber<adbot_msgs::SprMsg> sub("separate", &separateRingCallback);

void setup() {
  Serial.begin(9600);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  // すべてのモータ，エンコーダの初期化
  Cubic::begin();
  nh.getHardware()->setBaud(9600);

  // ROSの通信を開始
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(30);
  Serial.print(time_now);
  Serial.print(",");
  Serial.print(time_prev);
  Serial.print(",");
  Serial.print(pause_time);
  Serial.print(",");
  Serial.print(dt);
  Serial.print(",");
  Serial.print(corrected_dt);
  Serial.print(",");
  Serial.println(is_separating);


  // 立ち上がり(スイッチを押した瞬間)で分離実行を切り替え
  if (separate_sign && !separate_pre_sign) {
    is_separating = !is_separating;
  }

  if (is_separating) {
    digitalWrite(23, LOW);
    time_now = millis();

    if (!is_pausing) {
      pause_end_time = time_now;
      is_pausing = false;
    }
    pause_time = pause_end_time - pause_start_time;

    dt = time_now - time_prev;
    corrected_dt = dt - pause_time; // pausetime is bad

    if (corrected_dt < ONE_WAY_TIME) {
      pause_time = 0;
      digitalWrite(22, LOW);
      DC_motor::put(SPR_MOTOR, spr_duty);
    } else if (corrected_dt < ONE_WAY_TIME + STOP_TIME) {
      DC_motor::put(SPR_MOTOR, 0);
    } else if (corrected_dt < ONE_WAY_TIME * 2 + STOP_TIME) {
      digitalWrite(22, LOW);
      DC_motor::put(SPR_MOTOR, -spr_duty);
    } else if (corrected_dt < ONE_WAY_TIME * 2 + STOP_TIME * 2) {
      DC_motor::put(SPR_MOTOR, 0);
    } else {
      time_prev = time_now;
    }
  } else {
    digitalWrite(23, HIGH);
    is_pausing = true;
    pause_start_time = time_now;
  }

  // データの送受信を行う
  Cubic::update();
}
