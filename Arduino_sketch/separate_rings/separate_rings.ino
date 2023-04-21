#include "cubic_arduino.h"
// #include <ros.h>
// #include <adbot_msgs/SprMsg.h>

#define SPR_MOTOR 8        // 分離のモーター番号
#define SPR_ENC_NUM 0      // 分離のエンコーダ番号
#define ONE_WAY_COUNT 400  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5
#define STOP_TIME 500  // ミリ秒

int spr_duty = 70;    //指定するDutyの絶対値ROSメッセージから指定できる
int duty = spr_duty;  //実際にCubicに指定する値

bool separate_sign = false;
bool separate_pre_sign = false;
bool is_go_separating = false;
bool is_come_separating = false;
bool is_stopping = false;

// bool is_checking = false;
unsigned long stop_start_time = 0;



// ros::NodeHandle nh;

// トピックのコールバック関数
// void separateRingCallback(const adbot_msgs::SprMsg &spr_msg) {
//   separate_pre_sign = separate_sign;
//   separate_sign = spr_msg.isOn;
//   spr_duty = spr_msg.duty;
// }

// トピックを受け取るためのサブスクライバーを作成
// ros::Subscriber<adbot_msgs::SprMsg> sub("separate", &separateRingCallback);

void setup() {
  Serial.begin(115200);
  pinMode(24, OUTPUT);
  pinMode(23, OUTPUT);
  // すべてのモータ，エンコーダの初期化
  Cubic::begin(3.0);
  Inc_enc::reset();
  // nh.getHardware()->setBaud(9600);

  // ROSの通信を開始
  // nh.initNode();
  // nh.subscribe(sub);
}

void loop() {
  if (Serial.available() > 0) {
    char mode = Serial.read();
    if (mode == 'd') {
      // spr_duty = Serial.readStringUntil(':').toInt();
      // is_go_separating = Serial.readStringUntil('\n').toInt();
      is_go_separating = true;
    }

    else if (mode == 'r') {
      spr_duty = 0;
      is_go_separating = false;
      is_come_separating = false;
    }
  }
  int16_t enc_diff = Inc_enc::get_diff(SPR_ENC_NUM);
  int32_t enc_count = abs(Inc_enc::get(SPR_ENC_NUM));
  unsigned long time_now = micros() / 1000;
  Serial.print(time_now);
  Serial.print(",");
  Serial.print(stop_start_time);
  Serial.print(",");
  Serial.print(enc_diff);
  Serial.print(",");
  Serial.print(enc_count);
  Serial.print(",");
  Serial.print(spr_duty);
  Serial.print(",");
  Serial.println(duty);
  // nh.spinOnce();

  // 立ち上がり(スイッチを押した瞬間)で分離実行を切り替え
  if (separate_sign && !separate_pre_sign) {
    is_go_separating = true;
  }
  // // ストッパーに引っかかってたら分離中断
  // if ((abs(enc_diff) < ENC_DIFF_MIN) && (abs(duty) > 5)) {
  //   time_now = micros() / 1000;
  //   if (!is_checking) {
  //     is_checking = true;
  //     time_start = time_now;
  //     check_time = 0;
  //   } else {
  //     check_time = time_now - time_start;
  //   }
  //   if (check_time > 1000)  //1000ms以上エンコーダの差分が小さいかつ指定Dutyが0でなければ分離中断
  //     is_separating = false;
  // } else {
  //   is_checking = false;
  // }
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);
  if (is_go_separating && !is_come_separating) {
    if (enc_count < ONE_WAY_COUNT) {
      digitalWrite(24, LOW);
      duty = spr_duty;
      stop_start_time = micros() / 1000;
    } else {
      is_go_separating = false;
      is_stopping = true;
    }
  } else if (is_come_separating && !is_go_separating) {
    if (enc_count > 100) {
      digitalWrite(23, LOW);
      duty = -spr_duty;
    } else {
      is_come_separating = false;
      duty = 0;
    }
  } else if (is_stopping && (time_now - stop_start_time > STOP_TIME)) {
    is_come_separating = true;
    is_stopping = false;
  } else {
    is_go_separating = false;
    is_come_separating = false;
    duty = 0;
  }
  DC_motor::put(SPR_MOTOR, duty);

  // データの送受信を行う
  Cubic::update();
}