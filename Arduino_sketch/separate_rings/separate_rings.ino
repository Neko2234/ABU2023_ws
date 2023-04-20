#include "cubic_arduino.h"
#include <ros.h>
#include <adbot_msgs/SprMsg.h>

#define SPR_MOTOR 8    // 分離のモーター番号
#define SPR_ENC_NUM 0  // 分離のエンコーダ番号
#define ONE_WAY_COUNT 500  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5

int spr_duty = 50;   //指定するDutyの絶対値ROSメッセージから指定できる
int duty = spr_duty;  //実際にCubicに指定する値

bool separate_sign = false;
bool separate_pre_sign = false;
bool is_separating = false;
bool is_paused = true;

bool is_checking = false;
unsigned long check_time = 0;
unsigned long time_start = 0;
unsigned long time_now = 0;


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
  pinMode(24, OUTPUT);
  pinMode(23, OUTPUT);

  // すべてのモータ，エンコーダの初期化
  Cubic::begin(3.0);
  Inc_enc::reset();
  nh.getHardware()->setBaud(9600);

  // ROSの通信を開始
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  int16_t enc_diff = Inc_enc::get_diff(SPR_ENC_NUM);
  int32_t enc_count = Inc_enc::get(SPR_ENC_NUM);

  nh.spinOnce();

  // 立ち上がり(スイッチを押した瞬間)で分離実行を切り替え
  if (separate_sign && !separate_pre_sign) {
    is_separating = !is_separating;
  }

  if (is_separating) {

    digitalWrite(23, HIGH);
    if (enc_count > ONE_WAY_COUNT) {
      digitalWrite(24, LOW);
      duty = spr_duty;
    } else if (enc_count < 0) {
      digitalWrite(24, HIGH);
      duty = -spr_duty;
    } else if (is_paused) {
      duty = spr_duty;
      is_paused = false;
    }
  } else {
    digitalWrite(23, LOW);
    is_paused = true;
    duty = 0;
  }
  DC_motor::put(SPR_MOTOR, duty);

  // データの送受信を行う
  Cubic::update();
}
