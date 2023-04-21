#include "cubic_arduino.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <adbot_msgs/SprMsg.h>

///初期状態は下の刃でリングを受け止めている状態を想定///
#define SPR_MOTOR 11       // 分離のモーター番号
#define SPR_ENC_NUM 0      // 分離のエンコーダ番号
#define ONE_WAY_COUNT 500  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5

int spr_duty = 250;   //指定するDutyの絶対値ROSメッセージから指定できる メインなら70、サブなら250
int duty = spr_duty;  //実際にCubicに指定する値

bool separate_sign = false;
bool separate_pre_sign = false;
bool is_go_separating = false;    //初期位置から折り返しまでの間は真
bool is_come_separating = false;  //折り返しから初期位置までの間は真
bool is_paused = true;

ros::NodeHandle nh;

// トピックのコールバック関数
void separateRingCallback(const adbot_msgs::SprMsg &spr_msg) {
  separate_pre_sign = separate_sign;
  separate_sign = spr_msg.isOn;
}
void termCallback(const std_msgs::Int16 &msg) {
  spr_duty = msg.data;
}

// トピックを受け取るためのサブスクライバーを作成
ros::Subscriber<adbot_msgs::SprMsg> sub("separate", &separateRingCallback);
ros::Subscriber<std_msgs::Int16> term_sub("term_separate", &termCallback);

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
  nh.subscribe(term_sub);
}

void loop() {
  nh.spinOnce();

  int32_t enc_count = abs(Inc_enc::get(SPR_ENC_NUM));

  // 立ち上がり(スイッチを押した瞬間)で分離実行を切り替え
  if (separate_sign && !separate_pre_sign) {
    is_go_separating = true;
  }

  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);

  //両方とも真のときは停止してふたつとも偽とする
  if (is_go_separating && !is_come_separating) {
    if (enc_count < ONE_WAY_COUNT) {
      digitalWrite(24, LOW);
      duty = spr_duty;
    } else {
      is_go_separating = false;
      is_come_separating = true;
    }
  } else if (is_come_separating && !is_go_separating) {
    if (enc_count > 50) {
      digitalWrite(23, LOW);
      duty = -spr_duty;
    } else {
      is_come_separating = false;
      duty = 0;
    }
  } else {
    is_go_separating = false;
    is_come_separating = false;
    duty = 0;
  }
  DC_motor::put(SPR_MOTOR, duty);  //dutyをセット

  // データの送受信を行う
  Cubic::update();
}