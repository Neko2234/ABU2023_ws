#include "cubic_arduino.h"
#include <ros.h>
#include <adbot_msgs/SprMsg.h>

/*モーターの利用*/
// put関数で各モータのduty比指定，duty比の符号反転で逆回転
// Dutyの最大値はデフォルトで1000

/*インクリメントエンコーダの利用*/
// get関数で各エンコーダの累積値を取得．get_diff関数で各エンコーダの差分値を取得．

/*アブソリュートエンコーダの利用*/
// get関数で各エンコーダの絶対位置[0, 16383]を取得．

/*ソレノイドの利用*/
// put関数で各ソレノイドの状態指定
// get関数で各ソレノイドの状態を取得

// 各種print関数はSerial.begin()を実行したあとでないと使えない．(ROSと一緒には使えない)

// 詳しくはヘッダファイル参照
#define SPR_MOTOR 8    // 分離のモーター番号
#define SPR_ENC_NUM 0  // 分離のエンコーダ番号
#define ONE_WAY_COUNT 500  //片道のエンコーダカウント
#define STOP_COUNT 1000
#define ENC_DIFF_MIN 5

int spr_duty = 50;   //指定するDutyの絶対値ROSメッセージから指定できる
int duty = spr_duty;  //実際にCubicに指定する値
bool separate_sign = false;
bool separate_pre_sign = false;
bool is_separating = true;
bool is_paused = true;

ros::NodeHandle nh;

トピックのコールバック関数
void separateRingCallback(const adbot_msgs::SprMsg &spr_msg) {
  separate_pre_sign = separate_sign;
  separate_sign = spr_msg.isOn;
  spr_duty = spr_msg.duty;
}

トピックを受け取るためのサブスクライバーを作成
ros::Subscriber<adbot_msgs::SprMsg> sub("separate", &separateRingCallback);

void setup() {
  //デバッグ用
  pinMode(24, OUTPUT);
  pinMode(23, OUTPUT);
  
  // すべてのモータ，エンコーダの初期化
  // 第1引数に最大許容電流を与える．
  Cubic::begin(3.0);
  // Serial.begin(115200);

  Inc_enc::reset();
  nh.getHardware()->setBaud(9600);

  // time_prev = micros();
  ROSの通信を開始
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
