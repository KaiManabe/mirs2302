#include "param.h"
#include <EEPROM.h>
//#include<Servo.h>
//Servo servo;

/*グローバル変数の宣言*/
long l_spd_target = 0L;
long r_spd_target = 0L;

//フォトリフレクタ状態フラグ
const int photo_flg_err = 1;
const int photo_flg_ok = 0;
int photo_curr = 0; //現在の車体が浮いた回数

void init_pin(){
    //エンコーダのピン宣言
    pinMode(L_ENC_A,INPUT);
    pinMode(L_ENC_B,INPUT);
    pinMode(R_ENC_A,INPUT);
    pinMode(R_ENC_B,INPUT);

    //エンコーダのプルアップ抵抗を有効
    //2023-12-05 : エンコーダ用基板を作った　無効に変更した
    digitalWrite(L_ENC_B,0);
    digitalWrite(L_ENC_A,0);
    digitalWrite(R_ENC_A,0);
    digitalWrite(R_ENC_B,0);

    //モタドラの出力ピン宣言
    pinMode(L_MOT_DIR, OUTPUT);
    pinMode(R_MOT_DIR, OUTPUT);
    pinMode(L_MOT_PWM, OUTPUT);
    pinMode(R_MOT_PWM, OUTPUT);

    //ペルチェ素子リレー出力ピン宣言
    pinMode(PELTIER, OUTPUT);

    //サーミスタ値読み取りピン宣言
    pinMode(THERMISTOR1, INPUT);
    pinMode(THERMISTOR2, INPUT);

    //フォトリフレクタ入力ピン宣言
    pinMode(PHOTO, INPUT);

    //サーボモータの出力ピン宣言
    pinMode(SERVO, OUTPUT);
    //servo.attach(SERVO, 500, 2500);
}



void setup(){
    init_pin(); //ピンの初期化をする関数
    //エンコーダの割り込み関数を割り当て
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), l_enc_change, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), r_enc_change, CHANGE);

    //シリアル通信開始
    Serial.begin(115200);
    
    #ifdef ENABLE_EEP
        gain_eep_replace();
    #endif
}

void loop(){
    check_serial();
    pid();
    //send_odom();
}
