#include "param.h"
#include<stdio.h>
#include<Servo.h>
Servo servo;

void setup(){
    init_pin(); //ピンの初期化をする関数    

    //エンコーダの割り込み関数を割り当て
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), l_enc_change, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), r_enc_change, CHANGE);

    //シリアル通信開始
    Serial.begin(115200);
}

void loop(){
    //モータの動作確認をする
    //pwm_write(100,100);

    //直進のテストをする
    //straight(-1000.0, 200.0);
    //while(1){}//

    //回転のテストをする
    //rotate(300.0,100.0);
    //while(1){}

    //シリアル通信をする
    serial_receive(false);
    //delay(100);

    //バッテリ値読み取り
    //get_batt();
    //delay(100);
    //while(1){}
}

void init_pin(){
    //エンコーダのピン宣言
    pinMode(L_ENC_A,INPUT);
    pinMode(L_ENC_B,INPUT);
    pinMode(R_ENC_A,INPUT);
    pinMode(R_ENC_B,INPUT);

    //エンコーダのプルアップ抵抗を有効
    digitalWrite(L_ENC_B,1);
    digitalWrite(L_ENC_A,1);
    digitalWrite(R_ENC_A,1);
    digitalWrite(R_ENC_B,1);

    //モタドラの出力ピン宣言
    pinMode(L_MOT_DIR, OUTPUT);
    pinMode(R_MOT_DIR, OUTPUT);
    pinMode(L_MOT_PWM, OUTPUT);
    pinMode(R_MOT_PWM, OUTPUT);

    //バッテリの電圧測定のピン宣言
    pinMode(BATT, INPUT);
    digitalWrite(BATT, LOW);

    //サーボモータの出力ピン宣言
    pinMode(SERVO, OUTPUT);
    servo.attach(SERVO, 500, 2500);
}
