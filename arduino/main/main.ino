#include "param.h"
#include <EEPROM.h>
//#include<Servo.h>
//Servo servo;

/*グローバル変数の宣言*/
long l_spd_target = 0L;
long r_spd_target = 0L;


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

    //サーボモータの出力ピン宣言
    //pinMode(SERVO, OUTPUT);
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
}
