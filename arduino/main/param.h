//defineされている場合に、設定値ミリ秒ごとにエンコーダの値を送信する
#define SEND_ENC 50

/*
defineされている場合には、ゲインを保存する。
また、保存されたゲインを使用する

ただし、EEPROMには寿命があり、10万回以上書き込むと正常な動作を保証できなくなるとされている。
自動ゲイン調整など頻繁にゲイン変更が行われる操作をする前にはこれをコメントアウトしてから書き直すこと。
*/
#define ENABLE_EEP

#define GAIN_ACCURACY  10000.0

//defineされている場合に、エンコーダ値に加えて目標値を送信する
#define SEND_ENC_TARGET


#define GAIN_L {0.1, 0.01, 0.01}
#define GAIN_R {0.1, 0.01, 0.01}
#define GAIN_LR {0.1, 0.01, 0.1}
#define DELTA_T 20

//機体パラメータ
#define ENC_PPR 26  //エンコーダの回転毎のパルス
#define GEAR_RATIO 13.733564    //ギア比
#define TIRE_GEAR_TOOTH 20  //タイヤについてるギアの歯数
#define MOTOR_GEAR_TOOTH 10     //モータについてるギアの歯数
#define TIRE_DIAM 177   //タイヤの直径

#define TIRE_PITCH 555  //タイヤの間隔

#define PI 3.141592653589


//send_odometry関数でencの微分を求めるときのΔt [ms]
#define ODOM_DT 50

//エンコーダ入力ピン
#define R_ENC_A 3
#define R_ENC_B 7
#define L_ENC_A 2
#define L_ENC_B 4

//モタドラ出力ピン
#define L_MOT_DIR 12
#define L_MOT_PWM 11
#define R_MOT_DIR 8
#define R_MOT_PWM 9

//バッテリー監視ピン
#define BATT A5

//モジュールの抵抗測定ピン
#define MODULE1 A2  //1段目
#define MODULE2 A3  //2段目
#define MODULE3 A4  //3段目

//モジュール抵抗
#define MODULE_R1 200  //食品
#define MODULE_R2 300  //紙
#define MODULE_R3 430  //小物

//モジュール基本抵抗
#define MODULE_R 500

//バッテリ分圧比
#define V_RATIO 0.5 

//サーボpwm値出力ピン
#define SERVO 10

//ペルチェ素子リレー出力ピン
#define PELTIER 13

//サーミスタ値読み取りピン
#define THERMISTOR1 A0
#define THERMISTOR2 A1

//型ごとの最大値
#define UINTMAX 65535
#define LONGMAX 2147483647
#define LONGMIN -2147483648

//フォトリフレクタ入力ピン
#define PHOTO 5

//EEPの番地 変えるな
#define EEP_ORIGIN 22
