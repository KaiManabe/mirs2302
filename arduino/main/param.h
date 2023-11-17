#define GAIN_L {1.0, 0.01, 0.3}
#define GAIN_R {1.0, 0.01, 0.3}
#define GAIN_LR {0.1, 0.01, 0.1}
#define DELTA_T 20

//機体パラメータ
#define ENC_PPR 26  //エンコーダの回転毎のパルス
#define GEAR_RATIO 13.733564    //ギア比
#define TIRE_GEAR_TOOTH 20  //タイヤについてるギアの歯数
#define MOTOR_GEAR_TOOTH 10     //モータについてるギアの歯数
#define TIRE_DIAM 177   //タイヤの直径

#define TIRE_PITCH 320  //タイヤの間隔

#define PI 3.141592653589


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
#define MODULE_R 1000

//バッテリ分圧比
#define V_RATIO 0.5 

//サーボpwm値出力ピン
#define SERVO 10

//型ごとの最大値
#define UINTMAX 65535
#define LONGMAX 2147483647
#define LONGMIN -2147483648



//EEPの番地 変えるな
#define EEP_ORIGIN 22