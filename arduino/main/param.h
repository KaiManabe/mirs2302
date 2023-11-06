//PID用
#define STRAIGHT_GAIN_L_SPD {1.0, 0.04, 0.3}    //左のスピード
#define STRAIGHT_GAIN_R_SPD {1.0, 0.04, 0.3}    //右のスピード
#define STRAIGHT_LR_ENC {0.5, 0.2, 0.0}    //偏角差
#define STRAIGHT_DELTA_T 15 //[ms]

#define ROTATE_GAIN_L {1.0, 0.02, 0.3}    //左のスピード
#define ROTATE_GAIN_R {1.0, 0.02, 0.3}    //右のスピード
#define ROTATE_GAIN_LR {0.5, 0.2, 0.0}    //偏角差
#define ROTATE_DELTA_T 15

//機体パラメータ
#define ENC_PPR 26  //エンコーダの回転毎のパルス
#define GEAR_RATIO 13.733564    //ギア比
#define TIRE_DIAM 87.5 //タイヤ直径[mm]

#define TIRE_PITCH 320

#define PI 3.141592653589

#define DEFAULT_ALPHA 1.4


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
#define MODULE1 A2
#define MODULE2 A3
#define MODULE3 A4

//モジュール抵抗
#define MODULE_R[3] = {1000,1200,1500}

//バッテリ分圧比
#define V_RATIO 0.5 

//型ごとの最大値
#define UINTMAX 65535
#define LONGMAX 2147483647
#define LONGMIN -2147483648
