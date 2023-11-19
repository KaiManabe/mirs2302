#include "param.h"


//グローバル変数の宣言
long l_enc = long(0), r_enc = long(0);
long l_err_prev = long(0), r_err_prev = long(0);
long l_err_sum = long(0), r_err_sum = long(0);
long l_enc_target = long(0), r_enc_target = long(0);

//ゲインを動的に変更できるように、配列にしてある
float all_gain[3][3] = {GAIN_L, GAIN_R, GAIN_LR};

/*
pid中にシリアル通信でenc値を送るかどうか
0ならなにもしない
1ならencのみ送信
2ならencとenc_targetを送信
*/
int pid_serial_mode = 0;

//制御周期を動的に変更できるように、配列にしてある
int dt = DELTA_T;

bool stop_signal = 0;



void gain_eep_replace(){
    for(int i = 0; i < 3; i++){
        for(int ii = 0; ii < 3; ii++){
            int address = (i * 6) + (ii * 2) + EEP_ORIGIN;
            int hb = EEPROM.read(address);
            int lb = EEPROM.read(address + 1);

            float new_value = (float)((hb * 254) + lb) / (float)10000.0;
            all_gain[i][ii] = new_value;
        }
    }
    dt = (int)EEPROM.read(EEP_ORIGIN + 20);
}



void gain_eep_write(){
    for(int i = 0; i < 3; i++){
        for(int ii = 0; ii < 3; ii++){
            int hb = (int)(all_gain[i][ii] * 10000.0 / 254.0);
            int lb = (int)(all_gain[i][ii] * 10000.0) - (hb * 254);
            int address = (i * 6) + (ii * 2) + EEP_ORIGIN;
            EEPROM.update(address, hb);
            EEPROM.update(address+1, lb);
        }
    }
    EEPROM.update(EEP_ORIGIN + 20, dt);
}




/*
ロータリーエンコーダのパルスをmmに変換する関数

引数：
    long p : エンコーダ値

戻り値：
    距離[mm] -> long
*/
float pulse_to_mm(long p){
    float mm;
    mm = ((float)p / ((float)ENC_PPR * (float)GEAR_RATIO) * (float)TIRE_GEAR_TOOTH / (float)MOTOR_GEAR_TOOTH) * (float)(TIRE_DIAM * PI);
    return mm;
}



/*
mmをロータリーエンコーダのパルスに変換する関数

引数：
    long mm : 距離(mm)

戻り値：
    エンコーダ値 -> long
*/
long mm_to_pulse(long mm){
    float p;
    p = long((mm / (float)(TIRE_DIAM * PI)) * ((float)ENC_PPR * (float)GEAR_RATIO * (float)TIRE_GEAR_TOOTH / (float)MOTOR_GEAR_TOOTH));
    return p;
}

/*
pwm値でモータに電圧を印加する関数
引数１：左の電圧   (-255～255)
引数２：右の電圧   (-255～255)
（範囲外の値は強制的に-255～255に変換されます）
*/
void pwm_write(int l, int r){
    //正負を判定してDIRピンを出力する
    if (l > 0){
        digitalWrite(L_MOT_DIR,1);
    }else{
        digitalWrite(L_MOT_DIR,0);
    }

    if (r > 0){
        digitalWrite(R_MOT_DIR,1);
    }else{
        digitalWrite(R_MOT_DIR,0);
    }

    //PWMを出力
    if(abs(l) > 255){
        analogWrite(L_MOT_PWM, 255);
    }else{
        analogWrite(L_MOT_PWM, abs(l));
    }
    if(abs(r) > 255){
        analogWrite(R_MOT_PWM, 255);
    }else{
        analogWrite(R_MOT_PWM, abs(r));
    }
}


/*
左モータのpid制御を初期化する関数
停止状態から走行状態に移行するためには、これを実行する必要がある

引数:
    なし

戻り値:
    なし
*/
void pid_init_l(){
    l_err_prev = 0L;
    l_err_sum = 0L;

    l_enc_target = l_enc;
}


/*
右モータのpid制御を初期化する関数
停止状態から走行状態に移行するためには、これを実行する必要がある

引数:
    なし

戻り値:
    なし
*/
void pid_init_r(){
    r_err_prev = 0L;
    r_err_sum = 0L;

    r_enc_target = r_enc;
}




/*
pid制御の1周期を担う関数
void loop()の中にこれを入れて、制御周期よりも短い間隔でこれを実行する必要がある


引数:
    なし

戻り値:
    なし
*/
void pid(){
    //前回pid関数が実行された時刻を覚えておくstatic変数
    static long cmillis = 0L;

    //[制御周期dt]以上の時間が経過していなければ終了
    if(millis() < cmillis + (long)dt){
        return;
    }
    

    
    if(pid_serial_mode > 0){
        Serial.write(255);
        Serial.write(14);
        long send_value_l = l_enc + (long)2081157128;
        long send_value_r = r_enc + (long)2081157128;
        int send_byte = 0;
        for(int i = 0; i < 4; i++){
            send_byte = (int)((send_value_l % (long)pow(254,i+1)) / (long)pow(254,i));
            Serial.write(send_byte);
        }
        for(int i = 0; i < 4; i++){
            send_byte = (int)((send_value_r % (long)pow(254,i+1)) / (long)pow(254,i));
            Serial.write(send_byte);
        }
        Serial.write(254);
    }

    if(pid_serial_mode == 2){
        Serial.write(255);
        Serial.write(15);
        long send_value_l = l_enc_target + (long)2081157128;
        long send_value_r = r_enc_target + (long)2081157128;
        int send_byte = 0;
        for(int i = 0; i < 4; i++){
            send_byte = (int)((send_value_l % (long)pow(254,i+1)) / (long)pow(254,i));
            Serial.write(send_byte);
        }
        for(int i = 0; i < 4; i++){
            send_byte = (int)((send_value_r % (long)pow(254,i+1)) / (long)pow(254,i));
            Serial.write(send_byte);
        }
        Serial.write(254);
    }


    //目標量の計算
    if(l_spd_target > 0L){
        l_enc_target += long(abs((float)mm_to_pulse(l_spd_target) * (float)(dt) / (float)(1000.0)));
    }else{
        l_enc_target -= long(abs((float)mm_to_pulse(l_spd_target) * (float)(dt) / (float)(1000.0)));
    }

    if(r_spd_target > 0L){
        r_enc_target += long(abs((float)mm_to_pulse(r_spd_target) * (float)(dt) / (float)(1000.0)));
    }else{
        r_enc_target -= long(abs((float)mm_to_pulse(r_spd_target) * (float)(dt) / (float)(1000.0)));
    }
    /*
    Serial.print(r_enc_target);
    Serial.print(",");
    Serial.println(r_enc);
    */
    //pidする
    long l_err = l_enc - l_enc_target;
    long r_err = r_enc - r_enc_target;
    l_err_sum += l_err;
    r_err_sum += r_err;

    float l_pid = (float)l_err * all_gain[0][0] + (float)(l_err - l_err_prev) * all_gain[0][1] + (float)(l_err_sum) * all_gain[0][2]; 
    float r_pid = (float)r_err * all_gain[1][0] + (float)(r_err - r_err_prev) * all_gain[1][1] + (float)(r_err_sum) * all_gain[1][2]; 


    /*目標速度が0に設定されているならばモータに出力を加えないようにする*/
    if(l_spd_target != 0L && r_spd_target != 0L){
        pwm_write((int)l_pid, (int)r_pid);
    }

    if(l_spd_target == 0L && r_spd_target == 0L){
        pwm_write(0,0);
    }else if(l_spd_target == 0L){
        pwm_write(0, (int)r_pid);
    }else if(r_spd_target == 0L){
        pwm_write((int)l_pid, 0);
    }


    cmillis = millis();
    l_err_prev = l_err;
    r_err_prev = r_err;
}


/*
左エンコーダを監視してl_encを増減させる関数
詳細不明(コピペした)
*/
void l_enc_change(){
    static bool a_prev = 0, b_prev = 0;
    bool a, b;
    a = digitalRead(L_ENC_A);
    b = digitalRead(L_ENC_B);

    if(a == 1 && b == 0 && a_prev == 0 && b_prev == 1){
        l_enc -= 1L;
    }
    if(a == 0 && b == 1 && a_prev == 1 && b_prev == 0){
        l_enc -= 1L;
    }
    if(a == 1 && b == 1 && a_prev == 0 && b_prev == 0){
        l_enc += 1L;
    }
    if(a == 0 && b == 0 && a_prev == 1 && b_prev == 1){
        l_enc += 1L;
    }
    

    a_prev = a;
    b_prev = b;
}

/*
右エンコーダを監視してl_encを増減させる関数
詳細不明(コピペした)
*/
void r_enc_change(){
    //static long prev_time = 0L, prev_ct = 0L;
    static bool a_prev = 0, b_prev = 0;
    bool a, b;
    
    a = digitalRead(R_ENC_A);
    b = digitalRead(R_ENC_B);

    if(a == 1 && b == 0 && a_prev == 0 && b_prev == 1){
        r_enc += 1L;
    }
    if(a == 0 && b == 1 && a_prev == 1 && b_prev == 0){
        r_enc += 1L;
    }
    if(a == 1 && b == 1 && a_prev == 0 && b_prev == 0){
        r_enc -= 1L;
    }
    if(a == 0 && b == 0 && a_prev == 1 && b_prev == 1){
        r_enc -= 1L;
    }
 
    a_prev = a;
    b_prev = b;
}
