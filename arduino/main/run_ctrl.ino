#include "param.h"


//グローバル変数の宣言
long l_enc = long(0), r_enc = long(0);
long l_enc_prev = long(0), r_enc_prev = long(0);
float l_spd = (float)0 , r_spd = (float)0;
float l_err_prev = long(0), r_err_prev = long(0);
float l_err_sum = long(0), r_err_sum = long(0);

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

            float new_value = (float)((hb * 254) + lb) / (float)GAIN_ACCURACY;
            all_gain[i][ii] = new_value;
        }
    }
    dt = (int)EEPROM.read(EEP_ORIGIN + 20);
}



void gain_eep_write(){
    for(int i = 0; i < 3; i++){
        for(int ii = 0; ii < 3; ii++){
            int hb = (int)(all_gain[i][ii] * (float)GAIN_ACCURACY / 254.0);
            int lb = (int)(all_gain[i][ii] * (float)GAIN_ACCURACY) - (hb * 254);
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
    mm = ((float)p * (float)TIRE_DIAM * PI * (float)MOTOR_GEAR_TOOTH) / ((float)TIRE_GEAR_TOOTH * (float)ENC_PPR * (float)GEAR_RATIO);
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
    long p;
    p = (long)(((float)mm * (float)TIRE_GEAR_TOOTH * (float)ENC_PPR * (float)GEAR_RATIO) / ((float)TIRE_DIAM * (float)PI * (float)MOTOR_GEAR_TOOTH));
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
    l_err_prev = (float)0;
    l_err_sum = (float)0;
    l_enc_prev = l_enc;
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
    r_err_prev = (float)0;
    r_err_sum = (float)0;
    r_enc_prev = r_enc;
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
        int hb, lb;
        Serial.write(255);
        Serial.write(14);
        long send_value_l = (long)(l_spd * (float)1000) + (long)8193532;
        long send_value_r = (long)(r_spd * (float)1000) + (long)8193532;
        int send_byte = 0;
        for(int i = 0; i < 3; i++){
            send_byte = (int)((send_value_l % (long)pow(254,i+1)) / (long)pow(254,i));
            Serial.write(send_byte);
        }
        for(int i = 0; i < 3; i++){
            send_byte = (int)((send_value_r % (long)pow(254,i+1)) / (long)pow(254,i));
            Serial.write(send_byte);
        }

        hb = (int)((l_spd_target + (long)32258) / (long)254);
        lb = (int)((l_spd_target + (long)32258) % (long)254);
        Serial.write(hb);
        Serial.write(lb);
        hb = (int)((r_spd_target + (long)32258) / (long)254);
        lb = (int)((r_spd_target + (long)32258) % (long)254);
        Serial.write(hb);
        Serial.write(lb);

        Serial.write(254);
    }


    l_spd = (float)pulse_to_mm(l_enc - l_enc_prev) / (float)dt * (float)1000.0;
    r_spd = (float)pulse_to_mm(r_enc - r_enc_prev) / (float)dt * (float)1000.0;


    //pidする
    float l_err = l_spd - (float)l_spd_target;
    float r_err = r_spd - (float)r_spd_target;
    l_err_sum += l_err;
    r_err_sum += r_err;
/*
    Serial.print(l_err);
    Serial.print(",");
    Serial.print(pulse_to_mm(l_enc - l_enc_prev));
    Serial.print(",");
    Serial.println(l_spd);*/

    float l_pid = (l_err * all_gain[0][0]) + ((l_err - l_err_prev) * all_gain[0][1]) + ((l_err_sum) * all_gain[0][2]); 
    float r_pid = (r_err * all_gain[1][0]) + ((r_err - r_err_prev) * all_gain[1][1]) + ((r_err_sum) * all_gain[1][2]); 


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
    l_enc_prev = l_enc;
    r_err_prev = r_err;
    r_enc_prev = r_enc;
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

/*
回転させる関数
引数：回転速度(int),角度(int)
戻り値：無し
*/
void rotate(int omega, int theta){
    long target_enc_l = l_enc;
    long target_enc_r = r_enc;
    int dir_l, dir_r;

    if(omega*theta >= 0){
        dir_l = -1;
        dir_r = 1;
    }else{
        dir_l = 1;
        dir_r = -1;
    }
    target_enc_l += (long)(dir_l)*mm_to_pulse((long)((float)TIRE_PITCH * PI * (float)theta / 360.0));
    target_enc_r += (long)(dir_r)*mm_to_pulse((long)((float)TIRE_PITCH * PI * (float)theta / 360.0));

    if(l_spd_target == 0L){
        pid_init_l();
    }
    if(r_spd_target == 0L){
        pid_init_r();
    }

    l_spd_target = (long)(dir_l)*mm_to_pulse((long)((float)TIRE_PITCH * PI * (float)omega / 360.0));
    r_spd_target = (long)(dir_r)*mm_to_pulse((long)((float)TIRE_PITCH * PI * (float)omega / 360.0));

    while(1){
        if(dir_l == 1){
            if(l_enc > target_enc_l && r_enc > target_enc_r){
                l_spd_target = 0L;
                r_spd_target = 0L;
                break;
            }
        }else{
            if(l_enc < target_enc_l && r_enc > target_enc_r){
                l_spd_target = 0L;
                r_spd_target = 0L;
                break;
            }
        }
        
        pid();
        check_serial();
    }
}