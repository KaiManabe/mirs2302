


//グローバル変数の宣言
long l_enc = long(0), r_enc = long(0);

//ゲインを動的に変更できるように、配列にしてある
float all_gain[6][3] = {STRAIGHT_GAIN_L_SPD, 
                    STRAIGHT_GAIN_R_SPD,
                    STRAIGHT_LR_ENC,
                    ROTATE_GAIN_L,
                    ROTATE_GAIN_R,
                    ROTATE_GAIN_LR};

//制御周期を動的に変更できるように、配列にしてある
//[0]が直進用制御周期     [1]が回転用制御周期
int dt[2] = {STRAIGHT_DELTA_T, ROTATE_DELTA_T};
float alpha_var = DEFAULT_ALPHA;

bool stop_signal = 0;


/*
ロータリーエンコーダのパルスをmmに変換する関数
引数：long型のエンコーダ値
戻り値：距離(mm)をfloat型で
*/
float pulse_to_mm(long p){
    float mm;
    mm = ((float)p / ((float)ENC_PPR * (float)GEAR_RATIO)) * (float)(TIRE_DIAM * PI);
    return mm;
}

/*
mmをロータリーエンコーダのパルスに変換する関数
引数：float型で距離(mm)
戻り値：long型でエンコーダ値
*/
long mm_to_pulse(long mm){
    float p;
    p = long((mm / (float)(TIRE_DIAM * PI)) * ((float)ENC_PPR * (float)GEAR_RATIO));
    return p;
}

/*
pwm値でモータに電圧を印加する関数
引数１：左の電圧   (-255~255)
引数２：右の電圧   (-255~255)
（範囲外の値は強制的に-255~255に変換されます）
*/
void pwm_write(int l, int r){
    //正負を判定してDIRピンを出力する
    if (l < 0){
        digitalWrite(L_MOT_DIR,1);
    }else{
        digitalWrite(L_MOT_DIR,0);
    }

    if (r < 0){
        digitalWrite(R_MOT_DIR,0);
    }else{
        digitalWrite(R_MOT_DIR,1);
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


float calc_true_dist(float t, float dist, float spd, float a){
  float target_t = dist / spd;
  float ts = (-1.0 * target_t / a) + target_t;
  float b = a * spd / (2.0 * ts);
  
  if(t < ts){
    return (b * t * t);
  }else if (t < (target_t - ts)){
    return (a * spd * t) + (spd * (target_t / 2.0) * (1.0 - a));
  }else if(t < target_t){
    return ((target_t * spd) - (b * (t - target_t) * (t - target_t)));
  }else{
    return dist;
  }
}

/*
直進する関数                    整数でも小数で引数を与えること！
引数１：float型で距離[mm]
引数２：float型で速度[mm/s]
*/
void straight(float dist, float spd, float alpha){
    stop_signal = 0;
    bool forward = 0;   
    float true_dist = 0.0;
    long target_ct = 0L;
    long true_ct = 0L;
    long relat_l_enc = 0L, relat_r_enc = 0L;
    long curr_l_enc = 0L, curr_r_enc = 0L;
    int r_err[3] = {0,};
    int l_err[3] = {0,};
    int lr_err[3] = {0,};
    //float gain[3][3] = {STRAIGHT_GAIN_L_SPD, STRAIGHT_GAIN_R_SPD, STRAIGHT_LR_ENC};
    float pid[3] = {0.0,}; // L R LR
    long t = 0L;
    curr_l_enc = l_enc;
    curr_r_enc = r_enc;


    gain_send(0);
    gain_send(1);
    gain_send(2);

    if((dist * spd) > 0.0){
        forward = 1;
    }else if((dist * spd) < 0.0){
        forward = 0;
    }else{
        return 0;
    }
    long c_millis = millis();
    if (forward){
        target_ct = mm_to_pulse(dist);
        
        while(1){
            relat_l_enc = l_enc - curr_l_enc;
            relat_r_enc = r_enc - curr_r_enc;

            //true_dist = float(((float)t * (float)dt[0] / 1000.0) * spd);
            true_dist = calc_true_dist(((float)t * (float)dt[0] / 1000.0),
                                       dist,
                                       spd,
                                       alpha);
            if(true_dist < dist){
                true_ct = mm_to_pulse(true_dist);
            }else{
                true_ct = mm_to_pulse(dist);
            }

            r_err[0] = int(true_ct - relat_r_enc);
            l_err[0] = int(true_ct - relat_l_enc);
            lr_err[0] = int(relat_l_enc - relat_r_enc);

            l_err[1] += l_err[0];
            r_err[1] += r_err[0];
            lr_err[1] += lr_err[0];
            
            pid[0] = ((float)all_gain[0][0] * (float)l_err[0]) + ((float)all_gain[0][1] * (float)l_err[1]) + ((float)all_gain[0][2] * ((float)l_err[0] - (float)l_err[2]));
            pid[1] = ((float)all_gain[1][0] * (float)r_err[0]) + ((float)all_gain[1][1] * (float)r_err[1]) + ((float)all_gain[1][2] * ((float)r_err[0] - (float)r_err[2]));
            pid[2] = ((float)all_gain[2][0] * (float)lr_err[0]) + ((float)all_gain[2][1] * (float)lr_err[1]) + ((float)all_gain[2][2] * ((float)lr_err[0] - (float)lr_err[2]));
            
            /*
            Serial.print("true_ct");    //この１行がないと何故かまともに動かないので消さない
            Serial.print(":");
            Serial.print(true_ct);
            Serial.print(",");
            Serial.print("relat_l_enc");
            Serial.print(":");
            Serial.print(relat_l_enc);
            Serial.print(",");
            Serial.print("relat_r_enc");
            Serial.print(":");
            Serial.println(relat_r_enc);*/

            serial_send(true_ct, 0);
            serial_send(relat_l_enc, 1);
            serial_send(relat_r_enc, 2);
            
            
            pwm_write(int(pid[0]),int(pid[1] + pid[2]));

            if(relat_l_enc >= target_ct){
                pwm_write(0,int(pid[1] + pid[2]));
            }
            if(relat_r_enc >= target_ct){
                pwm_write(int(pid[0]),0);
            }

            if ((relat_l_enc >= target_ct) && (relat_r_enc >= target_ct)){
                pwm_write(0,0);
                break;
            }

            r_err[2] = r_err[0];
            l_err[2] = l_err[0];
            lr_err[2] = lr_err[0];
            //delay(dt[0]);
            while(1){
                if(millis() >= c_millis + long(dt[0])){
                    c_millis = millis();
                    break;
                }
            }
            t += 1L;
            
            serial_receive(true);
            if(stop_signal){
                pwm_write(0,0);
                break;
            }
        }
    }else{
        target_ct = long(-1) * mm_to_pulse(abs(dist));
        
        while(1){
            relat_l_enc = l_enc - curr_l_enc;
            relat_r_enc = r_enc - curr_r_enc;

            //true_dist = float(-1) * float(((float)t * (float)dt[0] / 1000.0) * abs(spd));
            true_dist = calc_true_dist(((float)t * (float)dt[0] / 1000.0),
                                       abs(dist),
                                       abs(spd),
                                       alpha);
            true_dist *= -1.0;
            if(true_dist > dist){
                true_ct = mm_to_pulse(true_dist);
            }else{
                true_ct = long(-1) * mm_to_pulse(abs(dist));
            }

            r_err[0] = int(true_ct - relat_r_enc);
            l_err[0] = int(true_ct - relat_l_enc);
            lr_err[0] = int(relat_l_enc - relat_r_enc);

            l_err[1] += l_err[0];
            r_err[1] += r_err[0];
            lr_err[1] += lr_err[0];
            
            pid[0] = ((float)all_gain[0][0] * (float)l_err[0]) + ((float)all_gain[0][1] * (float)l_err[1]) + ((float)all_gain[0][2] * ((float)l_err[0] - (float)l_err[2]));
            pid[1] = ((float)all_gain[1][0] * (float)r_err[0]) + ((float)all_gain[1][1] * (float)r_err[1]) + ((float)all_gain[1][2] * ((float)r_err[0] - (float)r_err[2]));
            pid[2] = ((float)all_gain[2][0] * (float)lr_err[0]) + ((float)all_gain[2][1] * (float)lr_err[1]) + ((float)all_gain[2][2] * ((float)lr_err[0] - (float)lr_err[2]));

            /*
            Serial.print("true_ct");
            Serial.print(":");
            Serial.print(true_ct);
            Serial.print(",");
            Serial.print("relat_l_enc");
            Serial.print(":");
            Serial.print(relat_l_enc);
            Serial.print(",");
            Serial.print("relat_r_enc");
            Serial.print(":");
            Serial.println(relat_r_enc);*/

            serial_send(true_ct, 0);
            serial_send(relat_l_enc, 1);
            serial_send(relat_r_enc, 2);

            pwm_write(int(pid[0]),(int(pid[1]) + int(pid[2])));

            if(relat_l_enc <= target_ct){
                pwm_write(0,(int(pid[1]) + int(pid[2])));
            }
            if(relat_r_enc <= target_ct){
                pwm_write(int(pid[0]),0);
            }

            if ((relat_r_enc <= target_ct) && (relat_l_enc <= target_ct)){
                pwm_write(0,0);
                break;
            }

            r_err[2] = r_err[0];
            l_err[2] = l_err[0];
            lr_err[2] = lr_err[0];
            //delay(dt[0]);
            while(1){
                if(millis() >= c_millis + long(dt[0])){
                    c_millis = millis();
                    break;
                }
            }
            t += 1L;

            serial_receive(true);
            if(stop_signal){
                pwm_write(0,0);
                break;
            }
        }
    }
}

/*
回転する関数                    整数でも小数で引数を与えること！
引数１：float型で角度[deg]
引数２：float型で角速度[deg/s]
*/
void rotate(float angle, float spd, float alpha){
    stop_signal = 0;
    int rot_dir = 0;
    float true_dist_l = 0.0, true_dist_r = 0.0;
    long true_ct_l = 0L, true_ct_r = 0L;
    float target_dist_l = 0.0, target_dist_r = 0.0;
    long target_ct_l = 0L, target_ct_r = 0L;
    long relat_l_enc = 0L, relat_r_enc = 0L;
    long curr_l_enc = 0L, curr_r_enc = 0L;
    int r_err[3] = {0,};
    int l_err[3] = {0,};
    int lr_err[3] = {0,};
    float pid[3] = {0.0,}; // L R LR
    long t = 0L;
    curr_l_enc = l_enc;
    curr_r_enc = r_enc;

    

    gain_send(3);
    gain_send(4);
    gain_send(5);

    //回転方向の決定
    if(angle * spd < 0.0){
        rot_dir = -1;
    }else if (angle * spd > 0.0){
        rot_dir = 1;
    }else{
        return 0;
    }
    target_dist_l = float(rot_dir) * float(-1) * (abs(angle) / 360.0) * float(PI) * float(TIRE_PITCH);
    target_dist_r = float(rot_dir) * (abs(angle) / 360.0) * float(PI) * float(TIRE_PITCH);
    target_ct_l = mm_to_pulse(target_dist_l);
    target_ct_r = mm_to_pulse(target_dist_r);
    Serial.println(target_dist_l);
    Serial.println(target_ct_l);
    long c_millis = millis();
    while(1){
        relat_l_enc = l_enc - curr_l_enc;
        relat_r_enc = r_enc - curr_r_enc;
        //true_dist_l = float(rot_dir) * float(-1) * abs((((float(t) * float(dt[1]) / 1000.0) * spd) / 360.0) * float(PI) * float(TIRE_PITCH));
        true_dist_l = calc_true_dist(((float)t * (float)dt[0] / 1000.0),
                                       abs(target_dist_l),
                                       ((abs(spd) / 360.0) * float(PI) * float(TIRE_PITCH)),
                                        alpha);
        true_dist_l *= float(rot_dir) * float(-1);
        
        //true_dist_r = float(rot_dir) * abs((((float(t) * float(dt[1]) / 1000.0) * spd) / 360.0) * float(PI) * float(TIRE_PITCH));
        true_dist_r = calc_true_dist(((float)t * (float)dt[0] / 1000.0),
                                       abs(target_dist_r),
                                       ((abs(spd) / 360.0) * float(PI) * float(TIRE_PITCH)),
                                       alpha);
        true_dist_r *= float(rot_dir);

        if(abs(true_dist_l) > abs(target_dist_l)){
            true_ct_l = target_ct_l;
        }else{
            true_ct_l = mm_to_pulse(true_dist_l);
        }

        if(abs(true_dist_r) > abs(target_dist_r)){
            true_ct_r = target_ct_r;
        }else{
            true_ct_r = mm_to_pulse(true_dist_r);
        }

        l_err[0] = int(true_ct_l - relat_l_enc);
        r_err[0] = int(true_ct_r - relat_r_enc);
        lr_err[0] = int(abs(relat_l_enc) - abs(relat_r_enc));

        l_err[1] += l_err[0];
        r_err[1] += r_err[0];
        lr_err[1] += lr_err[0];

        pid[0] = ((float)all_gain[3][0] * (float)l_err[0]) + ((float)all_gain[3][1] * (float)l_err[1]) + ((float)all_gain[3][2] * ((float)l_err[0] - (float)l_err[2]));
        pid[1] = ((float)all_gain[4][0] * (float)r_err[0]) + ((float)all_gain[4][1] * (float)r_err[1]) + ((float)all_gain[4][2] * ((float)r_err[0] - (float)r_err[2]));
        pid[2] = ((float)all_gain[5][0] * (float)lr_err[0]) + ((float)all_gain[5][1] * (float)lr_err[1]) + ((float)all_gain[5][2] * ((float)lr_err[0] - (float)lr_err[2]));

        /*
        Serial.print("true_ct_l");
        Serial.print(":");
        Serial.print(true_ct_l);
        Serial.print(",");
        Serial.print("relat_l_enc");
        Serial.print(":");
        Serial.print(relat_l_enc);
        Serial.print(",");
        Serial.print("true_ct_r");
        Serial.print(":");
        Serial.print(true_ct_r);
        Serial.print(",");
        Serial.print("relat_r_enc");
        Serial.print(":");
        Serial.println(relat_r_enc);*/

        serial_send(true_ct_l, 0);
        serial_send(relat_l_enc, 1);
        serial_send(relat_r_enc, 2);
        
        pwm_write(int(pid[0]), (int(pid[1])  + int(float(rot_dir) * pid[2])));

        if(abs(relat_l_enc) >= abs(target_ct_l)){
            pwm_write(0, (int(pid[1])  + int(float(rot_dir) * pid[2])));
        }
        if(abs(relat_r_enc) >= abs(target_ct_r)){
            pwm_write(pid[0], 0);
        }
        
        if((abs(relat_l_enc) >= abs(target_ct_l)) && (abs(relat_r_enc) >= abs(target_ct_r))){
            pwm_write(0,0);
            break;
        }

        r_err[2] = r_err[0];
        l_err[2] = l_err[0];
        lr_err[2] = lr_err[0];
        //delay(dt[1]);
        while(1){
                if(millis() >= c_millis + long(dt[1])){
                    c_millis = millis();
                    break;
                }
            }
        t += 1L;

        serial_receive(true);
        if(stop_signal){
            pwm_write(0,0);
            break;
        }
    }
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
