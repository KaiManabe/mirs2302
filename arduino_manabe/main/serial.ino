void check_serial(){
    int inc_bytes[64] = {-1,};

    /*ここから受信処理*/
    if(Serial.available() > 0){
        if(Serial.read() == 255){
            int sread = 0;
            int idx = 0;
            while(1){
                while(Serial.available() < 1){}
                sread = (int)Serial.read();
                if(sread == 254){
                    break;
                }else if(sread != -1){
                    inc_bytes[idx] = sread;
                    idx += 1;
                }
            }
        }
    }
    /*ここまで*/


    /*ここから受信データに応じた処理*/
    if(inc_bytes[0] == -1){
        //なんもなければ終了
        return;
    }
    switch(inc_bytes[0]){
        case 1:
            set_l_spd(inc_bytes[1], inc_bytes[2], inc_bytes[3]);
            break;
        
        case 2:
            set_r_spd(inc_bytes[1], inc_bytes[2], inc_bytes[3]);
            break;
        
        case 3:
            break;
        
        case 4:
            send_gain_param();
            break;
        
        case 5:
            break;
        
        case 6:
            break;
        
        case 7:
            break;
        
        case 8:
            change_gain(inc_bytes[1], inc_bytes[2], inc_bytes[3], inc_bytes[4]);
            break;
        
        case 9:
            change_param(inc_bytes[1], inc_bytes[2], inc_bytes[3]);
            break;
        
        default:
            break;
    }


}

void set_l_spd(int dir, int hb, int lb){
    if(l_spd_target == 0L){
        pid_init_l();
    }
    long value = (long)(hb * 254 + lb);
    if(dir == 1){
        value = value * (long)-1;
    }
    l_spd_target = value;
}


void set_r_spd(int dir, int hb, int lb){
    if(r_spd_target == 0L){
        pid_init_r();
    }
    long value = (long)(hb * 254 + lb);
    if(dir == 1){
        value = value * (long)-1;
    }
    r_spd_target = value;
}


void change_gain(int lr, int pid, int hb, int lb){
    float new_value = ((float)hb * (float)254.0) + (float)lb;
    new_value /= (float)10000.0;
    all_gain[lr][pid] = new_value;
    //gain_eep_write();
}

void change_param(int param, int hb, int lb){
    if(param == 0){
        dt = hb;
    }
}


void send_gain_param(){
    Serial.write(255);
    Serial.write(13);

    for(int i = 0; i < 3; i++){
        for(int ii = 0; ii < 3; ii++){
            int hb = (int)(all_gain[i][ii] * 10000.0 / 254.0);
            int lb = (int)(all_gain[i][ii] * 10000.0) - (hb * 254);
            Serial.write(hb);
            Serial.write(lb);
        }
    }

    Serial.write(dt);

    Serial.write(254);
}