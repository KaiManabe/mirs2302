/*
シリアルの受信をチェックし、受信内容に応じた処理を実行する関数

引数・戻り値なし
*/
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
            send_module();
            break;
        
        case 4:
            send_gain_param();
            break;
        
        case 5:
            send_batt();
            break;
        
        case 6:
            set_pid_ser_mode(inc_bytes[1]);
            break;
        
        case 7:
            break;
        
        case 8:
            change_gain(inc_bytes[1], inc_bytes[2], inc_bytes[3], inc_bytes[4]);
            break;
        
        case 9:
            change_param(inc_bytes[1], inc_bytes[2], inc_bytes[3]);
            break;
        
        case 10:
            //servo_open();
            break;

        default:
            break;
    }


}

/*
シリアル通信でエンコーダ値を送信するモードを設定する

引数:
    int mode : 0なら何も送信しない  1ならspd値を送信 

戻り値:
    なし
*/
void set_pid_ser_mode(int val){
    pid_serial_mode = val;
}




/*
左タイヤの速度をセットする関数

引数:
    int dir : 方向(0で直進, 1で後退)
    int hb : 速度値[mm/s]のハイビット
    int lb : 速度値[mm/s]のロービット

戻り値:
    なし
*/
void set_l_spd(int dir, int hb, int lb){
    if(l_spd_target == 0L){
        pid_init_l();
    }
    long value = (long)((long)hb * (long)254 + (long)lb);
    if(dir == 1){
        value = value * (long)-1;
    }
    l_spd_target = value;
}


/*
右タイヤの速度をセットする関数

引数:
    int dir : 方向(0で直進, 1で後退)
    int hb : 速度値[mm/s]のハイビット
    int lb : 速度値[mm/s]のロービット

戻り値:
    なし
*/
void set_r_spd(int dir, int hb, int lb){
    if(r_spd_target == 0L){
        pid_init_r();
    }
    long value = (long)((long)hb * (long)254 + (long)lb);
    if(dir == 1){
        value = value * (long)-1;
    }
    r_spd_target = value;
}



/*
ゲインを変更する関数

引数:
    int lr : ゲインの種類 (L/R/LR)
    int pid : ゲインの種類 (P/I/D)
    int hb : 新しい値*10000のハイビット
    int lb : 新しい値*10000のロービット

戻り値:
    なし
*/
void change_gain(int lr, int pid, int hb, int lb){
    float new_value = ((float)hb * (float)254.0) + (float)lb;
    new_value /= (float)GAIN_ACCURACY;
    all_gain[lr][pid] = new_value;

    #ifdef ENABLE_EEP
        gain_eep_write();
    #endif
}




/*
パラメータを変更する関数

引数:
    int param : パラメータの種類 (dt)
    int hb : 新しい値のハイビット／もしくは新しい値をそのままいれる
    int lb : 新しい値のロービット／もしくは任意の値をいれる（無視される）

戻り値:
    なし
*/
void change_param(int param, int hb, int lb){
    if(param == 0){
        dt = hb;
        #ifdef ENABLE_EEP
            gain_eep_write();
        #endif
    }
}



/*
現在のパラメータを送信する関数

引数:
    なし

戻り値:
    なし
*/
void send_gain_param(){
    Serial.write(255);
    Serial.write(13);

    for(int i = 0; i < 3; i++){
        for(int ii = 0; ii < 3; ii++){
            int hb = (int)(all_gain[i][ii] * (float)GAIN_ACCURACY / 254.0);
            int lb = (int)(all_gain[i][ii] * (float)GAIN_ACCURACY) - (hb * 254);
            Serial.write(hb);
            Serial.write(lb);
        }
    }

    Serial.write(dt);

    Serial.write(254);
}


/*
現在のパラメータを送信する関数

引数:
    バッテリのシリアル値

戻り値:
    なし
*/
void batt_send(int batt){

  Serial.write((byte)255);
  Serial.write((byte)batt);
  Serial.write((byte)254);

  return 0;
}




/*
モジュールの抵抗値を送信する

引数：
    なし

戻り値:
    なし
*/
void send_module(){
    int module_num[3] = {MODULE1,MODULE2,MODULE3};  //ピン番号

    Serial.write((byte)255);
    Serial.write((byte)12);

    for(int i = 0; i < 3; i++){
        double r = read_module(module_num[i]);
        int hb = (int)(r / 254.0);
        int lb = (int)(r - (double)(hb * 254.0));
        Serial.write(hb);
        Serial.write(lb);
    }

    Serial.write(254);
}




/*
バッテリ電圧を送信する

引数：
    なし

戻り値:
    なし
*/
void send_batt(){
    Serial.write((byte)255);
    Serial.write((byte)11);

    int value = (int)(io_get_batt() * 10.0);
    Serial.write(value);
    
    Serial.write(254);
}
