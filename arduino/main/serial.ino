
/*
この関数を実行すると、一生シリアル通信を監視して、受信したデータ（バイト）に応じた動作をするモードになる
シリアル通信のアルゴリズムを変えるときは、この関数をいじる
引数：moving(動いているかどうかのフラグ、bool)
*/
int serial_receive(bool moving){
    int inc_bytes[10] = {-1,};
    int idx = 0;
    int tmp = 0;

    while(Serial.available() > 0){
        tmp = int(Serial.read());
        if(tmp == 255){
            while(1){
              tmp = int(Serial.read());
              if(tmp == 254){
                break;
              }else if(tmp != -1){
                inc_bytes[idx] = tmp;
                idx += 1;
              }
              
            }
            while(Serial.available() > 0){
              Serial.read();
            }

          if(inc_bytes[0] == 1){
              if(moving){return(-1);}
              long dist_1, spd_1;
              dist_1 = long(inc_bytes[2]) * 253L + long(inc_bytes[3]);
              spd_1 = long(inc_bytes[4]) * 253L + long(inc_bytes[5]);
              if(inc_bytes[1] == 0){
                dist_1 = -dist_1;
              }
              straight(float(dist_1),float(spd_1),alpha_var);
              return(0);
          }else if(inc_bytes[0] == 2){
              if(moving){return(-1);}
              long dist_2, spd_2;
              dist_2 = long(inc_bytes[2]) * 253L + long(inc_bytes[3]);
              spd_2 = long(inc_bytes[4]) * 253L + long(inc_bytes[5]);
              if(inc_bytes[1] == 0){
                dist_2 = -dist_2;
              }
              rotate(float(dist_2),float(spd_2),alpha_var);
              return(0);
          }else if(inc_bytes[0] == 3){
              //if(moving){return(-1);}
              long spd_3;
              spd_3 = long(inc_bytes[2]) * 253L + long(inc_bytes[3]);
              if(inc_bytes[1] == 0){
                straight(-1000000.0,float(spd_3),1.0);
              }else{
                straight(1000000.0,float(spd_3),1.0);
              }
              return(0);
          }else if(inc_bytes[0] == 4){
              //if(moving){return(-1);}
              long spd_4;
              spd_4 = long(inc_bytes[2]) * 253L + long(inc_bytes[3]);
              if(inc_bytes[1] == 0){
                rotate(-1000000.0,float(spd_4),1.0);
              }else{
                rotate(1000000.0,float(spd_4),1.0);
              }
              return(0);
          }else if(inc_bytes[0] == 5){
              pwm_write(0,0);
              stop_signal = true;
              return(0);
          }else if(inc_bytes[0] == 6){
              long new_gain_int;
              float new_gain;
              int r,c;
              new_gain_int = long(inc_bytes[3]) * 253L + long(inc_bytes[4]);
              new_gain = float(new_gain_int) / 10000.0;
              r = inc_bytes[1];
              c = inc_bytes[2];
              all_gain[r][c] = new_gain;
              return(0);
          }else if(inc_bytes[0] == 7){
              int idx = inc_bytes[1];
              int new_value = inc_bytes[2];
              if(idx == 2){
                alpha_var = float(new_value / 100.0);
              }else{
                dt[idx] = new_value;
              }
              return(0);
          }else if(inc_bytes[0] == 8){
              if(moving){return(-1);}
              get_batt();
              return(0);
          }else if(inc_bytes[0] == 9){
              if(moving){return(-1);}
              get_module_mng();
              return(0);
          }/*else if(inc_bytes[0] == 10){
              if(moving){return(-1);}
              if(inc_bytes[1] == 0){
                servo_open();
              }else if(inc_bytes[1] == 1){
                servo_close();
              }
              return(0);
          }*/else{
            return(-1);
          }
        }
    }
}


/*指定されたゲインをラズパイに送信(100倍されたゲインを送信)
gain_num
0:STRAIGHT_GAIN_L_SPD
1:STRAIGHT_GAIN_R_SPD
2:STRAIGHT_GAIN_LR_ENC
3:ROTATE_GAIN_L
4:ROTATE_GAIN_R
5:ROTATE_GAIN_LR
*/
void gain_send(int gain_num){
    int send_gain[3] = {0,};
    int send_value[6] = {0,};
    
    for(int i=0; i<3; i++){
      send_gain[i] = (int)(all_gain[gain_num][i]*10000);
      send_value[2*i] = send_gain[i]/254;
      send_value[2*i+1] = send_gain[i]%254;
    }

    for(int i=0; i<6; i++){
      send_value[i] = (int)(send_gain[i]/254%254L);
    }
    
    Serial.write((byte)255);
    Serial.write((byte)3);
    Serial.write((byte)gain_num);
    for(int i=0; i<6; i++){
      Serial.write((byte)send_value[i]);
    }
    Serial.write((byte)254);

    return 0;
}

void serial_send(long enc, int mode){
  bool sign = 0;
  int send_value[4] = {0, 0, 0, 0};

  for(int i=0; i<4; i++){
    send_value[i] = (int)(abs(enc)/(long)pow(254, i)%254L);
  }
  if(enc<0L){
    sign = 1;
    }

  Serial.write((byte)255);
  Serial.write((byte)mode);
  Serial.write((byte)sign);
  for(int i=0; i<4; i++){
    Serial.write((byte)send_value[i]);
  }
  Serial.write((byte)254);
  
  return 0;
}

//バッテリ電圧送信
void batt_send(long batt){

  Serial.write((byte)255);
  Serial.write((byte)batt);
  Serial.write((byte)254);

  return 0;
}

//モジュール識別抵抗値送信
void module_send(double module_r){
  Serial.write((byte)255);
  Serial.write((byte)module_r);
  Serial.write((byte)254);
}
