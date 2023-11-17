void get_module_r(){
    int module_num[3] = {MODULE1,MODULE2,MODULE3};
    double module[3];

    for(int i=0; i<3; i++){
        module[i] = read_module(module_num[i]);
    }
    
    for(int i=0; i<3; i++){
        module_send(module[i]);
    }
}

double read_module(int module_num){
    double ans;
    ans = analogRead(module_num) * 5.0 / 1024.0 / V_RATIO;

    return MODULE_R*(5.0 / ans - 1);
}


//鍵開閉
void servo_open(){
  servo.write(150);
  //analogWrite(SERVO, 70);
  delay(10);
}

void servo_close(){
  servo.write(70);
  //analogWrite(SERVO, 150);
  delay(10);
}
