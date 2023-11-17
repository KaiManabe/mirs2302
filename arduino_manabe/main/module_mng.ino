/*
モジュールの抵抗値を返す

引数：
    int module_num : 電圧を読むアナログピン番号

戻り値:
    抵抗値 -> double
*/
double read_module(int module_num){
    double v;   //アナログ入力の電圧
    v = (double)analogRead(module_num) * 5.0 / 1024.0;
    
    return ((double)MODULE_R * v) /  (5.0 - v);
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
