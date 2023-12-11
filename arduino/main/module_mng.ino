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

/*
//鍵開閉
void servo_open(){
  servo.write(0);
  //analogWrite(SERVO, 0);
  delay(2000);
  servo.write(80);
  //analogWrite(SERVO, 80);
}

*/

/*
モジュールの鍵用サーボの開閉を行う
解錠→3秒待機→施錠

引数：なし

戻り値：なし
*/
void key(){
  for(int i=0; i<10; i++){
    digitalWrite(SERVO,HIGH);
    delay(2);
    digitalWrite(SERVO,LOW);
    delay(2);
  }
  delay(1000);
  for(int i=0; i<10; i++){
    digitalWrite(SERVO,HIGH);
    delay(1);
    digitalWrite(SERVO,LOW);
    delay(1);
  }
}