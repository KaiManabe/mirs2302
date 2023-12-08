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

//鍵開閉
void key(){
  for(int i=0; i<10; i++){
    digitalWrite(SERVO,HIGH);
    delayMicroseconds(2400);//マイクロ秒
    digitalWrite(SERVO,LOW);
    delay(20);
  }
  delay(3000);
  for(int i=0; i<10; i++){
    digitalWrite(SERVO,HIGH);
    delayMicroseconds(1300);//マイクロ秒
    digitalWrite(SERVO,LOW);
    delay(20);
  }
}