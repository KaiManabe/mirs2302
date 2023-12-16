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
  for(int i=0; i<100; i++){
    digitalWrite(SERVO,HIGH);
    delay(3);
    digitalWrite(SERVO,LOW);
    delay(3);
  }
  delay(1000);
  for(int i=0; i<250; i++){
    digitalWrite(SERVO,HIGH);
    delay(1);
    digitalWrite(SERVO,LOW);
    delay(2);
  }
}

/*
サーミスタの読み取りとペルチェ素子制御

引数：なし

戻り値：なし
*/
/*
void module_temp(){
  double target_temp1 = ;
  double target_temp2 = ;
  float temp1 = analogRead(THERMISTOR1);
  float temp2 = analogRead(THERMISTOR2);
  //保冷用モジュール
  if(temp1 <= target_temp1){
    digitalWrite(PELTIER,HIGH);
  }else{
    digitalWrite(PELTIER,LOW);
  }

  //保温用モジュール
  if(temp2 >= target_temp2){
    digitalWrite(PELTIER,HIGH);
  }else{
    digitalWrite(PELTIER,LOW);
  }
}
*/

/*
ペルチェ素子の制御（ペルチェ単体試験のみ）

引数：
      ペルチェon：1
      ペルチェoff:0

戻り値：なし

void peltier(int p){
  if(p == 1){
    digitalWrite(PELTIER,HIGH);
  }else if(p == 0){
    digitalWrite(PELTIER,LOW);
  }  
}
*/

/*
フォトリフレクタの監視
この関数を定期的に繰り返す。10回以上車体が浮いたら異常判定

引数：なし

戻り値：
      持ち去られてるor転倒してる：1
      問題なし：0
*/
int photo(){
  int p;
  int state = 0;
  
  p = digitalRead(PHOTO);
  if(p == 0){
    photo_curr = 0;
  }else{
    photo_curr ++;
  }

  if(photo_curr >= 10){
    state = photo_flg_err;
  }else{
    state = photo_flg_ok;
  }
  
  return(state);
}
