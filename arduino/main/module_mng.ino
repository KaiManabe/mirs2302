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

void module_temp(){
  double target_temp_h = 50;
  double target_temp_c = 10;

  float volt_h;
  volt_h = analogRead(THERMISTOR1);
  float volt_c;
  volt_c = analogRead(THERMISTOR2);
  Serial.println("moto");
  Serial.print(volt_h);
  Serial.println(volt_c);

  volt_h = volt_h*5.0/1023.0;
  volt_c = volt_c*5.0/1024.0;
  float r_h = (volt_h+5.0)*1000.0/volt_h;
  float r_c = (volt_c+5.0)*1000.0/volt_c;
  float temp_h = 1.0/(log(r_h/10000.0)/3435.0+1.0/298.0)-273.0;
  float temp_c = 1.0/(log(r_c/10000.0)/3435.0+1.0/293.0)-273.0;

  Serial.print((int)temp_h);
  Serial.println((int)temp_c);
  
  //保温用モジュール
  
  if(temp_h <= target_temp_h){
    digitalWrite(PELTIER,HIGH);
  }else{
    digitalWrite(PELTIER,LOW);
  }

  //保冷用モジュール
  if(temp_c >= target_temp_c){
    digitalWrite(PELTIER,HIGH);
  }else{
    digitalWrite(PELTIER,LOW);
  }
}


/*
ペルチェ素子の制御（ペルチェ単体試験のみ）

引数：
      ペルチェon：1
      ペルチェoff:0

戻り値：なし
*/
void peltier(int p){
  if(p == 1){
    digitalWrite(PELTIER,HIGH);
  }else if(p == 0){
    digitalWrite(PELTIER,LOW);
  }
}


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
