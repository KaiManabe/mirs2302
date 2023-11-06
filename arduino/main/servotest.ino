#include <Servo.h>
Servo servo;

void setup() {
  pinMode(3, OUTPUT);
  servo.attach(3, 500, 2400);
  pinMode(4, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void loop() {
  /*サーボを150(deg)の位置まで回転させる
    servo.write(150);
    while(1){}
  */


  /*サーボを70(deg)の位置まで回転させる
    servo.write(70);
    while(1){}
  */

  /*4番ピンの入力をそのまま13に出す
    while(1){
      digitalWrite(13, !digitalRead(4));
    }
  */


}
