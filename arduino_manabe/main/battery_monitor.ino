/*
バッテリの電圧を読んで返す関数

引数：
    なし

戻り値：
    バッテリ電圧 -> double
*/
double io_get_batt(){
  float batt;
  batt = analogRead(BATT) * 5.0 / 1024.0;
  return batt / V_RATIO;
}