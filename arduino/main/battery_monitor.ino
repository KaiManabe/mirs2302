double io_get_batt(){
  return analogRead(BATT) * 5.0 / 1024.0 / V_RATIO;
}

void get_batt() {
  double batt;
  char str[100], str_batt[10];

  batt = io_get_batt();

/*
//バッテリ値　元の値：batt, 文字表現：str_batt
  while (1) {
    batt = io_get_batt();
    sprintf(str, "volt = %s\n", dtostrf(batt, 4, 2, str_batt));
    Serial.print(str);
    delay(10);
  }
 */
}
