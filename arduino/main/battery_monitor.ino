double io_get_batt(){
  float batt;
  batt = analogRead(BATT) * 5.0 / 1024.0;
  return batt / V_RATIO * 10.0;
}

void get_batt(){
  int batt;
  
  batt = io_get_batt();
  batt_send(batt);
  //Serial.println(batt);
  delay(10);
}
