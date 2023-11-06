double io_get_batt(){
  return analogRead(BATT) * 5.0 / 1024.0 / V_RATIO;
}

void get_batt(){
  double batt;
  

  batt = io_get_batt();
  batt *= 10;
  batt_send(batt);

}
