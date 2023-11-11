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

