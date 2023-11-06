void gett_module_mng(){
    int module_num[3] = {MODULE1,MODULE2,MODULE3};
    double module_mng[3];

    for(int i=0; i<3; i++){
        module_r = MODULE_R[i];
        module_mng[i] = read_module(module_num[i] , module_r);
    }
    
    for(int i=0; i<3; i++){
        module_send(module_mng[i]);
    }
}

void read_module(int module_num , int module_r){
    double ans;
    ans = analogRead(module_num) * 5.0 / 1024.0 / V_RATIO;

    return ans*module_r/(5-ans);
}