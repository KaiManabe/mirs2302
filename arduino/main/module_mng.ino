void get_module_mng(){
    int module_num[3] = {MODULE1,MODULE2,MODULE3};
    double module_mng[3];

    for(int i=0; i<3; i++){
        module_mng[i] = read_module(module_num[i]);
    }
    
    for(int i=0; i<3; i++){
        module_send(module_mng[i]);
    }
}

double read_module(int module_num){
    double ans;
    ans = analogRead(module_num) * 5.0 / 1024.0 / V_RATIO;

    return ans*MODULE_R/(5-ans);
}
