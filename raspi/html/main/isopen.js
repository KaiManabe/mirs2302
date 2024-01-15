function isopen(){
    //受付するときにここをtrueにする
    //falseにしないと注文が入ってしまう
    return false;
}


if (!isopen()){
    window.location.href = "./outofservice.html";
}