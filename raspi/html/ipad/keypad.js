function keypad_pressed(ele){
    var num = ele.getAttribute("id");
    window.pass += num;
    disp();
}

function disp(){
    disp_ele.innerHTML = "";
    for (let i = 0; i < window.pass.length; i++){
        disp_ele.innerHTML += "＊";
    }
}

function keypad_clear(){
    window.pass = "";
    disp();
}


const disp_ele = document.querySelector("#number_display");
window.pass = "";