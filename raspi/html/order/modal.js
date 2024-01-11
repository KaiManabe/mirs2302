var timer_id;
var body_ele;
var sorry;

sorry = document.createElement("p");
sorry.innerHTML = "処理に時間がかかっています。<br>1分程度かかる場合があります。"
sorry.setAttribute("style", "font-size: 0.75em;");

function dispmsg(){
    body_ele.appendChild(sorry);
}


function show_modal(modal_id){
    window.clearTimeout(timer_id);
    timer_id = window.setTimeout(dispmsg, 5000);

    back_ele = document.querySelector("#modal");
    back_ele.classList.remove("hidden");
    if(modal_id[0] == "#"){
        id = modal_id;
    }else{
        id = "#" + modal_id;
    }
    body_ele = document.querySelector(id);
    body_ele.classList.remove("hidden");
}


function hide_modal(modal_id){
    back_ele = document.querySelector("#modal");
    back_ele.classList.add("hidden");

    if(modal_id[0] == "#"){
        id = modal_id;
    }else{
        id = "#" + modal_id;
    }
    body_ele = document.querySelector(id);
    body_ele.classList.add("hidden");
    body_ele.removeChild(sorry);
}