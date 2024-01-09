function show_modal(modal_id){
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
}