function read_status(){
    var xhr = new XMLHttpRequest();
    var url = "robot_state_reader.php"; // httpリクエスト先
    xhr.open("GET", url, false);
    xhr.setRequestHeader('Content-Type', 'application/json');
    xhr.send();
    var resp = JSON.parse(xhr.responseText);
    return resp;
}



function confirm(){
    //待機モーダル表示
    open_modal("auth_prog");

    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function(){
        if (xhr.readyState == 4){
            if(xhr.status == 200){
                let resp = xhr.responseText;
                if(resp != "True"){
                    open_modal("error");
                }
            }else{
                open_modal("error");
            }
        }
    }

    xhr.open("GET", "./confirm.php?PASS=" + window.pass, true);
    xhr.send()
}




function auth_button_pressed(){
    //待機モーダル表示
    open_modal("auth_prog");

    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function(){
        if (xhr.readyState == 4){
            if(xhr.status == 200){
                let resp = xhr.responseText;
                if(resp == "True"){
                    open_modal("auth_success");
                }else{
                    open_modal("auth_failed");
                }
            }else{
                open_modal("error");
            }
        }
    }

    xhr.open("GET", "./auth.php?PASS=" + window.pass, true);
    xhr.send()
}

function door_closed(){
    if(read_status()["DOOR"] == "CLOSE") {
        close_modal();
        confirm_button_elements.forEach(function(ele){
            ele.style.visibility = "visible";
        })
    }else{
        open_modal("door_failed");
    }
}


function onload(){
    confirm_button_elements.forEach(function(ele){
        ele.style.visibility = "hidden";
    })
    door_number_element.innerHTML = read_status()["DOOR_NUM"];
}

function view_open_button(){
    open_button_elements.forEach(function(ele){
        ele.style.removeProperty("visibility");
    })
}

function open_modal(modal_name){
    /*
    モーダルを表示する関数
    引数に表示したいモーダルのIDを入れる(#はいらない)
    */
    let modal_ele = document.querySelector(".modal");
    modal_ele.classList.remove("hidden");

    let modal_body_ele = document.querySelectorAll(".modal_body");
    modal_body_ele.forEach(function(ele){
        ele.classList.add("hidden");
    })

    document.querySelector("#" + modal_name).classList.remove("hidden");
}

function close_modal(){
    /*
    モーダルを閉じる関数
    */
    let modal_ele = document.querySelector(".modal");
    modal_ele.classList.add("hidden");

    let modal_body_ele = document.querySelectorAll(".modal_body");
    modal_body_ele.forEach(function(ele){
        ele.classList.add("hidden");
    })

}


const confirm_button_elements = document.querySelectorAll(".confirm_button");
const door_number_element = document.querySelector("#door_number");
onload();



function wait(ms) {
    return new Promise(resolve => {
      setTimeout(resolve, ms);
    });
  }
  