/*
ページを読み込んだタイミングで依頼者と依頼物を書き換える
*/
function onLoad(){
    let current_url = new URL(window.location.href);
    let params = current_url.searchParams;
    var order_id = params.get('id'); //order_idを取得

    let request = new XMLHttpRequest();
    var url = "get_order_data.php"; //リクエスト先のphpのurl
    request.open("GET", url + "?id=" + encodeURIComponent(JSON.stringify(order_id)), false);
    request.send();

    var order_info = JSON.parse(request.responseText);

    var sender_ele = document.querySelectorAll(".sender");
    var item_name_ele = document.querySelectorAll(".item_name");
    sender_ele.forEach(function(ele){
        ele.innerHTML = order_info.SENDER;
    })
    item_name_ele.forEach(function(ele){
        ele.innerHTML = order_info.ITEM_NAME;
    })
    if ("NOTE" in order_info){
        var note_ele = document.querySelectorAll(".note");
        note_ele.forEach(function(ele){
            ele.innerHTML = order_info.NOTE;
        })
    }


    var form_ele = document.querySelector("#form_ele");
    var id_ele = document.createElement("input");
    id_ele.setAttribute("checked", "checked");
    id_ele.setAttribute("style", "display:none;");
    id_ele.setAttribute("name", "id");
    id_ele.setAttribute("value", order_id);
    
    form_ele.appendChild(id_ele);

    selectableTime();

}


/*
選択可能な時間を制限する関数

引数(なしでもいける)：
    ITEM_TYPE: str

***クライアントがITEM_TYPEを選択した時に実行するやつ***
*/
function selectableTime() {
    let current_url = new URL(window.location.href);
    let params = current_url.searchParams;
    var order_id = params.get('id');
    // httpリクエストを送信して選択可能な時間を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("GET", url + "?id=" + encodeURIComponent(order_id), false); // 同期通信GETメソッド
    xhr.send();

    var timeList = xhr.responseText.split(",").filter(Boolean); // httpレスポンスを配列にして受け取る

    // 既存の選択肢の配列を作成
    var existingOptions = Array.from(pickingTimeElement.options).map(option => option.value);

    // 既存の選択肢にない選択可能な集荷時間を追加
    timeList.forEach(function(time) {
        if (!existingOptions.includes(time)) {
            var option = document.createElement("option");
            option.value = time;
            option.text = time;
            pickingTimeElement.appendChild(option);
        }
    });
    // 既存の選択肢にある選択可能な集荷時間ではないものを削除
    existingOptions.forEach(function(optionValue) {
        if (!timeList.includes(optionValue) && optionValue != 'init') {
            pickingTimeElement.querySelectorAll('option[value="' + optionValue + '"]').forEach(option => option.remove());
        }
    });
    // 集荷時間の選択肢をソート
    Array.from(pickingTimeElement.options)
    .filter(option => option.value !== 'init')
    .sort((a, b) => {
        if (a.value < b.value) {
            return -1;
        }
        if (a.value > b.value) {
            return 1;
        }
        return 0;
    })
    .forEach(option => pickingTimeElement.appendChild(option));
}



/*
チェックボックスの処理
*/

function chbox(obj){
    var form_label = document.querySelector("#input_form");
    var forms = document.querySelectorAll("#input_form select, #input_form input");
    var label = document.querySelector("#checkbox_label");
    if(obj.checked){
        forms.forEach(function(ele){
            ele.removeAttribute("disabled");
        })
        form_label.classList.remove("unchosen");
        form_label.classList.add("chosen");
        label.classList.add("checked_label");
        label.classList.remove("unchecked_label");
        obj.value = "accept";
    }else{
        forms.forEach(function(ele){
            ele.setAttribute("disabled","disabled");
        })
        form_label.classList.add("unchosen");
        form_label.classList.remove("chosen");
        label.classList.remove("checked_label");
        label.classList.add("unchecked_label");
        obj.value = "denied";
    }
}




const pickingTimeElement = document.getElementById("pickup_time");
const submit_button = document.getElementById("submit_button");

// ページが読み込まれた時に実行する処理
document.addEventListener("DOMContentLoaded", function(event) {
    pickingTimeElement.innerHTML = "<option value='init' selected disabled>選択してください</option>";
    onLoad();
});

// 集荷時間が選択された時の処理
pickingTimeElement.addEventListener('change', function(event) {
    selectableTime();
});



// データを送信するボタンが押された時の処理
submit_button.addEventListener("click", function(){
    let form_ele = document.querySelector("#form_ele");
    let select_ele = document.querySelectorAll('#form_ele select');
    let input_ele = document.querySelectorAll('#form_ele input');
    let isempty = false;

    for (let i = 0; i < select_ele.length; i++){
        if(select_ele[i].value == "init"){
            isempty = true;
        }
    }

    
    for (let i = 0; i < input_ele.length; i++){
        if(input_ele[i].value == ""){
            isempty = true;
        }
    }

    if(document.querySelector("#accept").value == "denied"){
        isempty = false;
    }

    if(isempty){
        window.alert("すべての必須項目を入力してください。");
    }else{
        form_ele.submit();
    }
});
