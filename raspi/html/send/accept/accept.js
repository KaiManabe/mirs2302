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
    場所: str

***クライアントが集荷場所を選択した時に実行するやつ***
*/
function selectableTime(place) {
    // httpリクエストを送信して選択可能な時間を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("GET", url + "?place=" + encodeURIComponent(JSON.stringify(place)), false); // 同期通信GETメソッド
    xhr.send();
    var timeList = xhr.responseText.split(",").filter(Boolean); // httpレスポンスを配列にして受け取る

    // 集荷時間の要素を取得
    var selectElement = document.getElementById("picking_time");

    // 既存の選択肢の配列を作成
    var existingOptions = timelist_available;

    // 既存の選択肢にない選択可能な集荷時間を追加
    timeList.forEach(function(time) {
        if (!existingOptions.includes(time)) {
            //var option = document.createElement("option");
            option.value = time;
            option.text = time;
            //selectElement.appendChild(option);
        }
    });
    // 既存の選択肢にある選択可能な集荷時間ではないものを削除
    existingOptions.forEach(function(optionValue) {
        if (!timeList.includes(optionValue) && optionValue != 'init') {
            selectElement.querySelectorAll('option[value="' + optionValue + '"]').forEach(option => option.remove());
        }
    });

    // 集荷時間の選択肢をソート
    Array.from(selectElement.options)
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
    .forEach(option => selectElement.appendChild(option));
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


/*
formデータ読み込む関数

引数：
    formのID
戻り値：
    formデータの配列
*/
function readFormData(formIds) {
    var formData = {};

    // 各IDの入力フォームの値をformDataに格納
    formIds.forEach(function (id) {
        formData[id] = document.getElementById(id).value;
    });

    let current_url = new URL(window.location.href);
    let params = current_url.searchParams;
    var order_id = params.get('id'); //order_idを取得

    formData["id"] = order_id;
    return formData;
}

/*
送信ボタンに関する処理
*/
function submitProcessing(){
    // formデータの読み込み
    var formIds = ['accept', 'receive_pincode','receive_time', 'receive_place']; // formのID
    var sendData = readFormData(formIds);

    // formデータを送信
    var result = sendDataToPhp(sendData);
    return result
    // 送信結果の画面を表示
    var sendingElement = document.getElementById("sending");
    // 正常終了
    if(result == 0){
        sendingElement.innerHTML = "<div id='success'>正常に取引を承りました</div>";
    }
    // 異常終了
    else{
        sendingElement.innerHTML = "<div id='error'>ERROR : 取引を承れませんでした</div>";
    }
}

/*
phpに送信する関数

引数：
    phpに送りたいデータ(json形式で送信する)
*/
function sendDataToPhp(sendData) {
    // XMLHttpRequestオブジェクトを使用してPHPにHTTPリクエストを送信
    var xhr = new XMLHttpRequest();
    var url = "picking.php";
    
    xhr.open("POST", url, false); // 第一引数：メソッド　第二引数：接続先のurl 第三引数：同期(false)か非同期(true)かを指定
    xhr.setRequestHeader("Content-Type", "application/json;charset=UTF-8"); // ヘッダを設定(文字列も送れるjson形式を指定)
    xhr.send(JSON.stringify(sendData)); // データを送信

    return xhr.responseText;
}

//イベントリスナー
const formElement = document.getElementById("form");
const pickingTimeElement = document.getElementById("receive_time");

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
formElement.addEventListener('submit', function(event) {
    event.preventDefault(); // ページのリロードを防ぐ
    submitProcessing();
});