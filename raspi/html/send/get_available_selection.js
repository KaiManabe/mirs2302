/*
全体を実行するやつ
*/
function execution(){
    var formIds = ['client_address', 'client_address_type', 'target_address', 'target_address_type', 'item_type', 'item_name', 'picking_place', 'picking_time', 'picking_pincode', 'note'];
    sendData = readFormData(formIds);
    exchangeDataPhp(sendData);
}

/*
formデータ読み込む関数

引数：formのID
戻り値：formデータの配列
*/
function readFormData(formIds){
    var formData = {};

    // 各IDの入力フォームの値をdataに格納
    formIds.forEach(function(id) {
        formData[id] = document.getElementById(id).value;
    });

    return formData;
}

/*
phpとやり取りする関数

引数：phpに送りたいデータ(json形式で送信する)
戻り値：結果
*/
function exchangeDataPhp(sendData) {
    // XMLHttpRequestオブジェクトを使用してPHPにHTTPリクエストを送信
    var xhr = new XMLHttpRequest();
    var url = "receive.php";
    
    xhr.open("POST", url, false); // 第一引数：メソッド　第二引数：接続先のurl 第三引数：同期(false)か非同期(true)かを指定
    xhr.setRequestHeader("Content-Type", "application/json;charset=UTF-8"); // ヘッダを設定(文字列も送れるjson形式を指定)
    xhr.send(JSON.stringify(sendData)); // データを送信

    return xhr.responseText;
}

/*
選択可能時間を制限する関数

引数：
    場所 -> str (2byte文字ダメかも)

***クライアントが場所を選択した時に実行するやつ***
*/
function selectableTime(place) {
    // HTTPリクエストを送信して選択可能時間を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先

    // GETメソッドでデータを送信
    xhr.open("GET", url + "?place=" + encodeURIComponent(JSON.stringify(place)), false);
    xhr.send();

    // httpレスポンスを配列にして受け取る
    timeList = xhr.responseText.split(",").filter(Boolean);

    // 集荷時間の要素を取得
    var selectElement = document.getElementById("picking_time");

    // 既存の選択肢の配列を作成
    var existingOptions = Array.from(selectElement.options).map(option => option.value);

    // 既存の選択肢にない選択可能な時間を追加
    timeList.forEach(function(time) {
        if (!existingOptions.includes(time)) {
            var option = document.createElement("option");
            option.value = time;
            option.text = time;
            selectElement.appendChild(option);
        }
    });
    // 既存の選択肢にある選択可能な時間ではないものを削除
    existingOptions.forEach(function(optionValue) {
        if (!timeList.includes(optionValue)) {
            selectElement.querySelectorAll('option[value="' + optionValue + '"]').forEach(option => option.remove());
        }
    });
    // 時間をソート(したかったけどまだできてない)
    Array.from(selectElement.options)
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
時間が選択された時に場所を選択する関数

引数：
    時間 -> str

***クライアントが時間を選択した時に実行するやつ***
*/
function selectablePlace(time) {
    // HTTPリクエストを送信して場所を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // HTTPリクエスト先

    // GETメソッドでデータを送信
    xhr.open("GET", url + "?time=" + encodeURIComponent(JSON.stringify(time)), false);
    xhr.send();

    // HTTPレスポンス
    placeList = xhr.responseText.split(",").filter(Boolean);

    // 場所の要素を取得
    var selectElement = document.getElementById("picking_place");

    // selectablePlace関数内で、新しい<option>要素を条件文で追加し、既存の要素を削除する方法
    var existingOptions = Array.from(selectElement.options).map(option => option.value);
    placeList.forEach(function(place) {
        if (!existingOptions.includes(place)) {
            var option = document.createElement("option");
            option.value = place;
            option.text = place;
            selectElement.appendChild(option);
        }
    });
    existingOptions.forEach(function(optionValue) {
        if (!timeList.includes(optionValue)) {
            selectElement.querySelectorAll('option[value="' + optionValue + '"]').forEach(option => option.remove());
        }
    });
}

/*
ページが読み込まれた時に実行する処理
*/
document.addEventListener("DOMContentLoaded", function(event) {
    document.getElementById("picking_place").innerHTML = "<option value='' selected disabled>選択してください</option>";
    selectablePlace();
    document.getElementById("picking_time").innerHTML = "<option value='' selected disabled>選択してください</option>";
    selectableTime();
});