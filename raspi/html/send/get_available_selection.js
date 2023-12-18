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
function limitSelectTime(place) {
    // HTTPリクエストを送信して選択可能時間を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先

    // GETメソッドでデータを送信
    xhr.open("GET", url + "?place=" + encodeURIComponent(JSON.stringify(place)), false);
    xhr.send();

    // httpレスポンス
    timeList = xhr.responseText.split(",").filter(Boolean);

    // 時間の要素を取得
    var selectElement = document.getElementById("picking_time");

    // 既存の<option>を削除
    selectElement.innerHTML = "";

    // 新しい<option>を追加
    timeList.forEach(function(time) {
        var option = document.createElement("option");
        option.value = time;
        option.text = time;
        selectElement.appendChild(option);
    });
}