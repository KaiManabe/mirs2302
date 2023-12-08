/*
全体を実行するやつ
*/
function execution(){
    data = receiveFormData();
    sendData(data);
}

/*
formデータを受け取り、データを送信する関数
引数・戻り値なし
*/
function receiveFormData(){
    var inputIds = ['client_address', 'client_address_type', 'target_address', 'target_address_type', 'item_type', 'item_name', 'picking_place', 'picking_time', 'picking_pincode', 'note'];
    var data = {};

    // 各IDの入力フォームの値をdataに格納
    inputIds.forEach(function(id) {
        data[id] = document.getElementById(id).value;
    });

    return data;
}

/*
データを送信し結果を返す関数

引数：送りたいデータ(配列も可能)
戻り値：結果(にしたい)
*/
function sendData(inputData) {
    // XMLHttpRequestオブジェクトを使用してPHPにHTTPリクエストを送信
    var xhr = new XMLHttpRequest();
    var url = "receive.php";
    
    xhr.open("POST", url, false); // 第一引数：メソッド　第二引数：接続先のurl 第三引数：同期(true)か非同期(false)かを指定
    xhr.setRequestHeader("Content-Type", "application/json"); // ヘッダを設定(文字列も送れるjson形式を指定)
    xhr.send(JSON.stringify(inputData)); // データを送信

    // レスポンスが帰ってきた際の処理
    xhr.onreadystatechange = function () {
        // xhr.readyState == 4 は通信完了、xhr.status == 200 は通信成功を表す
        if (xhr.readyState == 4 && xhr.status == 200) {
            console.log(xhr.responseText); //　開発者デバッグ用
            document.getElementById("result").innerHTML = xhr.responseText; // id="result"の要素に返ってきた値を書き込む

            // 戻り値を結果にしようとしてるけどうまくいかない
            // result = xhr.responseText;
            // return result;
        }
    };
}