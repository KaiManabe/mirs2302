/*
全体を実行するやつ
*/
function execution(){
    sendData = readFormData();
    receiveData = exchangeDataPhp(sendData);
}

/*
formデータ読み込む関数

引数：なし
戻り値：formデータの配列
*/
function readFormData(){
    var inputIds = ['client_address', 'client_address_type', 'target_address', 'target_address_type', 'item_type', 'item_name', 'picking_place', 'picking_time', 'picking_pincode', 'note'];
    var formData = {};

    // 各IDの入力フォームの値をdataに格納
    inputIds.forEach(function(id) {
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
    xhr.setRequestHeader("Content-Type", "application/json"); // ヘッダを設定(文字列も送れるjson形式を指定)
    xhr.send(JSON.stringify(sendData)); // データを送信

    return xhr.responseText;
}

/*
デバッグ用関数（うまく動かん）

引数：htmlに追加したいデータ（配列）
戻り値：なし
*/
function addHTML(data){
    var exdata = data;
    // 配列の各要素をliタグで囲んでinnerHTMLに挿入
    document.getElementById("result").innerHTML = "<ul>" + exdata.map(function(item) {
        return "<li>" + item + "</li>";
    }).join("") + "</ul>";  // 配列を文字列に結合し、ulタグで囲む
}