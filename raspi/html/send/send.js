/*
データを送信し結果を返す関数

引数：送りたいデータ(文字列-半角英数)
戻り値：結果
*/
function judg(inputData) {
    // 入力データを取得（getElementById()：任意のHTMLタグで指定したIDにマッチするドキュメント要素を取得するメソッド）
    // htmlがjsに渡すのではなく、jsがhtmlを読み込んでいる
    // var inputData = document.getElementById("inputData").value; // document.：文字コードがUTF-8のHTMLのソースコード

    // XMLHttpRequestオブジェクトを使用してPHPにHTTPリクエストを送信
    var xhr = new XMLHttpRequest();
    var url = "send.php";
    xhr.open("POST", url, true); // 第一引数：メソッド　第二引数：接続先のurl 第三引数：同期か非同期かを指定(Trueは非同期)
    xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded"); // 第一引数：ヘッダ名　第二引数：ヘッダの値

    // 判定するデータを送信
    var data = "inputData=" + inputData;

    // レスポンスが帰ってきた際の処理
    xhr.onreadystatechange = function () {
        if (xhr.readyState == 4 && xhr.status == 200) {
            // PHPから受け取った判定結果を表示
            // id="result"の要素に返ってきた値を書き込む
            document.getElementById("result").innerHTML = xhr.responseText;

            // 戻り値を結果にしようとしてるけどうまくいかない
            // result = xhr.responseText;
            // return result;
        }
    };

    // エンコードされたデータを送信
    xhr.send(data);
}

var values = {}; // ユーザが入力した値を保持しておくオブジェクト

/*
ユーザが入力した値を変数に代入する関数
引数・戻り値なし
*/
function setValues() {
    var inputIds = ['client_address', 'client_address_type', 'target_type', 'target_address', 'item_type', 'item_name', 'picking_place', 'picking_time', 'picking_pincode', 'note'];
    var values = {};

    inputIds.forEach(function(id) {
        values[id] = document.getElementById(id).value;
    });

    // values オブジェクトに各要素の値がセットされた状態
    console.log(values);
}

// デバッグ用
function addHTML(){
    document.getElementById("result").innerHTML = values.client_address;
}