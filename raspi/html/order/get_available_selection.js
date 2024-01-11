/*
--------------- 要素の定義 ---------------
*/
const formElement = document.getElementById("form");
const itemSelectorElement = document.querySelectorAll(".item_icon");
const pickingTimeElement = document.getElementById("picking_time");

/*
--------------- イベントリスナー ---------------
*/

// ページが読み込まれた時に実行する処理
document.addEventListener("DOMContentLoaded", function(event) {
    pickingTimeElement.innerHTML = "<option value='init' selected disabled>選択してください</option>";
    selectableTime();
});
// 商品が選択された時の処理 -> item_selector.js に記述
// データを送信するボタンが押された時の処理
formElement.addEventListener('submit', function(event) {
    event.preventDefault(); // ページのリロードを防ぐ
    submitProcessing();
});

/*
送信ボタンに関する処理を行う関数
*/
function submitProcessing(){
    // formデータの読み込み
    var formIds = ['client_address', 'client_address_type', 'item_selector', 'picking_place', 'picking_time', 'picking_pincode', 'note']; // formのID
    var couplingIds = [['client_address', 'client_address_type']]; // 結合する要素のID
    var sendData = readFormData(formIds, couplingIds);

    // formデータを送信
    var result = sendDataToPhp(sendData);

    if (result == 0){
        window.alert("依頼を承りました\n依頼相手が依頼を承認するまでお待ちください。");
    }else{
        window.alert("エラーが発生しました。\nもう一度お試しください。");
    }
}

/*
formデータ読み込む関数

引数：
    formのID, 結合したい要素のID（先頭のIDの要素に結合）
戻り値：
    formデータの配列
*/
function readFormData(formIds, couplingIds) {
    var formData = {};

    // 各IDの入力フォームの値をformDataに格納
    formIds.forEach(function (id) {
        formData[id] = document.getElementById(id).value;
    });

    // 特定のフォームの値を結合
    couplingIds.forEach(function (coupling) {
        var combinedValue = coupling.slice(1).reduce(function (combined, id) {
            return combined + formData[id];
        }, '');

        // 結合
        formData[coupling[0]] += combinedValue;

        // 結合後に不要なプロパティを削除
        coupling.slice(1).forEach(function (id) {
            delete formData[id];
        });
    });

    return formData;
}

/*
phpに送信する関数

引数：
    phpに送りたいデータ(json形式で送信する)
*/
function sendDataToPhp(sendData) {
    // XMLHttpRequestオブジェクトを使用してPHPにHTTPリクエストを送信
    var xhr = new XMLHttpRequest();
    var url = "receive.php";
    
    xhr.open("POST", url, false); // 第一引数：メソッド　第二引数：接続先のurl 第三引数：同期(false)か非同期(true)かを指定
    xhr.setRequestHeader("Content-Type", "application/json;charset=UTF-8"); // ヘッダを設定(文字列も送れるjson形式を指定)
    xhr.send(JSON.stringify(sendData)); // データを送信

    return xhr.responseText;
}

/*
選択可能な時間を制限する関数

引数(なしでもいける)：
    ITEM_TYPE: str

***クライアントがITEM_TYPEを選択した時に実行するやつ***
*/
function selectableTime(item_selector) {
    // httpリクエストを送信して選択可能な時間を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("GET", url + "?item_selector=" + encodeURIComponent(JSON.stringify(item_selector)), false); // 同期通信GETメソッド
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