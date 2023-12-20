/*
全体を実行するやつ
*/
function execution(){
    var formIds = ['client_address', 'client_address_type', 'target_address', 'target_address_type', 'item_type', 'item_name', 'picking_place', 'picking_time', 'picking_pincode', 'note']; // formのID
    var couplingIds = [['client_address', 'client_address_type'], ['target_address', 'target_address_type']]; // 結合する要素のID
    sendData = readFormData(formIds, couplingIds);
    exchangeDataPhp(sendData);
}

/*
formデータ読み込む関数

引数：formのID, 結合したい要素のID（先頭のIDの要素に結合）
戻り値：formデータの配列
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

    console.log(formData); // デバッグ出力

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
選択可能な時間を制限する関数

引数(なしでもいける)：
    場所 -> str

***クライアントが集荷場所を選択した時に実行するやつ***
*/
function selectableTime(place) {
    // httpリクエストを送信して選択可能な時間を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("GET", url + "?place=" + encodeURIComponent(JSON.stringify(place)), false); // 同期通信GETメソッド
    xhr.send();
    timeList = xhr.responseText.split(",").filter(Boolean); // httpレスポンスを配列にして受け取る

    // 集荷時間の要素を取得
    var selectElement = document.getElementById("picking_time");

    // 既存の選択肢の配列を作成
    var existingOptions = Array.from(selectElement.options).map(option => option.value);

    // 既存の選択肢にない選択可能な集荷時間を追加
    timeList.forEach(function(time) {
        if (!existingOptions.includes(time)) {
            var option = document.createElement("option");
            option.value = time;
            option.text = time;
            selectElement.appendChild(option);
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
時間が選択された時に場所を選択する関数

引数(なしでもいける)：
    時間 -> str

***クライアントが集荷時間を選択した時に実行するやつ***
*/
function selectablePlace(time) {
    // httpリクエストを送信して選択可能な集荷場所を取得
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("GET", url + "?time=" + encodeURIComponent(JSON.stringify(time)), false); // 同期通信GETメソッド
    xhr.send();
    placeList = xhr.responseText.split(",").filter(Boolean); // httpレスポンスを配列にして受け取る

    // 集荷場所の要素を取得
    var selectElement = document.getElementById("picking_place");

    // 既存の選択肢の配列を作成
    var existingOptions = Array.from(selectElement.options).map(option => option.value);

    // 既存の選択肢にない選択可能な集荷場所を追加
    placeList.forEach(function(place) {
        if (!existingOptions.includes(place)) {
            var option = document.createElement("option");
            option.value = place;
            option.text = place;
            selectElement.appendChild(option);
        }
    });
    // 既存の選択肢にある選択可能な集荷場所ではないものを削除
    existingOptions.forEach(function(optionValue) {
        if (!placeList.includes(optionValue) && optionValue != 'init') {
            selectElement.querySelectorAll('option[value="' + optionValue + '"]').forEach(option => option.remove());
        }
    });
    // 集荷場所の選択肢をソート
    const sortRule = ['共通棟', 'D科棟', 'E科棟', 'S科棟', 'M科棟', 'C科棟']; // 集荷場のソート規則（要素の早い順にソートされる）
    Array.from(selectElement.options)
    .filter(option => option.value !== 'init')
    .sort(function(a, b) {
        return sortRule.indexOf(a.value) - sortRule.indexOf(b.value);
    })
    .forEach(option => selectElement.appendChild(option));
}

/*
---------- 以下イベントリスナー ----------
*/

// 要素の定義
const pickingPlaceElement = document.getElementById("picking_place");
const pickingTimeElement = document.getElementById("picking_time");

// ページが読み込まれた時に実行する処理
document.addEventListener("DOMContentLoaded", function() {
    pickingPlaceElement.innerHTML = "<option value='init' selected disabled>選択してください</option>";
    pickingTimeElement.innerHTML = "<option value='init' selected disabled>選択してください</option>";
    selectablePlace();
    selectableTime();
});
// 集荷場所が選択された時の処理
pickingPlaceElement.addEventListener('change', function(event) {
    selectableTime(event.target.value);
});
// 集荷時間が選択された時の処理
pickingTimeElement.addEventListener('change', function(event) {
    selectablePlace(event.target.value);
});