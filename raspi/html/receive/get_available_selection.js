/*
--------------- 要素の定義 ---------------
*/
const formElement = document.getElementById("form");
const itemTypeElement = document.getElementById("item_type");
const pickingTimeElement = document.getElementById("picking_time");

/*
--------------- イベントリスナー ---------------
*/

// ページが読み込まれた時に実行する処理
document.addEventListener("DOMContentLoaded", function(event) {
    itemTypeElement.innerHTML = "<option value='init' selected disabled>選択してください</option>";
    var labels = ["小物", "書類", "食品（保冷）", "食品（保温）"]
    for(let i  = 0; i < 4; i++){
        newele = document.createElement("option");
        newele.setAttribute("value", labels[i]);
        newele.innerHTML = labels[i]
        itemTypeElement.appendChild(newele);
    }
    pickingTimeElement.innerHTML = "<option value='init' selected disabled>選択してください</option>";
    //selectableItem();
    selectableTime();
});
// ITEM_TYPEが選択された時の処理
itemTypeElement.addEventListener('change', function(event) {
    selectableTime(event.target.value);
});
// 集荷時間が選択された時の処理
pickingTimeElement.addEventListener('change', function(event) {
    selectableItem(event.target.value);
});
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
    var formIds = ['client_address', 'client_address_type', 'target_address', 'target_address_type', 'item_type', 'item_name', 'picking_place', 'picking_time', 'picking_pincode', 'note']; // formのID
    var couplingIds = [['client_address', 'client_address_type'], ['target_address', 'target_address_type']]; // 結合する要素のID
    var sendData = readFormData(formIds, couplingIds);

    // formデータを送信
    var result = sendDataToPhp(sendData);

    if (result == 0){
        window.alert("依頼を承りました\n依頼相手が依頼を承認するまでお待ちください。");
    }else{
        window.alert("エラーが発生しました。\nもう一度お試しください。");
    }

    


    /*   2023-12-31  alertに変更

    // 送信中の画面を表示（なぜかできねえ！！！！！！！！！！！！！！）
    var formElement = document.getElementById("form");
    formElement.innerHTML = "<div id='sending'>取引情報を送信中です...</div>";

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
    */
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
function selectableTime(item_type) {
    show_modal("modal_loading");
    // httpリクエストを送信して選択可能な時間を取得
    var xhr = new XMLHttpRequest();
    
    xhr.onreadystatechange = function(){
        if (xhr.readyState == 4){
            if(xhr.status == 200){
                
                let resp = xhr.responseText;
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
            hide_modal("modal_loading");
        }
    }

    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("GET", url + "?item_type=" + encodeURIComponent(JSON.stringify(item_type)), true); // 同期通信GETメソッド
    xhr.send();
    
}

/*
選択可能なITEM_TYPEを制限する関数

引数(なしでもいける)：
    時間: str

***クライアントが集荷時間を選択した時に実行するやつ***
*/
function selectableItem(time) {
    show_modal("modal_loading");
    // httpリクエストを送信して選択可能なITEM_TYPEを取得
    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function(){
        if (xhr.readyState == 4){
            if(xhr.status == 200){
                var itemList = xhr.responseText.split(",").filter(Boolean); // httpレスポンスを配列にして受け取る

                // 既存の選択肢の配列を作成
                var existingOptions = Array.from(itemTypeElement.options).map(option => option.value);

                // 既存の選択肢にない選択可能なITEM_TYPEを追加
                itemList.forEach(function(item) {
                    if (!existingOptions.includes(item)) {
                        var option = document.createElement("option");
                        option.value = item;
                        option.text = item;
                        itemTypeElement.appendChild(option);
                    }
                });
                // 既存の選択肢にある選択可能なITEM_TYPEではないものを削除
                existingOptions.forEach(function(optionValue) {
                    if (!itemList.includes(optionValue) && optionValue != 'init') {
                        itemTypeElement.querySelectorAll('option[value="' + optionValue + '"]').forEach(option => option.remove());
                    }
                });
                // ITEM_TYPEの選択肢をソート
                const sortRule = ['小物', '資料', '食品（保冷）', '食品（保温）']; // ITEM_TYPEのソート規則（要素の早い順にソートされる）
                Array.from(itemTypeElement.options)
                .filter(option => option.value !== 'init')
                .sort(function(a, b) {
                    return sortRule.indexOf(a.value) - sortRule.indexOf(b.value);
                })
                .forEach(option => itemTypeElement.appendChild(option));
            }
            hide_modal("modal_loading");
        }
    }

    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("GET", url + "?time=" + encodeURIComponent(JSON.stringify(time)), true); // 同期通信GETメソッド
    xhr.send();
}