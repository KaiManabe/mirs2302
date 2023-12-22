/*
ページを読み込んだタイミングで依頼者と依頼物を書き換える
*/
let current_url = new URL(window.location.href);
let params = current_url.searchParams;
var order_id = params.get('id'); //order_idを取得

var request = new XMLHttpRequest();
var url = "get_order_data.php"; //リクエスト先のphpのurl
request.open("GET", url + "?id=" + encodeURIComponent(JSON.stringify(order_id)), false);
request.send();
var dataList = request.responseText.split(/\s+/); //連想配列をコンマで区切った普通の配列に変換

var mail = dataList.slice(16,17); //dataList配列のうち、16番目の要素をmailに代入
var thing = dataList.slice(7,8); //dataList配列のうち、8番目の要素をthingに代入

document.querySelector('#text').textContent = `${mail}さんから${thing}を${time}時までに集荷するように依頼が来ています`; //テキスト差し替え

/*
ページを読み込んだタイミングで、依頼者の設定した時間に基づき選択できる時間を制限する処理
*/
var time = dataList.slice(22,23); //依頼者が設定した時間をtimeに代入
var timekey = null; //それまでに積み込んで欲しいという時間のキー
var timelist = null; //制限された後の時間(連想配列)
var timelist_base = { //最初のとりあえずの時間
    1:"15:00-15:10",
    2:"15:10-15:20",
    3:"15:20-15:30",
    4:"15:30-15:40",
    5:"15:40-15:50",
    6:"16:00-16:10",
    7:"16:10-16:20",
    8:"16:20-16:30"
}
const keys = Object.keys(timelist_base); //timelist_baseのkeyを配列keysに
for(let i = 0; i < keys.length; i++){
    if(timelist_base[keys[i]] === time){
        timekey = keys[i];
    }
}

for(let i = 0; i < timekey; i++){
    timelist["i"] = timelist_base[i];
}

timelist.forEach(function(time) {
    for (let i = 0; i < timekey; i++) {
        var option = document.createElement("option");
        option.value = time;
        option.text = time;
        selectElement.appendChild(option);
    }
});

/*
選択できるitemTypeを制限する関数

引数(なしでもいける)：
    場所: str
    時間: str

***クライアントが集荷時間を選択した時に実行するやつ***
*/
function selectableItem(place, time) {
    // httpリクエストを送信
    var xhr = new XMLHttpRequest();
    var url = "get_available_selection.php"; // httpリクエスト先
    xhr.open("POST", url, false); // 同期通信POSTメソッド
    xhr.setRequestHeader("Content-Type", "application/json;charset=UTF-8"); // ヘッダを設定(文字列も送れるjson形式を指定)
    // 送信するデータを連想配列にする
    var Data = {
        "place": JSON.stringify(place),
        "time": JSON.stringify(time)
    };
    xhr.send(JSON.stringify(Data)); // データを送信

    // httpレスポンスを配列にして受け取る
    var itemList = xhr.responseText.split(",").filter(Boolean); 

    // 既存の選択肢を取得
    var itemTypeElement = document.getElementById("item_type");
    var existingOptions = Array.from(itemTypeElement.options).map(option => option.value); // 既存の選択肢の配列を作成

    // 既存の選択肢にない選択可能なitemTypeを追加
    itemList.forEach(function(item) {
        if (!existingOptions.includes(item)) {
            var option = document.createElement("option");
            option.value = item;
            option.text = item;
            itemTypeElement.appendChild(option);
        }
    });
    // 既存の選択肢にある選択可能な集荷場所ではないものを削除
    existingOptions.forEach(function(optionValue) {
        if (!itemList.includes(optionValue) && optionValue != 'init') {
            itemTypeElement.querySelectorAll('option[value="' + optionValue + '"]').forEach(option => option.remove());
        }
    });
    // 集荷場所の選択肢をソート
    const sortRule = ['共通棟', 'D科棟', 'E科棟', 'S科棟', 'M科棟', 'C科棟']; // 集荷場のソート規則（要素の早い順にソートされる）
    Array.from(itemTypeElement.options)
    .filter(option => option.value !== 'init')
    .sort(function(a, b) {
        return sortRule.indexOf(a.value) - sortRule.indexOf(b.value);
    })
    .forEach(option => itemTypeElement.appendChild(option));
}

/*
チェックボックスの処理
*/

function chbox(obj){
    let that = obj;
    if (document.getElementById(that.id).checked == true) {
        let boxes = document.querySelectorAll('input[type="checkbox"]');

        for (let i = 0; i < boxes.length; i++) {
            boxes[i].checked = false;
        }
        document.getElementById(that.id).checked = true;
    }
}

function view(){
    var target = document.getElementById('target')
	target.classList.replace('hide' , 'view');
}
function notview(){
    var target = document.getElementById('target')
	target.classList.replace('view' , 'hide');
    
}