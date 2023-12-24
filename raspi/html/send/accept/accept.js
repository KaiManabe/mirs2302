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
var timelist_available = available_order_time(dataList.slice(22,23)); //依頼主の受取時間から制限される時間を配列で取得

/*
依頼者の設定した時間に基づき選択できる時間を制限する処理
*/
function available_order_time(order_time){ //依頼者が設定した時間をorder_timeに代入
    var limit_time //依頼者の設定した時間までに届けるために発送しなきゃならない限界の時間
    var timelist_available; //制限後の選択可能な時間
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

    for(let i = 0; i < Object.keys(timelist_base).length; i++){ //limit_timeを代入するためのループ
        if(timelist_base[keys(i)] == order_time){
            limit_time = timelist_base[keys(i-6)] //最低限必要な移動時間によって調節
            break;
        }
        timelist_available[i] = timelist_base[keys(i)];
    }
    return timelist_available; //制限後の選択可能時間を返す
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