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