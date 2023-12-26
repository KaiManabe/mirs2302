/*
ページを読み込んだタイミングで依頼者と依頼物を書き換える
*/
//function onLoad(){
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

document.querySelector('#text').textContent = `${mail}さんから${thing}を${time}までに集荷するように依頼が来ています`; //テキスト差し替え

/*
依頼者の設定した時間に基づき選択できる時間を制限する処理↓
*/
var order_time = dataList.slice(22,23);
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
    timelist_available[i] = timelist_base[keys(i)]; //制限後の時間の配列
}
//}
    
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
        var existingOptions = timelist_available;
    
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
        時間: str
    
    ***クライアントが集荷時間を選択した時に実行するやつ***
    */
    function selectablePlace(time) {
        // httpリクエストを送信して選択可能な集荷場所を取得
        var xhr = new XMLHttpRequest();
        var url = "get_available_selection.php"; // httpリクエスト先
        xhr.open("GET", url + "?time=" + encodeURIComponent(JSON.stringify(time)), false); // 同期通信GETメソッド
        xhr.send();
        var placeList = xhr.responseText.split(",").filter(Boolean); // httpレスポンスを配列にして受け取る
    
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
    
    /*
    //イベントリスナー
    const loadingElement = document.getElementById()
    
    //ページが読み込まれたときの処理
    document.addEventListener("DOMContentLoaded", function(event) {
        onLoad();
    });
    // 集荷場所が選択された時の処理
    pickingPlaceElement.addEventListener('change', function(event) {
        selectableTime(event.target.value);
    });
    // 集荷時間が選択された時の処理
    pickingTimeElement.addEventListener('change', function(event) {
        selectablePlace(event.target.value);
    });
    */