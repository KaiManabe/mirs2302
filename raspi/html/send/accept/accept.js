let current_url = new URL(window.location.href);
let params = current_url.searchParams;
var order_id = params.get('id');

var request = new XMLHttpRequest();
var url = "get_order_data.php"; //リクエスト先のphpのurl
request.open("GET", url + "?id=" + encodeURIComponent(JSON.stringify(order_id)), false);
request.send();
var dataList = request.responseText.split(/\s+/); //連想配列をコンマで区切った普通の配列に変換

var mail = dataList.slice(16,17); //dataList配列のうち、16番目の要素をmailに代入
var thing = dataList.slice(7,8); //dataList配列のうち、8番目の要素をthingに代入

document.querySelector('#text').textContent = `${mail}さんから${thing}を集荷するように依頼が来ています`;

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