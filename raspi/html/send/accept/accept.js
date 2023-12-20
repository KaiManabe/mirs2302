let url = new URL(window.location.href);
let params = url.searchParams;

var order_id = params.get('id');

pywebview.order_manager().get_order("ORDER_TYPE","order_id")

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