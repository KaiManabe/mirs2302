function read_status(){
    // httpリクエストを送信して選択可能なITEM_TYPEを取得
    var xhr = new XMLHttpRequest();
    var url = "robot_state_reader.php"; // httpリクエスト先
    xhr.open("GET", url, false);
    xhr.setRequestHeader('Content-Type', 'application/json');
    xhr.send();
    var resp = JSON.parse(xhr.responseText);
    return resp;
}

function monitor(){
    robot_status = read_status();
    if (robot_status["ROBOT_STATUS"] !== current_status){
        current_status = robot_status["ROBOT_STATUS"];
        switch (current_status){
            case "MOVING":
                iframe_ele.setAttribute("src", "status_moving.html");
                break;
            case "WAITING":
                iframe_ele.setAttribute("src", "status_waiting.html");
                break;
            case "WAITING_FOR_RECEIVE":
                iframe_ele.setAttribute("src", "receive.html");
                break;
            case "WAITING_FOR_PICKUP":
                iframe_ele.setAttribute("src", "pickup.html");
                break;
            default:
                break;
        }

    }
}

var current_status = "PAGE_INIT";
const intervalId = setInterval(monitor, 1000);
const iframe_ele = document.querySelector("#iframe_status");