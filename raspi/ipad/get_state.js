function sendData() {
    var time = document.getElementById("deliveryTime").value;
    var location = document.getElementById("deliveryLocation").value;

    // phpにhttpリクエストを送る
    var xhr = new XMLHttpRequest();
    xhr.open("POST", "receive.php", true);
    xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
    xhr.send("time=" + time + "&location=" + location);
}