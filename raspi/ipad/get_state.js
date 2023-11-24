function sendData() {
    var time = document.getElementById("deliveryTime").value;
    var location = document.getElementById("deliveryLocation").value;

    // AJAX request to send data to PHP
    var xhr = new XMLHttpRequest();
    xhr.open("POST", "process.php", true);
    xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
    xhr.send("time=" + time + "&location=" + location);
}