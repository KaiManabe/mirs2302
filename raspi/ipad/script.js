function performJudgment() {
    // 入力データを取得
    var inputData = document.getElementById("inputData").value;

    // XMLHttpRequestオブジェクトを使用してPHPにHTTPリクエストを送信
    var xhr = new XMLHttpRequest();
    var url = "perform_judgment.php";
    xhr.open("POST", url, true);
    xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");

    // 判定するデータを送信
    var data = "inputData=" + encodeURIComponent(inputData);

    // レスポンスが帰ってきた際の処理
    xhr.onreadystatechange = function () {
        if (xhr.readyState == 4 && xhr.status == 200) {
            // PHPから受け取った判定結果を表示
            document.getElementById("result").innerHTML = xhr.responseText;
        }
    };

    // データを送信
    xhr.send(data);
}
