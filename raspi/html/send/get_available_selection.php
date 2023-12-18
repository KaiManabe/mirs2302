<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    // HTTPリクエストを受け取り、Pythonを実行して注文状況を取得
    $pythonScriptPath = "order_mng.py";
    $orderStatus = exec("python $pythonScriptPath");

    // 取得した注文状況をHTTPレスポンスとして送信
    echo $orderStatus;
}
?>