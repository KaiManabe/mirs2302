<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    if(isset($_GET['id'])){
        $pythonScriptPath = "/home/pi/git/mirs2302/raspi/select_option_api.py";
        $receivedData = json_decode($_GET['id']); // GETメソッドのクエリ文字列のtimeデータを取得
    }

    // pythonスクリプトを実行
    $command = "sudo -u pi python3 $pythonScriptPath mode=RECEIVE id=$receivedData"; // なぜかescapeshellarg()を使用すると2byte文字が消えてしまう
    $result = shell_exec($command);
    
    // httpレスポンスを返す
    echo $result;
    
}
?>