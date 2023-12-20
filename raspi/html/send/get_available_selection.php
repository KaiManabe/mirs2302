<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    if(isset($_GET['time'])){
        $pythonScriptPath = "/home/pi/git/mirs2302/raspi/available_picking_place.py";
        $receivedData = json_decode($_GET['time']); // GETメソッドのクエリ文字列のtimeデータを取得
    } elseif(isset($_GET['place'])){
        $pythonScriptPath = "/home/pi/git/mirs2302/raspi/available_picking_time.py";
        $receivedData = json_decode($_GET['place']); // GETメソッドのクエリ文字列のplaceデータを取得
    }

    // pythonスクリプトを実行
    $command = "python3 $pythonScriptPath $receivedData"; // なぜかescapeshellarg()を使用すると2byte文字が消えてしまう
    $result = shell_exec($command);
    
    // httpレスポンスを返す
    echo $result;
}
?>