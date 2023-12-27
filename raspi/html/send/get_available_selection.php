<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    if(isset($_GET['time'])){
        $type = 'time'; // 引数の値のタイプ
        $receivedData = json_decode($_GET['time']); // GETメソッドのクエリ文字列のtimeデータを取得
    } elseif(isset($_GET['item_type'])){
        $type = 'type'; // 引数の値のタイプ
        $receivedData = json_decode($_GET['item_type']); // GETメソッドのクエリ文字列のplaceデータを取得
    }

    // pythonスクリプトを実行
    $pythonScriptPath = "/home/pi/git/mirs2302/raspi/select_option_api.py";
    $command = "python3 $pythonScriptPath $type $receivedData"; // なぜかescapeshellarg()を使用すると2byte文字が消えてしまう
    $result = shell_exec($command);
    
    // httpレスポンスを返す
    echo $result;
}
?>