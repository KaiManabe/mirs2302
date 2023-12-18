<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    $pythonScriptPath = "../../available_picking_time.py";
    $receivedData = json_decode($_GET['place']); // GETメソッドのクエリ文字列のplaceデータを取得

    // pythonスクリプトを実行
    $command = "python3 $pythonScriptPath " . escapeshellarg($receivedData);
    $result = shell_exec($command);
    
    // httpレスポンスを返す
    echo $result;
}
?>