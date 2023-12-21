<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){ //メソッドがGETでリクエストが来ているか
    if(isset($_GET['id'])){ //クエリ文字列のidデータあるか
        $pythonScriptPath = "/home/pi/git/mirs2302/raspi/order_mng.py"; //実行するpython
        $receivedData = json_decode($_GET['id']); //クエリ文字列のidデータを取得したい
    }
}

$command = "python3 $pythonScriptPath get_order $receivedData";
$result = shell_exec($command);

echo $result;

?>