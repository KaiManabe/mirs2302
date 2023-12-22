<?php
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $pythonScript = "/home/pi/git/mirs2302/raspi/";
    $selectData = json_decode(file_get_contents('php://input'), true); // 受け取ったデータをJSONから連想配列に変換

    // 選択情報を取得
    $place = $selectData["place"];
    $time = $selectData["time"];

    // pythonスクリプトを実行
    $command = "python3 $pythonScriptPath '$place' '$time'";
    echo $command;
    // $selectableData = shell_exec($command);

    // httpレスポンスを返す
    // echo $selectableData;
}
?>