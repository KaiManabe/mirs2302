<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    // pythonスクリプトを実行
    $pythonScriptPath = "/home/pi/git/mirs2302/raspi/select_option_api.py";
    $command = "python3 $pythonScriptPath mode=ORDER"; // なぜかescapeshellarg()を使用すると2byte文字が消えてしまう
    $result = shell_exec($command);
    
    // httpレスポンスを返す
    echo $result;
}
?>