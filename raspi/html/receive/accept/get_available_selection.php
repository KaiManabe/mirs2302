<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    if(isset($_GET['id'])){
        $pythonScriptPath = "/home/pi/git/mirs2302/raspi/select_option_api.py";
    }

    // pythonスクリプトを実行
    $command = "sudo -u pi python3 $pythonScriptPath mode=RECEIVE id=" . $_GET['id']; // なぜかescapeshellarg()を使用すると2byte文字が消えてしまう
    $result = shell_exec($command);
    
    // httpレスポンスを返す
    echo $result;
    
}
?>