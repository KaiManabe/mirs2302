<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    if(isset($_GET['item_selector'])){
        $type = 'selected_box=';
        $receivedData = json_decode($_GET['item_selector']); // GETメソッドのクエリ文字列のplaceデータを取得
        if ($receivedData === 'カフェオレホット'){
            $box = '食品（保温）';
        }else{
            $box = '食品（保冷）';
        }
    }

    // pythonスクリプトを実行
    $pythonScriptPath = "/home/pi/git/mirs2302/raspi/select_option_api.py";
    $command = "python3 $pythonScriptPath mode=ORDER $type$box"; // なぜかescapeshellarg()を使用すると2byte文字が消えてしまう
    $result = shell_exec($command);
    
    // httpレスポンスを返す
    echo $result;
}
?>