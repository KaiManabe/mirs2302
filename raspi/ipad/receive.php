<?php
/* 
$_SERVERはスーパーグローバル変数（phpですでに定義されているもの）
ページにアクセスする際に使われたメソッド名がPOSTの場合
*/
if ($_SERVER["REQUEST_METHOD"] == "POST") {
    // POSTメソッドから渡された値
    $deliveryTime = $_POST["time"];
    $deliveryLocation = $_POST["location"];

    // 実行するpythonファイルのパス
    $pythonScriptPath = "~/git/mirs2302/raspi/httpTest.py";

    // シェルコマンドを実行(戻り値は文字列)
    $command = "python3 $pythonScriptPath \"$deliveryTime\" \"$deliveryLocation\"";
    $output = shell_exec($command); 

    // 文字列を出力
    echo "Data sent to Python script. Response: $output";
} else {
    echo "Invalid request method";
}
?>
ß