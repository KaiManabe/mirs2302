<?php
// 入力データを取得
$inputData = $_POST['inputData'];

// Pythonスクリプトを実行して判定を行う
$pythonScript = "perform_judgment.py";
$command = "python3 $pythonScript " . escapeshellarg($inputData);
$result = shell_exec($command);

// 判定結果を表示
echo $result;
?>
