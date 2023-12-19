<?php

// POSTリクエストがあるか確認
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    // 受け取ったデータをJSONから連想配列に変換
    $receivedData = json_decode(file_get_contents('php://input'), true);

    // $command = "python3 $pythonScript " . escapeshellarg($receivedData);
    // $result = shell_exec($command);

    // echo json_encode(['status' => 'success', 'data' => $receivedData]); //　とりあえずそのまま返す
    print($receivedData);
} else {
    // POSTリクエスト以外はエラーとする
    echo json_encode(['status' => 'error', 'message' => 'Invalid request method']);
}

?>
