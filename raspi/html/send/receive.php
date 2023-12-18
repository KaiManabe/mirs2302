<?php

// POSTリクエストがあるか確認
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    // 受け取ったデータをJSONから連想配列に変換
    $receivedData = json_decode(file_get_contents('php://input'), true);

    // とりあえず受け取ったデータをそのままhttpレスポンスとして返す
    // echo json_encode(['status' => 'success', 'data' => $receivedData]);
    $command = "python3 $pythonScript " . escapeshellarg($receivedData);
} else {
    // POSTリクエスト以外はエラーとする
    echo json_encode(['status' => 'error', 'message' => 'Invalid request method']);
}

?>
