<?php
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $pythonScript = "/home/pi/git/mirs2302/raspi/order_mng.py";
    $receivedData = json_decode(file_get_contents('php://input'), true); // 受け取ったデータをJSONから連想配列に変換

    // 要素を取得（できることを確認済み）
    $ORDER_TYPE = 'SEND';
    $SENDER = $receivedData['client_address'];
    $RECEIVER = $receivedData['target_address'];
    $ITEM_TYPE = $receivedData['item_type'];
    $ITEM_NAME = $receivedData['item_name'];
    $PICKING_PLACE = $receivedData['picking_place'];
    $PICKING_TIME = $receivedData['picking_time'];
    $PICKING_PINCODE = $receivedData['picking_pincode'];
    $NOTE = $receivedData['note'];

    $command = "python3 $pythonScript new_order $ORDER_TYPE $SENDER $RECEIVER $PICKING_PLACE $PICKING_TIME"; // ここまではしっかり行ってる
    exec($command); // ここができない
    // echo $command; // デバッグ用
}
?>
