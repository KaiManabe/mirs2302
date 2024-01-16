<?php
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $pythonScript = "/home/pi/git/mirs2302/raspi/order_mng.py";
    $orderData = json_decode(file_get_contents('php://input'), true); // 受け取ったデータをJSONから連想配列に変換

    // 注文情報を要素ごと取得
    $ORDER_TYPE = 'ORDER';
    $RECEIVER = $orderData['client_address'];
    if ($orderData['item_selector'] === 'カフェオレホット'){
        $ITEM_TYPE = '食品（保温）';
    }
    else{
        $ITEM_TYPE = '食品（保冷）';
    }
    $ITEM_NAME = $orderData['item_selector'];
    $PICKING_PLACE = $orderData['picking_place'];
    $PICKING_TIME = $orderData['picking_time'];
    $PICKING_PINCODE = $orderData['picking_pincode'];
    $NOTE = $orderData['note'];

    // 注文情報を書き込み
    $command = "sudo -u pi python3 $pythonScript new_order '$ORDER_TYPE' '$ITEM_TYPE' '$ITEM_NAME' '$RECEIVER' '$PICKING_PLACE' '$PICKING_TIME' '$PICKING_PINCODE' '$NOTE'";
    $result = exec($command, $output, $return_var);

    // 管理者に新オーダーを通知
    if ($return_var === 0){
        $pythonScript = "/home/pi/git/mirs2302/raspi/web_app.py";
        $command = "sudo -u pi python3 $pythonScript new_order";
        $result = shell_exec($command);
    }

    echo $return_var; // 正常終了:0 異常終了:0以外
}
?>