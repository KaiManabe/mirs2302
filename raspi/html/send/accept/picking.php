<?php
//var_dump($_POST);
$ID = $_POST["id"];

if(isset($_POST["accept"]) && $_POST["accept"] == "accept"){
    if(isset($_POST["receive_pincode"])){
        $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order " . $ID . " RECEIVE_PIN " . $_POST['receive_pincode'];
        $result = shell_exec($cmd);
    }
    if(isset($_POST["receive_time"])){
        $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order " . $ID . " RECEIVE_TIME " . $_POST['receive_time'];
        $result = shell_exec($cmd);
    }
    if(isset($_POST["receive_place"])){
        $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order " . $ID . " RECEIVE_PLACE " . $_POST['receive_place'];
        $result = shell_exec($cmd);
    }


    $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order " . $ID . " STATUS ACCEPTED";
    $result = shell_exec($cmd);


}else{
    $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order $ID STATUS DENIED";
    $result = shell_exec($cmd);
}

header("Location: ../../main");
exit(); 

?>