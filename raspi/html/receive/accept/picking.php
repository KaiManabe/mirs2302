<?php
//var_dump($_POST);
$ID = $_POST["id"];

if(isset($_POST["accept"]) && $_POST["accept"] == "accept"){
    if(isset($_POST["pickup_pincode"])){
        $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order " . $ID . " PICKUP_PIN " . $_POST['pickup_pincode'];
        $result = shell_exec($cmd);
    }
    if(isset($_POST["pickup_time"])){
        $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order " . $ID . " PICKUP_TIME " . $_POST['pickup_time'];
        $result = shell_exec($cmd);
    }
    if(isset($_POST["pickup_place"])){
        $cmd = "sudo -u pi python3 /home/pi/git/mirs2302/raspi/order_mng.py modify_order " . $ID . " PICKUP_PLACE " . $_POST['pickup_place'];
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