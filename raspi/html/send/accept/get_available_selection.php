<?php
if($_SERVER['REQUEST_METHOD'] === 'GET'){
    if(isset($_GET[''])){
        $pythonScriptPath = "/home/pi/git/mirs2302/raspi/available_picking_place.py"
        $recerivedData = json_decode($_GET['time']);
    }

    $command = "python3 $pythonScriptPath $receivedData";
    $result = shell_exec($command);
}
?>