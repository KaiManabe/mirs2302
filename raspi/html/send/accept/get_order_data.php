<?php
if ($_SERVER['REQUEST_METHOD'] === 'GET'){
    if(isset($_GET['id'])){
        $pythonScriptPath = ../../order_mng.py;
        $receivedData = json_decode($_GET['id']);
    }
}

$command = "python3 $pythonScriptPath "

?>