<?php
if(isset($_GET["PASS"])){
    $auth_result = shell_exec("sudo -u pi python3 /home/pi/git/mirs2302/raspi/confirm.py " . $_GET["PASS"]);
    echo($auth_result);
}
?>