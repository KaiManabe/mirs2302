<?php
// JSONファイルのパス
$jsonFile = '/home/pi/git/mirs2302/raspi/robot_status.json';
header('Content-Type: application/json');
readfile($jsonFile);
?>