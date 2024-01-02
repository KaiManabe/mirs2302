<?php
$jsonFile = '/home/pi/git/mirs2302/raspi/robot_status.json';
$jsonData = file_get_contents($jsonFile);
$robot_status = json_decode($jsonData, true); 
echo($robot_status["GOAL"]);

?>