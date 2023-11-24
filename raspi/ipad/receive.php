<?php
if ($_SERVER["REQUEST_METHOD"] == "POST") {
    // Retrieve data from the POST request
    $deliveryTime = $_POST["time"];
    $deliveryLocation = $_POST["location"];

    // Pass data to Python script
    $pythonScriptPath = "./script.py";
    $command = "python3 $pythonScriptPath \"$deliveryTime\" \"$deliveryLocation\"";
    $output = shell_exec($command);

    // You can process or use the $output as needed
    echo "Data sent to Python script. Response: $output";
} else {
    echo "Invalid request method";
}
?>
ß