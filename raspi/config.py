MAX_DATA_LENGTH_OF_SERIAL_BUFFER = 500      #シリアル通信のバッファをため込む量
MAX_DATA_LENGTH_OF_SOCKET_BUFFER = 100      #ソケット通信のバッファをため込む量
LIDAR_CLASS_BUFFER_MAX_LENGTH = 10000       #暫定未使用

NUMBER_OF_DATA_TYPE_ON_SERIAL = 20      #シリアル通信で扱うデータの種類(の範囲？)

JETSON_IP = "192.168.1.3"       #ルータにおけるJETSONのIP
RASPI_IP = "192.168.1.4"        #ルータにおけるRASPIのIP
RASPI_IP_NCT = "172.25.19.2"    #学内LANにおけるRASPIのIP
SOCKET_PORT_RTA = 56789             #ros_to_arduino用のソケット通信のIP
SOCKET_PORT_RTR = 55555             #raspi_to_ros用のソケット通信のIP
SOCKET_PORT_MDL = 56674             #module_mng用のソケット通信のIP

GAIN_ACCURACY = 10000