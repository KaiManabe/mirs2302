import serial_com as ser
import ros_to_arduino as rta
import run_ctrl as ctrl
import order_mng
import module_mng
import sock
import web_app
import config


if __name__ == "__main__":
    #シリアルポート
    serial_port = ser.arduino_serial()
    
    #走行制御コントローラ
    controller = ctrl.run_controller(serial_port)
    
    #rosからarduinoに速度指令値を送るためのソケット(サーバ)
    sock_server_rta = sock.sock_server(config.RASPI_IP, 56789)
    
    #rosからarduinoに速度指令値を送るやつ　これ以降別スレッドで無限ループ
    rta.ros_to_arduino(controller, sock_server_rta)

    #オーダー管理用クラス
    order_manager = order_mng.order_manager()