import serial_com as ser
import ros_to_arduino as rta
import raspi_to_ros as rtr
import run_ctrl as ctrl
import order_mng
import module_mng
import sock
import web_app
import config
import datetime
import time
import sys


def set_goal():
     for i in range(3):
        idx = 2 - i
        print(f"[INFO][main.py] : 目的地を{idx}に設定しました")
        ros_controller.set_goal(idx)
        while(1):
            time.sleep(0.2)
            if ros_controller.status == "ACTIVE":
                break
        print(f"[INFO][main.py] : ロボットが走行中です... ROSステータス : {ros_controller.status}")
        while(1):
            time.sleep(0.2)
            if ros_controller.status == "SUCCEEDED":
                print(f"[INFO][main.py] : 目的地{idx}に到着しました ROSステータス : {ros_controller.status}")
                break
            elif ros_controller.status != "ACTIVE":
                print(f"[INFO][main.py] : 目的地{idx}に到着できませんでした ROSステータス : {ros_controller.status}")
                sys.exit(1)
        
        time.sleep(3)    



if __name__ == "__main__":
    #シリアルポート
    serial_port = ser.arduino_serial()
    
    #走行制御コントローラ
    controller = ctrl.run_controller(serial_port)
    
    
    #rosからarduinoに速度指令値を送るためのソケット(サーバ)
    sock_server_rta = sock.sock_server(config.RASPI_IP, 56789)
    #rosからarduinoに速度指令値を送るやつ　これ以降別スレッドで無限ループ
    rta.ros_to_arduino(controller, sock_server_rta)
    
    
    #raspiからrosに指令を送るためのソケット(サーバ)
    sock_server_rtr = sock.sock_server(config.RASPI_IP, 55555)
    #rosからarduinoに速度指令値を送るやつ　これ以降別スレッドで無限ループ
    ros_controller = rtr.raspi_to_ros(sock_server_rtr)


    #オーダー管理用クラス
    order_manager = order_mng.order_manager()
    
    time.sleep(5)
    
   