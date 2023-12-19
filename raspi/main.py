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
    
    
    """テスト用"""
    called = 0
    while(1):
        next_movement_time, next_movement_goal = order_manager.get_next_movement(called)
        print(f"\r 次の移動時刻：{next_movement_time}     目的地 : {next_movement_goal}       " , end = "")
        if next_movement_time <= datetime.datetime.now():
            print("移動の時間だ！")
            ros_controller.set_goal(next_movement_goal)
            called += 1
        
        time.sleep(1)