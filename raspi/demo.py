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
import robot_state_publisher as rsp
import shutil


if __name__ == "__main__":
    #シリアルポート
    serial_port = ser.arduino_serial()
    
    #走行制御コントローラ
    controller = ctrl.run_controller(serial_port)
    
    shutil.copy("/home/pi/git/mirs2302/raspi/demo_order.csv", "/home/pi/git/mirs2302/raspi/order_info.csv")
    
    #rosからarduinoに速度指令値を送るためのソケット(サーバ)
    sock_server_rta = sock.sock_server(config.RASPI_IP, 56789)
    #rosからarduinoに速度指令値を送るやつ　これ以降別スレッドで無限ループ
    print(f"[INFO][main.py] : 機体を初期位置にセットしてから、ROS ノードを立ち上げてください")
    rta.ros_to_arduino(controller, sock_server_rta)
    
    
    #raspiからrosに指令を送るためのソケット(サーバ)
    sock_server_rtr = sock.sock_server(config.RASPI_IP, 55555)
    #rosからarduinoに速度指令値を送るやつ　これ以降別スレッドで無限ループ
    ros_controller = rtr.raspi_to_ros(sock_server_rtr)


    #オーダー管理用クラス
    order_manager = order_mng.order_manager()
    
    #メール送信用クラス
    #mail_sender = web_app.mails(order_manager)
    
    #モジュール管理用クラス
    sock_server_mdl = sock.sock_server("127.0.0.1", 56674)
    module_manager = module_mng.module_controller(serial_port = serial_port,
                                                  order_manager = order_manager,
                                                  serv = sock_server_mdl)
    #機体管理用クラス
    # airframe_manager = module_mng.airframe_controller(serial_port, mail_sender)
    
    
    rsp.update("ROBOT_STATUS", "WAITING")
#ドアをすべてあけて閉じるという動作をする(初期化するときにサーボがちびちび動いてしまうため)
    rsp.update("DOOR", "CLOSE")
    
    print(f"[INFO][main.py] : 初期化終了 メインループを開始します")
    
    #1ループの処理時間は短めにすること
    while(1):
        order_manager.reflesh()
        for i in range(len(order_manager.df)):
            current_order = order_manager.get_order(i)
            id = current_order["ID"]
            status = current_order["STATUS"]
            

            
            
#*****************************************************************************************************
#状態    ACCEPTED
#デバッグ進捗 : 未完
#*****************************************************************************************************
            if status == "ACCEPTED":
                if True:
                    #ゴール設定
                    ros_controller.set_goal("DEMO2")
                    
                    #走り出すまで待機
                    time.sleep(0.5)
                    for i in range(3):
                        if ros_controller.status == "ACTIVE":
                            print(f"[INFO][main.py][ID:{id}] : ロボットが走行中です...")
                            order_manager.modify_order(id, "STATUS", "MOVING_FOR_PICKUP")
                            rsp.update("ROBOT_STATUS", "MOVING")
                            rsp.update("GOAL", current_order['PICKUP_PLACE'])
                            rsp.update("DOOR_NUM", current_order['ITEM_TYPE'])
                            break
                        else:
                            time.sleep(5)
                    if i == 2:
                        print(f"[FATAL][main.py][ID:{id}]  : 移動を開始できませんでした ROSステータス : {ros_controller.status}", file = sys.stderr)
                    
                    
            
#*****************************************************************************************************
#状態    MOVING_FOR_PICKUP
#デバッグ進捗 : 142のみ
#*****************************************************************************************************                
            elif status == "MOVING_FOR_PICKUP":
                if ros_controller.status == "SUCCEEDED":
                    print(f"[INFO][main.py][ID:{id}]  : ロボットがDEMO2に到着")
                    order_manager.modify_order(id, "STATUS", "WAITING_FOR_PICKUP")
                    rsp.update("ROBOT_STATUS", "WAITING_FOR_PICKUP")
                else:
                    if ros_controller.status == "ACTIVE":#まだ動いているなら
                        if datetime.datetime.now() >= current_order["PICKUP_TIME"] + datetime.timedelta(minutes = 5):
                            print(f"[FATAL][main.py][ID:{id}]  : ロボットがスタックしている可能性があります ROSステータス : {ros_controller.status}", file = sys.stderr)
#スタックしてる場合にバックさせる
                    else:#動いてないし到着してない
                        print(f"[FATAL][main.py][ID:{id}]  : ロボットが到着できませんでした リトライします ROSステータス : {ros_controller.status}", file = sys.stderr)
                        order_manager.modify_order(id, "STATUS", "ACCEPTED")
            
            
#*****************************************************************************************************
#状態    WAITING_FOR_PICKUP
#デバッグ進捗 : 未完
#***************************************************************************************************** 
            elif status == "WAITING_FOR_PICKUP":
                pass
            
            
#*****************************************************************************************************
#状態    PICKED_UP
#デバッグ進捗 : 未完
#***************************************************************************************************** 
            elif status == "PICKED_UP":
#ラボ前に帰る?　予約がどれだけの頻度で入っているかによる
                #今のところ10分前に移動させるようにしているが調整するならどうにかする
                if True:
                    #ゴール設定
                    ros_controller.set_goal("DEMO1")
                    
                    #走り出すまで待機
                    time.sleep(0.5)
                    for i in range(3):
                        if ros_controller.status == "ACTIVE":
                            print(f"[INFO][main.py][ID:{id}] : ロボットが走行中です...")
                            order_manager.modify_order(id, "STATUS", "MOVING_FOR_RECEIVE")
                            rsp.update("ROBOT_STATUS", "MOVING")
                            rsp.update("GOAL", current_order['RECEIVE_PLACE'])
                            rsp.update("DOOR_NUM", current_order['ITEM_TYPE'])
                            break
                        else:
                            time.sleep(5)
                    if i == 2:
                        print(f"[FATAL][main.py][ID:{id}]  : 移動を開始できませんでした ROSステータス : {ros_controller.status}", file = sys.stderr)



            
#*****************************************************************************************************
#状態    MOVING_FOR_RECEIVE
#デバッグ進捗 : 未完
#*****************************************************************************************************                
            elif status == "MOVING_FOR_RECEIVE":
                if ros_controller.status == "SUCCEEDED":
                    print(f"[INFO][main.py][ID:{id}]  : ロボットがDEMO1に到着")
                    order_manager.modify_order(id, "STATUS", "WAITING_FOR_RECEIVE")
                    rsp.update("ROBOT_STATUS", "WAITING_FOR_RECEIVE")
                else:
                    if ros_controller.status == "ACTIVE":#まだ動いているなら
                        if datetime.datetime.now() >= current_order["RECEIVE_TIME"] + datetime.timedelta(minutes = 5):
                            print(f"[FATAL][main.py][ID:{id}]  : ロボットがスタックしている可能性があります ROSステータス : {ros_controller.status}", file = sys.stderr)
#スタックしてる場合にバックさせる
                    else:#動いてないし到着してない
                        print(f"[FATAL][main.py][ID:{id}]  : ロボットが到着できませんでした リトライします ROSステータス : {ros_controller.status}", file = sys.stderr)
                        order_manager.modify_order(id, "STATUS", "PICKED_UP")
            
            
            
#*****************************************************************************************************
#状態    WAITING_FOR_RECEIVE
#デバッグ進捗 : 未完
#***************************************************************************************************** 
            elif status == "WAITING_FOR_RECEIVE":
                pass
            
            elif status == "RECEIVED":
                order_manager.modify_order(id, "STATUS", "ACCEPTED")

