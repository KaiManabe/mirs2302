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
import logger


ENABLETS = False
DISABLE_ROS = False
SENDEN = False


if "log" in sys.argv:
    ENABLETS = True

if "server" in sys.argv:
    DISABLE_ROS = True
    

if "senden" in sys.argv:
    SENDEN = True



    
if ENABLETS:
    import logger
    sys.stdout = logger.timestamp_logger(sys.stdout)
    sys.stderr = logger.timestamp_logger(sys.stderr)





if __name__ == "__main__":
    print("[INFO][main.py] : TENQ内部アプリケーションの初期化を開始します...")
    #シリアルポート
    serial_port = ser.arduino_serial()
    
    #走行制御コントローラ
    controller = ctrl.run_controller(serial_port)
    
    
    
    if not(DISABLE_ROS):
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
    mail_sender = web_app.mails(order_manager)
    
    #モジュール管理用クラス
    sock_server_mdl = sock.sock_server("127.0.0.1", 56674)
    module_manager = module_mng.module_controller(serial_port, order_manager, mail_sender, sock_server_mdl)
    #機体管理用クラス
    # airframe_manager = module_mng.airframe_controller(serial_port, mail_sender)
    
    
    rsp.update("ROBOT_STATUS", "WAITING")
#ドアをすべてあけて閉じるという動作をする(初期化するときにサーボがちびちび動いてしまうため)
    rsp.update("DOOR", "CLOSE")
    
    print(f"[INFO][main.py] : 初期化終了 メインループを開始します")
    
    #1ループの処理時間は短めにすること
    next_point = "散歩1"
    while(1):
        availavle_orders = 0
        order_manager.reflesh()
        for i in range(len(order_manager.df)):
            current_order = order_manager.get_order(i)
            id = current_order["ID"]
            status = current_order["STATUS"]
            
#*****************************************************************************************************
#状態    NOT_ACCEPTED_YET
#デバッグ進捗 : 完
#*****************************************************************************************************
            if status == "NOT_ACCEPTED_YET":
                #取引依頼メールを送信
                print(f"[INFO][main.py] : ID[{id}]の依頼メールを送信しています...")
                mail_sender.request(id)
                order_manager.modify_order(id, "STATUS", "MAIL_SENT")
            
            
#*****************************************************************************************************
#状態    MAIL_SENT
#デバッグ進捗 : 完
#*****************************************************************************************************        
            elif status == "MAIL_SENT":
                if datetime.datetime.now() > current_order["RECEIPT_TIME"] + datetime.timedelta(hours = 2):
                    order_manager.modify_order(id, "STATUS", "ACCEPT_TIMEOUT")
                    mail_sender.request_result(id, "timeout")
                    print(f"[INFO][main.py] : ID[{id}]の依頼がタイムアウトしました")
            
            
#*****************************************************************************************************
#状態    ACCEPTED
#デバッグ進捗 : 完
#*****************************************************************************************************
            elif status == "ACCEPTED":
                #今のところ10分前に移動させるようにしているが調整するならどうにかする
                if datetime.datetime.now() >= current_order["PICKUP_TIME"] - datetime.timedelta(minutes = 10):
                    #ゴール設定
                    if ros_controller.set_goal(current_order["PICKUP_PLACE"]) > 0:
                        print(f"[INFO][main.py][ID:{id}] : ロボットが走行中です...")
                        order_manager.modify_order(id, "STATUS", "MOVING_FOR_PICKUP")
                        rsp.update("ROBOT_STATUS", "MOVING")
                        rsp.update("GOAL", current_order['PICKUP_PLACE'])
                        rsp.update("DOOR_NUM", current_order['ITEM_TYPE'])
                        print(f"[INFO][main.py] : ID[{id}]の走行中メールを送信しています...")
                        mail_sender.notice(id, "pickup", "moving")
                    else:
                        time.sleep(1.5)
                        print(f"[FATAL][main.py][ID:{id}]  : 移動を開始できませんでした", file = sys.stderr)
                    
                    
            
#*****************************************************************************************************
#状態    MOVING_FOR_PICKUP
#デバッグ進捗 : 142のみ
#*****************************************************************************************************                
            elif status == "MOVING_FOR_PICKUP":
                if ros_controller.status == "SUCCEEDED":
                    print(f"[INFO][main.py][ID:{id}]  : ロボットが{current_order['PICKUP_PLACE']}に到着")
                    order_manager.modify_order(id, "STATUS", "WAITING_FOR_PICKUP")
                    rsp.update("ROBOT_STATUS", "WAITING_FOR_PICKUP")
                    print(f"[INFO][main.py] : ID[{id}]の到着メールを送信しています...")
                    mail_sender.notice(id, "pickup", "arrived")
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
                if datetime.datetime.now() >= current_order["PICKUP_TIME"] + datetime.timedelta(minutes = 15):
                    order_manager.modify_order(id, "STATUS", "PICKUP_TIMEOUT")
                    print(f"[INFO][main.py] : ID[{id}]の集荷タイムアウトメールを送信しています...")
                    mail_sender.notice(id, "pickup", "timeout")
            
            
#*****************************************************************************************************
#状態    PICKED_UP
#デバッグ進捗 : 未完
#***************************************************************************************************** 
            elif status == "PICKED_UP":
#ラボ前に帰る?　予約がどれだけの頻度で入っているかによる
                #今のところ10分前に移動させるようにしているが調整するならどうにかする
                if datetime.datetime.now() >= current_order["RECEIVE_TIME"] - datetime.timedelta(minutes = 10):
                    #ゴール設定
                    if ros_controller.set_goal(current_order["RECEIVE_PLACE"]) > 0:
                        print(f"[INFO][main.py][ID:{id}] : ロボットが走行中です...")
                        order_manager.modify_order(id, "STATUS", "MOVING_FOR_RECEIVE")
                        rsp.update("ROBOT_STATUS", "MOVING")
                        rsp.update("GOAL", current_order['RECEIVE_PLACE'])
                        rsp.update("DOOR_NUM", current_order['ITEM_TYPE'])
                        print(f"[INFO][main.py] : ID[{id}]の走行中メールを送信しています...")
                        mail_sender.notice(id, "receive", "moving")
                    else:
                        time.sleep(1.5)
                        print(f"[FATAL][main.py][ID:{id}]  : 移動を開始できませんでした", file = sys.stderr)



            
#*****************************************************************************************************
#状態    MOVING_FOR_RECEIVE
#デバッグ進捗 : 未完
#*****************************************************************************************************                
            elif status == "MOVING_FOR_RECEIVE":
                if ros_controller.status == "SUCCEEDED":
                    print(f"[INFO][main.py][ID:{id}]  : ロボットが{current_order['RECEIVE_PLACE']}に到着")
                    order_manager.modify_order(id, "STATUS", "WAITING_FOR_RECEIVE")
                    rsp.update("ROBOT_STATUS", "WAITING_FOR_RECEIVE")
                    print(f"[INFO][main.py] : ID[{id}]の受け取り到着メールを送信しています...")
                    mail_sender.notice(id, "receive", "arrived")
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
                if datetime.datetime.now() >= current_order["RECEIVE_TIME"] + datetime.timedelta(minutes = 15):
                    order_manager.modify_order(id, "STATUS", "RECEIVE_TIMEOUT")
                    print(f"[INFO][main.py] : ID[{id}]の受け取りタイムアウトメールを送信しています...")
                    mail_sender.notice(id, "receive", "timeout")


#*****************************************************************************************************
#状態    暇
#デバッグ進捗 : 未完
#***************************************************************************************************** 
        
        if order_manager.get_next_movement() > datetime.datetime.now() + datetime.timedelta(minutes = 15) \
        and not(DISABLE_ROS)\
        and SENDEN\
        and ros_controller.status != "ACTIVE"\
        and rsp.current_status()["ROBOT_STATUS"] == "WAITING":
            
            #ゴール設定
            if ros_controller.set_goal(next_point) > 0:
                if next_point == "散歩1":
                    next_point = "散歩2"
                else:
                    next_point = "散歩1"
                    
                print(f"[INFO][main.py] : ロボットが散歩中です...")
            else:
                time.sleep(1.5)
                print(f"[FATAL][main.py][ID:{id}]  : 移動を開始できませんでした", file = sys.stderr)