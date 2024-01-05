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


#デバッグ用
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
    
    #モジュール管理用クラス
    module_manager = module_mng.module_controller(serial_port)
    
    #機体管理用クラス
    airframe_manager = module_mng.airframe_controller(serial_port)
    
    #メール送信用クラス
    mail = web_app.mails(order_manager)
    
    
    print(f"[INFO][main.py] : 初期化終了 メインループを開始します")
    
    #1ループの処理時間は短めにすること
    while(1):
        for i in range(len(order_manager.df)):
            current_order = order_manager.get_order(i)
            id = current_order["ID"]
            status = current_order["STATUS"]
            
#*****************************************************************************************************
#状態    NOT_ACCEPTED_YET
#*****************************************************************************************************
            if status == "NOT_ACCEPTED_YET":
#メールを送る処理
                # web_app.approval(id)
                order_manager.modify_order(id, "STAUTS", "MAIL_SENT")
            
            
#*****************************************************************************************************
#状態    MAIL_SENT
#*****************************************************************************************************        
            elif status == "MAIL_SENT":
#条件文                     ↓
                if """メールのレスポンスがあったか？""":
#条件文                     ↓
                    if """承認されたか？""":
                        order_manager.modify_order(id, "STAUTS", "ACCEPTED")
                        order_manager.modify_order(id, "ACCEPTED_TIME", datetime.datetime.now())
#承認されましたメール送信
                    else:#拒否された
                        order_manager.modify_order(id, "STAUTS", "DENIED")
#拒否されましたメール送信
                else:#レスポンスなし
#条件文                     ↓
                    if """タイムアウトしてるか？""":
                        order_manager.modify_order(id, "STAUTS", "ACCEPT_TIMEOUT")
#タイムアウトしたので拒否しますメール送信
            
            
            
#*****************************************************************************************************
#状態    ACCEPTED
#*****************************************************************************************************
            elif status == "ACCEPTED":
                #今のところ10分前に移動させるようにしているが調整するならどうにかする
                if datetime.datetime.now() >= current_order["PICKUP_TIME"] - datetime.timedelta(minutes = 10):
                    #ゴール設定
                    ros_controller.set_goal(current_order["PICKUP_PLACE"])
                    
                    #走り出すまで待機
                    time.sleep(0.5)
                    for i in range(3):
                        if ros_controller.status == "ACTIVE":
                            print(f"[INFO][main.py][ID:{id}] : ロボットが走行中です...")
                            order_manager.modify_order(id, "STAUTS", "MOVING_FOR_PICKUP")
                            rsp.update("ROBOT_STATUS", "MOVING")
                            rsp.update("GOAL", current_order['PICKUP_PLACE'])
                            rsp.update("DOOR_NUM", current_order['ITEM_TYPE'])
#今向かってますてきなメールあれば◎  やらないなら無視
                            break
                        else:
                            time.sleep(5)
                    if i == 2:
                        print(f"[FATAL][main.py][ID:{id}]  : 移動を開始できませんでした ROSステータス : {ros_controller.status}", file = sys.stderr)
                    
                    
            
#*****************************************************************************************************
#状態    MOVING_FOR_PICKUP
#*****************************************************************************************************                
            elif status == "MOVING_FOR_PICKUP":
                if ros_controller.stauts == "SUCCEEDED":
                    print(f"[INFO][main.py][ID:{id}]  : ロボットが{current_order['PICKUP_PLACE']}に到着")
                    order_manager.modify_order(id, "STAUTS", "WAITING_FOR_PICKUP")
                    rsp.update("ROBOT_STATUS", "WAITING_FOR_PICKUP")
#到着しましたてきなメールあれば◎  やらないなら無視
                else:
                    if ros_controller.stauts == "ACTIVE":#まだ動いているなら
                        if datetime.datetime.now() >= current_order["PICKUP_TIME"] + datetime.timedelta(minutes = 5):
                            print(f"[FATAL][main.py][ID:{id}]  : ロボットがスタックしている可能性があります ROSステータス : {ros_controller.status}", file = sys.stderr)
#スタックしてる場合にバックさせる
                    else:#動いてないし到着してない
                        print(f"[FATAL][main.py][ID:{id}]  : ロボットが到着できませんでした リトライします ROSステータス : {ros_controller.status}", file = sys.stderr)
                        order_manager.modify_order(id, "STAUTS", "ACCEPTED")
            
            
#*****************************************************************************************************
#状態    WAITING_FOR_PICKUP
#***************************************************************************************************** 
            elif status == "WAITING_FOR_PICKUP":
#条件文                     ↓
                if """ユーザきたか？""":
                    order_manager.modify_order(id, "STAUTS", "PICKED_UP")
                else:#きてない
                    if datetime.datetime.now() >= current_order["PICKUP_TIME"] + datetime.timedelta(minutes = 15):
                        order_manager.modify_order(id, "STAUTS", "PICKUP_TIMEOUT")
#タイムアウトしたので拒否しますメール送信
            
            
#*****************************************************************************************************
#状態    PICKED_UP
#***************************************************************************************************** 
            elif status == "PICKED_UP":
#ラボ前に帰る?　予約がどれだけの頻度で入っているかによる
                #今のところ10分前に移動させるようにしているが調整するならどうにかする
                if datetime.datetime.now() >= current_order["RECEIVE_TIME"] - datetime.timedelta(minutes = 10):
                    #ゴール設定
                    ros_controller.set_goal(current_order["RECEIVE_PLACE"])
                    
                    #走り出すまで待機
                    time.sleep(0.5)
                    for i in range(3):
                        if ros_controller.status == "ACTIVE":
                            print(f"[INFO][main.py][ID:{id}] : ロボットが走行中です...")
                            order_manager.modify_order(id, "STAUTS", "MOVING_FOR_PICKUP")
                            rsp.update("ROBOT_STATUS", "MOVING")
                            rsp.update("GOAL", current_order['RECEIVE_PLACE'])
                            rsp.update("DOOR_NUM", current_order['ITEM_TYPE'])
#今向かってますてきなメールあれば◎  やらないなら無視
                            break
                        else:
                            time.sleep(5)
                    if i == 2:
                        print(f"[FATAL][main.py][ID:{id}]  : 移動を開始できませんでした ROSステータス : {ros_controller.status}", file = sys.stderr)



            
#*****************************************************************************************************
#状態    MOVING_FOR_RECEIVE
#*****************************************************************************************************                
            elif status == "MOVING_FOR_RECEIVE":
                if ros_controller.stauts == "SUCCEEDED":
                    print(f"[INFO][main.py][ID:{id}]  : ロボットが{current_order['RECEIVE_PLACE']}に到着")
                    order_manager.modify_order(id, "STAUTS", "WAITING_FOR_RECEIVE")
                    rsp.update("ROBOT_STATUS", "WAITING_FOR_RECEIVE")
#到着しましたてきなメールあれば◎  やらないなら無視
                else:
                    if ros_controller.stauts == "ACTIVE":#まだ動いているなら
                        if datetime.datetime.now() >= current_order["RECEIVE_TIME"] + datetime.timedelta(minutes = 5):
                            print(f"[FATAL][main.py][ID:{id}]  : ロボットがスタックしている可能性があります ROSステータス : {ros_controller.status}", file = sys.stderr)
#スタックしてる場合にバックさせる
                    else:#動いてないし到着してない
                        print(f"[FATAL][main.py][ID:{id}]  : ロボットが到着できませんでした リトライします ROSステータス : {ros_controller.status}", file = sys.stderr)
                        order_manager.modify_order(id, "STAUTS", "PICKED_UP")
            
            
            
#*****************************************************************************************************
#状態    WAITING_FOR_RECEIVE
#***************************************************************************************************** 
            elif status == "WAITING_FOR_RECEIVE":
#条件文                     ↓
                if """ユーザきたか？""":
                    order_manager.modify_order(id, "STAUTS", "RECEIVED")
                else:#きてない
                    if datetime.datetime.now() >= current_order["RECEIVE_TIME"] + datetime.timedelta(minutes = 15):
                        order_manager.modify_order(id, "STAUTS", "RECEIVE_TIMEOUT")
#タイムアウトしたので拒否しますメール送信

