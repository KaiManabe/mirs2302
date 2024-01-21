import order_mng as om
import sys
import hashlib
import robot_state_publisher
import time
import sock


USEHASH = False

def get_hash(s: str):
    return hashlib.sha256(s.encode()).hexdigest()

o = om.order_manager()

def auth(PASS):
    MODE = "PICKUP"
    order = o.get_order("STATUS", "WAITING_FOR_" + MODE)
    if type(order) == int and order == -1:
        MODE = "RECEIVE"
        order = o.get_order("STATUS", "WAITING_FOR_" + MODE)
        if type(order) == int and order == -1:
            print("[ERR][authentication.py] : 集荷・受取待ち状態のオーダが存在しません")
            sys.exit(-1)

    order = order.iloc[0]
    if MODE == "PICKUP":
        if USEHASH:
            if get_hash(PASS) == tostr(order["PICKUP_PIN"]):
                return True
            else:
                return False
        else:
            if PASS == tostr(order["PICKUP_PIN"]):
                return True
            else:
                return False
    elif MODE == "RECEIVE":
        if USEHASH:
            if get_hash(PASS) == tostr(order["RECEIVE_PIN"]):
                return True
            else:
                return False
        else:
            if PASS == tostr(order["RECEIVE_PIN"]):
                return True
            else:
                return False
    
    return -1
            

def tostr(raw):
    truth = 0
    try:
        truth = str(int(float(raw))).zfill(4)
    except:
        truth = raw
    return truth


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        if auth(sys.argv[1]) == True:
            
            # モジュール用サーバに接続
            client = sock.sock_client("127.0.0.1", 56674)
            client.send([1])
            
            # ドアが開くまで待機
            while True:
                if client.read() == [1]:
                    robot_state_publisher.update("DOOR", "OPEN")
                    break
                time.sleep(1)
            print("SuccessfullyOpenedDoor")
    
    else:
        print("usage : python3 authentication.py PASSWORD")