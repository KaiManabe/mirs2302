import serial_com as ser
import time
import sys
import threading
import run_ctrl as controller
import sock
import config

def getgain(s:ser.arduino_serial, output:bool = True):
    """
    ゲインを取得する関数

    引数：
        arduino_serialクラスオブジェクト
        
        output(任意) -> bool

    戻り値
        ゲインの可変長リスト
        
        失敗した場合 -1
    """
    pid = [[0,0,0],[0,0,0],[0,0,0],0]
    while(1):
        s.read()
        s.send([255,4,254])
        time.sleep(0.1)
        response = s.read()
        resp_int = []
        for resp in response:
            resp_int.append(int.from_bytes(resp, byteorder=sys.byteorder))
        
        if len(resp_int) >= 18:
            break
    
    if resp_int[0] != 255 or resp_int[1] != 13 or resp_int[-1] != 254:
        print("異常なデータを受け取りました")
        return -1
    else:
        arr = resp_int[2:-1]
    
    if(output):
        print("    P               I               D")
    
    for i in range(3):
        for ii in range(3):
            value = arr[i * 6 + ii * 2] * 254 + arr[i * 6 + ii * 2 + 1]
            value /= 10000.0
            
            if(output):
                print(str(value).ljust(9,"0"), end = "  ,  ")
            
            pid[i][ii] = value
        
        if(output):
            print("")
        
    if(output):
        print("\ndt : ", arr[-1])
    
    pid[-1] = arr[-1]
    return pid
    

def setgain(s: ser.arduino_serial, LR:str, PID:str, value:float):
    """
    ゲインを変更する関数

    引数：
        arduino_serialクラスオブジェクト
        
        "L" or "R" or "LR" -> string
        
        "P" or "I" or "D" -> string
        
        value -> float

    戻り値
        なし
    """
    
    current_param = getgain(s, output = False)
    
    hb = int(value * 10000 / 254)
    lb = int(value * 10000) - (hb * 254)
    
    if LR == "l" or LR == "L":
        lr = 0
    elif LR == "r" or LR == "R":
        lr = 1
    elif LR == "lr" or LR == "LR":
        lr = 2
    
    if PID == "p" or PID == "P":
        pid = 0
    elif PID == "i" or PID == "I":
        pid = 1
    elif PID == "D" or PID == "d":
        pid = 2
    
    retries = 0
    while(1):
        s.send([255, 8 ,lr , pid, hb, lb])
        time.sleep(0.25)
        retries += 1
        if getgain(s, output = False) != current_param:
            print("[INFO][setgain()] : ゲインの変更に成功しました")
            break
        if retries > 5:
            print("[ERROR][setgain()] : ゲインの変更に失敗しました")
            break
    

def setparam(s:ser.arduino_serial, dt:int):
    """
    パラメータを変更する関数

    引数：
        arduino_serialクラスオブジェクト
        
        dt -> int

    戻り値
        なし
    """
    
    current_param = getgain(s, output = False)
    
    if dt < 1 or dt >= 254:
        print("[ERROR][setparam()] : パラメータの値が不正です")
        return -1
    
    retries = 0
    while(1):
        s.send([255, 9 ,0 , dt, 254])
        time.sleep(0.25)
        retries += 1
        if getgain(s, output = False) != current_param:
            print("[INFO][setparam()] : パラメータの変更に成功しました")
            break
        if retries > 5:
            print("[ERROR][setparam()] : パラメータの変更に失敗しました")
            break


def receive_enc(bytes_arr):
    """
    bytes型の走行データ(エンコーダ値)をintに変換する関数
    
    引数 : 
        bytes_arr : s.read()の値をそのまま渡す
        
    戻り値：
        l : 左エンコーダの値
        r : 右エンコーダの値
    """
    int_arr = []
    
    for b in bytes_arr:
        int_arr.append(int.from_bytes(b, byteorder=sys.byteorder))
                       
    r = []
    l = []
    
    for i in range(len(int_arr)):
        if(int_arr[i] == 255):
            if len(int_arr) < i + 10:
                continue
            
            if(int_arr[i+1] == 14 and int_arr[i+10] == 254):
                val = -2081157128
                for ii in range(4):
                    val += int_arr[i+2+ii] * pow(254,ii)
                r.append(val)
                
                
                val = -2081157128
                for ii in range(4):
                    val += int_arr[i+6+ii] * pow(254,ii)
                l.append(val)
    
    return l, r


def receive_enc_target(bytes_arr):
    """
    bytes型の走行データ(エンコーダ値の目標値)をintに変換する関数
    
    引数 : 
        bytes_arr : s.read()の値をそのまま渡す
        
    戻り値：
        l : 左エンコーダの目標値
        r : 右エンコーダの目標値
    """
    int_arr = []
    
    for b in bytes_arr:
        int_arr.append(int.from_bytes(b, byteorder=sys.byteorder))
                       
    r = []
    l = []
    
    for i in range(len(int_arr)):
        if(int_arr[i] == 255):
            if len(int_arr) < i + 10:
                continue
            
            if(int_arr[i+1] == 15 and int_arr[i+10] == 254):
                val = -2081157128
                for ii in range(4):
                    val += int_arr[i+2+ii] * pow(254,ii)
                r.append(val)
                
                
                val = -2081157128
                for ii in range(4):
                    val += int_arr[i+6+ii] * pow(254,ii)
                l.append(val)
    
    return l, r


def record(s:ser.arduino_serial, speed:int, rectime:int):
    """
    エンコーダ値を監視しながらタイヤを動かす
    
    引数：
        arduino_serialクラスオブジェクト
        
        speed : 速度[mm/s] -> int
        
        rectime : 計測する時間[s] -> int
        
    戻り値：
        エンコーダ値のリスト
    """
    ctrl = controller.run_controller(s)
    s.read()
    s.send([255,6,2,254])
    ctrl.send_straight(speed)
    time.sleep(rectime)
    ctrl.send_straight(0)
    s.send([255,6,0,254])
    bytes_data = s.read()
    
    l_enc , r_enc = receive_enc(bytes_data)
    l_enc_target , r_enc_target = receive_enc_target(bytes_data)
    
    return [l_enc, l_enc_target, r_enc, r_enc_target]

#record()関数の戻り値をそのまま与えること.
def sendresult(result):
    #学内LANにサーバを公開
    serv = sock.sock_server(config.RASPI_IP_NCT,55555)
    while(1):
        if serv.isconnected() > 0:
            break
    
    for arr in result:
        serv.send([255])
        serv.send(arr)
        serv.send([254])
    

if __name__ == "__main__":
    s = ser.arduino_serial()
    ctrl = controller.run_controller(s)
    