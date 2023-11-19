import serial_com as ser
import time
import sys
import threading
import run_ctrl as controller


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
    
    current_param = getgain(output = False)
    
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
        if getgain(output = False) != current_param:
            print("[INFO][setgain()] : ゲインの変更に成功しました")
            break
        if retries > 5:
            print("[ERROR][setgain()] : ゲインの変更に失敗しました")
            break
    


if __name__ == "__main__":
    s = ser.arduino_serial()
    ctrl = controller.run_controller(s)
    