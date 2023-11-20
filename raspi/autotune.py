import serial_com as ser
import time
import sys
import threading
import run_ctrl as controller
import sock
import config
import tuning
import numpy as np


current_gain = [[0.1, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.0, 0.0], 25]
STEP = [0.1, 0.01, 0.01]
SPEED = 500
RECTIME = 5


def loss(data):
    """
    損失を求める関数
    
    引数：
        data : 左のデータ、左の指令値、右のデータ、右の値を含む2次元配列 -> list
    
    戻り値：
        l, r 誤差の積分 -> tuple
    """
    
    imax = max([len(data[0]),
                len(data[1]),
                len(data[2]),
                len(data[3])])
    
    l_loss = 0
    r_loss = 0
    
    for i in range(imax):
        if data[1][i] != 0:
            l_loss += (data[0][i] - data[1][i])
        
        if data[3][i] != 0:
            r_loss += (data[2][i] - data[3][i])
    
    return l_loss, r_loss



def setgain_arr(s, gain_arr):
    pid = ["P", "I", "D"]
    current = tuning.getgain(s)
    
    for i in range(3):
        if current[0][i] != gain_arr[0][i]:
            tuning.setgain(s, "L", pid[i], gain_arr[0][i])
            
        if current[1][i] != gain_arr[1][i]:
            tuning.setgain(s, "R", pid[i], gain_arr[1][i])



def autotune(s):
    res = np.zeros([0,5])
    gain = current_gain
    for p in range(1,6):
        gain[0][0] = STEP[0] * p
        gain[1][0] = STEP[0] * p
        for i in range(1,6):
            gain[0][1] = STEP[1] * i
            gain[1][1] = STEP[1] * i
            for d in range(1,6):
                gain[0][2] = STEP[2] * d
                gain[1][2] = STEP[2] * d
                loss_l, loss_r = execute(s,gain)
                app = np.array([gain[0][0], gain[0][1], gain[0][2], loss_l, loss_r])
                res = np.append(res, app.reshape(1,5))
    return res
                


def execute(s, gain_arr):
    print("\n[INFO][autotune.execute()] : ゲイン\n[INFO][autotune.execute()] : ",
          gain_arr, "による応答を計測します\n")
    
    setgain_arr(s, gain_arr)
    
    time.sleep(1)
    
    result = tuning.convert_data(tuning.record(s, SPEED, RECTIME))
    
    loss_l , loss_r = loss(result)
    
    print("\n[INFO][autotune.execute()] : loss_l -> ", loss_l)
    print("[INFO][autotune.execute()] : loss_r -> ", loss_r, "\n")
    
    return loss_l, loss_r
    

if __name__ == "__main__":
    s = ser.arduino_serial()