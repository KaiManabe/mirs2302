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
LR_P = pow(10, -7)
LR_I = pow(10, -7)
LR_D = pow(10, -7)

stop_sign = False

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
    
    return l_loss**2 , r_loss**2



def setgain_arr(s, gain_arr):
    pid = ["P", "I", "D"]
    current = tuning.getgain(s)
    
    for i in range(3):
        if current[0][i] != gain_arr[0][i]:
            tuning.setgain(s, "L", pid[i], gain_arr[0][i])
            
        if current[1][i] != gain_arr[1][i]:
            tuning.setgain(s, "R", pid[i], gain_arr[1][i])

def input_receiver():
    global stop_sign
    
    while(1):
        i = input()
        if i == "stop":
            stop_sign = True
        if i == "start":
            stop_sign = False
            

def grad(s, p, i, d, epsilonp = 0.05, epsiloni = 0.01, epsilond = 0.01):
    global stop_sign
    
    while(stop_sign):
        time.sleep(0.1)
    
    tuning.setgain(s, "L", "P", p)
    tuning.setgain(s, "R", "P", p)
    tuning.setgain(s, "L", "I", i)
    tuning.setgain(s, "R", "I", i)
    tuning.setgain(s, "L", "D", d)
    tuning.setgain(s, "R", "D", d)
    result = tuning.convert_data(tuning.record(s, SPEED, RECTIME))
    loss_l , loss_r = loss(result)
    loss_value = abs(loss_l) + abs(loss_r)

    
    while(stop_sign):
        time.sleep(0.1)
    
    tuning.setgain(s, "L", "P", p + epsilonp)
    tuning.setgain(s, "R", "P", p + epsilonp)
    result = tuning.convert_data(tuning.record(s, SPEED, RECTIME))
    loss_l , loss_r = loss(result)
    loss_value_d = abs(loss_l) + abs(loss_r)
    grad_p = loss_value_d - loss_value
    
    while(stop_sign):
        time.sleep(0.1)
    
    tuning.setgain(s, "L", "P", p)
    tuning.setgain(s, "R", "P", p)
    tuning.setgain(s, "L", "I", i + epsiloni)
    tuning.setgain(s, "R", "I", i + epsiloni)
    result = tuning.convert_data(tuning.record(s, SPEED, RECTIME))
    loss_l , loss_r = loss(result)
    loss_value_d = abs(loss_l) + abs(loss_r)
    grad_i = loss_value_d - loss_value
    
    while(stop_sign):
        time.sleep(0.1)
    
    tuning.setgain(s, "L", "I", i)
    tuning.setgain(s, "R", "I", i)
    tuning.setgain(s, "L", "D", d + epsilond)
    tuning.setgain(s, "R", "D", d + epsilond)
    result = tuning.convert_data(tuning.record(s, SPEED, RECTIME))
    loss_l , loss_r = loss(result)
    loss_value_d = abs(loss_l) + abs(loss_r)
    grad_d = loss_value_d - loss_value
    
    return [loss_value, grad_p, grad_i, grad_d]
    

def autotune(s):
    
    LOG_PATH = "/home/pi/git/autotune.log"
    first_gain = [0.1, 0.01, 0.01]
    
    p = first_gain[0]
    i = first_gain[1]
    d = first_gain[2]
    
    
    t = threading.Thread(target = input_receiver)
    t.setDaemon(True)
    t.start()
        
    
    while(1):
        
        grads = grad(s, p, i, d)
        with open(LOG_PATH, "a") as f:
            print(p, ",", i, ",", d, ",", grads[0], ",", grads[1], ",", grads[2], ",", grads[3], file = f)
        
        print(f"\n[INFO][autotune] : {p:.5f},{i:.5f},{d:.5f}における")
        print(f"[INFO][autotune] : 損失 = {grads[0]:10.5f}")
        print(f"[INFO][autotune] : grad_p = {grads[1]:10.5f}")
        print(f"[INFO][autotune] : grad_p = {grads[2]:10.5f}")
        print(f"[INFO][autotune] : grad_d = {grads[3]:10.5f}\n")
        
        p -= LR_P * grads[1]
        i -= LR_I * grads[2]
        d -= LR_D * grads[3]
        


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