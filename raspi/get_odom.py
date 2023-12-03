import serial_com as ser
import run_ctrl as ctrl
import datetime
import numpy as np
import threading
import sys
import time



#arduinoと定数を合わせること
PI = 3.1415926536
ENC_PPR = 26
GEAR_RATIO = 13.733564
TIRE_GEAR_TOOTH = 20 
MOTOR_GEAR_TOOTH = 10 
TIRE_DIAM = 177  
TIRE_PITCH = 555
ODOM_DT = 50




class odom():
    def __init__(self, ser:ser.arduino_serial):
        self.s = ser
        
        self.theta = [0, 0]
        self.omega = [0, 0]
        self.robot_verocity = 0.0
        self.robot_theta = 0.0
        self.robot_omega = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        while(1):
            if len(self.s.buffer[15]) >= 2:
                break
            self.s.send(6,[3])
            time.sleep(0.1)
        print("[INFO][get_odom.py] : arduinoからのデータ取得を開始しました")
        
        t = threading.Thread(target = get_odom, args=(self,))
        t.setDaemon(True)
        t.start()
        
        

def get_odom(o:odom):
    ctime = datetime.datetime.now()
    while(1):
        while(1):
            if o.s.last_update[15] > ctime and len(o.s.buffer[15]) >= 2:
                ctime = o.s.last_update[15]
                break
            
        resp = o.s.read(15)[-2]
        enc_l = -8193532
        enc_r = -8193532
        d_enc_l = -32258
        d_enc_r = -32258
        
        for i in range(3):
            enc_l += resp[i] * pow(254, i) 
            enc_r += resp[i + 3] * pow(254, i)
        
        for i in range(2):
            d_enc_l += resp[i + 6] * pow(254, i) 
            d_enc_r += resp[i + 8] * pow(254, i)
        
        vr = pulse_to_mm(d_enc_r) / (ODOM_DT / 1000)
        vl = pulse_to_mm(d_enc_l) / (ODOM_DT / 1000)
        
        o.robot_verocity = (vr + vl) / 2
        o.robot_omega = (vr - vl) / (TIRE_PITCH)
        
        
        rr = pulse_to_mm(enc_r)
        rl = pulse_to_mm(enc_l)
        
        o.robot_theta = (rr - rl) / (TIRE_PITCH)
        
        o.robot_x += o.robot_verocity * np.cos(o.robot_theta) * (ODOM_DT / 1000)
        o.robot_y += o.robot_verocity * np.sin(-1 * o.robot_theta) * (ODOM_DT / 1000)
        

def p(o:odom):
    while(1):
        print(f"\r v = {o.robot_verocity:4.1f}, theta = {o.robot_theta:4.1f}, omega = {o.robot_omega:4.1f}, x = {o.robot_x:4.1f}, y = {o.robot_y:4.1f}" ,end = "         ")


def pulse_to_mm(p):
    return (p * TIRE_DIAM * PI * MOTOR_GEAR_TOOTH) / (TIRE_GEAR_TOOTH * ENC_PPR * GEAR_RATIO)


if __name__ == "__main__":
    s = ser.arduino_serial()
    c = ctrl.run_controller(s)
    o = odom(s)
    
    t = threading.Thread(target = p, args = (o,))
    t.setDaemon(True)
    t.start()
    
    c.keyboard_controller(output=False)
    
    