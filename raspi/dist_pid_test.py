import run_ctrl as ctrl
import lidar
import serial_com as ser
import sys
import time

p = 0.015
i = 0.0025
d = 0.0025


def pid(c:ctrl.run_controller, l:lidar.lidar, spd:int):
    err_sum = 0
    err_prev = 0
    while(1):
        l_spd = spd
        r_spd = spd
        
        l_dist = 30000
        r_dist = 30000
        
        raw = l.getdata()
        ct = 0
        for i in range(len(raw)):
            if raw[-1 * i , 0] < 300 and raw[-1 * i , 0] > 240:
                if raw[-1 * i , 1] > 0 and raw[-1 * i , 1] < l_dist:
                    l_dist = raw[-1 * i , 1]
            
            if raw[-1 * i , 0] < 120 and raw[-1 * i , 0] > 60:
                if raw[-1 * i , 1] > 0 and raw[-1 * i , 1] < r_dist:
                    r_dist = raw[-1 * i , 1]

            if raw[-1 * i, 0] < raw[-1 * (i + 1), 0]:
                ct += 1
                if ct >= 2:
                    break
        
        if l_dist == 30000 or r_dist == 30000:
            continue
            
        err = r_dist - l_dist
        err_sum += err
        l_spd += ((err * p) + ((err - err_prev) * i) + (err_sum * d))
        r_spd -= ((err * p) + ((err - err_prev) * i) + (err_sum * d))
        
        if abs(l_spd) > 700:
            l_spd = 700
        
        
        if abs(r_spd) > 700:
            r_spd = 700
            
        if l_spd < 0 or r_spd < 0:
            continue
        
        c.set_l_speed(l_spd)
        c.set_r_speed(r_spd)
        print(f"\r l : {l_dist:4.1f} r : {r_dist:4.1f}   |   l : {l_spd:4.1f} r : {r_spd:4.1f}  |  err : {err:4.1f}" , end = "         ")
        time.sleep(0.05)
        err_prev = err



if __name__ == "__main__":
    s = ser.arduino_serial()
    c = ctrl.run_controller(s)
    l = lidar.lidar()
    
    pid(c, l, 400)
    c.send_straight(0)
    time.sleep(1)
    c.send_rotate(180, 60)