import sys
import threading
import sock
import config
import keyboard
import time
import re


base_spd = 200

def input_monitor():
    global base_spd
    while(1):
        string = input()
        if "spd" in string:
            num = re.sub("\\D", "", string)
            print(f"\nset speed to {num}\n")
            base_spd = int(num)
        


def calcspeed():
    global base_spd
    spd_l = 0
    spd_r = 0
    if keyboard.is_pressed("w"):
        spd_l += base_spd
        spd_r += base_spd
        
    if keyboard.is_pressed("s"):
        spd_l -= base_spd
        spd_r -= base_spd
        
    if keyboard.is_pressed("a"):
        spd_r += base_spd * 0.5
        spd_l += -base_spd * 0.5
    
    if keyboard.is_pressed("d"):
        spd_r += -base_spd * 0.5
        spd_l += base_spd * 0.5
    
    return spd_l, spd_r

if __name__ == "__main__":
    c = sock.sock_client(config.RASPI_IP, 56565)
    t = threading.Thread(target=input_monitor)
    t.setDaemon(True)
    t.start()
    
    while(1):
        l, r = calcspeed()
        print(f"\r {l:3},  {r:3}")
        l += 1270
        l = int(l / 10)
        r += 1270
        r = int(r / 10)
        
        c.send([l, r, 255])
        
        time.sleep(0.05)