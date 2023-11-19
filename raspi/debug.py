import serial_com as ser
import time
import sys
import threading


s = ser.arduino_serial()

def getgain():
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
    
    print("    P               I               D")
    for i in range(3):
        for ii in range(3):
            value = arr[i * 6 + ii * 2] * 254 + arr[i * 6 + ii * 2 + 1]
            value /= 10000.0
            print(str(value).ljust(9,"0"), end = "  ,  ")
        print("")
        
    print("\ndt : ", arr[-1])
    

def setgain(LR, PID, value):
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
    
    s.send([255, 8 ,lr , pid, hb, lb])
    print("GAIN CHANGED.")
    


def sendstraight(spd):
    if spd > 0:
        dir = 0
    else:
        dir = 1
    hb = int(abs(spd) / 254)
    lb = int(abs(spd) % 254)
    s.send([255,1,dir, hb, lb, 254])
    s.send([255,2,dir, hb, lb, 254])
    


def sendrotate(spd):
    if spd > 0:
        dir = 0
        dir2=1
    else:
        dir = 1
        dir2 = 0
    hb = int(abs(spd) / 254)
    lb = int(abs(spd) % 254)
    s.send([255,1,dir, hb, lb, 254])
    s.send([255,2,dir2, hb, lb, 254])
    

def yukkuri():
    for i in range(11):
        sendstraight( i*30)
        time.sleep(0.1)
    sendstraight(300)
    time.sleep(5)
    for i in range(11):
        sendstraight(300 - i*30)
        time.sleep(0.1)
        
def servo():
    kai = [255,10,254]
    s.send(kai)
    print('send:'+str(kai))



def receive_enc(bytes_arr):
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
    
def rec():
  s.read()
  sendstraight(300)
  time.sleep(2)
  sendstraight(0)
  return s.read()

if __name__ == "__main__":
    #servo()
    sendstraight(100)
    time.sleep(1)
    sendstraight(0)