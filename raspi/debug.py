import serial_com as ser
import run_ctrl as ctrl
import config 
import time
import sys
import threading


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
    


if __name__ == "__main__":
    s = ser.arduino_serial()
    c = ctrl.run_controller(s)