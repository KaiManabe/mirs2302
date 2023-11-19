import sock
import matplotlib as plt
import config
import subprocess as sp
import sys
import time
import numpy as np


s = sock.sock_server(config.RASPI_IP, config.SOCKET_PORT)

def getdata():
    s.send([255,3,254])
    time.sleep(0.25)
    return s.read()

def convertdata(received_data):
    out = np.zeros([int(len(received_data) / 10), 3])
    idx = 0
    for i in range(len(received_data)):
        if received_data[i] == 255 and received_data[i+9] == 254:
            if received_data[i+1] == 11:
                for ii in range(3):
                    out[idx,0] += received_data[i+2+ii] * np.power(254,i) / 1000.0
                    out[idx,1] += received_data[i+5+ii] * np.power(254,i)
                out[idx,2] = received_data[i+8]
                
                idx += 1
    
    return out[:idx]
                
