import sock
import matplotlib.pyplot as plt
import config
import subprocess as sp
import sys
import time
import numpy as np


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
                    out[idx,0] += received_data[i+2+ii] * np.power(254,ii) / 1000.0
                    out[idx,1] += received_data[i+5+ii] * np.power(254,ii)
                out[idx,2] = received_data[i+8]
                
                idx += 1
    
    return out[:idx]
                
def disconnect():
    s.send([255,9,254])
    time.sleep(0.25)
    s.server.close()


def plotter(nparr):
    xy = np.zeros([len(nparr), 2])
    for i in range(len(nparr)):
        xy[i,0] = -1 * nparr[i,1] * np.cos(nparr[i,0] / 180.0 * np.pi)
        xy[i,1] = nparr[i,1] * np.sin(nparr[i,0] / 180.0 * np.pi)
    
    plt.scatter(xy[:,0], xy[:,1])
    plt.scatter([0],[0], c = "red", marker = "x")
    plt.show()

def p():
    s.read()
    plotter(convertdata(getdata()))

if __name__ == "__main__":
    s = sock.sock_server(config.RASPI_IP, config.SOCKET_PORT)
    time.sleep(1)
    sp.run(["ssh",
            "-i",
            "/home/pi/.ssh/id_rsa_jetson",
            "mirs2302@192.168.1.3",
            "sh",
            "/home/mirs2302/git/mirs2302/jetson/start_lidar.sh"])
    
    while(s.isconnected() < 1):
        time.sleep(0.1)
    
    time.sleep(1)
    
    p()