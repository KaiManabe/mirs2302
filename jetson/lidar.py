import sock
import matplotlib.pyplot as plt
import config
import subprocess as sp
import sys
import time
import numpy as np
import datetime

def getdata():
    s.read()
    s.send([255,3,254])
    while(1):
        if s.buffer_length() > 10:
            break
        time.sleep(0.1)
    
    return s.read()

def convertdata(received_data):
    out = np.zeros([int(len(received_data) / 10), 3])
    idx = 0
    for i in range(len(received_data)):
        if (i+9) >= len(received_data):
            break
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
    received_data = []
    while(1):
        received_data = getdata()
        if len(received_data) > 10:
            break
        time.sleep(0.5)
    nparr = convertdata(received_data)
    plotter(nparr)


def logger():
    out = np.zeros([10,4096,3])
    for i in range(10):
        a = np.zeros([4096,3])
        rawdata = convertdata(getdata())
        a[:len(rawdata)] = rawdata
        out[i] = a
    fname = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    np.save("/home/mirs2302/git/lidar_log/"+fname, out)


if __name__ == "__main__":
    print("[INFO][lidar.py] : jetsonのデーモンプロセスをキルしています...")
    sp.run(["sh", "/home/mirs2302/git/mirs2302/jetson/terminate_screen.sh"])
    time.sleep(1)
    s = sock.sock_server("127.0.0.1", config.CPP_PYTHON_SOCKET_PORT)
    while(1):
        if(s.server_started == True):
            break
    
    print("[INFO][lidar.py] : ultra_simpleとのソケット通信を確立しています...")
    sp.run(["/home/mirs2302/git/mirs2302/jetson/start_lidar.sh"])
    
    while(1):
        if (s.isconnected() > 0):
            break
    
    time.sleep(1)
    logger()
    a = convertdata(getdata())