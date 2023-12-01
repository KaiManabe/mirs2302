import sock
import matplotlib.pyplot as plt
import config
import subprocess as sp
import sys
import time
import numpy as np




class lidar():
    def __init__(self):
        print("[INFO][lidar.py] : jetsonのデーモンプロセスをキルしています...")
        sp.run(["ssh",
                "-i",
                "/home/pi/.ssh/id_rsa_jetson",
                ("mirs2302@" + config.JETSON_IP),
                "sh",
                "/home/mirs2302/git/mirs2302/jetson/terminate_screen.sh"])
        time.sleep(1)
        self.server = sock.sock_server(config.RASPI_IP, config.SOCKET_PORT)
        while(1):
            if(self.server.server_started == True):
                break
        
        print("[INFO][lidar.py] : jetsonとのソケット通信を確立しています...")
        sp.run(["ssh",
                "-i",
                "/home/pi/.ssh/id_rsa_jetson",
                ("mirs2302@" + config.JETSON_IP),
                "sh",
                "/home/mirs2302/git/mirs2302/jetson/start_lidar.sh"])
        
        while(1):
            if (self.server.isconnected() > 0):
                break
        
        self.buf = []
        time.sleep(0.5)
    
    
    def getdata(self):
        return self.buf
    
    
    def close(self):
        self.server.send([255,9,254])
        time.sleep(0.25)
        self.server.server.close()
        print("[INFO][lidar.py] : jetsonとの通信を終了しました")
    
    
    



def monitor(l:lidar):
    while(1):
        l.server.read()
        l.server.send([255,3,254])
        while(1):
            if l.server.buffer_length() > 100:
                break
            time.sleep(0.1)
        
        received_data = l.server.read()
        l.buf = convertdata(received_data)
        
        time.sleep(0.1)
   

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
           


if __name__ == "__main__":
    l = lidar()