import sock
import matplotlib.pyplot as plt
import config
import subprocess as sp
import sys
import time
import numpy as np
import threading




class lidar():
    """
    コンストラクタ
    
    引数：
        なし
        
    戻り値：
        なし
    """
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
        
        self.buf = np.zeros([0,3])
        time.sleep(0.5)
        t = threading.Thread(target = monitor, args = (self,))
        t.setDaemon(True)
        t.start()
    
    
    def getdata(self, angle = None):
        """
        LiDARの測距データを取得する関数
        angleに0および360付近の値を指定するとバグるので修正が必要
        
        引数：
            angle -> float : 指定すると指定した角度での距離を返す・指定がない場合はバッファをそのまま返す
            
        戻り値：
            なし
        """
        if angle == None:
            return self.buf
        
        for i in range(1,len(self.buf) - 2):
            dist = -1
            if self.buf[-1 * i, 0] > angle and self.buf[-1 * (i + 1) , 0] < angle:
                if self.buf[-1 * (i + 1) , 1] == 0:
                    continue
                else:
                    dist = self.buf[-1 * (i + 1) , 1]
                    break
        
        return dist
    

    
    
    def close(self):
        """
        jetsonとの通信を切断する
        """
        self.server.send([255,9,254])
        time.sleep(0.25)
        self.server.server.close()
        print("[INFO][lidar.py] : jetsonとの通信を終了しました")
    
    
    



def monitor(l:lidar):
    print("[INFO][lidar.py] : 測距データのバッファリングを開始しました")
    while(1):
        l.server.send([255,3,254])
        while(1):
            if l.server.buffer_length() > 2000:
                break
        
        received_data = l.server.read()
        l.buf = np.append(l.buf, convertdata(received_data), axis = 0)
        if len(l.buf) > config.LIDAR_CLASS_BUFFER_MAX_LENGTH:
            l.buf = l.buf[-1 * config.LIDAR_CLASS_BUFFER_MAX_LENGTH:]

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