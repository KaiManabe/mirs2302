import sock
import sys
import time
import threading

class raspi_to_ros():
    def __init__(self, serv:sock.sock_server):
        self.serv = serv
        self.x = 0.0
        self.y = 0.0
        while(1):
            if serv.isconnected() > 0:
                break
        
        t = threading.Thread(target = self.position_monitor)
        t.setDaemon(True)
        #t.start()
        
    def set_goal(self, goal_index:int):
        if self.serv.isconnected() > 0:
            return self.serv.send([255,goal_index,254])
            
        else:
            print("[ERR][raspi_to_ros] : ros側のクライアントサーバが起動していません", file = sys.stderr)
            return -1
    
    def position_monitor(self):
        while(1):
            if self.serv.isconnected() > 0:
                if len(self.serv.buf) >= 6:
                    tmp = self.serv.read()
                    for i in range(len(tmp)-6 , -1, -1):
                        if tmp[i] == 255 and len(tmp) > i + 3 and tmp[i+5] == 254:
                            self.x = (tmp[i+1] * 254 + tmp[i+2]) / 100
                            self.y = (tmp[i+3] * 254 + tmp[i+4]) / 100
                
            time.sleep(1.0)
                        