import sock
import sys
import time
import threading

STATUS = [
    "PENDING",
    "ACTIVE",
    "PREEMPTED",
    "SUCCEEDED",
    "ABORTED",
    "REJECTED",
    "PREEMPTING",
    "RECALLING",
    "RECALLED",
    "LOST"]



class raspi_to_ros():
    def __init__(self, serv:sock.sock_server):
        self.serv = serv
        self.status = ""
        while(1):
            if serv.isconnected() > 0:
                break
        
        t = threading.Thread(target = self.status_monitor)
        t.setDaemon(True)
        t.start()
        
    def set_goal(self, goal_index:int):
        if self.serv.isconnected() > 0:
            return self.serv.send([255,goal_index,254])
            
        else:
            print("[ERR][raspi_to_ros] : ros側のクライアントサーバが起動していません", file = sys.stderr)
            return -1
    
    def status_monitor(self):
        while(1):
            if self.serv.isconnected() > 0 and self.serv.buffer_length() > 0:
                st = self.serv.read()
                self.status = STATUS[st[-1]]
            time.sleep(1)