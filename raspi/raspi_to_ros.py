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


#ゴールのindexをjetsonと合わせる
"""
0:ラボ
1:印刷室
2:学生課
3:E科
4:S科
5:C科
"""

GOALS = {"D科棟" : 0,
         "印刷室" : 1,
         "学生課" : 2,
         "E科棟" : 3,
         "S科棟" : 4,
         "C科棟" : 5,
         "散歩1" : 6,
         "散歩2" : 7
         }



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
        
    def set_goal(self, goal:str):
        
        if not(goal in GOALS.keys()):
            print("[ERR][raspi_to_ros] : set_goalの引数が不正です", file = sys.stderr)
            return -1
        if self.serv.isconnected() > 0:
            print(f"[INFO][raspi_to_ros] : 目的地を{goal}に設定しました")
            self.serv.send([255,GOALS[goal],254])
            
            while(1):
                current = self.status
                ctime = time.time()
                while(1):
                    if self.status != current:
                        break
                    if time.time() > ctime + 10:
                        print(f"[ERR][raspi_to_ros] : ROSが応答していません  STATUS : {self.status}", file = sys.stderr)
                        return -1
                
                if self.status == "ACTIVE":
                    break
            return 1
        
        else:
            print("[ERR][raspi_to_ros] : ros側のクライアントサーバが起動していません", file = sys.stderr)
            return -1
    
    def status_monitor(self):
        while(1):
            if self.serv.isconnected() > 0 and self.serv.buffer_length() > 0:
                st = self.serv.read()
                self.status = STATUS[st[-1]]
            time.sleep(0.2)
    
    
    
