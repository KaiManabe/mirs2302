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
GOALS = {"D科棟" : 0,
         "学生係" : 1
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
            return self.serv.send([255,GOALS[goal],254])
        else:
            print("[ERR][raspi_to_ros] : ros側のクライアントサーバが起動していません", file = sys.stderr)
            return -1
    
    def status_monitor(self):
        while(1):
            if self.serv.isconnected() > 0 and self.serv.buffer_length() > 0:
                st = self.serv.read()
                self.status = STATUS[st[-1]]
            time.sleep(1)
    
    
    def movement_monitor(self):
        while(1):
            time.sleep(0.2)
            if self.status == "ACTIVE":
                break
        print(f"[INFO][raspi_to_ros] : ロボットが走行中です... ROSステータス : {self.status}")
        while(1):
            time.sleep(0.2)
            if self.status == "SUCCEEDED":
                print(f"[INFO][raspi_to_ros] : 目的地{idx}に到着しました ROSステータス : {self.status}")
                break
            elif self.status != "ACTIVE":
                print(f"[INFO][raspi_to_ros] : 目的地{idx}に到着できませんでした ROSステータス : {self.status}")
                sys.exit(1)