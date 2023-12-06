import Jetson.GPIO as g
import threading
import time


class enc():
    def __init__(self, dt = 20):
        """
        jetsonのGPIOからエンコーダ値を監視するクラス
        
        引数：
            dt(任意) : 速度を求めるときのΔt
        """
        g.setmode(g.BOARD)

        self.dt = dt
        self.ctime = time.time()
        self.PINASSIGN = {"LA" : 35, "LB" : 36, "RA" : 32, "RB" : 33}
        for label in self.PINASSIGN:
            g.setup(self.PINASSIGN[label], g.IN)


        self.prev = {"LA" : 0, "LB" : 0, "RA" : 0, "RB" : 0}
        self.current = {"LA" : 0, "LB" : 0, "RA" : 0, "RB" : 0}
        self.pulse_L = 0
        self.pulse_L_prev = 0
        self.pulse_L_d = 0
        self.pulse_R = 0
        self.pulse_R_prev = 0
        self.pulse_R_d = 0
    
    def start_monitor(self):
        t = threading.Thread(target = monitor, args = (self,))
        t.setDaemon(True)
        t.start()
        
        
    def reset(self):
        self.pulse_L = 0
        self.pulse_L_prev = 0
        self.pulse_L_d = 0
        self.pulse_R = 0
        self.pulse_R_prev = 0
        self.pulse_R_d = 0


    
def monitor(e:enc):
    while(1):
        for label in e.PINASSIGN:
            e.current[label] = g.input(e.PINASSIGN[label])
            
        if e.current["LA"] == 1 and e.prev["LA"] == 0:
            e.prev["LA"] = e.current["LA"]
            if e.current["LB"] == 0:
                e.pulse_L += 1
            else:
                e.pulse_L -= 1
                
        if e.current["LB"] == 1 and e.prev["LB"] == 0:
            e.prev["LB"] = e.current["LB"]
            if e.current["LA"] == 1:
                e.pulse_L += 1
            else:
                e.pulse_L -= 1
            
            
        if e.current["LA"] == 0 and e.prev["LA"] == 1:
            e.prev["LA"] = e.current["LA"]
            if e.current["LB"] == 1:
                e.pulse_L += 1
            else:
                e.pulse_L -= 1
                
        if e.current["LB"] == 0 and e.prev["LB"] == 1:
            e.prev["LB"] = e.current["LB"]
            if e.current["LA"] == 0:
                e.pulse_L += 1
            else:
                e.pulse_L -= 1
            
            
                
            
        if e.current["RA"] == 1 and e.prev["RA"] == 0:
            e.prev["RA"] = e.current["RA"]
            if e.current["RB"] == 1:
                e.pulse_R += 1
            else:
                e.pulse_R -= 1
                
        if e.current["RB"] == 1 and e.prev["RB"] == 0:
            e.prev["RB"] = e.current["RB"]
            if e.current["RA"] == 0:
                e.pulse_R += 1
            else:
                e.pulse_R -= 1
            
            
        if e.current["RA"] == 0 and e.prev["RA"] == 1:
            e.prev["RA"] = e.current["RA"]
            if e.current["RB"] == 0:
                e.pulse_R += 1
            else:
                e.pulse_R -= 1
                
        if e.current["RB"] == 0 and e.prev["RB"] == 1:
            e.prev["RB"] = e.current["RB"]
            if e.current["RA"] == 1:
                e.pulse_R += 1
            else:
                e.pulse_R -= 1
        
        if time.time() >= e.ctime + (e.dt / 1000):
            e.pulse_L_d = e.pulse_L - e.pulse_L_prev
            e.pulse_R_d = e.pulse_R - e.pulse_R_prev
            e.pulse_L_prev = e.pulse_L
            e.pulse_R_prev = e.pulse_R
            e.ctime = time.time()
    

if __name__ == "__main__":
    e = enc(dt = 100)
    e.start_monitor()
    
    while(1):
        print(f"\r L : {e.pulse_L} ,  R : {e.pulse_R} ,  dL : {e.pulse_L_d} ,  dL : {e.pulse_R_d} ", end = "     ")