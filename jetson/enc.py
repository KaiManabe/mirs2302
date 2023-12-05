import Jetson.GPIO as g
import threading

class enc():
    def __init__(self):
        g.setmode(g.BOARD)

        self.PINASSIGN = {"LA" : 35, "LB" : 36, "RA" : 32, "RB" : 33}
        for label in self.PINASSIGN:
            g.setup(self.PINASSIGN[label], g.IN)


        self.prev = {"LA" : 0, "LB" : 0, "RA" : 0, "RB" : 0}
        self.current = {"LA" : 0, "LB" : 0, "RA" : 0, "RB" : 0}
        self.pulse_L = 0
        self.pulse_R = 0
    
    def start_monitor(self):
        t = threading.Thread(target = monitor, args = (self,))
        t.setDaemon(True)
        t.start()
    
    def reset(self):
        self.pulse_L = 0
        self.pulse_R = 0

    def get(self):
        return self.pulse_L, self.pulse_R


    
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
        
    

