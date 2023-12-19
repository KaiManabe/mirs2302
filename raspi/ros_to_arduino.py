import sock
import serial_com as ser
import run_ctrl as ctrl
import threading
import time

class ros_to_arduino():
    def __init__(self, c:ctrl.run_controller, s:sock.sock_server):
        self.c = c
        self.s = s
        while(1):
            if self.s.isconnected() > 0:
                break
        t = threading.Thread(target = monitor, args = (self,))
        t.setDaemon(True)
        t.start()

    def send_to_arduino(self, l, r):
        self.c.set_l_speed(l)
        self.c.set_r_speed(r)

def monitor(rta):
    print("[INFO][ros_to_arduino] : cmd_velをarduinoに転送開始しました")
    while(1):
        if len(rta.s.buf) > 0:
            time.sleep(0.05)
            if rta.s.isconnected() > 0:
                tmp = rta.s.read()
                #print(tmp)
                for i in range(len(tmp)-3 , -1, -1):
                    if tmp[i] == 255 and len(tmp) > (i+2):
                        l = (tmp[i+1] - 127) * 6
                        r = (tmp[i+2] - 127) * 6
                        rta.send_to_arduino(l,r)
                        break
            

if __name__ == "__main__":
    s = ser.arduino_serial()
    c = ctrl.run_controller(s)
    ss = sock.sock_server("192.168.1.4", 56789)
    #ss = sock.sock_server("127.0.0.1", 56789)
    r = ros_to_arduino(c, ss)
