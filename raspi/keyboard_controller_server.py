import sys
import run_ctrl as ctrl
import serial_com as ser
import threading
import sock
import config



def input_monitor():
    while(1):
        if serv.isconnected() == 0:
            return 0, 0
        else:
            if serv.buffer_length() >= 5:
                tmp = serv.read()
                for i in range(len(tmp) - 1, -1, -1):
                    if (i - 2) < 0:
                        break
                    if tmp[i] == 255:
                        r = int((tmp[i - 1] * 10)) - 1270
                        l = int((tmp[i - 2] * 10)) - 1270
                        return r, l
                


if __name__ == "__main__":
    s = ser.arduino_serial()
    c = ctrl.run_controller(s)
    serv = sock.sock_server(config.RASPI_IP, 56565)
    while(1):
        l, r = input_monitor()
        c.set_l_speed(l)
        c.set_r_speed(r)