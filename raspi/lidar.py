import sock
import matplotlib as plt
import config
import subprocess as sp
import sys
import time

s = sock.sock_server(config.RASPI_IP, config.SOCKET_PORT)

def getdata():
    s.send([255,3,254])
    time.sleep(0.25)
    return s.read()


