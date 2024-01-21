import subprocess as sp
import time


def reset_navigation():
    sp.run(["ssh",
            "-i",
            "/home/pi/id_rsa_jetson",
            "mirs2302@192.168.1.5",
            "sh",
            "/home/mirs2302/kill_navigation.sh"])
    
    time.sleep(15)
    sp.run(["ssh",
            "-i",
            "/home/pi/id_rsa_jetson",
            "mirs2302@192.168.1.5",
            "sh",
            "/home/mirs2302/start_navigation.sh"])
    time.sleep(10)
    