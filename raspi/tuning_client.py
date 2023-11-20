import sock, config, tuning
import matplotlib.pyplot as plt
import time
import datetime
import numpy as np

if __name__ == "__main__":
    c = sock.sock_client(config.RASPI_IP_NCT, 55555)
    time.sleep(2)

def receive():
    a = c.read()
    b = tuning.convert_data(a)
    bma = moving_average(b,1)
    plotter(bma, dontshow=True)



def moving_average(data, dt:int):
    out = []
    for data_idx in range(4):
        arr = []
        for i in range(len(data[data_idx]) - dt):
            arr.append(np.mean(data[data_idx][i:i+dt]))
        out.append(arr)
    return out
    


