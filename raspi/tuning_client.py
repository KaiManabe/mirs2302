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
    plotter(b, dontshow=True)



def moving_average(data, dt:int):
    out = []
    for data_idx in range(len(data)):
        arr = []
        for i in range(len(data[data_idx]) - dt):
            arr.append(np.mean(data[data_idx][i:i+dt]))
        out.append(arr)
    return out
    
        

def plotter(data, dontshow=False):
    fig = plt.figure(figsize=(19.2,10.8))
    
    plt1 = plt.subplot(1,2,1)
    plt1.plot(data[0], label = "L")
    plt1.plot(data[1], label = "L_target")
    plt1.set_ylabel("spped[mm/s]")
    plt1.grid()
    plt1.legend()
    
    plt2 = plt.subplot(1,2,2)
    plt2.plot(data[2], label = "R")
    plt2.plot(data[3], label = "R_target")
    plt2.set_ylabel("spped[mm/s]")
    plt2.grid()
    plt2.legend()
    
    directory = "../../"
    fname = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + ".png"
    fig.savefig(directory+fname, format = "png", dpi = 600)
    if not(dontshow):
        plt.show()