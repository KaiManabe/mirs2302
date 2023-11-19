import sock, config, tuning
import matplotlib.pyplot as plt
import time
import datetime
import numpy as np


c = sock.sock_client(config.RASPI_IP_NCT, 55555)
time.sleep(2)

def receive():
    a = c.read()
    b = tuning.convert_data(a)
    c = moving_average(b,5)
    plotter(c, dontshow=False)



def moving_average(data, dt:int):
    out = []
    for data_idx in range(4):
        arr = []
        for i in range(len(data[data_idx]) - dt):
            arr.append(np.mean(data[data_idx][i:i+dt]))
        out.append(arr)
    return out
    
    


def plotter(data, dontshow = False):
    err_l = []
    err_r = []
    vl_target = []
    vr_target = []
    vl = []
    vr = []
    
    
    for i in range(max([len(data[0]),len(data[1]),len(data[2]),len(data[3])])):
        if i+1 < len(data[0]) and i+1 < len(data[1]):
            err_l.append(data[0][i] - data[1][i])
            vl.append(data[0][i+1] - data[0][i])
            vl_target.append(data[1][i+1] - data[1][i])
            
        if i+1 < len(data[2]) and i+1 < len(data[3]):
            err_r.append(data[2][i] - data[3][i])
            vr.append(data[2][i+1] - data[2][i])
            vr_target.append(data[3][i+1] - data[3][i])
    
    
    fig = plt.figure(figsize=(19.2,10.8))
    plt1 = plt.subplot(2,2,1)
    plt1.plot(vl, label = "v_l")
    plt1.plot(vl_target, label = "v_l_target")
    plt1.legend()
    
    plt2 = plt.subplot(2,2,2)
    plt2.plot(err_l, label = "l_enc_err")
    plt2.legend()
    
    plt3 = plt.subplot(2,2,3)
    plt3.plot(vr, label = "v_r")
    plt3.plot(vr_target, label = "v_r_target")
    plt3.legend()
    
    plt4 = plt.subplot(2,2,4)
    plt4.plot(err_r, label = "r_enc_err")
    plt4.legend()
    
    directory = "../../"
    fname = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + ".png"
    fig.savefig(directory + fname, format = "png", dpi = 600)
    if not(dontshow):
        plt.show()