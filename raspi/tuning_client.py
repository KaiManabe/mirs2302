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
    bma = moving_average(b,5)
    plotter(bma, dontshow=False)



def moving_average(data, dt:int):
    out = []
    for data_idx in range(4):
        arr = []
        for i in range(len(data[data_idx]) - dt):
            arr.append(np.mean(data[data_idx][i:i+dt]))
        out.append(arr)
    return out
    
    
def getamplitude(data):
    err_l = []
    err_r = []
    vl_target = []
    vr_target = []
    vl = []
    vr = []
    tl = []
    tr = []
    vl_relat = []
    vr_relat = []
    ampl = []
    ampr = []

    for i in range(max([len(data[0]),len(data[1]),len(data[2]),len(data[3])])):
        if i+1 < len(data[0]) and i+1 < len(data[1]):
            err_l.append(data[0][i] - data[1][i])
            vl.append(data[0][i+1] - data[0][i])
            vl_target.append(data[1][i+1] - data[1][i])
            vl_relat.append(vl[-1] - vl_target[-1])
            
        if i+1 < len(data[2]) and i+1 < len(data[3]):
            err_r.append(data[2][i] - data[3][i])
            vr.append(data[2][i+1] - data[2][i])
            vr_target.append(data[3][i+1] - data[3][i])
            vr_relat.append(vr[-1] - vr_target[-1])

    current_idx = 0
    for i in range(len(vl)):
        if vl_relat[i] < 0:
            for ii in range(i):
                if vl_relat[i - ii - 1] < 0:
                    break
                if vl_relat[i - ii - 1] > 0:
                    if current_idx != 0:
                        tl.append(i - current_idx)
                        ampl.append(max(vl_relat[current_idx:i]) - min(vl_relat[current_idx:i]))
                    current_idx = i
                    break
    
    current_idx = 0           
    for i in range(len(vr)):
        if vr_relat[i] < 0:
            for ii in range(i):
                if vr_relat[i - ii - 1] < 0:
                    break
                if vr_relat[i - ii - 1] > 0:
                    if current_idx != 0:
                        tr.append(i - current_idx)
                        ampr.append(max(vr_relat[current_idx:i]) - min(vr_relat[current_idx:i]))
                    current_idx = i
                    break
                
    return ampl, ampr
        
    


def getperiod(data):
    err_l = []
    err_r = []
    vl_target = []
    vr_target = []
    vl = []
    vr = []
    tl = []
    tr = []
    vl_relat = []
    vr_relat = []

    for i in range(max([len(data[0]),len(data[1]),len(data[2]),len(data[3])])):
        if i+1 < len(data[0]) and i+1 < len(data[1]):
            err_l.append(data[0][i] - data[1][i])
            vl.append(data[0][i+1] - data[0][i])
            vl_target.append(data[1][i+1] - data[1][i])
            vl_relat.append(vl[-1] - vl_target[-1])
            
        if i+1 < len(data[2]) and i+1 < len(data[3]):
            err_r.append(data[2][i] - data[3][i])
            vr.append(data[2][i+1] - data[2][i])
            vr_target.append(data[3][i+1] - data[3][i])
            vr_relat.append(vr[-1] - vr_target[-1])

    current_idx = 0
    for i in range(len(vl)):
        if vl_relat[i] < 0:
            for ii in range(i):
                if vl_relat[i - ii - 1] < 0:
                    break
                if vl_relat[i - ii - 1] > 0:
                    if current_idx != 0:
                        tl.append(i - current_idx)
                    current_idx = i
                    break
    current_idx = 0           
    for i in range(len(vr)):
        if vr_relat[i] < 0:
            for ii in range(i):
                if vr_relat[i - ii - 1] < 0:
                    break
                if vr_relat[i - ii - 1] > 0:
                    if current_idx != 0:
                        tr.append(i - current_idx)
                    current_idx = i
                    break
                
    return tl, tr



def plotter(data, dontshow = False):
    err_l = []
    err_r = []
    vl_target = []
    vr_target = []
    vl = []
    vr = []
    tl = []
    tr = []
    vl_relat = []
    vr_relat = []

    for i in range(max([len(data[0]),len(data[1]),len(data[2]),len(data[3])])):
        if i+1 < len(data[0]) and i+1 < len(data[1]):
            err_l.append(data[0][i] - data[1][i])
            vl.append(data[0][i+1] - data[0][i])
            vl_target.append(data[1][i+1] - data[1][i])
            vl_relat.append(vl[-1] - vl_target[-1])
            
        if i+1 < len(data[2]) and i+1 < len(data[3]):
            err_r.append(data[2][i] - data[3][i])
            vr.append(data[2][i+1] - data[2][i])
            vr_target.append(data[3][i+1] - data[3][i])
            vr_relat.append(vr[-1] - vr_target[-1])

    current_idx = 0
    for i in range(len(vl)):
        if vl_relat[i] < 0:
            for ii in range(i):
                if vl_relat[i - ii - 1] < 0:
                    break
                if vl_relat[i - ii - 1] > 0:
                    if current_idx != 0:
                        tl.append(i - current_idx)
                    current_idx = i
                    break
    current_idx = 0           
    for i in range(len(vr)):
        if vr_relat[i] < 0:
            for ii in range(i):
                if vr_relat[i - ii - 1] < 0:
                    break
                if vr_relat[i - ii - 1] > 0:
                    if current_idx != 0:
                        tr.append(i - current_idx)
                    current_idx = i
                    break
    
    
    fig = plt.figure(figsize=(19.2,10.8))
    plt1 = plt.subplot(2,3,1)
    plt1.plot(vl, label = "v_l")
    plt1.plot(vl_target, label = "v_l_target")
    plt1.legend()
    
    plt2 = plt.subplot(2,3,2)
    plt2.plot(err_l, label = "l_enc_err")
    plt2.legend()
    
    plt3 = plt.subplot(2,3,3)
    plt3.plot(tl, label = "t_l")
    plt3.legend()
    
    plt4 = plt.subplot(2,3,4)
    plt4.plot(vr, label = "v_r")
    plt4.plot(vr_target, label = "v_r_target")
    plt4.legend()
    
    plt5 = plt.subplot(2,3,5)
    plt5.plot(err_r, label = "r_enc_err")
    plt5.legend()
    
    plt6 = plt.subplot(2,3,6)
    plt6.plot(tr, label = "t_r")
    plt6.legend()
    
    directory = "../../"
    fname = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + ".png"
    fig.savefig(directory + fname, format = "png", dpi = 600)
    if not(dontshow):
        plt.show()