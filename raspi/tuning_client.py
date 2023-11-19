import sock, config, tuning
import matplotlib.pyplot as plt
import time
import datetime

c = sock.sock_client(config.RASPI_IP_NCT, 55555)
time.sleep(2)

def receive():
    a = c.read()
    b = tuning.convert_data(a)
    plotter(b, dontshow=False)


def plotter(data, dontshow = False):
    err_l = []
    err_r = []
    
    for i in range(max([len(data[0]),len(data[1]),len(data[2]),len(data[3])])):
        if i < len(data[0]) and i < len(data[1]):
            err_l.append(data[0][i] - data[1][i])
        if i < len(data[2]) and i < len(data[3]):
            err_r.append(data[2][i] - data[3][i])
    
    
    fig = plt.figure()
    plt1 = plt.subplot(2,2,1)
    plt1.plot(data[0], label = "l_enc")
    plt1.plot(data[1], label = "l_enc_target")
    plt1.legend()
    
    plt2 = plt.subplot(2,2,2)
    plt2.plot(err_l, label = "l_enc_err")
    plt2.legend()
    
    plt3 = plt.subplot(2,2,3)
    plt3.plot(data[2], label = "r_enc")
    plt3.plot(data[3], label = "r_enc_target")
    plt3.legend()
    
    plt4 = plt.subplot(2,2,4)
    plt4.plot(err_r, label = "r_enc_err")
    plt4.legend()
    
    directory = "../../"
    fname = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + ".png"
    fig.savefig(directory + fname, format = "png", dpi = 600)
    if not(dontshow):
        plt.show()