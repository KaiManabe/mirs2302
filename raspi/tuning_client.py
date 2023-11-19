import sock, config, tuning
import matplotlib.pyplot as plt
import time

c = sock.sock_client(config.RASPI_IP_NCT, 55555)
time.sleep(2)
a = c.read()
b = tuning.convert_data(a)

