import serial_com as ser
import run_ctrl as ctrl
import order_mng
import module_mng
import sock
import web_app
import config
import datetime
import time
import sys
import robot_state_publisher as rsp
import keyboard_controller_server
import threading
import shutil


ID = "e229d3f0-3a47-4333-b6cd-e78f68c1da01"

if __name__ == "__main__":
    s = ser.arduino_serial()
    c = ctrl.run_controller(s)
    
    ctrl_server = sock.sock_server(config.RASPI_IP, 56565)
    serv_server = sock.sock_server("127.0.0.1", 56674)
    shutil.copy("/home/pi/git/mirs2302/raspi/prsn_order.csv", "/home/pi/git/mirs2302/raspi/order_info.csv")
    om = order_mng.order_manager()
    
    controller_thread = threading.Thread(target = keyboard_controller_server.exec, args = (ctrl_server, c,))
    controller_thread.setDaemon(True)
    controller_thread.start()
    
    module_mng.module_controller(serial_port = s,order_manager = om, serv = serv_server)

    while(1):
        while(1):
            time.sleep(0.5)
            if om.get_order("ID",ID)["STATUS"][0] == "PICKED_UP":
                break
        
        time.sleep(0.5)
        print("ORDER STATUSを WAITING_FOR_RECEIVEに変更")
        print("ROBOT STATUSを WAITING_FOR_RECEIVEに変更")
        om.modify_order(ID, "STATUS", "WAITING_FOR_RECEIVE")
        rsp.update("ROBOT_STATUS", "WAITING_FOR_RECEIVE")
        
        while(1):
            time.sleep(0.5)
            if om.get_order("ID",ID)["STATUS"][0] == "RECEIVED":
                break
        
        time.sleep(0.5)
        print("ORDER STATUSを WAITING_FOR_PICKUPに変更")
        print("ROBOT STATUSを WAITING_FOR_PICKUPに変更")
        om.modify_order(ID, "STATUS", "WAITING_FOR_PICKUP")
        rsp.update("ROBOT_STATUS", "WAITING_FOR_PICKUP")
    
    