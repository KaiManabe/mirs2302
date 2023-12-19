import serial_com as ser
import run_ctrl as ctrl
import config 
import time
import sys
import threading
import ros_to_arduino
import raspi_to_ros
import sock


if __name__ == "__main__":
    s = ser.arduino_serial()
    c = ctrl.run_controller(s)
    ss_rta = sock.sock_server(config.RASPI_IP, 56789)
    ss_rtr = sock.sock_server(config.RASPI_IP, 55555)
    rta = ros_to_arduino.ros_to_arduino(c,ss_rta)
    rtr = raspi_to_ros.raspi_to_ros(ss_rtr)