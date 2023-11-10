import serial_com
import run_ctrl

ser = serial_com.arduino_serial()

controller = run_ctrl.run_controller(ser)

controller.set_l_speed(123)