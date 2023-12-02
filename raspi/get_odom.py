import serial_com as ser


class odom():
    def __init__(self, s:ser.arduino_serial):
        self.theta = [0, 0]
        self.omega = [0, 0]
        self.robot_verocity = 0.0
        self.robot_theta = 0.0
        self.robot_omega = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
    
    