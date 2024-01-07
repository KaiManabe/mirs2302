import serial_com as ser
import RPi.GPIO as GPIO
import time
import atexit

s = ser.arduino_serial()

PINS = [13,15,19,21,29,31]
"""
def servo(pin):
    GPIO.output(pin, 0)
    time.sleep(0.5)
    s.send(10, [0])
    time.sleep(3)
    
    GPIO.output(pin, 1)
    time.sleep(0.5)
    
"""
def init():
    GPIO.setmode(GPIO.BOARD)
    for pin in PINS:
        GPIO.setup(pin, GPIO.OUT)

def dele():
        GPIO.cleanup()

def r(p):
    for i in range(500):
        GPIO.output(p,1)
        time.sleep(0.001)
        GPIO.output(p,0)
        time.sleep(0.001)
    
    time.sleep(1)
    
    for i in range(250):
        GPIO.output(p,1)
        time.sleep(0.002)
        GPIO.output(p,0)
        time.sleep(0.002)


if __name__ == "__main__":
    init()
    
    atexit.register(dele)
    """
    for p in PINS:
        r(p)
        time.sleep(1)
    """



