import serial_com as ser
import RPi.GPIO as GPIO
import time
import atexit

s = ser.arduino_serial()

PINS = [13,15,19,21,29,31]

def servo(pin):
    GPIO.output(pin, 0)
    time.sleep(0.5)
    s.send(10, [0])
    time.sleep(3)
    
    GPIO.output(pin, 1)
    time.sleep(0.5)
    

def init():
    GPIO.setmode(GPIO.BOARD)
    for pin in PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin,1)

def dele():
    for pin in PINS:
        GPIO.cleanup(pin)

if __name__ == "__main__":
    init()
    atexit.register(dele)