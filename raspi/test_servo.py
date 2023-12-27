import serial_com as ser
import RPi.GPIO as GPIO
import time

s = ser.arduino_serial()

def servo(pin):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, True)
    time.sleep(0.5)
    s.send(10, [0])
    time.sleep(3)
    
    GPIO.output(pin, False)
    time.sleep(0.5)