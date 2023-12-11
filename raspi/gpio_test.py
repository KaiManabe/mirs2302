import RPi.GPIO as GPIO
import serial_com as ser
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(13,GPIO.OUT)
s = ser.arduino_serial()

def open(): 
    GPIO.output(13, 1)
    s.send(10,[0])   
    time.sleep(1)
    GPIO.output(13, 0)