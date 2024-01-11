import signal
import serial_com as ser
import RPi.GPIO as GPIO
import time
import atexit
import threading

s = ser.arduino_serial()

PINS = [13,15,19,21,29,31]


def init():
    GPIO.setmode(GPIO.BOARD)
    for pin in PINS:
        GPIO.setup(pin, GPIO.OUT)



def pwm(pin, cycle):
    if not(pin in PINS):
        return
    pwm_cycle = 1/ 50
    duty = cycle
    duty /= 1000
    duty /= 1000
    for i in range(40):
        GPIO.output(pin, 1)
        ctime = time.time()
        while(ctime + duty > time.time()):
            pass
        GPIO.output(pin,0)
        while(ctime + pwm_cycle > time.time()):
            pass
        


def open_servo(pin, side = "left"):
    if side == "left":
        #左の扉
        #蝶番が左　サーボが右
        pwm(pin, 900)
        time.sleep(1)
        pwm(pin, 1900)
    elif side == "right":
        #右の扉
        #蝶番が右　サーボが左
        pwm(pin, 1900)
        time.sleep(1)
        pwm(pin, 900)
        

if __name__ == "__main__":
    init()
    for pin in PINS:
        if pin in [13, 29]:
            t = threading.Thread(target = open_servo, args = (pin, "right",))
        else:
            t = threading.Thread(target = open_servo, args = (pin, "left",))
        time.sleep(1)
        t.start()


