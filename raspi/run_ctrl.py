import csv
import time
import serial_com as ser
import keyboard
import threading
import re
import sys

class sequence_file():
    """
    シーケンスファイルの入出力をするクラス
    """
    
    def read(self):
        with open("./sequence.csv", mode='r', encoding="utf-8") as csv_file:
            #２次元配列にする ※数値も文字列として代入されるので注意が必要
            sequence_list = list(csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True))
        
        print(sequence_list)
        
    def write(self, data):
        with open('./sequence.csv', mode='a', newline = '') as csv_file:
            writer = csv.writer(csv_file)
            
            for i in data:
                writer.writerow(i)
                
    def delete(self):
        with open('./sequence.csv', mode='w'):
            pass
        
class run_controller():
    """
    走行を制御するクラス
    """
    
    def __init__(self, serial_port:ser.arduino_serial):
        """
        コンストラクタ
        
        引数：
            serial_port -> serial object : serial_com.pyのarduino_serialクラスのオブジェクトを渡す
        """
        self.serial = serial_port
    
    def set_l_speed(self, speed):
        """
        左モータのスピードをセットする
        
        引数：
            speed -> float : 速度[mm/s]
        """
        
        if speed > 0:
            dir = 1
        else:
            dir = 0
        hb = int(abs(speed) / 254)
        lb = int(abs(speed) % 254)
        self.serial.send([255,1,dir, hb, lb, 254])
        
    def set_r_speed(self, speed):
        """
        右モータのスピードをセットする
        
        引数：
            speed -> float : 速度[m/s]
        """
        
        if speed > 0:
            dir = 1
        else:
            dir = 0
        hb = int(abs(speed) / 254)
        lb = int(abs(speed) % 254)
        self.serial.send([255,2,dir, hb, lb, 254])
    
    
    def send_straight(self, speed):
        """
        左右モータのスピードをセットする
        
        引数：
            speed -> float : 速度[mm/s]
        """
        
        if speed > 0:
            dir = 1
        else:
            dir = 0
        hb = int(abs(speed) / 254)
        lb = int(abs(speed) % 254)
        if sys.argv[0] == "autotune.py":
            self.serial.send([255,1,1, hb, lb, 254])
            self.serial.send([255,2,0, hb, lb, 254])
        else:
            self.serial.send([255,1,dir, hb, lb, 254])
            self.serial.send([255,2,dir, hb, lb, 254])
        print([255,2,dir, hb, lb, 254])
    
    def send_rotate(self, theta, omega):
        """
        機体を回転させる
        
        引数：
            theta -> int : 回転角[degree]
            omega -> int : 回転速度[degree/s]
        """
        
        if theta * omega > 0:
            dir = 1
        else:
            dir = 0
        thb = int(abs(theta) / 254)
        tlb = int(abs(theta) % 254)
        
        ohb = int(abs(omega) / 254)
        olb = int(abs(omega) % 254)
        print([255,7,dir, thb, tlb, ohb, olb, 254])
        self.serial.send([255,7,dir, thb, tlb, ohb, olb, 254])
    
    
    def keyboard_controller(self):
        """
        wasdで機体を動かす
        sshからのキー入力を受け付けないため、usdに挿したキーボードで操作する
        実行中にspd400　とか入れると速度が変わる　デフォルトは200
        実行中にstopと入力するとメソッドが終了する
        
        デバッグ用と以外では使わないこと
        """
        self.spd = 200
        self.stop = False
        spd_l_prev = 0
        spd_r_prev = 0
        
        t = threading.Thread(target = input_monitor, args = (self, ))
        t.setDaemon(True)
        t.start()
        
        while(not(self.stop)):
            spd_l, spd_r = calcspeed(self.spd)
            if spd_r != spd_r_prev or spd_l != spd_l_prev or True:
                print(f"\r{spd_l:4}, {spd_r:4}", end = "               ")
                self.set_l_speed(spd_l)
                self.set_r_speed(spd_r)
                spd_l_prev = spd_l
                spd_r_prev = spd_r
        print("\n")
        
        

def input_monitor(ctrl:run_controller):
    while(1):
        string = input()
        if "spd" in string:
            num = re.sub("\\D", "", string)
            print(f"\nset speed to {num}\n")
            ctrl.spd = int(num)
        
        
        if "stop" in string:
            ctrl.stop = True
            break



def calcspeed(spd):
    spd_l = 0
    spd_r = 0
    if keyboard.is_pressed("w"):
        spd_l += spd
        spd_r += spd
        
    if keyboard.is_pressed("s"):
        spd_l -= spd
        spd_r -= spd
        
    if keyboard.is_pressed("a"):
        spd_r += spd * 0.5
        spd_l += -spd * 0.5
    
    if keyboard.is_pressed("d"):
        spd_r += -spd * 0.5
        spd_l += spd * 0.5
    
    return spd_l, spd_r

            


if __name__ == "__main__":
    """
    seq = sequence_file()
    print('status = read:')
    seq.read()
    print()
    
    time.sleep(1)
    
    seq.delete()
    print('status = delete:')
    print()
    
    time.sleep(1)
    
    data_to_write = [["rotate", 90, 90],["straight", 20, 0.5], ["straight", 5, -0.3]]
    seq.write(data_to_write)
    """

    import serial_com as ser
    s = ser.arduino_serial()
    c = run_controller(s)