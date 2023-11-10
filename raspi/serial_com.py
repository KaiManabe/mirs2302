import serial
import time

# クラス
class arduino_serial:
    # コンストラクタ（インスタンスの初期設定）
    def __init__(self):
        if(1): # raspi:1 Windows/Mac:0
            try:
                self.serial_port = serial.Serial("/dev/ttyACM0", 115200, timeout=1.0)
            except:
                self.serial_port = serial.Serial("/dev/ttyACM1", 115200, timeout=1.0)
        else:
            try:
                self.serial_port = serial.Serial("COM6", 115200, timeout=1.0) # Windows
            except:
                self.serial_port = serial.Serial("/dev/cu.usbmodem14101", 115200, timeout=1.0) # Mac
        time.sleep(2)
    
    # メソッドsend
    def send(self,b):
        self.serial_port.reset_input_buffer() # 送信前にバッファをクリア
        if type(b) == list: # b という引数が配列だった場合は要素を1つずつ取り出し、bytes型にキャストして書き込む
            for i in b:
                self.serial_port.write(bytes([i]))
        else:
            self.serial_port.write(bytes([b]))
    
    # メソッドs_read
    def s_read(self):
        self.buf = self.serial_port.in_waiting
        if self.buf > 0: # バッファにデータがある時のみ実行
            return self.serial_port.read()
        else:
            return -1
        
    # メソッドclear
    def clear(self):
        self.serial_port.reset_input_buffer() # バッファをクリア
    
    # メソッドclose
    def close(self):
        self.serial_port.close() # シリアルポートを閉じる（必須処理）

s = arduino_serial()

# 送信用にデータを分解する関数
def run_data(mode,dist,spd):
    # 距離、速度をhigh bit, low bit に分解
    dhb = int(abs(dist)/253)
    dlb = int(abs(dist)%253)
    shb = int(abs(spd)/253)
    slb = int(abs(spd)%253)
    
    # 前進/後退の判定
    if spd*dist < 0:
        d = 0
    else:
        d = 1
        
    # 送信するデータを配列に代入
    sb = [255,mode,d,dhb,dlb,shb,slb,254]

    for i in sb:
        s.send(i)
        print(i) # デバッグ用

# 変数の更新を監視し、変更があった場合はデータを送信する関数
def update_serv(mode, dist, spd):
    if tmp != [mode, dist, spd]:
        tmp = [mode, dist, spd] # 要素を更新
        run_data(tmp[0], tmp[1], tmp[2])


# 以下テスト用
if __name__ == "__main__":
    tmp = [0, 0, 0] # tmpの初期化
    
    #run_data(1,3000,500)
    test = [255,1,0,10,0,1,247,254]
    #test = [255,5,254]
    for i in test:
        s.send(i)
        print('send:'+str(i))
        
    s.close()