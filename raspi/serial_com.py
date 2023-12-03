""" 
arduinoと通信をするためのクラスが定義されている

このファイルをインポートして、arduino_serial()というオブジェクトを定義してつかう
"""

import serial
import threading
import time
import sys
import config
import numpy as np
import datetime

#使用されうるポートを列挙しておく
#ここにあるポートに順番に接続を試みる
PORTS =  ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9", "/dev/cu.usbmodem14101","/dev/cu.usbmodem14201"]







class arduino_serial():
    def __init__(self):
        """
        コンストラクタ
        
        引数・戻り値なし
        """
        
        initialize_result  = False  #初期化の成功/失敗フラグ
        print("[INFO][serial_com.arduino_serial] : Arduinoに接続を試みています...")
        for p in PORTS:
            try:
                #タイムアウト1秒でarduinoに接続を試みる
                self.serial_port = serial.Serial(p, 115200, timeout = 1.0)
            except:
                continue
            
            """以下、接続に成功した場合の処理"""
            time.sleep(1.5)#接続後すぐに処理が走らないようキープする
            initialize_result = True
            print("[INFO][serial_com.arduino_serial] : ポート", p, "のArduinoと接続を確立しました")
            break
        
        if not(initialize_result):
            #エラー処理
            print("[ERROR][serial_com.arduino_serial] : arduinoとの接続に失敗しました")
            sys.exit(1)
        
        
        #n番目のバッファが最後にいつ更新されたか覚えておく
        self.last_update = [datetime.datetime.now()] * config.NUMBER_OF_DATA_TYPE_ON_SERIAL
        
        #バッファ　大きさはconfig.pyで宣言しておく
        #self.buffer = np.full([config.NUMBER_OF_DATA_TYPE_ON_SERIAL,config.MAX_DATA_LENGTH_OF_SERIAL_BUFFER] , -1 , dtype = np.uint8)
        self.buffer = []
        for i in range(config.NUMBER_OF_DATA_TYPE_ON_SERIAL):
            self.buffer.append([])
        
        #バッファを監視するスレッドを走らせる(これ以降常時実行)
        buf_monitor_thread = threading.Thread(target = receive_buffer, args = (self,))
        buf_monitor_thread.setDaemon(True)
        buf_monitor_thread.start()
        print("[INFO][serial_com.arduino_serial] : シリアルバッファの監視を開始しました")

    
    def read(self, idx):
        """
        バッファを返す関数
        
        引数：
            idx -> クリアする番号
        
        戻り値：
            バッファ
        """
        
        return self.buffer[idx]
        
    
    def clear_buffer(self, idx):
        """
        バッファをクリアする関数
        
        引数：
            idx -> クリアする番号
        
        戻り値なし
        """
        self.buffer[idx] = []
    
    
    def send(self,idx, data):
        """"
        シリアルポートにデータを送る関数
        
        引数：
            idx -> 送るデータの番号
            send_data -> list(int) : 送るデータ
        戻り値：
            正常に送信したデータの長さ
        """
        
        """データのチェック"""
        if type(data) != list:
            print("[ERROR][serial_com.arduino_serial] : send()メソッドの引数はint型のリストである必要があります")
            return -1
        
        for d in data:
            if type(d) != int or d >= 254:
                print("[ERROR][serial_com.arduino_serial] : send()メソッドの引数に254以上の値が存在します")
                return -1
        
        data.insert(0,255)
        data.insert(1,idx)
        data.append(254)
        
        return self.serial_port.write(bytes(data))
    
    
    def send_and_read_response(self,idx, data, response_idx):
        """"
        シリアルポートにデータを送って、返答があれば返す関数
        
        引数：
            idx -> 送るデータの番号
            send_data -> list(int) : 送るデータ
            response_idx -> 受け取るデータの番号
        戻り値：
            レスポンスの配列
        """
        
        """データのチェック"""
        if type(data) != list:
            print("[ERROR][serial_com.arduino_serial] : send()メソッドの引数はint型のリストである必要があります")
            return -1
        
        for d in data:
            if type(d) != int or d >= 254:
                print("[ERROR][serial_com.arduino_serial] : send()メソッドの引数に254以上の値が存在します")
                return -1
        
        data.insert(0,255)
        data.insert(1,idx)
        data.append(254)
        
        self.clear_buffer(response_idx)
        send_time = datetime.datetime.now()
        self.serial_port.write(bytes(data))
        
        while(1):
            if self.last_update[response_idx] > send_time:
                return self.read(response_idx)
            
                
        
    
    


def receive_buffer(s:arduino_serial):
    """
    シリアルポートのバッファを受け続ける

    Args:
        s (arduino_serial object): arduinoと通信するserialオブジェクト
    """
        
    while(1):
        if s.serial_port.in_waiting > 0:
            if int.from_bytes(s.serial_port.read(), byteorder=sys.byteorder) == 255:
                data_idx = int.from_bytes(s.serial_port.read(), byteorder=sys.byteorder)
                s.buffer[data_idx].append([])
                while(1):
                    tmp = int.from_bytes(s.serial_port.read(), byteorder=sys.byteorder)
                    if tmp == 254:
                        s.last_update[data_idx] = datetime.datetime.now()
                        break
                    
                    s.buffer[data_idx][-1].append(tmp)
                
                if len(s.buffer[data_idx]) >= config.MAX_DATA_LENGTH_OF_SERIAL_BUFFER:
                    s.buffer[data_idx] = s.buffer[data_idx][(-1 * config.MAX_DATA_LENGTH_OF_SERIAL_BUFFER):]
                    
    

if __name__ == "__main__":
    """モジュールではなく、コマンドラインから呼ばれた場合に以下の処理を実行"""
    s = arduino_serial()
    s.send([255,1,0,1,247,254])
    
    sys.exit(0)