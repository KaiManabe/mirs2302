""" 
arduinoと通信をするためのクラスが定義されている

このファイルをインポートして、arduino_serial()というオブジェクトを定義してつかう
"""

import serial
import threading
import time
import sys
import config

#使用されうるポートを列挙しておく
#ここにあるポートに順番に接続を試みる
PORTS =  ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9", "/dev/cu.usbmodem14101","/dev/cu.usbmodem14201"]



buffer = []

def receive_buffer(serial_port):
    """
    シリアルポートのバッファを受け続ける

    Args:
        serial_port (serial.serial object): arduinoと通信するserialオブジェクト
    """
    global buffer
    while(1):
        time.sleep(0.05)
        while(serial_port.in_waiting > 0):
            buffer.append(serial_port.read())
            
            #バッファの長さが制限を超えたらバッファの頭を削る
            if len(buffer) > config.MAX_DATA_LENGTH_OF_SERIAL_BUFFER and config.MAX_DATA_LENGTH_OF_SERIAL_BUFFER != -1:
                buffer = buffer[(-1 * config.MAX_DATA_LENGTH_OF_SERIAL_BUFFER):]
    


class arduino_serial():
    global buffer
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
            time.sleep(3)#接続後すぐに処理が走らないようキープする
            initialize_result = True
            print("[INFO][serial_com.arduino_serial] : ポート", p, "のArduinoと接続を確立しました")
            break
        
        if not(initialize_result):
            #エラー処理
            print("[ERROR][serial_com.arduino_serial] : arduinoとの接続に失敗しました")
            sys.exit(1)
        
        #バッファを監視するスレッドを走らせる(これ以降常時実行)
        buf_monitor_thread = threading.Thread(target = receive_buffer, args = (self.serial_port,))
        buf_monitor_thread.setDaemon(True)
        buf_monitor_thread.start()
        print("[INFO][serial_com.arduino_serial] : シリアルバッファの監視を開始しました")
        
    def length(self):
        """
        バッファの長さを返す関数
        
        引数：
            なし
        
        戻り値：
            バッファの長さ
        """
        global buffer
        return len(buffer)
    
    def read(self):
        """
        バッファを返す関数
        
        引数・戻り値なし
        """
        global buffer
        buf = buffer.copy()
        buffer = []
        return buf
    
    def send(self,send_data):
        """"
        シリアルポートにデータを送る関数
        
        引数：
            send_data -> list(int) : 送るデータ
        戻り値：
            正常に送信したデータの長さ
        """
        
        """データのチェック"""
        if type(send_data) != list:
            print("[ERROR][serial_com.arduino_serial] : send()メソッドの引数はint型のリストである必要があります")
            return -1
        
        for d in send_data:
            if type(d) != int or d > 255:
                print("[ERROR][serial_com.arduino_serial] : send()メソッドの引数に256以上の値が存在します")
                return -1
        
        
        return self.serial_port.write(bytes(send_data))
    


if __name__ == "__main__":
    """モジュールではなく、コマンドラインから呼ばれた場合に以下の処理を実行"""
    s = arduino_serial()
    s.send([255,1,0,1,247,254])
    
    sys.exit(0)