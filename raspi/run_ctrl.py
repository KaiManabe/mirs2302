import csv
import time

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
            
            #if type(data) != list(list()):
                #print("[エラー] run_ctrl : writerow()メソッドの引数はリストである必要があります")
            #else:
            for i in data:
                writer.writerow(i)
                
    def delete(self):
        with open('./sequence.csv', mode='w'):
            pass
        
class run_controller():
    """
    走行を制御するクラス(作成途中)
    """
    
    def __init__(self, serial_port):
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
            speed -> float : 速度[m/s]
        """
        self.serial.send([speed])
        
    def set_r_speed(self, speed):
        """
        右モータのスピードをセットする
        
        引数：
            speed -> float : 速度[m/s]
        """
        self.serial.send([speed])


if __name__ == "__main__":
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