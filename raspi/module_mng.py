import serial_com as ser
import RPi.GPIO as GPIO
import threading
import time

class module_controller():
    """
    モジュールを制御するクラス
    """
    def __init__(self, serial_port: ser.arduino_serial):
        """
        コンストラクタ
        
        引数：
            serial_port -> serial object : serial_com.pyのarduino_serialクラスのオブジェクトを渡す
        """
        self.serial = serial_port
        
        # 各段のモジュール情報
        self.module_rank_info = {
            "module1": {
                "name": "",
                "door1": {
                    "pin": {
                        "servo":13,
                        "switch":16
                    },
                    "current_state": False
                },
                "door2": {
                    "pin": {
                        "servo":15,
                        "switch":18
                    },
                    "current_state": False
                }
            },
            "module2": {
                "name": "",
                "door1": {
                    "pin": {
                        "servo":19,
                        "switch":22
                    },
                    "current_state": False
                },
                "door2": {
                    "pin": {
                        "servo":21,
                        "switch":23
                    },
                    "current_state": False
                }
            },
            "module3": {
                "name": "",
                "door1": {
                    "pin": {
                        "servo":29,
                        "switch":32
                    },
                    "current_state": False
                },
                "door2": {
                    "pin": {
                        "servo":31,
                        "switch":33
                    },
                    "current_state": False
                }
            }
        }
        self.identify_module() # 各段のモジュール名を振り付け
        
        # 各モジュールについての情報
        self.each_module_info = {
            "base": {
                "height": 230
            },
            "accessories": {
                "height": 120
            },
            "document": {
                "height": 160
            },
            "insulation": {
                "height": 200
            },
            "unconnected": {
                "height": 0
            }
        }
        
    def identify_module(self):
        """
        モジュールを識別をする
        
        各段のモジュールの名前をセット：
            self.module_rank_info[module_num]["name"]: str
        """
        # 抵抗値の許容誤差範囲を定義
        err_rate=20 # 許容誤差範囲[%]
        acc_res_range = [240 * (100 - err_rate) / 100, 240 * (100 + err_rate) / 100] # 小物抵抗値範囲
        doc_res_range = [1000 * (100 - err_rate) / 100, 1000 * (100 + err_rate) / 100] # 資料抵抗値範囲
        ins_res_range = [2000 * (100 - err_rate) / 100, 2000 * (100 + err_rate) / 100] # 保冷・保温抵抗値範囲
        
        # 回路ができたらこいつ使う↓
        # response = self.serial.send_and_read_response(3,[],12)
        response = [0, 240, 3, 238, 0, 20] # 仮の値 240Ω 1000Ω 2000Ω
        
        # 抵抗値を計算
        resistance_list = [
            response[0]*254+response[1], # 1段目
            response[2]*254+response[3], # 2段目
            response[4]*254+response[5] # 3段目
            ]
        
        # 各段のモジュール名を振り付け
        for i, res in enumerate(resistance_list):
            if acc_res_range[0] <= res <= acc_res_range[1]:
                self.module_rank_info[f"module{i + 1}"]["name"] = "accessories"
            elif doc_res_range[0] <= res <= doc_res_range[1]:
                self.module_rank_info[f"module{i + 1}"]["name"] = "document"
            elif ins_res_range[0] <= res <= ins_res_range[1]:
                self.module_rank_info[f"module{i + 1}"]["name"] = "insulation"
            else: # 未接続の場合の値を決めてArduinoから送るようにしたほうがいいか？？？？？？？
                self.module_rank_info[f"module{i + 1}"]["name"] = "unconnected"
    
    def door_surv(self, module_num: str, door_num: str):
        """
        扉の開閉状態を監視する
        
        引数：
            module_num : 監視したい扉のモジュール番号 -> str
            
            door_num : 監視したい扉の番号 -> str
            
        ドアの開閉状態のフラグをセット：
            self.each_module_info[module_num][door_num]["current_state"]: bool
        """
        # 扉が閉じている時、開いている時のPINの状態どっちだ？？
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.module_rank_info[module_num][door_num]["PIN"]["switch"], GPIO.IN)
        
        # 扉の状態フラグの初期化
        self.module_rank_info[module_num][door_num]["current_state"]: bool = GPIO.input(self.module_rank_info[module_num][door_num]["PIN"]["switch"])
        previous_state = self.module_rank_info[module_num][door_num]["current_state"]
        
        while True:
            self.module_rank_info[module_num][door_num]["current_state"]: bool = GPIO.input(self.module_rank_info[module_num][door_num]["PIN"]["switch"])
            # PINの状態が変わった時
            if self.module_rank_info[module_num][door_num]["current_state"] != previous_state:
                if self.module_rank_info[module_num][door_num]["current_state"]:
                    print(f"扉{door_num}が開きました")
                else:
                    print(f"扉{door_num}が閉じました")
                previous_state = self.module_rank_info[module_num][door_num]["current_state"]
            time.sleep(0.1) # 0.1sごと監視
            
    def door_open(self, module_num: str, door_num: str):
        """
        扉の鍵を解錠する
        
        引数：
            module_num : 解錠したい扉のモジュール番号 -> str
            
            door_num : 解錠したい扉の番号 -> str
        """
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.module_rank_info[module_num][door_num]["servo"], GPIO.OUT)
        
        GPIO.output(self.module_rank_info[module_num][door_num]["servo"], True)
        time.sleep(0.5) # なぜか待たないと動かない
        self.serial.send(10, [])
        time.sleep(4) # Arduino側のサーボを開けてから閉じるまでの時間が3sなので、0.5s余裕を持たせておく
        GPIO.output(self.module_rank_info[module_num][door_num]["servo"], False)
            
    def battery_surv(self):
        """
        バッテリー電圧を監視する
        
        戻り値：
            バッテリー電圧[V]
        """
        response = self.serial.send_and_read_response(5,[],11)
        batt_vol = response[0][0]/10
        
        return batt_vol
    
    def height_calculate(self):
        """
        機体の高さを算出する関数
            
        戻り値：
            機体全体の高さ[mm]
        """
        total_height = self.each_module_info["base"]["height"]

        total_height += self.each_module_info[ self.module_rank_info["module1"]["name"] ]["height"]
        total_height += self.each_module_info[ self.module_rank_info["module2"]["name"] ]["height"]
        total_height += self.each_module_info[ self.module_rank_info["module3"]["name"] ]["height"]
            
        return total_height
            
            
# if __name__ == '__main__':
#     s = ser.arduino_serial()
#     m = module_controller(s)
#     m.door_open('paper_under')
#     m.door_open('paper_upper')
#     m.door_open('accessories_right')
#     m.door_open('accessories_left')
#     m.door_open('hot')
#     m.door_open('cold')