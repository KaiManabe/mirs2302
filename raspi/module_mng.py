import serial_com as ser
import RPi.GPIO as GPIO
import time

# Arduinoでモジュールの種類を識別して、ラベル名を適宜変えたい！！！！！！！
# エレキの配線決まり次第直す エレキ詳細設計書参照
door_pin_num = {
            'paper_under': {
                'servo':13, 'switch':16
                },
            
            'paper_upper': {
                'servo':15, 'switch':18
                },
            
            'accessories_right': {
                'servo':19, 'switch':22
                },
            
            'accessories_left': {
                'servo':21, 'switch':23
                },
            
            'hot': {
                'servo':29, 'switch':32
                },
            
            'cold': {
                'servo':31, 'switch':33
                     }
            }

class module_controller():
    """
    モジュールを制御するクラス
    """
    def __init__(self, serial_port:ser.arduino_serial):
        """
        コンストラクタ
        
        引数：
            serial_port -> serial object : serial_com.pyのarduino_serialクラスのオブジェクトを渡す
            
        シリアル通信
        """
        self.serial = serial_port
        self.module_name_list = self.identify_module()
        
    def door_open(self, door_name):
        """
        扉の鍵を解錠する
        
        引数：
            解錠したい扉の名前
        """
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(door_pin_num[door_name]['servo'], GPIO.OUT)
        
        GPIO.output(door_pin_num[door_name]['servo'], 1)
        time.sleep(0.5) # なぜか0.5秒待たないと動かない
        self.serial.send(10, [])
        time.sleep(4) # Arduino側のサーボを開けてから閉じるまでの時間が3sなので、0.5s余裕を持たせておく
        GPIO.output(door_pin_num[door_name]['servo'], 0)
    
    def microSW_surv(self, door_name):
        """
        扉の開閉を監視する
        
        引数：
            監視したい扉の名前
        戻り値：ああああああ
        """
        # 扉が閉じている時、開いている時のPINの状態どっちだ？？
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(door_pin_num[door_name]['switch'], GPIO.IN)
        
        previous_state = GPIO.input(door_pin_num[door_name]['switch'])
        
        while True:
            current_state = GPIO.input(door_pin_num[door_name]['switch'])
            # PINの状態が変わった時
            if current_state != previous_state:
                if current_state:
                    print("扉が開きました")
                else:
                    print("扉が閉じました")
                previous_state = current_state
            time.sleep(0.1)
            
    def battery_surv(self):
        """
        バッテリー電圧を監視する
        
        引数：
            なし
        戻り値：
            バッテリー電圧[V]
        """
        response = self.serial.send_and_read_response(5,[],11)
        batt_vol = response[0][0]/10
        
        return batt_vol

    def identify_module(self):
        """
        モジュールを識別をする
        
        引数：
            なし
        戻り値
            各段のモジュール名 -> ["module1": "モジュール名", "module2": "モジュール名", "module3": "モジュール名"]
            
            接続されていない もしくは 正しく読み取れていない場合 -> -1
        """
        # 抵抗値の許容誤差範囲を定義
        err_rate=20 # 許容誤差範囲[%]
        acc_res_range = [240 * (100 - err_rate) / 100, 240 * (100 + err_rate) / 100] # 小物抵抗値範囲
        doc_res_range = [1000 * (100 - err_rate) / 100, 1000 * (100 + err_rate) / 100] # 資料抵抗値範囲
        ins_res_range = [2000 * (100 - err_rate) / 100, 2000 * (100 + err_rate) / 100] # 保冷・保温抵抗値範囲
        
        # 回路ができたらこいつ使う↓
        # response = self.serial.send_and_read_response(3,[],12)
        response = [0, 240, 3, 238, 7, 222] # 仮の値 240Ω 1000Ω 2000Ω
        
        # 抵抗値を計算
        resistance_list = [
            response[0]*254+response[1], # 1段目
            response[2]*254+response[3], # 2段目
            response[4]*254+response[5] # 3段目
            ]
        
        # 判定結果を配列にする
        module_name = {}
        for i, res in enumerate(resistance_list):
            if acc_res_range[0] <= res <= acc_res_range[1]:
                module_name[f"module{i + 1}"] = "accessories"
            elif doc_res_range[0] <= res <= doc_res_range[1]:
                module_name[f"module{i + 1}"] = "document"
            elif ins_res_range[0] <= res <= ins_res_range[1]:
                module_name[f"module{i + 1}"] = "insulation"
            else: # 未接続の場合の値を決めてArduinoから送るようにしたほうがいいか？？？？？？？
                module_name[f"module{i + 1}"] = "unconnected"
                
        return module_name
    
    def height_calculate(self):
        """
        機体の高さを算出する関数
        
        引数：
            なし
        戻り値：
            機体全体の高さ[mm]
        """
        # ↓メカに聞いて数値直さなきゃ！！！！！！！！！
        # 各モジュールの高さ[mm]
        module_height_list = {
            "base": 230,
            "accessories": 120,
            "document": 100,
            "insulation": 100,
            "unconnected": 0
        }
        total_height = module_height_list["base"]

        for module_num, module_name in self.module_name_list.items():
            total_height += module_height_list[module_name]
            
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