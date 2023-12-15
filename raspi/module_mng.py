import serial_com as ser
import RPi.GPIO as GPIO
import threading
import time

IDEN_CYCLE = 0.1 # 搭載モジュール情報更新周期[s]
SURV_CYCLE = 0.01 # 監視周期[s]
"""統合時こいつらは調整する"""

# 抵抗値の許容誤差範囲を定義
err_rate = 20 # 許容誤差範囲[%]
acc_res_range = [240 * (100 - err_rate) / 100, 240 * (100 + err_rate) / 100] # 小物抵抗値範囲
doc_res_range = [1000 * (100 - err_rate) / 100, 1000 * (100 + err_rate) / 100] # 資料抵抗値範囲
ins_res_range = [2000 * (100 - err_rate) / 100, 2000 * (100 + err_rate) / 100] # 保冷・保温抵抗値範囲

# モジュール空き状況未実装
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
        print(f"[INFO][module_mng.py] : モジュール情報初期化中...")
        self.serial = serial_port
        time.sleep(1) # インスタンスを渡し切るまでキープ ※必須なので消さないこと！！！！！
        
        # 排他制御：リソースに同時にアクセスするのを防ぐ
        self.lock = threading.Lock()  # Lockオブジェクトを作成
        time.sleep(1) # インスタンスを渡し切るまでキープ
    
        self.module_info = {
            "module1": {
                "name": "",
                "door1": {
                    "name": "",
                    "pin": {
                        "servo":13,
                        "switch":16
                    },
                    "unlocked": False,
                    "current_state": True
                },
                "door2": {
                    "name": "",
                    "pin": {
                        "servo":15,
                        "switch":18
                    },
                    "unlocked": False,
                    "current_state": True
                }
            },
            "module2": {
                "name": "",
                "door1": {
                    "name": "",
                    "pin": {
                        "servo":19,
                        "switch":22
                    },
                    "unlocked": False,
                    "current_state": True
                },
                "door2": {
                    "name": "",
                    "pin": {
                        "servo":21,
                        "switch":23
                    },
                    "unlocked": False,
                    "current_state": True
                }
            },
            "module3": {
                "name": "",
                "door1": {
                    "name": "",
                    "pin": {
                        "servo":29,
                        "switch":32
                    },
                    "unlocked": False,
                    "current_state": True
                },
                "door2": {
                    "name": "",
                    "pin": {
                        "servo":31,
                        "switch":33
                    },
                    "unlocked": False,
                    "current_state": True
                }
            }
        }
        """搭載モジュール情報"""
        
        self.module_height = {
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
        """各モジュールの高さ[mm]"""
        
        """モジュールを識別し続けるスレッドを走らせる(これ以降常時実行)"""
        identify_module_thread = threading.Thread(target = self.identify_module)
        identify_module_thread.setDaemon(True)
        identify_module_thread.start()
        time.sleep(1) # モジュールを識別し終わるまでキープ
        print(f"[INFO][module_mng.py] : モジュール情報初期化完了")
        
        """モジュールの状態と扉の開閉状態を監視し、取り外しとこじ開けを検知するスレッドを走らせる(これ以降常時実行)"""
        module_surv_thread = {} # モジュール監視スレッド用配列
        door_surv_thread = {} # 扉監視スレッド用配列
        for module_num, module_info in self.module_info.items():
            # モジュール監視スレッドを開始
            module_surv_thread[module_num] = threading.Thread(target = self.module_surv, args = (module_num,))
            module_surv_thread[module_num].setDaemon(True)
            module_surv_thread[module_num].start()
            print(f"[DEBUG][module_mng.py] : {module_num} -> {module_surv_thread[module_num]}") # デバッグ用出力
            
            # 扉監視スレッド用配列のを2次元配列に設定
            door_surv_thread.setdefault(module_num, {})
            for door_num, door_info in module_info.items(): # for文でモジュール番号、扉番号を取り出す
                if "door" in door_num:
                    # 扉監視スレッドを開始
                    door_surv_thread[module_num][door_num] = threading.Thread(target = self.door_surv, args = (module_num, door_num,))
                    door_surv_thread[module_num][door_num].setDaemon(True)
                    door_surv_thread[module_num][door_num].start()
                    print(f"[DEBUG][module_mng.py] : {module_num}-{door_num} -> {door_surv_thread[module_num][door_num]}") # デバッグ用出力
        print(f"[INFO][module_mng.py] : モジュールと扉の状態の監視を開始しました")
        
    def identify_module(self):
        """
        搭載モジュール情報を更新する
        
        モジュールの名前をセット：
            self.module_info[module_num]["name"]: str
        """
        # 一定周期で識別し続ける
        while True:
            # Arduinoに抵抗値測定指令を出す
            result = self.serial.send_and_read_response(3, [], 12)
            response = result[0]
        
            # 抵抗値を計算
            self.resistance_list = [
                response[0]*254+response[1], # 1段目
                response[2]*254+response[3], # 2段目
                response[4]*254+response[5] # 3段目
            ]
            
            with self.lock:  # ロックを取得
                # 搭載モジュール情報を更新
                for num, res in enumerate(self.resistance_list, start=1):
                    if acc_res_range[0] <= res <= acc_res_range[1]:
                        self.module_info[f"module{num}"]["name"] = "accessories"
                        self.module_info[f"module{num}"]["door1"]["name"] = "left"
                        self.module_info[f"module{num}"]["door2"]["name"] = "right"
                    elif doc_res_range[0] <= res <= doc_res_range[1]:
                        self.module_info[f"module{num}"]["name"] = "document"
                        self.module_info[f"module{num}"]["door1"]["name"] = "under"
                        self.module_info[f"module{num}"]["door2"]["name"] = "upper"
                    elif ins_res_range[0] <= res <= ins_res_range[1]:
                        self.module_info[f"module{num}"]["name"] = "insulation"
                        self.module_info[f"module{num}"]["door1"]["name"] = "left"
                        self.module_info[f"module{num}"]["door2"]["name"] = "right"
                    else:
                        self.module_info[f"module{num}"]["name"] = "unconnected"
                        self.module_info[f"module{num}"]["door1"]["name"] = ""
                        self.module_info[f"module{num}"]["door2"]["name"] = ""
                
            time.sleep(IDEN_CYCLE)
            
    def module_surv(self, module_num: str):
        """
        モジュールの状態を監視し、取り外しを検知する
        
        引数：
            module_num : 監視したいモジュールの番号 -> str
        """
        # モジュールと扉の情報を初期化
        name_module_previous = self.module_info[module_num]["name"]
        
        # 一定周期で監視し続ける
        while True:
            with self.lock:  # ロックを取得
                # 現在のモジュールの情報を取得
                name_module_current = self.module_info[module_num]["name"]
                
                # モジュールの状態が変わった時
                if name_module_current != name_module_previous:
                    # 未接続の場合
                    if name_module_current == "unconnected":
                        print(f"[INFO][module_mng.py] : {name_module_previous}が取り外されました")
                    # 取り付けられた場合
                    else:
                        print(f"[INFO][module_mng.py] : {name_module_current}が取り付けられました")
                    # フラグを更新
                    name_module_previous = name_module_current
                    
            time.sleep(SURV_CYCLE)
    
    def door_surv(self, module_num: str, door_num: str):
        """
        扉の開閉状態を監視し、こじ開けを検知する
        
        引数：
            module_num : 監視したい扉のモジュール番号 -> str
            
            door_num : 監視したい扉の番号 -> str
            
        扉の開閉状態のフラグをセット：
            self.module_info[module_num][door_num]["current_state"]: bool
        """
        # 扉が開いているときにTrueとした（内部プルアップ） -> マイクロスイッチが押されている時に導通
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.module_info[module_num][door_num]["pin"]["switch"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # モジュールと扉の情報を初期化
        self.module_info[module_num][door_num]["current_state"]: bool = GPIO.input(self.module_info[module_num][door_num]["pin"]["switch"])
        state_door_previous = self.module_info[module_num][door_num]["current_state"]
        
        # 一定周期で監視し続ける
        while True:
            with self.lock:  # ロックを取得
                # 現在の扉の情報を取得
                name_module_current = self.module_info[module_num]["name"]
                name_door_current = self.module_info[module_num][door_num]["name"]
                self.module_info[module_num][door_num]["current_state"]: bool = GPIO.input(self.module_info[module_num][door_num]["pin"]["switch"])
                
                # 扉の状態が変わった時
                if self.module_info[module_num][door_num]["current_state"] != state_door_previous:
                    # 扉が開いた場合
                    if self.module_info[module_num][door_num]["current_state"]:
                        # サーボで解錠した場合
                        if self.module_info[module_num][door_num]["unlocked"]:
                            print(f"[INFO][module_mng.py] : {name_module_current}-{name_door_current}を解錠しました")
                        # サーボで解錠していない場合
                        else:
                            print(f"[INFO][module_mng.py] : {name_module_current}-{name_door_current}のこじ開けを検知しました")
                    # 扉が閉じた場合
                    else:
                        self.module_info[module_num][door_num]["unlocked"]: bool = False # こじ開け検知用フラグをもとに戻す
                        print(f"[INFO][module_mng.py] : {name_module_current}-{name_door_current}が閉じました")
                    # フラグを更新
                    state_door_previous = self.module_info[module_num][door_num]["current_state"]
                    
            time.sleep(SURV_CYCLE)
            
    def door_open(self, module_num: str, door_num: str):
        """
        扉の鍵を解錠する
        
        引数：
            module_num : 解錠したい扉のモジュール番号 -> str
            
            door_num : 解錠したい扉の番号 -> str
        """
        # ピンの初期化
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.module_info[module_num][door_num]["pin"]["servo"], GPIO.OUT)
        
        # ピンにHIGHを出力
        self.module_info[module_num][door_num]["unlocked"]: bool = True # こじ開け検知用フラグを"解錠した"に設定
        GPIO.output(self.module_info[module_num][door_num]["pin"]["servo"], True)
        time.sleep(0.5) # なぜか待たないと動かない
        
        # ArduinoにPWM出力指令を出す
        self.serial.send(10, [])
        print(f"[INFO][module_mng.py] : 解錠中...")
        time.sleep(3) # Arduino側のサーボを開けてから閉じるまでの時間が3s
        time.sleep(0.5) # 0.5s余裕を持たせておく

        # ピンにLOWを出力
        GPIO.output(self.module_info[module_num][door_num]["pin"]["servo"], False)
        time.sleep(0.5) # 0.5s余裕を持たせておく
        
        # GPIOピンを解放
        GPIO.cleanup(self.module_info[module_num][door_num]["pin"]["servo"])
            
    def battery_surv(self):
        """
        バッテリー電圧を監視する
        
        戻り値：
            バッテリー電圧[V]
        """
        # Arduinoに電圧測定指令を出す
        response = self.serial.send_and_read_response(5, [], 11)
        batt_vol = response[0][0]/10
        
        return batt_vol
    
    def height_calculate(self):
        """
        機体の高さを算出する関数
            
        戻り値：
            機体全体の高さ[mm]
        """
        # 土台の高さを設定
        total_height = self.module_height["base"]["height"]
        
        # 接続されているモジュールの高さを足し合わせる
        for module in self.module_info.values():
            total_height += self.module_height[module["name"]]["height"]
            
        return total_height
            
            
if __name__ == '__main__':
    s = ser.arduino_serial()
    m = module_controller(s)