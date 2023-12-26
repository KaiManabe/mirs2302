""" 
モジュールを制御,機体を制御するクラスが定義されている

このファイルをインポートして、module_controller(), airframe_controllerというオブジェクトを定義してつかう
"""

import serial_com as ser
import web_app
import RPi.GPIO as GPIO
import threading
import time

IDEN_CYCLE = 0.1 # 搭載モジュール情報更新周期[s]
AIR_CYCLE = 0.1 # 機体持ち去り検知周期[s]
SURV_CYCLE = 0.01 # 監視周期[s]
RES_ERR_RATE = 10 # 抵抗値許容誤差範囲[%]
"""統合時こいつらは調整する"""

# 詳細設計書には書いてあるけど、モジュール空き状況はweb_app.pyでやった方がいいかも
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
        print("[INFO][module_mng.py] : モジュール情報初期化中...")
        self.serial = serial_port
        time.sleep(1) # インスタンスを渡し切るまでキープ ※必須なので消さないこと！！！！！
        
        # 排他制御：リソースに同時にアクセスするのを防ぐ
        self.lock = threading.Lock()  # Lockオブジェクトを作成
        time.sleep(1) # インスタンスを渡し切るまでキープ
    
        self.onb_module_info = {
            "module1": {
                "name": "",
                "door1": {
                    "name": "",
                    "pin": {
                        "SERVO":13,
                        "SWITCH":16
                    },
                    "unlocked": False,
                    "open": True
                },
                "door2": {
                    "name": "",
                    "pin": {
                        "SERVO":15,
                        "SWITCH":18
                    },
                    "unlocked": False,
                    "open": True
                }
            },
            "module2": {
                "name": "",
                "door1": {
                    "name": "",
                    "pin": {
                        "SERVO":19,
                        "SWITCH":22
                    },
                    "unlocked": False,
                    "open": True
                },
                "door2": {
                    "name": "",
                    "pin": {
                        "SERVO":21,
                        "SWITCH":23
                    },
                    "unlocked": False,
                    "open": True
                }
            },
            "module3": {
                "name": "",
                "door1": {
                    "name": "",
                    "pin": {
                        "SERVO":29,
                        "SWITCH":32
                    },
                    "unlocked": False,
                    "open": True
                },
                "door2": {
                    "name": "",
                    "pin": {
                        "SERVO":31,
                        "SWITCH":33
                    },
                    "unlocked": False,
                    "open": True
                }
            }
        }
        """搭載モジュール情報"""
        
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
        """各モジュールの高さ[mm]"""
        
        """モジュールを識別し続けるスレッドを走らせる(これ以降常時実行)"""
        identify_module_thread = threading.Thread(target = self.identify_module)
        identify_module_thread.setDaemon(True)
        identify_module_thread.start()
        time.sleep(1) # モジュールを識別し終わるまでキープ
        print("[INFO][module_mng.py] : モジュール情報初期化完了")
        
        """モジュールの状態を監視して取り外しを検知するスレッド・扉の開閉状態を監視してこじ開けを検知するスレッド走らせる(これ以降常時実行)"""
        module_surv_thread = {} # モジュール監視スレッド用配列
        door_surv_thread = {} # 扉監視スレッド用配列
        for module_num, module_info in self.onb_module_info.items():
            # モジュール監視スレッドを開始
            module_surv_thread[module_num] = threading.Thread(target = self.module_surv, args = (module_num,))
            module_surv_thread[module_num].setDaemon(True)
            module_surv_thread[module_num].start()
            
            # 扉監視スレッド用配列のを2次元配列に設定
            door_surv_thread.setdefault(module_num, {})
            for door_num, door_info in module_info.items(): # for文でモジュール番号、扉番号を取り出す
                if "door" in door_num:
                    # 扉監視スレッドを開始
                    door_surv_thread[module_num][door_num] = threading.Thread(target = self.door_surv, args = (module_num, door_num,))
                    door_surv_thread[module_num][door_num].setDaemon(True)
                    door_surv_thread[module_num][door_num].start()
        print("[INFO][module_mng.py] : モジュールと扉の状態の監視を開始しました")
        
    def identify_module(self):
        """
        *** module_controller()をオブジェクト化した際に、自動でスレッドが立ち上げられるので使用しないこと ***
        搭載モジュール情報を更新し続ける
        
        モジュールと扉の名前をセット：
            self.onb_module_info[module_num]["name"] -> str
            self.onb_module_info[module_num][door_num]["name"] -> str
        """
        # 抵抗値の許容誤差範囲を定義
        acc_res_range = [240 * (100 - RES_ERR_RATE) / 100, 240 * (100 + RES_ERR_RATE) / 100] # 小物抵抗値範囲
        doc_res_range = [1000 * (100 - RES_ERR_RATE) / 100, 1000 * (100 + RES_ERR_RATE) / 100] # 資料抵抗値範囲
        ins_res_range = [2000 * (100 - RES_ERR_RATE) / 100, 2000 * (100 + RES_ERR_RATE) / 100] # 保冷・保温抵抗値範囲
        
        # 一定周期で搭載モジュール情報を更新し続ける
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
                        self.onb_module_info[f"module{num}"]["name"] = "accessories"
                        self.onb_module_info[f"module{num}"]["door1"]["name"] = "right"
                        self.onb_module_info[f"module{num}"]["door2"]["name"] = "left"
                    elif doc_res_range[0] <= res <= doc_res_range[1]:
                        self.onb_module_info[f"module{num}"]["name"] = "document"
                        self.onb_module_info[f"module{num}"]["door1"]["name"] = "under"
                        self.onb_module_info[f"module{num}"]["door2"]["name"] = "upper"
                    elif ins_res_range[0] <= res <= ins_res_range[1]:
                        self.onb_module_info[f"module{num}"]["name"] = "insulation"
                        self.onb_module_info[f"module{num}"]["door1"]["name"] = "right"
                        self.onb_module_info[f"module{num}"]["door2"]["name"] = "left"
                    else:
                        self.onb_module_info[f"module{num}"]["name"] = "unconnected"
                        self.onb_module_info[f"module{num}"]["door1"]["name"] = ""
                        self.onb_module_info[f"module{num}"]["door2"]["name"] = ""
                
            time.sleep(IDEN_CYCLE)
            
    def module_surv(self, module_num: str):
        """
        *** module_controller()をオブジェクト化した際に、自動でスレッドが立ち上げられるので使用しないこと ***
        モジュールの状態を監視し、取り外しを検知する
        
        引数：
            module_num : 監視したいモジュールの番号 -> str
        """
        # モジュールと扉の情報を初期化
        name_module_previous = self.onb_module_info[module_num]["name"]
        
        # 一定周期で監視し続ける
        while True:
            with self.lock:  # ロックを取得
                # 現在のモジュールの情報を取得
                name_module_current = self.onb_module_info[module_num]["name"]
                
                # モジュールの状態が変わった時
                if name_module_current != name_module_previous:
                    # 未接続の場合
                    if name_module_current == "unconnected":
                        print(f"[INFO][module_mng.py] : {name_module_previous}が取り外されました")
                        web_app.warning(warn_type="door", module=name_module_current) # 異常検知メールを送信
                    # 取り付けられた場合
                    else:
                        print(f"[INFO][module_mng.py] : {name_module_current}が取り付けられました")
                    # フラグを更新
                    name_module_previous = name_module_current
                    
            time.sleep(SURV_CYCLE)
    
    def door_surv(self, module_num: str, door_num: str):
        """
        *** module_controller()をオブジェクト化した際に、自動でスレッドが立ち上げられるので使用しないこと ***
        扉の開閉状態を監視し、こじ開けを検知する
        
        引数：
            module_num : 監視したい扉のモジュール番号 -> str
            
            door_num : 監視したい扉の番号 -> str
            
        扉の開閉状態のフラグをセット：
            self.onb_module_info[module_num][door_num]["open"]: bool
        """
        # 扉が開いているときにTrueとした（内部プルアップ） -> マイクロスイッチが押されている時に導通
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.onb_module_info[module_num][door_num]["pin"]["SWITCH"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # モジュールと扉の情報を初期化
        self.onb_module_info[module_num][door_num]["open"]: bool = GPIO.input(self.onb_module_info[module_num][door_num]["pin"]["SWITCH"])
        openFlag_door_previous = self.onb_module_info[module_num][door_num]["open"]
        
        # 一定周期で監視し続ける
        while True:
            with self.lock:  # ロックを取得
                # 現在の扉の情報を取得
                name_module_current = self.onb_module_info[module_num]["name"]
                name_door_current = self.onb_module_info[module_num][door_num]["name"]
                self.onb_module_info[module_num][door_num]["open"]: bool = GPIO.input(self.onb_module_info[module_num][door_num]["pin"]["SWITCH"])
                
                # 扉の状態が変わった時
                if self.onb_module_info[module_num][door_num]["open"] != openFlag_door_previous and name_module_current != "unconnected":
                    # 扉が開いた場合
                    if self.onb_module_info[module_num][door_num]["open"]:
                        # サーボで解錠した場合
                        if self.onb_module_info[module_num][door_num]["unlocked"]:
                            print(f"[INFO][module_mng.py] : {name_module_current}-{name_door_current}を解錠しました")
                        # サーボで解錠していない場合
                        else:
                            print(f"[INFO][module_mng.py] : {name_module_current}-{name_door_current}のこじ開けを検知しました")
                            web_app.warning(warn_type="door", module=name_module_current, door=name_door_current) # 異常検知メールを送信
                    # 扉が閉じた場合
                    else:
                        self.onb_module_info[module_num][door_num]["unlocked"]: bool = False # こじ開け検知用フラグをもとに戻す
                        print(f"[INFO][module_mng.py] : {name_module_current}-{name_door_current}が閉じました")
                    # フラグを更新
                    openFlag_door_previous = self.onb_module_info[module_num][door_num]["open"]
                    
            time.sleep(SURV_CYCLE)
            
    def door_open(self, module_name: str, door_name: str):
        """
        扉の鍵を解錠する
        
        引数：
            module_name : 解錠したい扉のモジュール名 -> str
            
            door_name : 解錠したい扉の名 -> str
        """
        # モジュール名・扉名に対応するモジュール番号・扉番号を引っ張る
        module_num, door_num = self.reverse_lookup(module_name, door_name)
        
        if module_num != None and door_num != None:
            # ピンの初期化
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.onb_module_info[module_num][door_num]["pin"]["SERVO"], GPIO.OUT)
            
            # ピンにHIGHを出力
            self.onb_module_info[module_num][door_num]["unlocked"]: bool = True # こじ開け検知用フラグを"解錠した"に設定
            GPIO.output(self.onb_module_info[module_num][door_num]["pin"]["SERVO"], True)
            time.sleep(0.5) # なぜか待たないと動かない
            
            # ArduinoにPWM出力指令を出す
            if door_name == "right":
                rot_dir = 0
            else:
                rot_dir = 1
            self.serial.send(10, [rot_dir])
            print(f"[INFO][module_mng.py] : 解錠中...")
            time.sleep(3) # Arduino側のサーボを開けてから閉じるまでの時間が3s
            time.sleep(0.5) # 0.5s余裕を持たせておく

            # ピンにLOWを出力
            GPIO.output(self.onb_module_info[module_num][door_num]["pin"]["SERVO"], False)
            time.sleep(0.5) # 0.5s余裕を持たせておく
            
            # GPIOピンを解放
            GPIO.cleanup(self.onb_module_info[module_num][door_num]["pin"]["SERVO"])
    
    def height_calculate(self):
        """
        機体の高さを算出する関数
            
        戻り値：
            機体全体の高さ[mm]
        """
        # 土台の高さを設定
        total_height = self.each_module_info["base"]["height"]
        
        # 接続されているモジュールの高さを足し合わせる
        for module in self.onb_module_info.values():
            total_height += self.each_module_info[module["name"]]["height"]
            
        return total_height
    
    def reverse_lookup(self, module_name: str, door_name: str):
        """
        モジュールの名前・扉の名前からモジュール番号・扉番号を逆算する関数（動作未確認）
        
        引数：
            module_name : モジュールの名前
            door_name : 扉の名前
            
        戻り値：
            module_num : モジュール番号(str)
            door_num : 扉番号(str)
        """
        with self.lock: # ロックを取得
            # 各モジュールの名前・各扉の名前を比較し、一致した時の値を返す
            for module_num, module_data in self.onb_module_info.items():
                for door_num, door_data in module_data.items():
                    # ドア情報にアクセスするために辞書内の文字列キーを確認
                    if door_num.startswith("door") and isinstance(door_data, dict):
                        org_module_name = module_data.get("name")
                        org_door_name = door_data.get("name")
                        # モジュールの名前・扉の名前が指定されたものと一致した場合
                        if org_module_name == module_name and org_door_name == door_name:
                            return module_num, door_num
            # 何も一致しなかった場合、空の文字列を返す
            return None, None

            
class airframe_controller():
    """
    機体を制御するクラス
    """
    def __init__(self, serial_port: ser.arduino_serial):
        """
        コンストラクタ
        
        引数：
            serial_port -> serial object : serial_com.pyのarduino_serialクラスのオブジェクトを渡す
        """
        self.serial = serial_port
        time.sleep(1) # インスタンスを渡し切るまでキープ ※必須なので消さないこと！！！！！
        
        self.airframe_taken: bool = False # 機体持ち去り検知フラグ
        
        """機体の持ち去りを検知し続けるスレッドを走らせる(これ以降常時実行)"""
        airframe_surv_thread = threading.Thread(target = self.airframe_surv)
        airframe_surv_thread.setDaemon(True)
        airframe_surv_thread.start()
        print("[INFO][module_mng.py] : 機体持ち去りの監視を開始しました")
        
    def airframe_surv(self):
        """
        *** airframe_controller()をオブジェクト化した際に、自動でスレッドが立ち上げられるので使用しないこと ***
        機体の持ち去りを検知する
        """
        # 一定周期で検知し続ける
        while True:
            # Arduinoに機体持ち去り検知指令を出す
            response = self.serial.send_and_read_response(11, [], 16)
            self.airframe_taken = response[0][0]
            
            # 持ち去りを検知した時（1回のみ実行される）
            if self.airframe_taken:
                print("[INFO][module_mng.py] : 機体の持ち去りを検知しました")
                web_app.warning(warn_type="airframe") # 異常検知メールを送信
                break
            
            time.sleep(AIR_CYCLE)
        
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
            
if __name__ == '__main__':
    s = ser.arduino_serial()
    m = module_controller(s)
    a = airframe_controller(s)