import pandas as pd
import uuid
import time
import threading
import datetime
import sys
import os
import warnings


#warnings.filterwarnings('ignore', category=pd.core.common.SettingWithCopyWarning)

def option_to_datetime(opt):
    strdt = opt.split("-")[-1]
    hour = int(strdt.split(":")[0])
    minute = int(strdt.split(":")[-1])
    return datetime.datetime.combine(datetime.datetime.today(), datetime.time(hour, minute))

#移動時間の見積もり 10分単位
TIME_MARGIN = 20

STATUS_LABELS = ["NOT_ACCEPTED_YET",
                "MAIL_SENT",
                "DENIED",
                "ACCEPT_DENIED",
                "ACCEPTED",
                "MOVING_FOR_PICKUP",
                "WAITING_FOR_PICKUP",
                "PICKUP_TIMEOUT",
                "PICKED_UP",
                "MOVING_FOR_RECEIVE",
                "WAITING_FOR_RECEIVE",
                "RECEIVE_TIMEOUT",
                "RECEIVED"]




class order_manager():
    """
    オーダーを管理するクラス
    ・order_info.csvに対する【書き込み】はこのオブジェクト経由で行うこと  
    複数プロセスがcsvに対してアクセスすると同期がとれなかったりpermission errorになる可能性がある
    """
    def __init__(self, path:str = "/home/pi/git/mirs2302/raspi/order_info.csv"):
        """
        オーダーを管理するクラス
                
        ・order_info.csvに対する【書き込み】はこのオブジェクト経由で行うこと  
        
        複数プロセスがcsvに対してアクセスすると同期がとれなかったりpermission errorになる可能性がある
                
                
        コンストラクタ
        
        引数：
            path : csvファイルのパス（任意・デフォルト値あり）
        """            
        self.file_path = path
        self.reflesh()

    
    def reflesh(self):
        """
        変更に応じてcsvを更新する関数
        """
        retries = 0
        while(1):
            try:
                self.df = pd.read_csv(self.file_path, parse_dates=["RECEIPT_TIME", "ACCEPTED_TIME", "PICKUP_TIME", "RECEIVE_TIME"] )
                break
            except:
                time.sleep(0.5)
                retries += 1
            if retries > 5:
                print("[ERR][order_mng] : 例外をキャッチしました", file = sys.stderr)
                return -1
            
        self.read = datetime.datetime.now()
    
    
    def update(self):
        #最後にcsvを読んだ時刻　以降に　csvに変更が加えられていた場合
        if self.read < datetime.datetime.fromtimestamp(os.stat(self.file_path).st_mtime):
            print("[WARN][order_mng.py] : ファイルの書き込みに失敗しました", file = sys.stderr)
            return -1

        self.df.to_csv(self.file_path, index = None, date_format='%Y-%m-%d %H:%M:%S')
        return 1
    
    def get_order(self, label:str = None, value = None):
        """
        オーダー情報を取得する関数
        
        使い方 1: インデックス
            get_order(0) : 0番目のオーダーを取得する
        
        使い方 2: ラベルと値
            get_order("ORDER_TYPE", "RECEIVE") : ORDER_TYPEがRECEIVEになっているものだけを抽出
            
        
        戻り値：
            該当する注文情報(DataFrame)
            ない場合には-1を返す
        """
        self.reflesh()
        if type(label) == int:
            return self.df.iloc[label]
        elif label in self.df.columns and value != None:
            try:
                ret = self.df.set_index(label, drop=False).loc[value]
                if len(ret.shape) == 1:
                    return pd.DataFrame(ret).T
                else:
                    return ret
            except KeyError:
                #print("[WARN][order_mng.py] : order_manager.get_order()の引数に該当するオーダーは見つかりませんでした", file = sys.stderr)
                return -1
        else:
            print("[ERR][order_mng.py] : order_manager.get_order()の引数が不正です", file = sys.stderr)
            return -1
    
    
    def modify_order(self, ID:str, key:str, value):
        """
        オーダーを更新する関数
        
        引数：
            ID : オーダーID
            key : 変更する値の種類
            value : 変更後の値
        """
        while(1):
            self.reflesh()
            row =  self.df.query(f"ID == '{ID}'").index[-1]
            self.df.iloc[row, self.df.columns.get_loc(key)] = value
            if self.update() > 0:
                break
    
    def new_order(self,
                  ORDER_TYPE:str = None,
                  ITEM_TYPE:str = None,
                  ITEM_NAME:str = None,
                  STATUS:str = "NOT_ACCEPTED_YET",
                  RECEIPT_TIME:datetime.datetime = datetime.datetime.now(),
                  ACCEPTED_TIME:datetime.datetime = None,
                  SENDER:str = None,
                  RECEIVER:str = None,
                  PICKUP_PLACE:int = None,
                  PICKUP_TIME:datetime.datetime = None,
                  PICKUP_PIN:str = None,
                  RECEIVE_PLACE:int = None,
                  RECEIVE_TIME:datetime.datetime = None,
                  RECEIVE_PIN:str = None,
                  NOTE:str = None,
                  ) -> str:
        
        """
        オーダーを作成し追加する関数
        引数はすべて任意・いくつかデフォルト値がある
        
        引数：
                  ORDER_TYPE: オーダーの種類 [SEND, RECEIVE, DELIVERY]
                  STATUS: ステータス (デフォルト値あり -> "NOT_ACCEPTED_YET")
                  RECEIPT_TIME: オーダーを受け取った時間 (デフォルト値あり -> 現在時刻のdatetimeオブジェクト)
                  ACCEPTED_TIME: オーダーが承認された時間
                  SENDER: 発送をすることになる人間のメールアドレス
                  RECEIVER: 受け取りをすることになる人間のメールアドレス
                  PICKUP_PLACE: 発送をする場所のインデックス
                  PICKUP_TIME: 発送場所への到着予定時間
                  RECEIVE_PLACE: 受け取りをする場所のインデックス
                  RECEIVE_TIME: 受け取り場所への到着予定時間
                  
        戻り値：
                追加されたオーダーのID
        """
        while(1):
            self.reflesh()
            ID = str(uuid.uuid4())
            #s = pd.DataFrame([[ID,ORDER_TYPE,STATUS,RECEIPT_TIME,ACCEPTED_TIME,SENDER,RECEIVER,PICKUP_PLACE,PICKUP_TIME,RECEIVE_PLACE,RECEIVE_TIME]], columns = self.df.columns)
            s = pd.DataFrame([[ID,ORDER_TYPE,ITEM_TYPE,ITEM_NAME,STATUS,RECEIPT_TIME,ACCEPTED_TIME,SENDER,RECEIVER,PICKUP_PLACE,PICKUP_TIME,PICKUP_PIN,RECEIVE_PLACE,RECEIVE_TIME,RECEIVE_PIN,NOTE]], columns = self.df.columns)
            self.df = pd.concat([self.df, s], axis = 0, join = "outer")
            if self.update() > 0:
                break
        return ID

    
    def get_box_usage(self):
        """
        箱ごとに、すでに入っている予約を抽出する関数
        """
        usage = {"小物1" : {"begin": [], "end" : []},
                "小物2" :  {"begin": [], "end" : []},
                "書類1" :  {"begin": [], "end" : []},
                "書類2" :  {"begin": [], "end" : []},
                "食品（保冷）" :  {"begin": [], "end" : []},
                "食品（保温）" :  {"begin": [], "end" : []}}

        #すべての箱について抽出する
        for box in usage.keys():
            df = self.get_order("ITEM_TYPE", box)
            #なければ無視
            if type(df) == int and df == -1:
                continue
            
            
            for i in range(len(df)):
                #箱が埋まっていないオーダーについては除外する
                if df["STATUS"].iloc[i] in ["DENIED", "ACCEPT_DENIED", "PICKUP_TIMEOUT", "RECEIVED"]:
                    continue
                
                if pd.isna(df["PICKUP_TIME"].iloc[i]):
                    #積み込み時刻が未定ならば今から枠を確保しておく
                    usage[box]["begin"].append(datetime.datetime.now())
                else:
                    #積み込み時刻がわかっているならその時間から枠を確保する
                    usage[box]["begin"].append(df["PICKUP_TIME"].iloc[i])
                
                if pd.isna(df["RECEIVE_TIME"].iloc[i]):
                    #受け取り時刻が未定ならばめちゃ未来まで枠を確保しておく
                    usage[box]["end"].append(datetime.datetime(year = 2050, month = 12, day = 31, hour = 23, minute = 50))
                else:
                    #受け取り時刻がわかっているならそれまで枠を確保しておく
                    usage[box]["end"].append(df["RECEIVE_TIME"].iloc[i])
                
        return usage
        
        
    def get_moving_schedule(self):
        """
        移動の予定を取得する関数
        """
        movements = []
        
        
        #すべてのオーダーに対して
        for status in STATUS_LABELS:
            #もう移動の必要がないオーダについては除外する
            if status in ["DENIED", "ACCEPT_DENIED", "PICKUP_TIMEOUT", "RECEIVE_TIMEOUT", "RECEIVED"]:
                continue
            
            df = self.get_order("STATUS", status)
            if type(df) == int and df == -1:
                continue
            
            for i in range(len(df)):
                #積み込み時間・場所が未定でなければ
                if not(pd.isna(df["PICKUP_TIME"].iloc[i])):
                    movements.append({"begin": (df["PICKUP_TIME"].iloc[i] - datetime.timedelta(minutes = TIME_MARGIN + 1)),
                                      "end": (df["PICKUP_TIME"].iloc[i] + datetime.timedelta(minutes = TIME_MARGIN + 1))})
                    
                
                #受け取り時間・場所が未定でなければ
                if not(pd.isna(df["RECEIVE_TIME"].iloc[i])):
                    movements.append({"begin": (df["RECEIVE_TIME"].iloc[i] - datetime.timedelta(minutes = TIME_MARGIN + 1)),
                                      "end": (df["RECEIVE_TIME"].iloc[i] + datetime.timedelta(minutes = TIME_MARGIN + 1))})
        
        return movements
    
    
    
    def box_decider(self, box_type:str, PICKUP_TIME:datetime.datetime = None, RECEIVE_TIME:datetime.datetime = None):
        """
        箱を決める関数
        引数：
            box_type : {"小物", "書類", "食品（保冷）", "食品（保温）"} のどれかであること
            PICKUP_TIME : 【任意】未定でない限り指定すること
            RECEIVE_TIME : 【任意】未定でない限り指定すること
        使える箱がないときは-1を返す
        """
        #移動時間の余裕があるかチェック
        time_isok = True
        movements = self.get_moving_schedule()
        if PICKUP_TIME != None:
            for m in movements:
                if m["begin"] <= PICKUP_TIME <= m["end"]:
                    time_isok = False
                    break
        
        if RECEIVE_TIME != None:
            for m in movements:
                if m["begin"] <= RECEIVE_TIME <= m["end"]:
                    time_isok = False
                    break
        
        if not(time_isok):
            return -1
        
        
        
        #箱が開いてるかチェック
        box_usage = self.get_box_usage()
        
        if PICKUP_TIME == None:
            begin = datetime.datetime.now()
        else:
            begin = PICKUP_TIME
        
        if RECEIVE_TIME == None:
            end = datetime.datetime(year = 2050, month = 12, day = 31, hour = 23, minute = 50)
        else:
            end = RECEIVE_TIME
        
        if begin >= end:
            return -1
        if end - begin <= datetime.timedelta(minutes = TIME_MARGIN):
            return -1
        
        if box_type == "小物":
            for box in ["小物1", "小物2"]:
                available = True
                for i in range(len(box_usage[box]["begin"])):
                    #箱の使用時間が被っているなら無効
                    if box_usage[box]["begin"][i] <= begin <= box_usage[box]["end"][i]\
                    or\
                    box_usage[box]["begin"][i] <= end <= box_usage[box]["end"][i]\
                    or\
                    begin <= box_usage[box]["begin"][i] <= box_usage[box]["end"][i] <= end\
                    :
                        available = False
                        break
                #有効なら使える箱のラベルを返す
                if available:
                    return box
        
        elif box_type == "書類":
            for box in ["書類1", "書類2"]:
                available = True
                for i in range(len(box_usage[box]["begin"])):
                    #箱の使用時間が被っているなら無効
                    if box_usage[box]["begin"][i] <= begin <= box_usage[box]["end"][i]\
                    or\
                    box_usage[box]["begin"][i] <= end <= box_usage[box]["end"][i]\
                    or\
                    begin <= box_usage[box]["begin"][i] <= box_usage[box]["end"][i] <= end\
                    :
                        available = False
                        break
                #有効なら使える箱のラベルを返す
                if available:
                    return box
        
        elif box_type == "食品（保冷）":
            box = "食品（保冷）"
            available = True
            for i in range(len(box_usage[box]["begin"])):
                #箱の使用時間が被っているなら無効
                if box_usage[box]["begin"][i] <= begin <= box_usage[box]["end"][i]\
                or\
                box_usage[box]["begin"][i] <= end <= box_usage[box]["end"][i]\
                or\
                begin <= box_usage[box]["begin"][i] <= box_usage[box]["end"][i] <= end\
                :
                    available = False
                    break
            #有効なら使える箱のラベルを返す
            if available:
                return box
        
        elif box_type == "食品（保温）":
            box = "食品（保温）"
            available = True
            for i in range(len(box_usage[box]["begin"])):
                #箱の使用時間が被っているなら無効
                if box_usage[box]["begin"][i] <= begin <= box_usage[box]["end"][i]\
                or\
                box_usage[box]["begin"][i] <= end <= box_usage[box]["end"][i]\
                or\
                begin <= box_usage[box]["begin"][i] <= box_usage[box]["end"][i] <= end\
                :
                    available = False
                    break
            #有効なら使える箱のラベルを返す
            if available:
                return box
            
        return -1
        
        
        
        
        
        
        
                
    

if __name__ == '__main__':
    """
    コマンドライン(phpなど)から実行された時の処理
    """
    o = order_manager()
    """
    o.new_order(
            ORDER_TYPE="RECEIVE",
            ITEM_NAME="TEST2",
            ITEM_TYPE="書類",
            PICKUP_PLACE="PLACE2",
            PICKUP_TIME=option_to_datetime("15:30-15:40"))

    
    exit()
    """
    # 追加のコマンドライン引数がある場合
    if(len(sys.argv) > 1):
        
        # 新しい注文情報を書き込む
        if(sys.argv[1] == 'new_order'):
            # 備考の有無判別
            if sys.argv[2] != 'ORDER':
                if(len(sys.argv) > 10):
                    note = sys.argv[10]
                else:
                    note = ""
            # "送る"の場合
            if(sys.argv[2] == 'SEND'):
                if o.box_decider(sys.argv[3]) != -1:
                    o.new_order(
                        ORDER_TYPE = sys.argv[2],
                        ITEM_TYPE = o.box_decider(sys.argv[3]),
                        ITEM_NAME = sys.argv[4],
                        SENDER = sys.argv[5],
                        RECEIVER = sys.argv[6],
                        PICKUP_PLACE = sys.argv[7],
                        PICKUP_TIME = option_to_datetime(sys.argv[8]),
                        PICKUP_PIN = sys.argv[9],
                        NOTE = note
                    )
            # "送ってもらう"の場合
            elif(sys.argv[2] == 'RECEIVE'):
                if o.box_decider(sys.argv[3]) != -1:
                    o.new_order(
                        ORDER_TYPE = sys.argv[2],
                        ITEM_TYPE = o.box_decider(sys.argv[3]),
                        ITEM_NAME = sys.argv[4],
                        SENDER = sys.argv[5],
                        RECEIVER = sys.argv[6],
                        RECEIVE_PLACE = sys.argv[7],
                        RECEIVE_TIME = option_to_datetime(sys.argv[8]),
                        RECEIVE_PIN = sys.argv[9],
                        NOTE = note
                    )
            # "注文する"の場合
            elif(sys.argv[2] == 'ORDER'):
                if(len(sys.argv) > 9):
                    note = sys.argv[9]
                else:
                    note = ""
                if o.box_decider(sys.argv[3]) != -1:
                    o.new_order(
                        ORDER_TYPE = sys.argv[2],
                        ITEM_TYPE = o.box_decider(sys.argv[3]),
                        ITEM_NAME = sys.argv[4],
                        RECEIVER = sys.argv[5],
                        RECEIVE_PLACE = sys.argv[6],
                        RECEIVE_TIME = option_to_datetime(sys.argv[7]),
                        RECEIVE_PIN = sys.argv[8],
                        NOTE = note
                    )
                
        # 指定のIDのオーダー情報(DataFrame)を入手する
        if(sys.argv[1] == 'get_order'):
            result = o.get_order(
                "ID",
                sys.argv[2]
            )
            out = f"\"SENDER\":\"{result.iloc[0]['SENDER']}\",\
                   \"ITEM_NAME\":\"{result.iloc[0]['ITEM_NAME']}\""
            if not(pd.isna(result.iloc[0]['NOTE'])):
                out += f",\"NOTE\":\"{result.iloc[0]['NOTE']}\""
            
            print("{" + out.replace(" ","") + "}", end = "")

        #承認されたデータでcsvを更新する
        if(sys.argv[1] == 'modify_order'):
            if "TIME" in sys.argv[3]:
                result = o.modify_order(
                    ID = sys.argv[2],
                    key = sys.argv[3],
                    value = option_to_datetime(sys.argv[4])
                    )
            else:
                result = o.modify_order(
                    ID = sys.argv[2],
                    key = sys.argv[3],
                    value = sys.argv[4]
                    )
                