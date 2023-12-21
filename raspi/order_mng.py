import pandas as pd
import uuid
import time
import threading
import datetime
import sys



class order_manager():
    """
    オーダーを管理する関数
    order_info.csvに対する【書き込み】はこのオブジェクト経由で行うこと  
    複数プロセスがcsvに対してアクセスすると同期がとれなかったりpermission errorになる可能性がある
    """
    def __init__(self, path:str = "/home/pi/git/mirs2302/raspi/order_info.csv"):
        """
        コンストラクタ
        
        引数：
            path : csvファイルのパス（任意・デフォルト値あり）
        """
        self.file_path = path
        self.changed = datetime.datetime.now()
        self.wrote = datetime.datetime.now()
        self.df = pd.read_csv(path)
        t = threading.Thread(target = self.reflesh)
        t.setDaemon(True)
        t.start()

    
    def reflesh(self):
        """
        【自動実行するため呼び出し不要】
        変更に応じてcsvを更新する関数
        """
        while(1):
            if self.changed > self.wrote:
                self.df.to_csv(self.file_path, index = None)
                self.wrote = datetime.datetime.now()
            time.sleep(1)
    
    
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
        if type(label) == int:
            return self.df.iloc[label]
        elif label in self.df.columns and value != None:
            try:
                return self.df.set_index(label, drop=False).loc[value]
            except KeyError:
                print("[WARN][order_mng.py] : order_manager.get_order()の引数に該当するオーダーは見つかりませんでした", file = sys.stderr)
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
        self.df.loc[ID].loc[key] = value
        self.changed = datetime.datetime.now()
    
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
        ID = str(uuid.uuid4())
        #s = pd.DataFrame([[ID,ORDER_TYPE,STATUS,RECEIPT_TIME,ACCEPTED_TIME,SENDER,RECEIVER,PICKUP_PLACE,PICKUP_TIME,RECEIVE_PLACE,RECEIVE_TIME]], columns = self.df.columns)
        s = pd.DataFrame([[ID,ORDER_TYPE,ITEM_TYPE,ITEM_NAME,STATUS,RECEIPT_TIME,ACCEPTED_TIME,SENDER,RECEIVER,PICKUP_PLACE,PICKUP_TIME,PICKUP_PIN,RECEIVE_PLACE,RECEIVE_TIME,RECEIVE_PIN,NOTE]], columns = self.df.columns)
        self.df = pd.concat([self.df, s], axis = 0, join = "outer")
        self.changed = datetime.datetime.now()
        return ID

    
    def get_next_movement(self, debug_index):
        """
        【未完】
        次の移動時刻と目的地を取得する
        
        戻り値：
            time: 次の出発時刻
            place_idx: 目的地の番号
        """
        
        
        
        return self.TIME[debug_index], debug_index
    
    

if __name__ == '__main__':
    """
    コマンドライン(phpなど)から実行された時の処理
    """
    o = order_manager()
    time.sleep(0.5) # インスタンス化し終わるまで待つ
    
    if(len(sys.argv) > 1):
        # 新しい注文情報を書き込む
        if(sys.argv[1] == 'new_order'):
            # "送る"の場合
            if(sys.argv[2] == 'SEND'):
                o.new_order(
                    ORDER_TYPE = sys.argv[2],
                    ITEM_TYPE = sys.argv[3],
                    ITEM_NAME = sys.argv[4],
                    SENDER = sys.argv[5],
                    RECEIVER = sys.argv[6],
                    PICKUP_PLACE = sys.argv[7],
                    PICKUP_TIME = sys.argv[8],
                    PICKUP_PIN = sys.argv[9],
                    NOTE = sys.argv[10]
                )
                time.sleep(0.5) # 注文情報を書き込み切るまで待つ
            # "送ってもらう"の場合
            if(sys.argv[2] == 'RECEIVE'):
                o.new_order(
                    ORDER_TYPE = sys.argv[2],
                    ITEM_TYPE = sys.argv[3],
                    ITEM_NAME = sys.argv[4],
                    SENDER = sys.argv[5],
                    RECEIVER = sys.argv[6],
                    REVEIVE_PLACE = sys.argv[7],
                    RECEIVE_TIME = sys.argv[8],
                    RECEIVE_PIN = sys.argv[9],
                    NOTE = sys.argv[10]
                )
                time.sleep(0.5) # 注文情報を書き込み切るまで待つ
                
        # 指定のIDのオーダー情報(DataFrame)を入手する
        if(sys.argv[1] == 'get_order'):
            result = o.get_order(
                "ID",
                sys.argv[2]
            )
            time.sleep(0.5) # 注文情報を読み込み切るまで待つ
            print(result)