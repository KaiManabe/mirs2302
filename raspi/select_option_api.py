import order_mng
import datetime
import pandas as pd
from math import ceil
import sys

#稼働可能な時間帯
open_hour = [{"begin" : datetime.datetime.combine(datetime.datetime.today(), datetime.time(10, 0)),
              "end" : datetime.datetime.combine(datetime.datetime.today(), datetime.time(12, 0))},
             {"begin" : datetime.datetime.combine(datetime.datetime.today(), datetime.time(14, 0)),
              "end" : datetime.datetime.combine(datetime.datetime.today(), datetime.time(23, 0))}
            ]


#移動時間の見積もり
TIME_MARGIN = 20

ITEM_TYPES = ["小物",
              "書類",
              "食品（保冷）",
              "食品（保温）"
              ]

o = order_mng.order_manager()


def get_movements():
    """
    いま予約されている「到着時間」とその「目的地」をとる関数
    
    この関数の結果をつかって「予約を入れてもいい移動時間」をみつける
    
    引数：
        なし
        
    戻り値：
        n行2列 リスト
        
        1列目 到着時間
        
        2列目 目的地
    """
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
    arrive_times = []
    goals = []
    o.reflesh()
    for status in STATUS_LABELS:
        if status in ["DENIED", "ACCEPT_DENIED", "PICKUP_TIMEOUT", "RECEIVE_TIMEOUT", "RECEIVED"]:
            continue
        
        df = o.get_order("STATUS", status)
        if type(df) == int and df == -1:
            continue
        
        for i in range(len(df)):
            if not(pd.isna(df["RECEIVE_TIME"][i]) or pd.isna(df["RECEIVE_PLACE"][i])):
                arrive_times.append(df["RECEIVE_TIME"][i])
                goals.append(df["RECEIVE_PLACE"][i])
            
            if not(pd.isna(df["PICKUP_TIME"][i]) or pd.isna(df["PICKUP_PLACE"][i])):
                arrive_times.append(df["PICKUP_TIME"][i])
                goals.append(df["PICKUP_PLACE"][i])
        
    return list(zip(arrive_times, goals))


def get_items():
    """
    いま予約されている「ITEMTYPE」とその「占有時間」をとる関数
    
    この関数の結果をつかって「予約を入れてもいい時間」をみつける
    
    引数：
        なし
        
    戻り値：
        n行2列 リスト
        
        1列目 ITEMTYPE
        
        2列目 PICKUP_TIME
        
        3列目 RECEIVE_TIME
    """
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
    item_types = []
    pickup = []
    receive = []
    o.reflesh()
    for status in STATUS_LABELS:
        if status in ["DENIED", "ACCEPT_DENIED", "PICKUP_TIMEOUT", "RECEIVED"]:
            continue
        
        df = o.get_order("STATUS", status)
        if type(df) == int and df == -1:
            continue
        
        for i in range(len(df)):
            item_types.append(df["ITEM_TYPE"][i])
            if pd.isna(df["PICKUP_TIME"][i]):
                pickup.append(datetime.datetime.now())
            else:
                pickup.append(df["PICKUP_TIME"][i])
                
            if pd.isna(df["RECEIVE_TIME"][i]):
                receive.append(datetime.datetime(year = 2050, month = 12, day = 31))
            else:
                receive.append(df["RECEIVE_TIME"][i])
    
    return list(zip(item_types, pickup, receive))


def get_available_time(ITEM_TYPE:str = None):
    """
    選択可能な時間を返す関数
    
    引数：
        ITEM_TYPE:str -> 【任意】：選択されたITEMTYPEを渡す　何も入っていなければ初期化動作として扱う
        
    戻り値：
        選択可能な時間をdatetime型で返す
        これをdatetime_to_option()で変換する必要アリ
    """
    movements = get_movements()
    items = get_items()
    available = []
    
    #今からTIME_MARGIN分後の時間を、10分単位で切り上げる
    ctime = datetime.datetime.now() + datetime.timedelta(minutes = TIME_MARGIN)
    margin = (ceil(ctime.minute / 10) * 10) - ctime.minute
    ctime += datetime.timedelta(minutes = margin)
    
    latest = datetime.datetime.now()
    for t in open_hour:
        if t["end"] > latest:
            latest = t["end"]
    
    
    ctime -= datetime.timedelta(minutes = 10)
    while(1):
        kaburi = False
        ctime += datetime.timedelta(minutes = 10)
        
        #すべての時間について検証を終えたらbreak
        if ctime > latest:
            break
                
        #ctimeが営業時間内か検証する
        isopen = False
        for oh in open_hour:
            if oh["begin"] <= ctime or oh["end"] >= ctime:
                isopen = True
        
        #ITEMTYPE引数があるならば
        if ITEM_TYPE != None:
            KOMONOcount = 0
            SHORUIcount = 0
            for item in items:
                if ITEM_TYPE == item[0]:
                    #時間が被っているならば
                    if item[1] <= ctime and item[2] >= ctime:
                        
                        #小物と書類は被りを1回まで許容する
                        if ITEM_TYPE == "小物":
                            KOMONOcount += 1
                        elif ITEM_TYPE == "書類":
                            SHORUIcount += 1
                        
                        #被りがあるのでアウト
                        if KOMONOcount == 2 or SHORUIcount == 2 or "食品" in ITEM_TYPE:
                            kaburi = True
        
        
        
        #ctimeが他の予約とぶつからないか検証する    
        for mt in movements:
            
            #予約mtがctimeの前後TIME_MARGIN分以内に存在するならば
            if ctime >= (mt[0] - datetime.timedelta(minutes = TIME_MARGIN)) and ctime <= (mt[0] + datetime.timedelta(minutes = TIME_MARGIN)):
                #print("yoyaku")
                #print(ctime)
                kaburi = True
            
        if not(kaburi) and isopen:
            available.append(ctime)
            #print("append" , ctime)
    
    return available



def get_available_itemtype(TIME_BEGIN:datetime.datetime = None, TIME_END:datetime.datetime = None):
    """
    選択可能な時間を返す関数
    
    引数：
        TIME_BEGIN:datetime.datetime -> 【任意】：選択されたPICKUP時間を渡す
        TIME_END:datetime.datetime -> 【任意】：選択されたRECEIVE時間を渡す
        
    TIME_BEGIN, TIME_ENDどちらもNoneの場合には初期化動作として扱う
    どちらか一方のみがNoneの場合には[発送 | 受け取り時間]未定として扱う
       
    
    戻り値：
        選択可能なITEMTYPEを返す
    """
    if TIME_BEGIN == None and TIME_END == None:
        return ITEM_TYPES

    else:
        if TIME_BEGIN == None:
            time_begin = datetime.datetime.now()
        else:
            time_begin = TIME_BEGIN
            
        
        if TIME_END == None:
            #time_end = datetime.datetime(year = 2050, month = 12, day = 31)
            time_end = TIME_BEGIN
        else:
            time_end = TIME_END
    
    item_list = get_items()
    available = []
    
    for item in ITEM_TYPES:
        kaburi = 0
        for exist in item_list:
            #既存
            if exist[0] == item:
                #print(exist, time_begin, time_end)
                if not(time_begin <= time_end < exist[1] or exist[2] < time_begin <= time_end):
                    kaburi += 1
        
        
        #被りがあるのでアウト
        if not("食品" in item) and kaburi > 1:
            continue
        elif ("食品" in item) and kaburi > 0:
            continue
        
        available.append(item)
    
    return available
    
    
    

def option_to_datetime(opt):
    strdt = opt.split("-")[0]
    hour = int(strdt.split(":")[0])
    minute = int(strdt.split(":")[-1])
    return datetime.datetime.combine(datetime.datetime.today(), datetime.time(hour, minute))


def datetime_to_option(dt:datetime.datetime):
    out = dt.strftime("%H:%M")
    out += "-"
    out += (dt + datetime.timedelta(minutes = 10)).strftime("%H:%M")
    return out


def printer(args):
    #与えられた引数の種類
    #typeが与えられたので今から返すのは時間
    if args[1] == "type":
        
        #初期化処理ならば
        if len(args) == 2:
            times = get_available_time()
        else:
            times = get_available_time(args[2])
        
        for t in times:
            print(datetime_to_option(t), end = ",")
        return
    
    #timeが与えられたので今から返すのはtype
    elif args[1] == "time":
        #初期化処理ならば
        if len(args) == 2:
            itemtypes = get_available_itemtype()
        else:
            itemtypes = get_available_itemtype(TIME_BEGIN = option_to_datetime(args[2]))
        
        for t in itemtypes:
            print(t, end = ",")
        return
        
        
    


            
if __name__ == "__main__":
    
    #コマンドライン引数があればそれに応じた処理をさせる
    if len(sys.argv) > 1:
        printer(sys.argv)