import order_mng
import datetime
import pandas as pd
from math import ceil
import sys

#稼働可能な時間帯
open_hour = [{"begin" : datetime.datetime.combine(datetime.datetime.today(), datetime.time(8, 0)),
              "end" : datetime.datetime.combine(datetime.datetime.today(), datetime.time(23, 59))}
            ]


#移動時間の見積もり 10分単位
TIME_MARGIN = order_mng.TIME_MARGIN

o = order_mng.order_manager()

   
    

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



def simulator1():
    """
    【送る】の手配ページで【何も選択されていない】場合に選択肢をシミュレートする
    """
    available = []
    BOX_TYPES = ["小物", "書類", "食品（保冷）", "食品（保温）"]
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
            if oh["begin"] <= ctime <= oh["end"]:
                isopen = True
        
        isok = False
        for box in BOX_TYPES:
            if o.box_decider(box_type = box, PICKUP_TIME = ctime) != -1:
                isok = True
                break
        
        
            
        if isok and isopen:
            ctime2 = ctime
            while(1):
                ctime2 += datetime.timedelta(minutes = 10)
                if ctime2 > latest:
                    break
                
                #ctime2が営業時間内か検証する
                isopen2 = False
                for oh in open_hour:
                    if oh["begin"] <= ctime2 <= oh["end"]:
                        isopen2 = True
                
                if not(isopen2):
                    continue

                
                if o.box_decider(box_type = box, PICKUP_TIME = ctime, RECEIVE_TIME = ctime2) != -1:
                    available.append(ctime)
                    break
                
    
    return available


def simulator2(box_type):
    """
    【送る】の手配ページで【箱のみ選択されている】場合に選択肢をシミュレートする
    """
    available = []
    BOX_TYPES = ["小物", "書類", "食品（保冷）", "食品（保温）"]
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
            if oh["begin"] <= ctime <= oh["end"]:
                isopen = True
        
        isok = False
        if o.box_decider(box_type = box_type, PICKUP_TIME = ctime) != -1:
            isok = True
        
        
        if isok and isopen:
            ctime2 = ctime
            while(1):
                ctime2 += datetime.timedelta(minutes = 10)
                if ctime2 > latest:
                    break
                
                #ctime2が営業時間内か検証する
                isopen2 = False
                for oh in open_hour:
                    if oh["begin"] <= ctime2 <= oh["end"]:
                        isopen2 = True
                
                if not(isopen2):
                    continue

                
                if o.box_decider(box_type = box_type, PICKUP_TIME = ctime, RECEIVE_TIME = ctime2) != -1:
                    available.append(ctime)
                    break
    
    return available


def simulator3(PICKUP_TIME):
    """
    【送る】の手配ページで【集荷時間のみ選択されている】場合に選択肢をシミュレートする
    """
    available = []
    BOX_TYPES = ["小物", "書類", "食品（保冷）", "食品（保温）"]
       
    latest = datetime.datetime.now()
    for t in open_hour:
        if t["end"] > latest:
            latest = t["end"]
    
    for box in BOX_TYPES:
        if o.box_decider(box_type = box, PICKUP_TIME = option_to_datetime(PICKUP_TIME)) != -1:
            ctime = option_to_datetime(PICKUP_TIME)
            ctime -= datetime.timedelta(minutes = 10)
            while(1):
                ctime += datetime.timedelta(minutes = 10)
                if ctime > latest:
                    break
                
                #ctimeが営業時間内か検証する
                isopen2 = False
                for oh in open_hour:
                    if oh["begin"] <= ctime <= oh["end"]:
                        isopen2 = True
                
                if not(isopen2):
                    continue

                if o.box_decider(box_type = box, PICKUP_TIME = option_to_datetime(PICKUP_TIME), RECEIVE_TIME = ctime) != -1:
                    available.append(box)
                    break
    
    return available


def simulator4(ID):
    """
    【送る】の【承認ページ】で選択肢をシミュレートする
    """
    available = []
    order = o.get_order("ID", ID)
    
    
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
            if oh["begin"] <= ctime <= oh["end"]:
                isopen = True
        
        isok = True
        movements = o.get_moving_schedule()
        for m in movements:
            if m["begin"] <= ctime <= m["end"]:
                isok = False
                break
        
        if order["PICKUP_TIME"].iloc[0] + datetime.timedelta(minutes = order_mng.TIME_MARGIN)\
        >=\
        ctime:
            isok = False
                
        if isok and isopen:
            available.append(ctime)
    
    return available



def simulator5():
    """
    【受け取る】の手配ページで【何も選択されていない】場合に選択肢をシミュレートする
    """
    available = []
    BOX_TYPES = ["小物", "書類", "食品（保冷）", "食品（保温）"]
    #今からTIME_MARGIN分後の時間を、10分単位で切り上げる
    ctime = datetime.datetime.now() + datetime.timedelta(minutes = TIME_MARGIN)
    margin = (ceil(ctime.minute / 10) * 10) - ctime.minute
    ctime += datetime.timedelta(minutes = margin)
    
    latest = datetime.datetime.now()
    for t in open_hour:
        if t["end"] > latest:
            latest = t["end"]
    
    
    earliest = datetime.datetime.combine(datetime.datetime.today(), datetime.time(23, 50))
    for t in open_hour:
        if t["begin"] < earliest:
            earliest = t["begin"]
    
    
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
            if oh["begin"] <= ctime <= oh["end"]:
                isopen = True
        
        isok = False
        for box in BOX_TYPES:
            if o.box_decider(box_type = box, RECEIVE_TIME = ctime) != -1:
                isok = True
                break
        
        
            
        if isok and isopen:
            ctime2 = ctime
            while(1):
                ctime2 -= datetime.timedelta(minutes = 10)
                if ctime2 < earliest:
                    break
                
                #ctime2が営業時間内か検証する
                isopen2 = False
                for oh in open_hour:
                    if oh["begin"] <= ctime <= oh["end"]:
                        isopen2 = True
                
                if not(isopen2):
                    continue
                
                if o.box_decider(box_type = box, PICKUP_TIME = ctime2, RECEIVE_TIME = ctime) != -1:
                    available.append(ctime)
                    break
    
    return available



def simulator6(box_type):
    """
    【受け取る】の手配ページで【箱のみ選択されている】場合に選択肢をシミュレートする
    """
    available = []
    BOX_TYPES = ["小物", "書類", "食品（保冷）", "食品（保温）"]
    #今からTIME_MARGIN分後の時間を、10分単位で切り上げる
    ctime = datetime.datetime.now() + datetime.timedelta(minutes = TIME_MARGIN)
    margin = (ceil(ctime.minute / 10) * 10) - ctime.minute
    ctime += datetime.timedelta(minutes = margin)
    
    latest = datetime.datetime.now()
    for t in open_hour:
        if t["end"] > latest:
            latest = t["end"]
    
    
    earliest = datetime.datetime.combine(datetime.datetime.today(), datetime.time(23, 50))
    for t in open_hour:
        if t["begin"] < earliest:
            earliest = t["begin"]
    
    
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
            if oh["begin"] <= ctime <= oh["end"]:
                isopen = True
        
        isok = False
        
        if o.box_decider(box_type = box_type, RECEIVE_TIME = ctime) != -1:
            isok = True
        
        
            
        if isok and isopen:
            ctime2 = ctime
            while(1):
                ctime2 -= datetime.timedelta(minutes = 10)
                if ctime2 < earliest:
                    break
                
                #ctime2が営業時間内か検証する
                isopen2 = False
                for oh in open_hour:
                    if oh["begin"] <= ctime <= oh["end"]:
                        isopen2 = True
                
                if not(isopen2):
                    continue
                
                if o.box_decider(box_type = box_type, PICKUP_TIME = ctime2, RECEIVE_TIME = ctime) != -1:
                    available.append(ctime)
                    break
    
    return available


def simulator7(RECEIVE_TIME):
    """
    【受け取る】の手配ページで【集荷時間のみ選択されている】場合に選択肢をシミュレートする
    """
    available = []
    BOX_TYPES = ["小物", "書類", "食品（保冷）", "食品（保温）"]
    
    
    earliest = datetime.datetime.combine(datetime.datetime.today(), datetime.time(23, 50))
    for t in open_hour:
        if t["begin"] < earliest:
            earliest = t["begin"]
    
    
    for box in BOX_TYPES:
        if o.box_decider(box_type = box, RECEIVE_TIME = option_to_datetime(RECEIVE_TIME)) != -1:
            ctime = option_to_datetime(RECEIVE_TIME)
            while(1):
                ctime -= datetime.timedelta(minutes = 10)
                if ctime < earliest:
                    break
                
                #ctimeが営業時間内か検証する
                isopen = False
                for oh in open_hour:
                    if oh["begin"] <= ctime <= oh["end"]:
                        isopen = True
                
                if not(isopen):
                    continue
            
                if o.box_decider(box_type = box, RECEIVE_TIME = option_to_datetime(RECEIVE_TIME), PICKUP_TIME = ctime) != -1:
                    available.append(box)
                    break
    
    return available


def simulator8(ID):
    """
    【受け取る】の【承認ページ】で選択肢をシミュレートする
    """
    available = []
    order = o.get_order("ID", ID)
    
    
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
            if oh["begin"] <= ctime <= oh["end"]:
                isopen = True
        
        isok = True
        movements = o.get_moving_schedule()
        for m in movements:
            if m["begin"] <= ctime <= m["end"]:
                isok = False
                break
        
        if ctime + datetime.timedelta(minutes = order_mng.TIME_MARGIN)\
        >=\
        order["RECEIVE_TIME"].iloc[0]:
            isok = False
                
        if isok and isopen:
            available.append(ctime)
    
    return available


def arg_to_dict(args):
    out = {"mode" : None,
            "id" : None,
            "selected_box" : None,
            "selected_receive_time" : None,
            "selected_pickup_time" : None}
    
    for arg in args[1:]:
        if arg.split(r"=")[0] in out.keys():
            if arg.split(r"=")[-1] != "":
                out[arg.split(r"=")[0]] = arg.split(r"=")[-1]
            else:
                out[arg.split(r"=")[0]] = None
                
        else:
            print(f"[ERR][select_option_api] : 不正な引数 -> {arg}", file = sys.stderr)
    
    return out
    

def time_printer(array):
    for dt in array:
        print(datetime_to_option(dt), end = ",")

def box_printer(array):
    for box_type in array:
        print(box_type, end = ",")


def api(args):
    option = arg_to_dict(args)
    
    if option["mode"] == "SEND":
        if option["id"] == None:
            #送る　の依頼
            if option["selected_pickup_time"] == None and option["selected_box"] == None:
                #何も選択されていない場合
                time_printer(simulator1())

            elif option["selected_pickup_time"] == None:
                #箱のみ選択した場合
                time_printer(simulator2(option["selected_box"]))
                
            elif option["selected_box"] == None:
                #時間のみ選択した場合
                box_printer(simulator3(option["selected_pickup_time"]))
                
        else:
            #送る　の承認
            time_printer(simulator4(option["id"]))
    
    elif option["mode"] == "RECEIVE":
        if option["id"] == None:
            #受け取る　の依頼
            if option["selected_receive_time"] == None and option["selected_box"] == None:
                #何も選択されていない場合
                time_printer(simulator5())

            elif option["selected_receive_time"] == None:
                #箱のみ選択した場合
                time_printer(simulator6(option["selected_box"]))
                
            elif option["selected_box"] == None:
                #時間のみ選択した場合
                box_printer(simulator7(option["selected_receive_time"]))
                
        else:
            #受け取る　の承認
            time_printer(simulator8(option["id"]))
    
    elif option["mode"] == "DELIVERY":
        time_printer(simulator6(option["selected_box"]))
    
            
    



            
if __name__ == "__main__":
    o.reflesh()
    
    #コマンドライン引数があればそれに応じた処理をさせる
    if len(sys.argv) > 1:
        api(sys.argv)