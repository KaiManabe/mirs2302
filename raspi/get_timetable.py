import order_mng as om
import pandas as pd
import datetime
import select_option_api



def datetime_to_time(dt:datetime.datetime):
    h = dt.hour
    m = round(dt.minute / 10) * 10
    if m >= 60:
        m = 0
        h += 1
    if h >= 24:
        h = 23
        m = 59
    
    return datetime.time(h,m)
    
    
    
timetable = {}


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

o = om.order_manager()

for status in STATUS_LABELS:
    if status in ["DENIED", "ACCEPT_DENIED", "PICKUP_TIMEOUT", "RECEIVED"]:
        continue
    df = o.get_order("STATUS", status)
    if type(df) == int and df == -1:
        continue
    
    for i in range(len(df)):
        key = df["ITEM_TYPE"][i]
        for ii in range(100):
            if not(key + str(ii) in timetable.keys()):
                if ii != 0:
                    key += "_" + str(ii)
                break
        
        if pd.isna(df["PICKUP_TIME"][i]):
            begin = datetime.datetime.now()
        else:
            begin = df["PICKUP_TIME"][i]
        
        if pd.isna(df["RECEIVE_TIME"][i]):
            end = datetime.datetime(year = 2050, month = 12, day = 31, hour = 23, minute = 59)
        else:
            end = df["RECEIVE_TIME"][i]
        
        
        timetable[key] = {"begin" : begin, "end" : end}

output = ""


output += r"<tr>"
output += r"<th width = 100px>収納部</th>"
for h in range(8,17):
    output += rf"<th width = '50px' colspan = '6'>{h:02}時</th>"
output += r"</tr>"


for box in timetable.keys():
    output += r"<tr>"
    output += rf"<th width = 100px>{box}</th>"
    for h in range(8,17):
        for m in range(0,60,10):
            element_class = ""
            if datetime.time(h,m) >= datetime_to_time(timetable[box]["begin"]) and datetime.time(h,m) <= datetime_to_time(timetable[box]["end"]):
                element_class = r"class = 'using'"
            
            if datetime_to_time(timetable[box]["begin"] - datetime.timedelta(minutes = om.TIME_MARGIN))\
            <=\
            datetime.time(h,m)\
            <=\
            datetime_to_time(timetable[box]["begin"] + datetime.timedelta(minutes = select_option_api.TIME_MARGIN))\
            or\
            datetime_to_time(timetable[box]["end"] - datetime.timedelta(minutes = select_option_api.TIME_MARGIN))\
            <=\
            datetime.time(h,m)\
            <=\
            datetime_to_time(timetable[box]["end"] + datetime.timedelta(minutes = select_option_api.TIME_MARGIN))\
            :
                element_class = r"class = 'moving'"
            
            
            if datetime.time(h,m) == datetime_to_time(timetable[box]["begin"]) or datetime.time(h,m) == datetime_to_time(timetable[box]["end"]):
                element_class = r"class = 'reserved'"
            
                
            output += rf"<th {element_class} ></th>"
output += r"</tr>"

print(output, end = "")





        
    