import pandas as pd
import uuid

def check_order_status():
    """
    注文状況をcsvファイルから取得する関数
    
    戻り値：
        注文状況の連想配列
    """
    # CSVファイルから注文状況を確認
    df = pd.read_csv('order_info.csv')
    return df


def add_new_order(order_type, receipt_time, accepted_time, sender, receiver, pickup_place, pickup_time, receive_place, receive_time):
    id = str(uuid.uuid4())
    
    status = 'NOT_ACCEPTED_YET' # 初期状態：未承認
    df = pd.read_csv('order_info.csv')
    new_order = pd.DataFrame([[order_type, status, receipt_time, accepted_time, sender, receiver, pickup_place, pickup_time, receive_place, receive_time]],
                             columns=['ID', 'ORDER_TYPE', 'STATUS', 'RECEIPT_TIME', 'ACCEPTED_TIME', 'SENDER', 'RECEIVER', 'PICKUP_PLACE', 'PICKUP_TIME', 'RECEIVE_PLACE', 'RECEIVE_TIME'])
    df = df.append(new_order, ignore_index=True)
    df.to_csv('order_info.csv', index=False)
    
    
d = pd.DataFrame()
d.index = ["1", "2", "3"]
d.columns = ["A", "B", "C"]