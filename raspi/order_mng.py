import pandas as pd
import uuid
import time
import threading
import datetime

class order_manager():
    def __init__(self, path = "/home/pi/git/mirs2302/raspi/order_info.csv"):
        self.file_path = path
        self.df = pd.read_csv(path)
        t = threading.Thread(target = self.reflesh)
        t.setDaemon(True)
        t.start()
    
    def reflesh(self):
        while(1):
            self.df = pd.read_csv(self.file_path)
            time.sleep(1)
    
    def get_order_from_id(self, id):
        return self.df.set_index("ID", drop=False).loc[id]
    
    def get_order_from_index(self, idx):
        return self.df.iloc[idx]
    
    def modify_order(self, ID, key, value):
        self.df.loc[ID].loc[key] = value
    
    def new_order(self,
                  ORDER_TYPE= None,
                  STATUS = "NOT_ACCEPTED_YET",
                  RECEIPT_TIME = datetime.datetime.now(),
                  ACCEPTED_TIME = None,
                  SENDER = None,
                  RECEIVER = None,
                  PICKUP_PLACE = None,
                  PICKUP_TIME = None,
                  RECEIVE_PLACE = None,
                  RECEIVE_TIME = None,
                  ):
        
        ID = str(uuid.uuid4())
        s = pd.DataFrame([[ID,ORDER_TYPE,STATUS,RECEIPT_TIME,ACCEPTED_TIME,SENDER,RECEIVER,PICKUP_PLACE,PICKUP_TIME,RECEIVE_PLACE,RECEIVE_TIME]], columns = self.df.columns)
        self.df = pd.concat([self.df, s], axis = 0, join = "outer")
        return ID
    