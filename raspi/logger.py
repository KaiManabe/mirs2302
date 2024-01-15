import sys
import time

class timestamp_logger:
    def __init__(self, original_stream):
        """
        sys.stdout = timestamp_logger(sys.stdout)
        とすると、それ以降標準出力の頭にタイムスタンプがつく
        1文字だと出力が捨てられるので注意
        """
        self.original_stream = original_stream
        self.enabled = True

    def write(self, message):
        if len(message) < 2:
            return
        if self.enabled:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.original_stream.write(f"{timestamp} : {message}\n")
        else:
            self.original_stream.write(f"{message}\n")
    
    def enable(self):
        self.enabled = True
        
    def disable(self):
        self.enabled = False
        

"""
import sys
import logger

tslogger = logger.timestamp_logger(sys.stdout)
sys.stdout = tslogger


"""