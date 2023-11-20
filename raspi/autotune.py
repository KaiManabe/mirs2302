import serial_com as ser
import time
import sys
import threading
import run_ctrl as controller
import sock
import config
import tuning
import numpy as np


def loss(data, target_speed):
    """
    損失を求める関数
    
    引数：
        data : 左のデータ、右のデータを含む2次元配列 -> list
        target_speed : 真値 -> int
    
    
    戻り値：
        誤差の積分
    """
    
    