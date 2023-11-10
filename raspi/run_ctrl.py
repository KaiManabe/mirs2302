import csv
import time
        
def read_sequence():
    with open("./sequence.csv", "r", encoding="utf-8") as csv_file:
        #リスト形式
        # ２次元配列にする ※数値も文字列として代入されるので注意が必要
        ls_2 = list(csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True))
        
    for read in ls_2:
        if read[0] == 'straight': # 直進のモード
            mode = 1
        elif read[0] == 'rotate': # 回転のモード
            mode = 2
        dist = int(read[1])
        spd = int(read[2])
        
        # デバッグ用
        print('mode:'+str(mode))
        print('dist:'+str(dist))
        print('spd:'+str(spd))
        
        print()
        
        time.sleep(1) # 1秒ごとにcsvファイルを読み込み更新
    
    print(ls_2[1][0])

if __name__ == "__main__":
    read_sequence()