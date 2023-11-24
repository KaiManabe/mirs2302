import sys

# 入力データを取得
input_data = sys.argv[1]

# ここで必要な判定の処理を行う（サンプルとして文字列の長さを判定しています）
if len(input_data) > 5:
    result = "入力データは5文字より長いです。"
else:
    result = "入力データは5文字以下です。"

# 判定結果を表示
print(result)
