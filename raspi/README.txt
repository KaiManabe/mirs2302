#####　ここに書いてあるもの以外は使わないこと #####

------------ サーボテスト方法 ------------

1. コマンドラインで python3 -i test_servo.py を実行
2. servo() を実行(引数には動かしたいリレーのPIN番号: int型)
3. 終了する際は exit() を実行

r() でラズパイからPWM信号を出せる(引数にはPIN番号: int型)

------------ モジュールテスト方法(全体) ------------

1. コマンドラインで python3 -i module_mng.py を実行
・モジュール抵抗値を見る -> m.resistance_list
・搭載モジュール情報を見る -> m.onb_module_info
・扉を解錠 -> m.door_open() | 第一引数: 解錠したいモジュール名("accessories" / "insulation" / "document") 第二引数: 解錠したい扉名("right" / "left" | "under" / "upper")
・機体の高さを算出 -> m.height_calculate()
・模型用バッテリーの電圧を見る -> a.battery_surv()
2. 終了する際は exit() を実行
