# TENQ projectについて
- 沼津工業高等専門学校 電子制御工学科4年次における科目 MIRS(通称)において製作した。
- 差動二輪駆動ロボットが学内を自律走行し、学生や教職員間で荷物を配達するプロジェクトである。
- 詳細は[MIRSデータベース](https://www2.denshi.numazu-ct.ac.jp/mirsdoc2/mirs2302/)を参照。
- [走行中の動画](https://www.youtube.com/watch?v=8aU9zPizS0Q)

<br>
<br>

# リポジトリの内容
- 機体にはRaspberry pi 4, arduino uno, jetson nanoおよびRPLiDAR S2Mを搭載した。  
- 各マイコンにて使用したソースコードを各ディレクトリにまとめて配置した。  
- 詳細は[ソフトウェア詳細設計書](https://www2.denshi.numazu-ct.ac.jp/mirsdoc2/mirs2302/soft/num0001b/)を参照。
## arduino
- モータの制御を行う
- 機体に搭載した各センサの監視を行う
- raspberry piとシリアル通信を行う

  
## raspberry pi
- webサーバを稼働し、ユーザからの手配を受け付ける
- メインプログラムを実行しながら、arduino / jetson と通信する

  
## jetson nano
- ROSによる自律走行を行う
- raspberry piとソケット通信を行う
