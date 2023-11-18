import socket
import threading
import time
import config
import sys


def receive_buffer(socket_class):
    """ 
    [常時実行]ソケットのバッファを監視して、データが来たらリストに格納する
    
    引数：
        socket_serverクラスオブジェクト もしくは socket_clientクラスオブジェクト
    
    戻り値:
        なし
    """
    while(1):
        received_data = socket_class.client.recv(1024)
        for r in received_data:
            socket_class.buf.append(r)
        
        #バッファの長さが制限を超えたらバッファの頭を削る
        if len(socket_class.buf) > config.MAX_DATA_LENGTH_OF_SOCKET_BUFFER and config.MAX_DATA_LENGTH_OF_SOCKET_BUFFER != -1:
                socket_class.buf = socket_class.buf[(-1 * config.MAX_DATA_LENGTH_OF_SOCKET_BUFFER):]


def server_starter(server_class):
    """ 
    クライアントからの接続を裏で待ってくれる関数
    
    引数：
        sock_serverクラスオブジェクト
    戻り値：
        なし
    """
    
    #クライアントから接続されるまで待機する
    print("[INFO][sock.sock_server] : サーバーへのソケット通信接続を待機しています...")
    server_class.client, server_class.client_addr = server_class.server.accept()
    
    
    #接続を確立したらバッファ変数を宣言して、ソケットのバッファ監視を開始する
    print("[INFO][sock.sock_server] : クライアント" , server_class.client_addr , "から接続を受け付けました")
    server_class.buf = []
    buf_monitor_thread = threading.Thread(target = receive_buffer, args = (server_class,))
    buf_monitor_thread.setDaemon(True)
    buf_monitor_thread.start()
    print("[INFO][sock.sock_server] : バッファの監視を開始しました")
    server_class.server_started = True


class sock_server():
    def __init__(self, address, port):
        """ 
        コンストラクタ

        引数：
            サーバのアドレス -> str
            サーバのポート (49152-65535が好ましい) -> int
            
        戻り値：
            なし
        """
        
        #サーバを立てる
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.server.bind((address, port))
        except:
            print("[ERROR][sock.sock_server] : ポート", port, "にてサーバーをリッスンできませんでした")
            return
        self.server.listen(0)
        print("[INFO][sock.sock_server] : ポート", port, "にてサーバーをリッスンしました")
        
        
        #クライアントからの接続を待機（並列処理なので注意）
        self.server_started = False
        server_starter_thread = threading.Thread(target = server_starter, args = (self,))
        server_starter_thread.setDaemon(True)
        server_starter_thread.start()
        
        
    
    
    def buffer_length(self):
        """ 
        サーバのバッファの長さを返すメソッド

        引数：
            なし
            
        戻り値：
            バッファ(リスト)の長さ -> int
            失敗した場合 -> -1
        """
        #サーバが立っているかチェック
        if not(self.server_started):
            print("[ERROR][sock.sock_server] : サーバがクライアントと接続していない状態でbuffer_length()メソッドが実行されました")
            return -1
        
        return len(self.buf)
    
    
    def read(self):
        """ 
        サーバのバッファを返すと同時に、バッファをクリアするメソッド

        引数：
            なし
            
        戻り値：
            バッファ -> list
            失敗した場合 -> -1
        """
        #サーバが立っているかチェック
        if not(self.server_started):
            print("[ERROR][sock.sock_server] : サーバがクライアントと接続していない状態でread()メソッドが実行されました")
            return -1
        
        
        buffer = self.buf.copy()
        self.buf = []
        return buffer
    
    
    def send(self, send_data):
        """ 
        クライアントにデータを送信するメソッド

        引数：
            送信するデータ -> list(int)
            
        戻り値：
            送信データの長さ
            失敗した場合 -> -1
        """
        #サーバが立っているかチェック
        if not(self.server_started):
            print("[ERROR][sock.sock_server] : サーバがクライアントと接続していない状態でsend()メソッドが実行されました")
            return -1
        
        
        """データのチェック"""
        if type(send_data) != list:
            print("[ERROR][sock.sock_server] : send()メソッドの引数はint型のリストである必要があります")
            return -1
        
        for d in send_data:
            if type(d) != int or d > 255:
                print("[ERROR][sock.sock_server] : send()メソッドの引数に256以上の値が存在します")
                return -1
        
        return self.client.sendall(bytes(send_data))


class sock_client():
    def __init__(self, address, port):
        """ 
        コンストラクタ

        引数：
            接続したいサーバのアドレス -> str
            接続したいサーバのポート (49152-65535が好ましい) -> int
            
        戻り値：
            なし
        """
        #サーバに接続する
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("[INFO][sock.sock_client] : ポート", port, "に接続を試みています...")
        try:
            self.client.connect((address, port))
        except:
            print("[ERROR][sock.sock_client] : ポート", port, "に接続できませんでした")
            return
        
        print("[INFO][sock.sock_client] : ポート", port, "のサーバとの接続を確立しました")
        
        
        #バッファ変数を宣言して、バッファの監視を開始
        self.buf = []
        buf_monitor_thread = threading.Thread(target = receive_buffer, args = (self,))
        buf_monitor_thread.setDaemon(True)
        buf_monitor_thread.start()
        print("[INFO][sock.sock_client] : バッファの監視を開始しました")
    
    
    def buffer_length(self):
        """ 
        クライアントのバッファの長さを返すメソッド

        引数：
            なし
            
        戻り値：
            バッファ(リスト)の長さ -> int
        """
        return len(self.buf)
    
    
    def read(self):
        """ 
        クライアントのバッファを返すと同時に、バッファをクリアするメソッド

        引数：
            なし
            
        戻り値：
            バッファ -> list
        """
        buffer = self.buf.copy()
        self.buf = []
        return buffer
    
    
    def send(self, send_data):
        
        """ 
        サーバにデータを送信するメソッド

        引数：
            送信するデータ -> list(int)
            
        戻り値：
            送信データの長さ・失敗したら-1
        """
        
        
        """データのチェック"""
        if type(send_data) != list:
            print("[ERROR][sock.sock_client] : send()メソッドの引数はint型のリストである必要があります")
            return -1
        
        for d in send_data:
            if type(d) != int or d > 255:
                print("[ERROR][sock.sock_client] : send()メソッドの引数に256以上の値が存在します")
                return -1
        
        return self.client.sendall(bytes(send_data))
