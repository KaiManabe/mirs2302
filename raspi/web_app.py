""" 
メールを送信するクラスが定義されている

このファイルをインポートして、mails()というオブジェクトを定義してつかう
"""

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import order_mng as om

# メール送信用関数
def send_email(sender_name, sender_email, app_password, receiver_email, subject, body):
    # メールの設定
    message = MIMEMultipart()
    message["From"] = sender_name
    message["To"] = receiver_email
    message["Subject"] = subject

    # メール本文の追加
    message.attach(MIMEText(body, "plain"))

    # GmailのSMTPサーバーとポート
    smtp_server = "smtp.gmail.com"
    port = 587

    # SMTPサーバーへの接続
    with smtplib.SMTP(smtp_server, port) as server:
        # 暗号化の開始
        server.starttls()

        # ログイン
        server.login(sender_email, app_password)

        # メールの送信
        server.sendmail(sender_email, receiver_email, message.as_string())
        
        
class mails():
    """
    メール送信用クラス
    """
    def __init__(self, order_manager: om.order_manager):
        self.sender_name = "学内配達ロボット TENQ" # 送信元の名前
        self.sender_email = "mirs2302tenq@gmail.com"  # 送信元のメールアドレス
        self.app_password = "lvst oefb zsfw kmmk"  # 送信元のアプリパスワード
        
        # 管理者のメールアドレスリスト
        self.manager_list = [
            "d20139@numazu.kosen-ac.jp"
            ]
        # "d20102@numazu.kosen-ac.jp",
        # "d20136@numazu.kosen-ac.jp",
        
        self.order_manager = order_manager
    
    def request(self, order_id):
        """
        取引依頼メールを送信する
        
        引数：
            オーダーID
        """
        # 送信先のメールアドレス（オーダーIDからオーダータイプ、送信先を引っ張ってくる）
        order_info = self.order_manager.get_order("ID", order_id)
        order_type = order_info["ORDER_TYPE"][0]
        if order_type == "SEND":
            receiver_list = [order_info["RECEIVER"][0]]
        elif order_type == "RECEIVE":
            receiver_list = [order_info["SENDER"][0]]
        else:
            receiver_list = self.manager_list # order/accept/も必要じゃねこれ
            
        subject = "依頼が来ています"
        transactions_link = f"http://172.25.60.44/{order_type.lower()}/accept/index.html?id={order_id}"
        usage_rules_link = "http://172.25.60.44/main/"
        
        body = f"""  D科4年のプロジェクト,「学内配達ロボットTENQ」です。\n\n  取引の依頼が来ています。以下のリンクから取引の承認・拒否を行なってください。\n  {transactions_link}\n\n  また、TENQの概要・使い方については以下のTENQホームページをご覧ください。\n  {usage_rules_link}\n\n※このメールは自動で送信されています。"""
        
        for receiver_email in receiver_list:
            # print(f"メールを送信します\n\n 送信元 : {self.sender_email}\n 送信先 : {receiver_email}\n 件名 : {subject}\n 内容 : {body}") # デバッグ用
            send_email(self.sender_name, self.sender_email, self.app_password, receiver_email, subject, body)

    def denied(self, order_id):
        """
        依頼が拒否されたというメールを送信する
        
        引数：
            オーダーID
        """
        # 送信先のメールアドレス（オーダーIDからオーダータイプ、送信先を引っ張ってくる）
        order_info = self.order_manager.get_order("ID", order_id)
        order_type = order_info["ORDER_TYPE"][0]
        if order_type == "SEND":
            receiver_list = [order_info["SENDER"][0]]
            opponent_email = order_info["RECEIVER"][0]
        elif order_type == "RECEIVE":
            receiver_list = [order_info["RECEIVER"][0]]
            opponent_email = order_info["SENDER"][0]
        else:
            receiver_list = [order_info["RECEIVER"][0]] # order/accept/も必要じゃねこれ
            opponent_email = "TENQ管理者"
            
        item_type = order_info["ITEM_TYPE"][0]
        
        subject = "依頼が拒否されました"

        body = f"""  D科4年のプロジェクト,「学内配達ロボットTENQ」です。\n\n  お相手の承認をとることができなかったため、以下の依頼はキャンセルされます。\n  再度依頼をする際は、お相手のメールアドレスに間違いが無いことをご確認ください。\n\n< 依頼内容 >\n   依頼の種類 : {order_type}\n   相手 : {opponent_email}\n   商品 : {item_type}\n\n※このメールは自動で送信されています。"""

        for receiver_email in receiver_list:
            # print(f"メールを送信します\n\n 送信元 : {self.sender_email}\n 送信先 : {receiver_email}\n 件名 : {subject}\n 内容 : {body}") # デバッグ用
            send_email(self.sender_name, self.sender_email, self.app_password, receiver_email, subject, body)

    def warning(self, warn_type: str, module: str = None, door: str = None):
        """
        異常検知メールを送信する

        引数：
            異常の種類、モジュールの名前、扉の名前
            異常の種類 -> "module":モジュール持ち去り "door":扉こじ開け "airframe":機体持ち去り
        """
        subject = "TENQの異常を検知しました"
        
        if warn_type == "module":
            body = f"""  TENQの異常を検知しました。\n  "{module}"モジュールが持ち去られた可能性があります。\n  直ちに確認作業を行ってください。\n\n※このメールは自動で送信されています。"""
            
        elif warn_type == "door":
            body = f"""  TENQの異常を検知しました。\n  "{module}"モジュールの扉"{door}"がこじ開けられた可能性があります。\n  直ちに確認作業を行ってください。\n\n※このメールは自動で送信されています。"""
            
        elif warn_type == "airframe":
            body = """  TENQの異常を検知しました。\n  機体が持ち去られた可能性があります。\n  直ちに確認作業を行ってください。\n\n※このメールは自動で送信されています。"""
            
        # 各管理者に送信
        for receiver_email in self.manager_list:
            send_email(self.sender_name, self.sender_email, self.app_password, receiver_email, subject, body)