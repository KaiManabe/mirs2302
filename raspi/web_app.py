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
    
    def request(self, order_id: str):
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
        elif order_type == "ORDER":
            # 依頼メールを各管理者に送信し、承認した人をオーダーリストに書き込むようにしたい
            receiver_list = self.manager_list # order/accept/も必要だなこれ
            
        subject = "依頼が来ています"
        transactions_link = f"http://172.25.60.44/{order_type.lower()}/accept/index.html?id={order_id}"
        usage_rules_link = "http://172.25.60.44/main/"
        
        body = f"""  D科4年のプロジェクト「学内配達ロボットTENQ」です。\n\n  取引の依頼が来ています。以下のリンクから取引の承認・拒否を行なってください。\n\n▼承認用ページ\n  {transactions_link}\n\n  また、TENQの概要・使い方については以下のTENQホームページをご覧ください。\n\n▼TENQホームページ\n  {usage_rules_link}\n\n※このメールは自動で送信されています。"""
        
        for receiver_email in receiver_list:
            send_email(self.sender_name, self.sender_email, self.app_password, receiver_email, subject, body)

    def accepted(self, order_id: str):
        """
        確認メールを送信（依頼された側に送る）
        
        引数：
            オーダーID
        """
        # 送信情報をオーダーリストから取得する
        order_info = self.order_manager.get_order("ID", order_id)
        order_type = order_info["ORDER_TYPE"][0]
        
        if order_type == "SEND":
            order_type_name = "荷物を受け取る"
            receiver_email = order_info["RECEIVER"][0]
            opponent_email = order_info["SENDER"][0]
            place = order_info["RECEIVE_PLACE"][0]
            time = order_info["RECEIVE_TIME"][0]
        elif order_type == "RECEIVE" or order_type == "ORDER":
            if order_type == "RECEIVE":
                order_type_name = "荷物を送る"
            elif order_type == "ORDER":
                order_type_name = "商品を発送する"
            receiver_email = order_info["SENDER"][0]
            opponent_email = order_info["RECEIVER"][0]
            place = order_info["PICKUP_PLACE"][0]
            time = order_info["PICKUP_TIME"][0]
            
        item_type = order_info["ITEM_TYPE"][0]
        item_name = order_info["ITEM_NAME"][0]
        
        subject = "あなたが依頼を承認しました"
        body = f"""  D科4年のプロジェクト「学内配達ロボットTENQ」です。\n\n  あなたが依頼を承認したため、以下の取引が成立しました。\n\n< 取引内容 >\n   あなたの行動 : {order_type_name}\n   取引相手 : {opponent_email}\n   荷物の種類 : {''.join(c for c in item_type if not c.isnumeric())}\n   荷物の名称 : {item_name}\n   場所 : {place}\n   時間 : {time}\n\n  上記の通りに取引を行ってください。また、その際に設定していただいたPINコードを使いますので、お忘れないようお願いいたします。\n\n※このメールは自動で送信されています"""
        
        send_email(self.sender_name, self.sender_email, self.app_password, receiver_email, subject, body)

    def request_result(self, order_id: str, result: str):
        """
        依頼結果メールを送信する
        
        引数：
            オーダーID
            依頼結果 -> 承認 : "accepted" / 拒否 : "denied" / タイムアウト : "timeout"
        """
        # 送信情報をオーダーリストから取得する
        order_info = self.order_manager.get_order("ID", order_id)
        order_type = order_info["ORDER_TYPE"][0]
        
        if order_type == "SEND":
            order_type_name = "荷物を送る"
            receiver_email = order_info["SENDER"][0]
            opponent_email = order_info["RECEIVER"][0]
        elif order_type == "RECEIVE":
            order_type_name = "荷物を受け取る"
            receiver_email = order_info["RECEIVER"][0]
            opponent_email = order_info["SENDER"][0]
        elif order_type == "ORDER":
            order_type_name = "商品を注文する"
            receiver_email = order_info["RECEIVER"][0]
            opponent_email = "TENQ管理者"
            
        item_type = order_info["ITEM_TYPE"][0]
        item_name = order_info["ITEM_NAME"][0]
        
        # 依頼結果に応じて内容を変更する
        if result == "accepted":
            if order_type == "SEND":
                your_action = order_type_name
                place = order_info["PICKUP_PLACE"][0]
                time = order_info["PICKUP_TIME"][0]
            elif order_type == "RECEIVE":
                your_action = order_type_name
                place = order_info["RECEIVE_PLACE"][0]
                time = order_info["RECEIVE_TIME"][0]
            elif order_type == "ORDER":
                your_action = "商品を受け取る"
                place = order_info["RECEIVE_PLACE"][0]
                time = order_info["RECEIVE_TIME"][0]
            subject = "依頼が承認されました"
            body = f"""  D科4年のプロジェクト「学内配達ロボットTENQ」です。\n\n  お相手の承認をとることができたため、以下の取引が成立しました。\n\n< 取引内容 >\n   あなたの行動 : {your_action}\n   取引相手 : {opponent_email}\n   荷物の種類 : {''.join(c for c in item_type if not c.isnumeric())}\n   荷物の名称 : {item_name}\n   場所 : {place}\n   時間 : {time}\n\n  上記の通りに取引を行ってください。また、その際に設定していただいたPINコードを使いますので、お忘れないようお願いいたします。\n\n※このメールは自動で送信されています"""
        elif result == "denied":
            subject = "依頼が拒否されました"
            body = f"""  D科4年のプロジェクト「学内配達ロボットTENQ」です。\n\n  お相手の承認をとることができなかったため、以下の取引はキャンセルされます。\n\n< 取引内容 >\n   依頼内容 : {order_type_name}\n   取引相手 : {opponent_email}\n   荷物の種類 : {''.join(c for c in item_type if not c.isnumeric())}\n   荷物の名称 : {item_name}\n\n  再度取引の依頼を行う際は、お相手のメールアドレスに間違いが無いことをご確認ください。\n\n※このメールは自動で送信されています"""
        elif result == "timeout":
            subject = "依頼がタイムアウトしました"
            body = f"""  D科4年のプロジェクト「学内配達ロボットTENQ」です。\n\n  お相手の承認/拒否を一定時間以内に確認できなかったため、以下の取引はキャンセルされます。\n\n< 取引内容 >\n   依頼内容 : {order_type_name}\n   取引相手 : {opponent_email}\n   荷物の種類 : {''.join(c for c in item_type if not c.isnumeric())}\n   荷物の名称 : {item_name}\n\n  再度取引の依頼を行う際は、お相手のメールアドレスに間違いが無いことをご確認ください。\n\n※このメールは自動で送信されています"""

        send_email(self.sender_name, self.sender_email, self.app_password, receiver_email, subject, body)
        
    def notice(self, order_id: str, type: str, movement: str):
        """
        お知らせメールを送信
        
        引数：
            オーダーID
            種類 picking / receive
            動作 moving / arrived
        """
        # 送信情報をオーダーリストから取得する
        order_info = self.order_manager.get_order("ID", order_id)
        
        if type == "picking":
            receiver_email = order_info["SENDER"][0]
            place = order_info["PICKUP_PLACE"][0]
            if movement == "moving":
                time = order_info["PICKUP_TIME"][0]
            elif movement == "arrived":
                text = "荷物を積みに早めにお越しください。"
        elif type == "receive":
            receiver_email = order_info["RECEIVER"][0]
            place = order_info["RECEIVE_PLACE"][0]
            if movement == "moving":
                time = order_info["RECEIVE_TIME"][0]
            elif movement == "arrived":
                text = "荷物を受け取りに早めにお越しください。"
        
        if movement == "moving":
            subject = "今向かってます"
            body = f"""  D科4年のプロジェクト「学内配達ロボットTENQ」です。\n\n------- 以下の場所に向かっています -------\n\n   場所 : {place}\n   到着予定時刻 : {time}\n\n------------------------------------------\n\n※このメールは自動で送信されています"""
        elif movement == "arrived":
            subject = "到着しました"
            body = f"""  D科4年のプロジェクト「学内配達ロボットTENQ」です。\n\n------- 以下の場所に到着しました -------\n\n   場所 : {place}\n\n----------------------------------------\n\n  {text}\n\n※このメールは自動で送信されています"""
            
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