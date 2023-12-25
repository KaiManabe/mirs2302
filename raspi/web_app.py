import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# 承認メール送信用関数
def send_email(sender_email, app_password, receiver_email, subject, body):
    # メールの設定
    message = MIMEMultipart()
    message["From"] = sender_email
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


# 以下は、実際にメールを送信するための例です。
if __name__ == "__main__":
    sender_email = "mirs2302tenq@gmail.com"  # 送信元のメールアドレス
    app_password = "lvst oefb zsfw kmmk"  # 送信元のアプリパスワード
    receiver_email = "recipient_email@example.com"  # 送信先のメールアドレス
    subject = "Test Subject"
    
    approval_link = "http://172.25.19.2"  # 承認リンク
    approval_body = f"""
    ※このメールは自動で送信されています。
    
    以下のリンクからメールの承認を行なってください。
    
    {approval_link}
    """
    
    warning_body = f"""
    TENQに異常が検知されました。直ちに確認作業を行なってください。
    
    ※このメールは自動で送信されています。
    """
    
    body = approval_body

    send_email(sender_email, app_password, receiver_email, subject, body)

def approval(order_id, mail, order_type):
    """
    承認メールを送信する関数
    引数：
        依頼のID,依頼者のメールアドレス,依頼の種類,依頼物

    戻り値：
        なし
    """
    sender_email = "mirs2302tenq@gmail.com"  # 送信元のメールアドレス
    app_password = "lvst oefb zsfw kmmk"  # 送信元のアプリパスワード
    receiver_email = mail #送信先のメールアドレス
    subject = "学内配達ロボットTENQ-依頼承認メール"

    approval_link = 'http://172.25.19.2/raspi/html/${order_type}/accept/index.html?id=${order_id}'
    body = f"""
    ※このメールは自動で送信されています。
    D科4年のプロジェクト、「学内配達TENQ」です。

    以下のリンクからメールの承認を行なってください。
    
    {approval_link}
    """
    send_email(sender_email, app_password, receiver_email, subject, body)

def warning(warn_type: str, module: str = None, door: str = None):
    """
    異常検知メールを送信する関数

    引数：
        異常の種類、モジュール番号、扉番号
        異常の種類 "module":モジュール持ち去り "door":扉こじ開け "airframe":機体持ち去り

    戻り値:
        なし
    """
    sender_email = "mirs2302tenq@gmail.com"  # 送信元のメールアドレス
    app_password = "lvst oefb zsfw kmmk"  # 送信元のアプリパスワード
    # 管理者のメールアドレスリスト
    manager_list = [
        "d20139@numazu.kosen-ac.jp"
        ]
    # "d20102@numazu.kosen-ac.jp",
    # "d20136@numazu.kosen-ac.jp",
    subject = "TENQ error"
    
    if warn_type == "module":
        body = f"""
        TENQの異常を検知しました。
        {module}モジュールが持ち去られた可能性があります。
        直ちに確認作業を行ってください。
        ※このメールは自動で送信されています。
        """
    elif warn_type == "door":
        body = f"""
        TENQの異常を検知しました。
        {module}モジュールの扉{door}がこじ開けられた可能性があります。
        直ちに確認作業を行ってください。
        ※このメールは自動で送信されています。
        """
    elif warn_type == "airframe":
        body = f"""
        TENQの異常を検知しました。
        機体が持ち去られた可能性があります。
        直ちに確認作業を行ってください。
        ※このメールは自動で送信されています。
        """
    # 各管理者に送信
    for receiver_email in manager_list:
        print([sender_email, app_password, receiver_email, subject, body]) # デバッグ用
        send_email(sender_email, app_password, receiver_email, subject, body)