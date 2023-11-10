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
    sender_email = "your_email@gmail.com"  # 送信元のメールアドレス
    app_password = "your_app_password"  # 送信元のアプリパスワード
    receiver_email = "recipient_email@example.com"  # 送信先のメールアドレス
    subject = "Test Subject"
    
    approval_link = "http://172.25.19.2"  # 承認リンク
    approval_body = f"""
    ※このメールは自動で送信されています。
    
    以下のリンクからメールの承認を行なってください。
    
    {approval_link}
    """
    
    warning_body = f"""
    ※このメールは自動で送信されています。
    
    TENQに異常が検知されました。直ちに確認作業を行なってください。
    """
    
    body = approval_body

    send_email(sender_email, app_password, receiver_email, subject, body)
