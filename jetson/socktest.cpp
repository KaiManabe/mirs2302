#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cerrno>
#define ADDRESS "127.0.0.1"

int main(){
    /*ソケットの作成*/
    const long port_number = 55555;
    int sock = 0;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0){
        printf("[ERROR][cpp] : ソケットの作成に失敗\n");
    }

    //ソケットをノンブロッキングモードに設定
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in server;
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = inet_addr(ADDRESS);
    server.sin_port        = htons(port_number);


    /*サーバに接続する*/
    if ((connect(sock, (struct sockaddr*)&server, sizeof(server))) < 0)
    {
        if(errno != 115){
            printf("[ERROR][cpp] : サーバに接続できませんでした\n");
        }
    }

    sleep(0.25);

    /*サーバに送信する*/
    /*
        int int_arr[10] = {1,2,3,4,5,6,7,8,9,10};
        unsigned char senddata[10];
        for(int i = 0; i < 10; i++){
            senddata[i] = static_cast<unsigned char>(int_arr[i]);
        }
        send(sock, senddata, sizeof(senddata), 0);
    */

    int num = 0;
    unsigned char send_num[1];
    while(1){
        send_num[0] = static_cast<unsigned char>(num);

        int data_sent = -1;
        while(data_sent == -1){
            data_sent = send(sock, send_num, sizeof(send_num), 0);
        }
        num += 1;
        sleep(1);


        /*サーバから受信する*/
        char buff[10] = {0,};
        int data_arrived = -1;
        data_arrived = recv(sock, buff, sizeof(buff), 0);

        if(data_arrived != -1){
            for(int i = 0; i < sizeof(buff); i++){
                printf("%d \n", static_cast<int>(static_cast<unsigned char>(buff[i])));
            }
            if(static_cast<int>(static_cast<unsigned char>(buff[0])) == 254){
                break;
            }
        }
    }

    close(sock);


    return 0;
}
