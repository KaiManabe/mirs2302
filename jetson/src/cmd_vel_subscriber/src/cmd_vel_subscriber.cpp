#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sstream>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cerrno>
#include <csignal>


//ソケット通信のバッファサイズ
//扱うデータの規模によって調整する
#define BUFFER_SIZE 32

#define TIRE_PITCH (double)0.555

/*サーバのアドレスとポートは適宜変更　もしくはプログラム内外で取得すること*/
#define PORT_NUMBER 56789
#define ADDRESS "192.168.1.4"



int sock = -1;
int client_fd = -1;

void testcallback(const geometry_msgs::Twist::ConstPtr& tw){
    //printf("linear : %f , %f,  %f     angular : %f , %f , %f  \n", tw->linear.x, tw->linear.y, tw->linear.z, tw->angular.x, tw->angular.y, tw->angular.z);
    unsigned char send_num[3];

    send_num[0] = static_cast<unsigned char>(255);
    double l_f = 0;
    double r_f = 0;
    l_f = tw->linear.x - (TIRE_PITCH * tw->angular.z / 2.0);
    r_f = tw->linear.x + (TIRE_PITCH * tw->angular.z / 2.0);

    int l_d = (int)(1000.0 * l_f / 6.0);
    int r_d = (int)(1000.0 * r_f / 6.0);
    
    l_d += 127;
    r_d += 127;

    if(l_d > 254){
        l_d = 254;
    }
    if(r_d > 254){
        r_d = 254;
    }

    send_num[1] = static_cast<unsigned char>(l_d);
    send_num[2] = static_cast<unsigned char>(r_d);
    //printf("%d,  %d  \n", l_d, r_d);
    int sent = -1;
    while(sent == -1){
        sent = send(sock, send_num, sizeof(send_num), 0);
    }

}

void close_sock(int signum){
    printf("[INFO][cmd_vel_pub] : ソケット通信 (%d, %d)をクローズします\n", client_fd, sock);
    close(client_fd);
    close(sock);
    exit(signum);
}

int main(int argc, char** argv){
    signal(SIGINT, close_sock);
    signal(SIGTERM, close_sock);


    /*ソケットの作成*/
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0){
        printf("[FATAL][cmd_vel_pub] : ソケットの作成に失敗\n");
        return -1;
    }else{
        printf("[INFO][cmd_vel_pub] : ソケットの作成に成功\n");
    }



    /*サーバ情報を宣言する？よくわからない*/
    struct sockaddr_in server;
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = inet_addr(ADDRESS);    //アドレスを変更するならばここも変える
    server.sin_port        = htons(PORT_NUMBER);    //ポートも同様
    printf("[INFO][cmd_vel_pub] : アドレス%s ポート%dに接続を試みています...\n", ADDRESS, PORT_NUMBER);


    /*サーバに接続する*/
    client_fd = connect(sock, (struct sockaddr*)&server, sizeof(server));
    if (client_fd < 0)
    {   
        /*
        ノンブロッキングモードにしているとconnect関数は115エラーを吐くらしい
        エラーを吐いてもソケット通信は確立されるので115エラーの場合は無視してよい
        */
        if(errno != 115){
            printf("[FATAL][cmd_vel_pub] : サーバに接続できませんでした\n");
            return -1;
        }
    }
    sleep(0.25);    //確立を待ってあげる　いらないかも
    printf("[INFO][cmd_vel_pub] : サーバとの接続を確立しました\n");


    /*
    ソケットをノンブロッキングモードに設定
    これをすることでrecv関数とsend関数が非同期処理になるので注意
    */
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    ros::init(argc, argv, "cmd_vel_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1, testcallback);
    ros::spin();
    close_sock(0);
    return 0;
}