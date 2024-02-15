/*
raspiからソケット通信で目的地の座標を受け取り、publishする
move_baseのステータスをsubscribeして、ソケット通信で送る
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>

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


#include "GOAL_POSITIONS.h"

//ソケット通信のバッファサイズ
//扱うデータの規模によって調整する
#define BUFFER_SIZE 32

/*サーバのアドレスとポートは適宜変更　もしくはプログラム内外で取得すること*/
#define PORT_NUMBER 55555
#define ADDRESS "192.168.1.4"



//ディスクリプタ
int sock = -1;
int client_fd = -1;



//move_baseのステータスをraspiにソケット通信で送信する
void send_status(const actionlib_msgs::GoalStatusArray::ConstPtr &status){
    int status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9

    if (!status->status_list.empty()){
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
    status_id = goalStatus.status;
    }
    
    unsigned char send_num[1];
    send_num[0] = static_cast<unsigned char>(status_id);
    int sent = -1;
    while(sent == -1){
        sent = send(sock, send_num, sizeof(send_num), 0);
    }
}



//ソケットを閉じて終了
void close_sock(int signum){
    printf("[INFO][goal_publisher] : ソケット通信 (%d, %d)をクローズします\n", client_fd, sock);
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
        printf("[FATAL][goal_publisher] : ソケットの作成に失敗\n");
        return -1;
    }else{
        printf("[INFO][goal_publisher] : ソケットの作成に成功\n");
    }


    // SO_REUSEADDR オプションをセット
    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        printf("[FATAL][goal_publisher] : SO_REUSEADDRオプションの設定に失敗\n");
        return -1;
    }else{
        printf("[INFO][goal_publisher] : SO_REUSEADDRオプションの設定を完了\n");
    }


    /*サーバ情報を宣言する？よくわからない*/
    struct sockaddr_in server;
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = inet_addr(ADDRESS);    //アドレスを変更するならばここも変える
    server.sin_port        = htons(PORT_NUMBER);    //ポートも同様
    printf("[INFO][goal_publisher] : アドレス%s ポート%dに接続を試みています...\n", ADDRESS, PORT_NUMBER);
    
    int retries = 0;
    while(1){
        /*サーバに接続する*/
        client_fd = connect(sock, (struct sockaddr*)&server, sizeof(server));
        if (client_fd < 0)
        {   
            /*
            ノンブロッキングモードにしているとconnect関数は115エラーを吐くらしい
            エラーを吐いてもソケット通信は確立されるので115エラーの場合は無視してよい
            */
            if(errno != 115){
                printf("[WARN][goal_publisher] : サーバに接続できませんでした。リトライ中...\n");
                usleep(5000000);
                retries += 1;
                if (retries > 5){
                    printf("[FATAL][goal_publisher] : サーバに接続できませんでした。ノードを終了します\n");
                    close_sock(-1);
                    return -1;
                }
            }
        }else{
            break;
        }
    }
    sleep(0.25);    //確立を待ってあげる　いらないかも
    printf("[INFO][goal_publisher] : サーバとの接続を確立しました\n");


    /*
    ソケットをノンブロッキングモードに設定
    これをすることでrecv関数とsend関数が非同期処理になるので注意
    */
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);




    ros::init(argc, argv, "goal_publisher");


    //ノードハンドラ
    ros::NodeHandle n;

    //move_baseのstatusをsubscribeするオブジェクト
    ros::Subscriber sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, &send_status);

    //goalをpublishするオブジェクト
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    ros::Time current_time, last_time;
    ros::Rate r(1.0);

    //ノードが建っている -> n.ok()がtrue
    while(n.ok()){
        r.sleep();

        //subscribeしているトピックの更新があればコールバックが実行される
        ros::spinOnce();
        
        char buff_raw[BUFFER_SIZE] = {0,};  //生の受信データを格納するバッファ
        int buff_int[BUFFER_SIZE] = {-1,};  //int型の受信データを格納するバッファ
        int received_data[BUFFER_SIZE] = {-1,};  //ヘッダとフッタを除いた受信データを格納する
        int data_arrived = -1;  //値が到着しているかをあらわすフラグ -1なら未到着
        
        data_arrived = recv(sock, buff_raw, sizeof(buff_raw), 0);   //データを受け取り

        if(data_arrived != -1){ //データが到着しているならば
            for(int i = 0; i < BUFFER_SIZE; i++){
                //char型をuchar型にキャストしてint型にキャストすることで0-255に収めている
                buff_int[i] = static_cast<int>(static_cast<unsigned char>(buff_raw[i]));
                
            }

            //received_dataにヘッダ・フッタを除いたデータを格納する
            for(int i = 0; i < BUFFER_SIZE; i++){
                if(buff_int[i] == 255 && (i + 1) < BUFFER_SIZE){
                    for(int ii = (i + 1); ii < BUFFER_SIZE; ii++){
                        if(buff_int[ii] == 254){
                            break;
                        }
                        received_data[ii - i - 1] = buff_int[ii];
                    }
                    break;
                }
            }
        }else{
            //データが到着していなければトピックは配信しない
            continue;
        }

        double GOAL_INDEX_X[] = GOAL_X;
        double GOAL_INDEX_Y[] = GOAL_Y;
        double GOAL_INDEX_QZ[] = GOAL_QZ;
        double GOAL_INDEX_QW[] = GOAL_QW;
        double goal_x = 0.0;
        double goal_y = 0.0;
        double goal_qz = 0.0;
        double goal_qw = 0.0;

        if(received_data[0] == -1){
            //データが欠けている（1つ以上のデータがない）場合はやりなおし
            continue;
        }
        
        goal_x = GOAL_INDEX_X[received_data[0]];
        goal_y = GOAL_INDEX_Y[received_data[0]];
        goal_qz = GOAL_INDEX_QZ[received_data[0]];
        goal_qw = GOAL_INDEX_QW[received_data[0]];

        printf("[INFO][goal_publisher] : 目的地を設定します (index : %d)\n", received_data[0]);
        printf("[INFO][goal_publisher] : x -> %lf \n", goal_x);
        printf("[INFO][goal_publisher] : y -> %lf \n", goal_y);



        //topicを受けたときのコールバック関数が指定されているなら
        //これを実行した時点でコールバックされる
        ros::spinOnce();

        //現在時刻
        current_time = ros::Time::now();


        //odomをtopicとして飛ばすぞ
        geometry_msgs::PoseStamped goal;

        goal.header.stamp = current_time;
        goal.header.frame_id = "map";

        //set the position
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.z = goal_qz;
        goal.pose.orientation.w = goal_qw;


        //publish the message
        goal_pub.publish(goal);
        
    }
    close_sock(0);
    return 0;
}