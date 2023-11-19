/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

/*******************************************************
**********************ここから追記***********************
*******************************************************/

//なんかよくわからないし使ってるかどうかもわからないけどライブラリをインクルードしておく
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cerrno>
#include <cmath>

//ultra_simpleに元々備わっていた出力機能(測距ログ)を無効化する場合にdefineする
#define DISABLE_DEFAULT_OUTPUTS


//ソケット通信のバッファサイズ
//扱うデータの規模によって調整する
#define BUFFER_SIZE 32


/*サーバのアドレスとポートは適宜変更　もしくはプログラム内外で取得すること*/
#define PORT_NUMBER 55555
#define ADDRESS "127.0.0.1"

/*******************************************************
**********************ここまで追記***********************
*******************************************************/


using namespace sl;

void print_usage(int argc, const char * argv[])
{
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
		   " For udp channel\n %s --channel --udp <ipaddr> [port NO.]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
	const char * opt_is_channel = NULL; 
	const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
	sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

	bool useArgcBaudrate = false;

    IChannel* _channel;

    /*******************************************************
    **********************ここから追記***********************
    *******************************************************/
    
    int send_data = 0;
    
    /*ソケットの作成*/
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0){
        printf("[FATAL][ultra_simple] : ソケットの作成に失敗\n");
        return -1;
    }else{
        printf("[INFO][ultra_simple] : ソケットの作成に成功\n");
    }


    /*
    ソケットをノンブロッキングモードに設定
    これをすることでrecv関数とsend関数が非同期処理になるので注意
    */
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);


    /*サーバ情報を宣言する？よくわからない*/
    struct sockaddr_in server;
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = inet_addr(ADDRESS);    //アドレスを変更するならばここも変える
    server.sin_port        = htons(PORT_NUMBER);    //ポートも同様


    /*サーバに接続する*/
    if ((connect(sock, (struct sockaddr*)&server, sizeof(server))) < 0)
    {   
        /*
        ノンブロッキングモードにしているとconnect関数は115エラーを吐くらしい
        エラーを吐いてもソケット通信は確立されるので115エラーの場合は無視してよい
        */
        if(errno != 115){
            printf("[FATAL][ultra_simple] : サーバに接続できませんでした\n");
            return -1;
        }
    }
    sleep(0.25);    //確立を待ってあげる　いらないかも
    printf("[INFO][ultra_simple] : サーバとの接続を確立しました\n");


    /*******************************************************
    **********************ここまで追記***********************
    *******************************************************/


    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);

	 
	if (argc>1)
	{ 
		opt_is_channel = argv[1];
	}
	else
	{
		print_usage(argc, argv);
		return -1;
	}

	if(strcmp(opt_is_channel, "--channel")==0){
		opt_channel = argv[2];
		if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
		{
			// read serial port from the command line...
			opt_channel_param_first = argv[3];// or set to a fixed value: e.g. "com3"
			// read baud rate from the command line if specified...
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);	
			useArgcBaudrate = true;
		}
		else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
		{
			// read ip addr from the command line...
			opt_channel_param_first = argv[3];//or set to a fixed value: e.g. "192.168.11.2"
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);//e.g. "8089"
			opt_channel_type = CHANNEL_TYPE_UDP;
		}
		else
		{
			print_usage(argc, argv);
			return -1;
		}
	}
	else
	{
		print_usage(argc, argv);
        return -1;
	}

	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
	{
		if (!opt_channel_param_first) {
#ifdef _WIN32
		// use default com port
		opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
		opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
		opt_channel_param_first = "/dev/ttyUSB0";
#endif
		}
	}

    
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
        if(useArgcBaudrate){
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result)) 
                {
	                connectSuccess = true;
                }
                else{
                    delete drv;
					drv = NULL;
                }
            }
        }
        else{
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
			for(size_t i = 0; i < baudRateArraySize; ++i)
			{
				_channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result)) 
                    {
	                    connectSuccess = true;
                        break;
                    }
                    else{
                        delete drv;
					    drv = NULL;
                    }
                }
			}
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP){
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) 
            {
	            connectSuccess = true;
            }
            else{
                delete drv;
				drv = NULL;
            }
        }
    }


    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));
		
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
    /*******************************************************
    **********************ここから変更点あり******************
    *******************************************************/

    send_data = 0;  //1ならソケット通信で測距データを送信する 0なら一時停止

    while (1) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                #ifndef DISABLE_DEFAULT_OUTPUTS
                    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                        (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                        (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                        nodes[pos].dist_mm_q2/4.0f,
                        nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                #endif

                float angle_in_degrees = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                long deg = (long)(angle_in_degrees * (float)1000);
                float distance_in_meters = nodes[pos].dist_mm_q2 / (1 << 2);
                long dist = (long)distance_in_meters;

                /*測距データの提供を命令されていたら送信*/
                if(send_data == 1){
                    unsigned char send_num[10];     
                    send_num[0] = static_cast<unsigned char>(255);
                    send_num[1] = static_cast<unsigned char>(11);
                    for(int i = 0; i < 3; i++){
                        int val;
                        val = (int)((deg % (long)std::pow(254,i+1)) / (long)pow(254,i))
                        send_num[i+2] = static_cast<unsigned char>(val);
                    }

                    for(int i = 0; i < 3; i++){
                        int val;
                        val = (int)((dist % (long)std::pow(254,i+1)) / (long)pow(254,i))
                        send_num[i+5] = static_cast<unsigned char>(val);
                    }
                    send_num[8] = nodes[pos].quality;
                    send_num[9] = static_cast<unsigned char>(254);

                    int data_sent = -1;
                    while(data_sent == -1){
                        data_sent = send(sock, send_num, sizeof(send_num), 0);
                    }
                }


            }
            break;
        }


        /*
        ソケット通信の受信データを確認
        データが到着していたら、それに応じた処理をする
        */
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


            /*データに応じた処理*/
            switch(received_data[0]){
                case 1:
                    //ソケット通信で測距データの提供を開始
                    send_data = 1;
                    break;
                
                case 2:
                    //ソケット通信での測距データの提供を停止
                    send_data = 0;
                    break;
                
                case 3:
                    //プログラムの強制終了
                    send_data = -1;
                    break;
                
                default:
                    break;
            }

        }

        /*ソケット通信で終了命令を受け取ったら終了*/
        if(send_data == -1){
            break;
        }

        /*ctrl + C で強制終了*/
        if (ctrl_c_pressed){ 
            break;
        }
    }

    /*******************************************************
    **********************変更点ここまで*********************
    *******************************************************/

    drv->stop();
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    // done!
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}

