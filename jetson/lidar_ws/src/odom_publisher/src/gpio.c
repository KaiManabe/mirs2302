#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>


//#define DEBUG_MODE
//GPIOピンをsysfs番号で定義する
#define SYSFS_LA 76
#define SYSFS_LB 51
#define SYSFS_RA 168
#define SYSFS_RB 38


//asciiで0と1が48, 49をあらわすが、これをintに変換するタイムロスを避けるためそのままつかう
#define HIGH (char)49
#define LOW (char)48


//各ピンのIO(sysfsの中身)を格納する
char LA = 0;
char LB = 0;
char RA = 0;
char RB = 0;

//前周期のIO
char LA_prev = 0;
char LB_prev = 0;
char RA_prev = 0;
char RB_prev = 0;


//エンコーダ値を覚えておくグローバル変数
long L = (long)0;
long R = (long)0;

//pin番号をsysfsにおける番号に変換し、配列にしておく
int pins[4] = {SYSFS_LA, SYSFS_LB, SYSFS_RA, SYSFS_RB};



/*
デバッグ用関数
一度呼ばれるとエンコーダ値を出力しつづける
pthreadと併用する必要有
*/
void *printer(void *arg){
    while(1){
        printf("\n %ld, %ld       ", L, R);
        //printf("\n %d      %d      %d      %d     ", (int)LA, (int)LB, (int)RA, (int)RB);
        usleep(10000);
    }
}

/*
一度呼ばれるとエンコーダ値をモニタリングしてグローバル変数を更新し続ける
pthreadと併用する必要有
*/
void *enc_monitor(void *arg){
    //ファイルディスクリプタを4つつくる
    //高速化のためディスクリプタは閉じない READ ONLYなのでアクセス権云々は気にしなくてよい（はず）
    int fd[4];
    for (int i = 0; i < 4; i++){
        char path[256];
        sprintf(path, "/sys/class/gpio/gpio%d/value", pins[i]);
        fd[i] = open(path, O_RDONLY);
    }
    

    while(1){
        //sysfsからGPIOのIOを読み取る
        lseek(fd[0], 0, SEEK_SET);
        lseek(fd[1], 0, SEEK_SET);
        lseek(fd[2], 0, SEEK_SET);
        lseek(fd[3], 0, SEEK_SET);
        read(fd[0], &LA, 1);
        read(fd[1], &LB, 1);
        read(fd[2], &RA, 1);
        read(fd[3], &RB, 1);

        
        //以下、IOに応じたエンコーダ値の加減
        if(LA != LA_prev){
            if (LA == HIGH){
                //LA立ち上がり
                if (LB == LOW){
                    L += 1L;
                }else{
                    L -= 1L;
                }
            }else{
                //LA立ち下がり
                if (LB == HIGH){
                    L += 1L;
                }else{
                    L -= 1L;
                }
            }
        }else if(LB != LB_prev){
            if (LB == HIGH){
                //LB立ち上がり
                if (LA == HIGH){
                    L += 1L;
                }else{
                    L -= 1L;
                }
            }else{
                //LB立ち下がり
                if (LA == LOW){
                    L += 1L;
                }else{
                    L -= 1L;
                }
            }
        }


        if(RA != RA_prev){
            if (RA == HIGH){
                //RA立ち上がり
                if (RB == HIGH){
                    R += 1L;
                }else{
                    R -= 1L;
                }
            }else{
                //RA立ち下がり
                if (RB == LOW){
                    R += 1L;
                }else{
                    R -= 1L;
                }
            }
        }else if(RB != RB_prev){
            if (RB == HIGH){
                //RB立ち上がり
                if (RA == LOW){
                    R += 1L;
                }else{
                    R -= 1L;
                }
            }else{
                //RB立ち下がり
                if (RA == HIGH){
                    R += 1L;
                }else{
                    R -= 1L;
                }
            }
        }


        LA_prev = LA;
        LB_prev = LB;
        RA_prev = RA;
        RB_prev = RB;
    }

    
    for (int i = 0; i < 4; i++){
        close(fd[i]);
    }
}


/*
外部からの呼び出し用
これを実行してからRenc()/Lenc()を使うべし
*/
int start_monitor(){

    //gpioを初期化する
    //使用する4つのピンに対して、入力モードをinにする　(directionファイルにinを書き込み)
    int f = open("/sys/class/gpio/export", O_WRONLY);
    for(int i = 0; i < 4; i++){
        char buffer[3];
        sprintf(buffer, "%d", pins[i]);
        int len = 2;
        if (pins[i] >= 100){
            len = 3;
        }
        write(f, buffer, len);
        char fpath[256];
        sprintf(fpath, "/sys/class/gpio/gpio%d/direction", pins[i]);
        int ff = open(fpath, O_WRONLY);
        write(ff, "in", 2);
        close(ff);
    }
    close(f);


    //DEBUG_MODEがdefineされているならばデバッグ用関数を別スレッドで走らせる
    #ifdef DEBUG_MODE
        pthread_t printer_thread;
        pthread_create(&printer_thread, NULL, printer, (void *)NULL);
    #endif
    
    //エンコーダ信号監視用スレッドの開始
    pthread_t monitor_thread;
    pthread_create(&monitor_thread, NULL, enc_monitor, (void *)NULL);
}


/*
右エンコーダ値を返す
*/
long Renc(){
    return R;
}


/*
左エンコーダ値を返す
*/
long Lenc(){
    return L;
}
