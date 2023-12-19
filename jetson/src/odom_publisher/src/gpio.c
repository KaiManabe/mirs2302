#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>


//#define DEBUG_MODE
#define SYSFS_LA 76
#define SYSFS_LB 51
#define SYSFS_RA 168
#define SYSFS_RB 38


//asciiで0と1が48, 49をあらわすが、これをintに変換するタイムロスを避けるためそのままつかう
#define HIGH (char)49
#define LOW (char)48


char LA = 0;
char LB = 0;
char RA = 0;
char RB = 0;

char LA_prev = 0;
char LB_prev = 0;
char RA_prev = 0;
char RB_prev = 0;

long L = (long)0;
long R = (long)0;


int pins[4] = {SYSFS_LA, SYSFS_LB, SYSFS_RA, SYSFS_RB};

void *printer(void *arg){
    //デバッグ用
    while(1){
        printf("\n %ld, %ld       ", L, R);
        //printf("\n %d      %d      %d      %d     ", (int)LA, (int)LB, (int)RA, (int)RB);
        usleep(10000);
    }
}


void *enc_monitor(void *arg){
    //ファイルディスクリプタを4つつくる
    int fd[4];
    for (int i = 0; i < 4; i++){
        char path[256];
        sprintf(path, "/sys/class/gpio/gpio%d/value", pins[i]);
        fd[i] = open(path, O_RDONLY);
    }

    char bufbuf[5];

    while(1){
        lseek(fd[0], 0, SEEK_SET);
        lseek(fd[1], 0, SEEK_SET);
        lseek(fd[2], 0, SEEK_SET);
        lseek(fd[3], 0, SEEK_SET);
        read(fd[0], &LA, 1);
        read(fd[1], &LB, 1);
        read(fd[2], &RA, 1);
        read(fd[3], &RB, 1);

        
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



int start_monitor(){

    //gpio初期化
       
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


    //スレッドを走らせる

    #ifdef DEBUG_MODE
        pthread_t printer_thread;
        pthread_create(&printer_thread, NULL, printer, (void *)NULL);
    #endif
    

    pthread_t monitor_thread;
    pthread_create(&monitor_thread, NULL, enc_monitor, (void *)NULL);
}

long Renc(){
    return R;
}

long Lenc(){
    return L;
}
