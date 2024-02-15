#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>



/*
機体パラメータ
差動二輪駆動ロボットを前提
動力伝達は
モータ&エンコーダ → ギア1(GEAR_RATIO) → ギア2(MOTOR_GEAR_TOOTH/TIRE_GEAR_TOOTH) → タイヤ(TIRE_DIAM)
の順
*/
#define ENC_PPR 52  //エンコーダの回転毎のパルス
#define GEAR_RATIO 13.733564    //ギア比
#define TIRE_GEAR_TOOTH 20  //タイヤについてるギアの歯数
#define MOTOR_GEAR_TOOTH 10     //モータについてるギアの歯数
#define TIRE_DIAM 177   //タイヤの直径
#define TIRE_PITCH 550  //タイヤの間隔
#define PI 3.141592653589


/*
C言語によって作成されたされたライブラリを呼び出し
*/
extern "C" {
  int start_monitor();
  long Renc();
  long Lenc();
}


/*
エンコーダ値をメートルに変換する関数
引数 -> エンコーダ値:long
戻り値 -> 長さ[m]:double
*/
double pulse_to_m(long p){
  double m = ((double)p * (double)TIRE_DIAM * PI * (double)MOTOR_GEAR_TOOTH) / ((double)TIRE_GEAR_TOOTH * (double)ENC_PPR * (double)GEAR_RATIO);
  return (m / (double)1000.0);
}



/*
左右のエンコーダ値から、いまの機体の偏角を計算する関数
引数 -> 左エンコーダ値:long
引数 -> 右エンコーダ値:long
戻り値 -> 機体の偏角[rad]:double
*/
double calc_theta(long L, long R){
  return (pulse_to_m(R - L) / ((double)TIRE_PITCH / (double)1000.0));
}



/*
  エンコーダ値からロボットの速度（進行方向）を求める
  引数：
      左エンコーダ -> long
      前周期左エンコーダ -> long
      右エンコーダ -> long
      前周期右エンコーダ -> long
      dt[s] -> double
*/
double calc_velocity(long L, long L_prev, long R, long R_prev, double dt){
  double vl = pulse_to_m(L - L_prev) / (double)dt;
  double vr = pulse_to_m(R - R_prev) / (double)dt;
  return (vl + vr) / (double)2.0;
}





int main(int argc, char** argv){
  //rosノードの初期化
  ros::init(argc, argv, "odometry_publisher");

  //ノードハンドラ
  ros::NodeHandle n;

  //odomをpublishするpublisherオブジェクト
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);

  //odom -> base_linkのtfをpublishするオブジェクト
  tf2_ros::TransformBroadcaster odom_broadcaster;

  //エンコーダ信号のモニタリング開始
  start_monitor();
  
  
  //機体の位置情報に関する諸々
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double th_prev = 0.0;

  double v = 0.0;
  double vth = 0.0;

  long L = 0L;
  long L_prev = 0L;
  long R = 0L;
  long R_prev = 0L;


  //ros時間の宣言
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  //publishする周波数を決める
  ros::Rate r(10.0);
  

  //ノードが建っている -> n.ok()がtrue
  while(n.ok()){

    //topicを受けたときのコールバック関数が指定されているなら
    //これを実行した時点でコールバックされる
    ros::spinOnce();

    //現在時刻
    current_time = ros::Time::now();

    //エンコーダ値の取得
    L = Lenc();
    R = Renc();


    /*
    th -> odom座標系で機体が向いている方向
    vth -> thの微分
    
    v -> thに向かって機体が進行している速度

    x -> v * cos(th) を積分し続けることで、odom座標系におけるbase_linkの位置を推測する
    y -> v * sin(th) を積分し続けることで、odom座標系におけるbase_linkの位置を推測する
    */
    double dt = (current_time - last_time).toSec();
    th = calc_theta(L, R);
    vth = (th - th_prev) / dt;
    
    v = calc_velocity(L, L_prev, R, R_prev, dt);
    x += v * cos(th) * dt;
    y += v * sin(th) * dt;


    //前周期の値を格納
    th_prev = th;
    L_prev = L;
    R_prev = R;



    //z軸方向の回転をクォータニオンにする
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, th);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

    //odom -> base_link のtfを飛ばす
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);



    //odom座標系におけるbase_link座標をトピックとして配信
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);


    last_time = current_time;
    r.sleep();
  }
}