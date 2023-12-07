#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

extern "C" {
  long sq(int num);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");


  //ノードを操るためのオブジェクト
  ros::NodeHandle n;

  //こいつがodomをtopicとして飛ばす
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  //こいつがodomをtfに飛ばす
  tf2_ros::TransformBroadcaster odom_broadcaster;



  //実際にはここの値をencから計算するべき
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 1.0;
  double vy = -1.0;
  double vth = 1.0;


  //ros時間を使うならこうやって宣言する
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

    //このへんの値は実際にはｅｎｃから求めることになる
    //x,yの軸はロボットの進行方向なのか？map座標によるものなのか？
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;



    //z軸方向の回転をクォータニオンにする
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, th);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

    //odomをtfに飛ばすぞ
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);



/*=============================debug===============================*/
//なにか座標をデバッグしたいならこのようにする

    //odomをtfに飛ばすぞ
    geometry_msgs::TransformStamped odom_trans2;
    odom_trans2.header.stamp = current_time;
    odom_trans2.header.frame_id = "odom";
    odom_trans2.child_frame_id = "debug_l";

    odom_trans2.transform.translation.x = sq(1);
    odom_trans2.transform.translation.y = y - 1.0;
    odom_trans2.transform.translation.z = 0.0;
    odom_trans2.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans2);


    //odomをtfに飛ばすぞ
    geometry_msgs::TransformStamped odom_trans3;
    odom_trans3.header.stamp = current_time;
    odom_trans3.header.frame_id = "odom";
    odom_trans3.child_frame_id = "debug_r";

    odom_trans3.transform.translation.x = x + 1.0;
    odom_trans3.transform.translation.y = y + 1.0;
    odom_trans3.transform.translation.z = 0.0;
    odom_trans3.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans3);




/*=============================debug===============================*/


    //odomをtopicとして飛ばすぞ
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}