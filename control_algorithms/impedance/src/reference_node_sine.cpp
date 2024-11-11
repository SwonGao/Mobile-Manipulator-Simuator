#include "ros/ros.h"
#include "cmath"
#include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "reference_node");


  ros::NodeHandle n;

  ros::Publisher car_state_pub = n.advertise<cartesian_state_msgs::PoseTwist>("reference_car_state", 1000);
  ros::Publisher ee_state_pub = n.advertise<cartesian_state_msgs::PoseTwist>("reference_ee_state", 1000);
  ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("reference_wrench", 1000);
  ros::Rate loop_rate(50);

  double secs = 0.0;
  double init_secs = ros::Time::now().toSec();
  cartesian_state_msgs::PoseTwist state_msg, car_state_msg;
  geometry_msgs::WrenchStamped wrench_msg;
  double t = 0.0;
  while (ros::ok())
  {
    secs = ros::Time::now().toSec();
    t = secs - init_secs;
    double t0 = 10;
    double t1 = 20;
    double te = 50;
    // 0 ~ t0 position 
    if(t < t0){  
    state_msg.pose.position.x = 0.2;
    state_msg.pose.position.y = 0.4;
    state_msg.pose.position.z = 0.8;
    state_msg.pose.orientation.w = 0.707;
    state_msg.pose.orientation.x = -0.707;
    state_msg.pose.orientation.y = 0.0;
    state_msg.pose.orientation.z = 0.0;
    state_msg.twist.linear.x = 0.0;
    state_msg.twist.linear.y = 0.0;
    state_msg.twist.linear.z = 0.0;
    state_msg.twist.angular.x = 0.0;
    state_msg.twist.angular.y = 0.0;
    state_msg.twist.angular.z = 0.0;
    ee_state_pub.publish(state_msg);

    car_state_msg.pose.position.x = 0.0;
    car_state_msg.pose.position.y = 0.0;
    car_state_msg.pose.position.z = 0.0;
    car_state_msg.pose.orientation.w = 1.0;
    car_state_msg.pose.orientation.x = 0.0;
    car_state_msg.pose.orientation.y = 0.0;
    car_state_msg.pose.orientation.z = 0.0;

    car_state_msg.twist.linear.x = 0.0;
    car_state_msg.twist.linear.y = 0;
    car_state_msg.twist.linear.z = 0;
    car_state_msg.twist.angular.x = 0;
    car_state_msg.twist.angular.y = 0;
    car_state_msg.twist.angular.z = 0;
    car_state_pub.publish(car_state_msg);
    
    wrench_msg.wrench.force.x = 0;
    wrench_msg.wrench.force.y = 0;
    wrench_msg.wrench.force.z = 0;
    wrench_msg.wrench.torque.x = 0;
    wrench_msg.wrench.torque.y = 0;
    wrench_msg.wrench.torque.z = 0;
    wrench_pub.publish(wrench_msg);
    }
    // t1 < te contact 
    else if(t >= t1 && t < te){
    state_msg.pose.position.x = 0.2 + 0.2 * (t - t0) ;
    state_msg.pose.position.y = 0.6;
    state_msg.pose.position.z = 0.8;
    state_msg.pose.orientation.w = 0.707;
    state_msg.pose.orientation.x = -0.707;
    state_msg.pose.orientation.y = 0.0;
    state_msg.pose.orientation.z = 0.0;
    state_msg.twist.linear.x = 0.2;
    state_msg.twist.linear.y = 0.0;
    state_msg.twist.linear.z = 0.0;
    state_msg.twist.angular.x = 0.0;
    state_msg.twist.angular.y = 0.0;
    state_msg.twist.angular.z = 0.0;
    ee_state_pub.publish(state_msg);
    
    car_state_msg.pose.position.x = 0.2 * (t - t0) ;
    car_state_msg.pose.position.y = 0.025 *  sin(0.25* M_PI * (t - t0));
    car_state_msg.pose.position.z = 0.0;
    car_state_msg.pose.orientation.x = 0.0;
    car_state_msg.pose.orientation.y = 0.0;
    car_state_msg.pose.orientation.z = atan2(0.025 * cos(0.25* M_PI *(t - t0)), 0.2);

    car_state_msg.twist.linear.x = 0.2;
    car_state_msg.twist.linear.y = 0.;
    car_state_msg.twist.linear.z = 0;
    car_state_msg.twist.angular.x = 0;
    car_state_msg.twist.angular.y = 0;
    car_state_msg.twist.angular.z = 0;
    car_state_pub.publish(car_state_msg);
    
    wrench_msg.wrench.force.x = 0;
    
    if(t<=(2*t1+te)/3.0 || t > (t1+2*te)/3.0){
        wrench_msg.wrench.force.y = -5;
    }
    else{
        wrench_msg.wrench.force.y = -10;
    }
    
    wrench_msg.wrench.force.z = 0;
    wrench_msg.wrench.torque.x = 0;
    wrench_msg.wrench.torque.y = 0;
    wrench_msg.wrench.torque.z = 0;
    wrench_pub.publish(wrench_msg);
    }
    // te: final position mode
    else if(t >= te){
    state_msg.pose.position.x = 0.2 + 0.2 * (t - t0) ;
    state_msg.pose.position.y = 0.4;
    state_msg.pose.position.z = 0.8;
    state_msg.pose.orientation.w = 0.707;
    state_msg.pose.orientation.x = -0.707;
    state_msg.pose.orientation.y = 0.0;
    state_msg.pose.orientation.z = 0.0;
    state_msg.twist.linear.x = 0.2;
    state_msg.twist.linear.y = 0.0;
    state_msg.twist.linear.z = 0.0;
    state_msg.twist.angular.x = 0.0;
    state_msg.twist.angular.y = 0.0;
    state_msg.twist.angular.z = 0.0;
    ee_state_pub.publish(state_msg);
    
    car_state_msg.pose.position.x = 0.2 * (t - t0) ;
    car_state_msg.pose.position.y = 0.0;
    car_state_msg.pose.position.z = 0.0;
    car_state_msg.pose.orientation.w = 1.0;
    car_state_msg.pose.orientation.x = 0.0;
    car_state_msg.pose.orientation.y = 0.0;
    car_state_msg.pose.orientation.z = 0.0;

    car_state_msg.twist.linear.x = 0.2;
    car_state_msg.twist.linear.y = 0;
    car_state_msg.twist.linear.z = 0;
    car_state_msg.twist.angular.x = 0;
    car_state_msg.twist.angular.y = 0;
    car_state_msg.twist.angular.z = 0;
    car_state_pub.publish(car_state_msg);
    
    wrench_msg.wrench.force.x = 0;
    wrench_msg.wrench.force.y = -0;
    wrench_msg.wrench.force.z = 0;
    wrench_msg.wrench.torque.x = 0;
    wrench_msg.wrench.torque.y = 0;
    wrench_msg.wrench.torque.z = 0;
    wrench_pub.publish(wrench_msg);
    }
    // t0 ~ t1
    else{
    //state_msg.pose.position.x = 0.4;
    state_msg.pose.position.x = 0.2 + 0.2 * (t - t0) ;
    state_msg.pose.position.y = 0.4;
    state_msg.pose.position.z = 0.8;
    state_msg.pose.orientation.w = 0.707;
    state_msg.pose.orientation.x = -0.707;
    state_msg.pose.orientation.y = 0.0;
    state_msg.pose.orientation.z = 0.0;
    //state_msg.twist.linear.x = 0.;
    state_msg.twist.linear.x = 0.2;
    state_msg.twist.linear.y = 0.0;
    state_msg.twist.linear.z = 0.0;
    state_msg.twist.angular.x = 0.0;
    state_msg.twist.angular.y = 0.0;
    state_msg.twist.angular.z = 0.0;
    ee_state_pub.publish(state_msg);
    
    car_state_msg.pose.orientation.w = 1.0;
    //car_state_msg.pose.position.x = 0.2;
    car_state_msg.pose.position.x = 0.0 + 0.2 * (t - t0) ;
    car_state_msg.pose.position.y = 0.025 *  sin(0.25* M_PI * (t - t0));
    car_state_msg.pose.position.z = 0.0;
    car_state_msg.pose.orientation.x = 0.0;
    car_state_msg.pose.orientation.y = 0.0;
    car_state_msg.pose.orientation.z = atan2(0.025 * cos(0.25* M_PI *(t - t0)), 0.2);
    //car_state_msg.twist.linear.x = 0;
    car_state_msg.twist.linear.x = 0.2;
    car_state_msg.twist.linear.y = 0.0;
    car_state_msg.twist.linear.z = 0;
    car_state_msg.twist.angular.x = 0;
    car_state_msg.twist.angular.y = 0;
    car_state_msg.twist.angular.z = 0;
    car_state_pub.publish(car_state_msg);
    
    wrench_msg.wrench.force.x = 0;
    wrench_msg.wrench.force.y = 0;
    wrench_msg.wrench.force.z = 0;
    wrench_msg.wrench.torque.x = 0;
    wrench_msg.wrench.torque.y = 0;
    wrench_msg.wrench.torque.z = 0;
    wrench_pub.publish(wrench_msg);
    }
   
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
