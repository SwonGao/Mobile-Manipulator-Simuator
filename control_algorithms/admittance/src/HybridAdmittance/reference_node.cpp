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

  ros::Publisher state_pub = n.advertise<cartesian_state_msgs::PoseTwist>("reference_ee_state", 1000);
  ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("reference_wrench", 1000);
  ros::Rate loop_rate(10);

  double secs = 0;
  cartesian_state_msgs::PoseTwist state_msg;
  geometry_msgs::WrenchStamped wrench_msg;
  while (ros::ok())
  {
    secs = ros::Time::now().toSec();

    state_msg.pose.position.x = 0.5;// + 0.2 * sin(M_PI * secs);
    state_msg.pose.position.y = 0.0;
    state_msg.pose.position.z = 0.5;
    state_msg.pose.orientation.x = 0;
    state_msg.pose.orientation.y = 0.707;
    state_msg.pose.orientation.z = 0;
    state_msg.pose.orientation.w = 0.707;
    state_pub.publish(state_msg);
    
    wrench_msg.wrench.force.x = 0;
    wrench_msg.wrench.force.y = -5;
    wrench_msg.wrench.force.z = 0;
    wrench_msg.wrench.torque.x = 0;
    wrench_msg.wrench.torque.y = 0;
    wrench_msg.wrench.torque.z = 0;
    wrench_pub.publish(wrench_msg);
    ros::spinOnce();
    
    loop_rate.sleep();
  }
  return 0;
}
