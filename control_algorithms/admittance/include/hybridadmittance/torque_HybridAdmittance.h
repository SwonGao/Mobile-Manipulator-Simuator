#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
//#include "pinocchio/urdf/details.hpp"
//#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "ros/ros.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include "joint_effort_msg/JointEffort.h"
#include "joint_effort_msg/JointEfforts.h"
#include "joint_state_msg/JointState.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include <stdlib.h>
#include <iomanip>
#include <complex>
#include "BiQuad.h"

#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>


using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
#define PI 3.1415926

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

class Admittance
{
protected:
  // ROS VARIABLES:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  // ADMITTANCE PARAMETERS:
  Matrix6d M_, D_, K_, K_f_, Dcar_;

  // Subscribers:
  ros::Subscriber sub_joint_state_;
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_reference_arm_state_;  
  ros::Subscriber sub_wrench_state_;
  ros::Subscriber sub_reference_wrench_state_;
  // Publishers:
  ros::Publisher  pub_arm_cmd_;
  ros::Publisher  pub_car_cmd_;

  // Variables:
  Vector3d      arm_position_;
  Quaterniond   arm_orientation_;
  Vector6d      arm_twist_;
  Vector3d      reference_arm_position_;
  Quaterniond   reference_arm_orientation_;
  Vector6d      reference_arm_twist_;
  Vector6d      wrench_external_;
  Vector6d      reference_wrench_;
  Vector6d      arm_desired_twist_adm_;
  Vector6d      arm_desired_accelaration;
  Vector6d      car_desired_twist_adm_;
  Vector6d      car_desired_accelaration;

  Vector7d      desired_pose_;
  Vector3d      desired_pose_position_;
  Quaterniond   desired_pose_orientation_;

  Vector6d      error;
  Vector6d      error_vel;
  Vector6d      error_acc; 
  Vector6d      error_f;
  Vector6d      F;
  Vector6d      F2;
  Vector6d      tau;
  
  const std::string urdf_filename = std::string("/home/swon/swon_ws/src/Mobile-Manipulator-Simulation/control_algorithms/admittance/src/HybridAdmittance/ur5model_for_torque_control.urdf");
  const int JOINT_ID = 6;
  //pinocchio::Model model;  
  Vector6d q;
  Vector6d q_dot;
  //pinocchio::Data::Matrix6x J;
  //Matrix6d J;

  // TF:
  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  double arm_max_vel_;
  double arm_max_acc_;
  
  BiQuad bq1;
  BiQuad bq2;
  BiQuadChain bqc1;
  BiQuadChain bqc2;
  BiQuad bq3;
  BiQuad bq4;
  BiQuadChain bqc3;
  BiQuadChain bqc4;  
  BiQuad bq5;
  BiQuad bq6;
  BiQuadChain bqc5;
  BiQuadChain bqc6;  
  BiQuad bq7;
  BiQuad bq8;
  BiQuadChain bqc7;
  BiQuadChain bqc8;  
  BiQuad bq9;
  BiQuad bq10;
  BiQuadChain bqc9;
  BiQuadChain bqc10;  
  BiQuad bq11;
  BiQuad bq12;
  BiQuadChain bqc11;
  BiQuadChain bqc12;
  bool Recieved_Joint_State;

  
  int step_;
  bool Init_Flag_;
  
  Vector6d init_arm_pose_;
  Vector6d Traj_current_state;
  Vector6d desired_arm_pose_;  
  Vector6d desired_arm_twist_; 
  Vector6d desired_arm_accel_;
  
public:
  Admittance(ros::NodeHandle &n, double frequency,
                      std::string topic_joint_state,
                      std::string topic_arm_state,
                      std::string topic_arm_reference_state,
                      std::string topic_arm_command,
                      std::string topic_wrench_state,
                      std::string topic_wrench_reference_state,
                      std::string topic_car_command,
                      std::vector<double> M,
                      std::vector<double> D,
                      std::vector<double> K,
                      std::vector<double> K_f,
                      std::vector<double> desired_pose,
                      std::vector<double> Dcar,
                      std::string base_link,
                      std::string end_link,
                      double arm_max_vel,
                      double arm_max_acc
                       );
  ~Admittance(){}
  void run();
private:

  void compute_admittance(bool flag);

  void state_joint_callback(const sensor_msgs::JointState msg);

  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);

  void state_wrench_callback(const geometry_msgs::WrenchStamped msg);

  void reference_state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);

  void reference_state_wrench_callback(const geometry_msgs::WrenchStamped msg);

  void send_commands_to_robot();

  void send_commands_to_car();

  void wait_for_transformations();

  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);
  void toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw);
  
private:
  std::string   base_link_;
  std::string   end_link_;
};

#endif // ADMITTANCE_H
