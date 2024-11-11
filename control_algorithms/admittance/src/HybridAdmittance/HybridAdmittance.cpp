#include <hybridadmittance/HybridAdmittance.h>

Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
    std::string topic_car_state,
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
    double arm_max_acc) :
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),K_f_(K_f.data()),desired_pose_(desired_pose.data()),
  Dcar_(Dcar.data()), arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
  base_link_(base_link), end_link_(end_link){

  //* Subscribers
  std::cout<<"topic_arm_state: "<<topic_arm_state<<std::endl;
  sub_car_state_              = nh_.subscribe(topic_car_state, 5, 
      &Admittance::state_car_callback, this,ros::TransportHints().reliable().tcpNoDelay());
      
  sub_arm_state_              = nh_.subscribe(topic_arm_state, 5, 
      &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_state_           = nh_.subscribe(topic_wrench_state, 5,
      &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  sub_reference_arm_state_    = nh_.subscribe(topic_arm_reference_state, 5, 
      &Admittance::reference_state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_reference_wrench_state_ = nh_.subscribe(topic_wrench_reference_state, 5, 
      &Admittance::reference_state_wrench_callback, this,ros::TransportHints().reliable().tcpNoDelay());

  //* Publishers
  pub_arm_cmd_              = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);
  pub_car_cmd_              = nh_.advertise<geometry_msgs::Twist>(topic_car_command, 5);

  // initializing the class variables
  arm_position_.setZero();
  arm_twist_.setZero();
  reference_arm_position_.setZero();
  reference_arm_twist_.setZero();
  wrench_external_.setZero();
  reference_wrench_.setZero();
  desired_pose_position_ << desired_pose_.topRows(3);
  desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();
  // =========================== transform to MM_base frame =========================
  Vector3d p_inertial2mmbase(0,0,0.35);
  desired_pose_position_ << desired_pose_position_ - p_inertial2mmbase;
  //std::cout << desired_pose_orientation_.matrix().eulerAngles(0,1,2).transpose();
  // Init integrator
  arm_desired_twist_adm_.setZero();
  car_desired_twist_adm_.setZero();

  while (nh_.ok() && !arm_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }

  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  wait_for_transformations();
}

//!-                   INITIALIZATION                    -!//

void Admittance::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  // Makes sure all TFs exists before enabling all transformations in the callbacks
  // while (!get_rotation_matrix(rot_matrix, listener, "world", base_link_)) {sleep(1);}
  base_world_ready_ = true;
  // while (!get_rotation_matrix(rot_matrix, listener, base_link_, "world")) {sleep(1);}
  world_arm_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}

//!-                    CONTROL LOOP                     -!//

void Admittance::run() {

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {

    compute_admittance();

    send_commands_to_robot();

    send_commands_to_car();

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

//!-                Admittance Dynamics                  -!//
void Admittance::compute_admittance() {

  error.topRows(3) = arm_position_ - desired_pose_position_;
  if(desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
  {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err (arm_orientation_ * desired_pose_orientation_.inverse());
  if(quat_rot_err.coeffs().norm() > 1e-3)
  {
    quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();

  // Translation error w.r.t. desired equilibrium
  Vector6d coupling_wrench_arm;
  Vector6d coupling_wrench_car;
  // ========================= car control =======================================
  //coupling_wrench_car = Dcar_ * (car_desired_twist_adm_);
  //car_desired_accelaration = M_.inverse() * ( - coupling_wrench_car + wrench_external_);
  double e1, e2, e3, k1, k2;
  k1 = 5.0;
  k2 = 0.3;
  double vr, wr;
  vr = 0.0; wr = 0.0;
  e1 = car_position_(0) - 0.0;
  e2 = car_position_(1) - 0.0;
  Vector3d car_angle = car_orientation_.matrix().eulerAngles(0,1,2);
  toEulerAngle(car_orientation_, car_angle(0), car_angle(1), car_angle(2));
  std::cout << "car_angle: " << car_angle.transpose() << std::endl;  
  e3 = car_angle(2) - 0.0;
  car_desired_twist_adm_(0) = - k1 * e1 + vr * cos(e3); // e1
  //std::cout << "e3: " << e3 << std::endl;
  car_desired_twist_adm_(5) = - vr * sin(e3)/e3 * e2 - k2 * e3 + wr; // e1  
  std::cout << "v and w: " << car_desired_twist_adm_(0) << "," << car_desired_twist_adm_(5) << std::endl;
  // https://zhuanlan.zhihu.com/p/524463640 traj tracking control
  // ===================== manipulator control ===================================
  
  error_vel = arm_twist_ - reference_arm_twist_ + arm_desired_twist_adm_;
  error_f   = reference_wrench_ - wrench_external_;
  coupling_wrench_arm = D_ * error_vel + K_ * error + K_f_ * error_f;
  arm_desired_accelaration = - M_.inverse() * coupling_wrench_arm;
  
  //coupling_wrench_arm =  D_ * (arm_desired_twist_adm_)  + K_*error;
  //arm_desired_accelaration = M_.inverse() * ( - coupling_wrench_arm + wrench_external_);
  // ====================== limit maximum sspeed ==========================
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  double b_acc_norm = (car_desired_accelaration.segment(0, 3)).norm();
  if (b_acc_norm > 10) {
    car_desired_accelaration.segment(0, 3) *= (1.5 / b_acc_norm);
  }

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_adm_ += arm_desired_accelaration *  duration.toSec();
  //car_desired_twist_adm_ += car_desired_accelaration *  duration.toSec();
}



//!-                     CALLBACKS                       -!//
//!-                     CALLBACKS                       -!//
//!-                     CALLBACKS                       -!//
void Admittance::state_car_callback(const nav_msgs::OdometryConstPtr msg){
  //std::cout << msg->pose.pose << msg->twist.twist<<std::endl;
  car_position_ <<  msg->pose.pose.position.x,
                  msg->pose.pose.position.y,
                  msg->pose.pose.position.z;
  car_orientation_.coeffs() <<  msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z,
                                msg->pose.pose.orientation.w;

  car_twist_ << msg->twist.twist.linear.x,
                msg->twist.twist.linear.y,
                msg->twist.twist.linear.z,
                msg->twist.twist.angular.x,
                msg->twist.twist.angular.y,
                msg->twist.twist.angular.z;
}

void Admittance::reference_state_car_callback(const nav_msgs::OdometryConstPtr msg){
  std::cout << msg->pose << msg->twist<<std::endl;
}


void Admittance::state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  arm_position_ <<  msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z;

  arm_orientation_.coeffs() <<  msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z,
                                msg->pose.orientation.w;

  arm_twist_ << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
}


void Admittance::reference_state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  reference_arm_position_ <<  msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z;

  reference_arm_orientation_.coeffs() <<  msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z,
                                msg->pose.orientation.w;

  reference_arm_twist_ << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
}

void Admittance::state_wrench_callback(
  const geometry_msgs::WrenchStamped msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {
    // wrench_ft_frame <<  0.0,0.0,msg->wrench.force.z,0,0,0;
    wrench_ft_frame <<  msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z;
    //std::cout << " wrench_external_ before rot"<< wrench_ft_frame.transpose() << std::endl;
    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
    wrench_external_ <<  rotation_ft_base * wrench_ft_frame;
    wrench_external_[2] += 1.841; // compesate for the gravity force;
    std::cout << " ee_wrench in inertial base"<< wrench_external_.transpose() << std::endl;
  }
}

void Admittance::reference_state_wrench_callback(
  const geometry_msgs::WrenchStamped msg) {
  reference_wrench_ << msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z;
}




//!-               COMMANDING THE ROBOT                  -!//
void Admittance::send_commands_to_robot() {
  geometry_msgs::Twist arm_twist_cmd;

  arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_adm_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_adm_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_adm_(5);
  pub_arm_cmd_.publish(arm_twist_cmd);
}

void Admittance::send_commands_to_car() {
  geometry_msgs::Twist car_twist_cmd;

  car_twist_cmd.linear.x  = car_desired_twist_adm_(0);
  car_twist_cmd.linear.y  = 0.0;
  car_twist_cmd.linear.z  = 0.0;
  car_twist_cmd.angular.x = 0.0;
  car_twist_cmd.angular.y = 0.0;
  car_twist_cmd.angular.z = car_desired_twist_adm_(5);
  pub_car_cmd_.publish(car_twist_cmd);
}


//!-                    UTILIZATION                      -!//
void Admittance::toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
double sinr_cosp = + 2.0 * (q.w() * q.x() + q.y() * q.z());
double cosr_cosp = + 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
if (fabs(sinp) >= 1)
pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
else
pitch = asin(sinp);

// yaw (z-axis rotation)
double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
yaw = atan2(siny_cosp, cosy_cosp);
}


bool Admittance::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                            ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }
  return true;
}
