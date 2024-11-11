#include <hybridadmittance/torque_HybridAdmittance.h>

Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
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
    double arm_max_acc) :
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),K_f_(K_f.data()),desired_pose_(desired_pose.data()),
  Dcar_(Dcar.data()), arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
  base_link_(base_link), end_link_(end_link){

  //* Subscribers
  sub_joint_state_              = nh_.subscribe(topic_joint_state, 5, 
      &Admittance::state_joint_callback, this,ros::TransportHints().reliable().tcpNoDelay());

  std::cout<<"topic_arm_state: "<<topic_arm_state<<std::endl;
  sub_arm_state_              = nh_.subscribe(topic_arm_state, 5, 
      &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_state_           = nh_.subscribe(topic_wrench_state, 5,
      &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  sub_reference_arm_state_    = nh_.subscribe(topic_arm_reference_state, 5, 
      &Admittance::reference_state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_reference_wrench_state_ = nh_.subscribe(topic_wrench_reference_state, 5, 
      &Admittance::reference_state_wrench_callback, this,ros::TransportHints().reliable().tcpNoDelay());

  //* Publishers
  pub_arm_cmd_              = nh_.advertise<joint_effort_msg::JointEfforts>(topic_arm_command, 5);
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
  F.setZero();
  F2.setZero();
  tau.setZero();
  q_dot.setZero();
  q.setZero();  
  // Init integrator
  arm_desired_twist_adm_.setZero();
  car_desired_twist_adm_.setZero();
  Recieved_Joint_State = false;
  Traj_current_state.setZero();
  while (nh_.ok() && !arm_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }
  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;
  
  //Fs = 100;
  //fc = 25;
  //Wn = (1/Fs) * fc;
  //[b,a] = butter(1,Wn);
  //Gf = tf(b,a)
  //
  //%1/(1-Gf)
  //a1 = [1 -1]/sqrt(2);
  //b1 = [1 -0.4142];
  //tf(b1,a1)
  //sos = tf2sos(b1,a1);
  //tf2cppbq( sos );
  //
  //%Gf/(1-Gf)
  //b2 = [0.2929 0.1716 -0.1213 0];
  //a2 = [0.7071 -1 0.2929];
  //tf(b2,a2)
  //sos2 = tf2sos(b2,a2);
  //tf2cppbq( sos2 );
  // for 1/(1-Gf)
  
  //bq1.set( 1.41421e+00, -5.85767e-01, 0.00000e+00, -1.00000e+00, 0.00000e+00 );
  bq1.set( 1.00000e+00, 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc1.add( &bq1 ); 
  bq2.set( 1.00000e+00, 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc2.add( &bq2 );   
  bq3.set( 1.00000e+00, 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc3.add( &bq3 );   
  bq4.set( 1.00000e+00, 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc4.add( &bq4 );   
  bq5.set( 1.00000e+00, 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc5.add( &bq5 );   
  bq6.set( 1.00000e+00, 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc6.add( &bq6 ); 
  // for s*Gf/(1-Gf)
  bq7.set( 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  //bq7.set( 4.14227e-01, 2.42681e-01, -1.71546e-01, -1.41423e+00, 4.14227e-01 );
  bqc7.add( &bq7 ); 
  bq8.set( 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 ); // 1/T
  bqc8.add( &bq8 );
  bq9.set( 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc9.add( &bq9 );
  bq10.set( 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc10.add( &bq10 );
  bq11.set( 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc11.add( &bq11 );
  bq12.set( 2.50000e+01, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
  bqc12.add( &bq12 );
  ROS_INFO("The BiQuad filter is ready to use.");
  //for(int i = 0; i< 10; i++) std::cout << bqc1.step(1) << ",";
  
  
  step_ = 0;
  Init_Flag_ = true;
  
  
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
    compute_admittance(Recieved_Joint_State);
    send_commands_to_robot();
    send_commands_to_car();
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

//!-                Admittance Dynamics                  -!//
void Admittance::compute_admittance(bool flag) {
  if(!flag) return;
  
  if (Init_Flag_){
    init_arm_pose_.topRows(3) = arm_position_;
    toEulerAngle(arm_orientation_,init_arm_pose_(3),init_arm_pose_(4),init_arm_pose_(5));
    Init_Flag_ = false;
    init_arm_pose_ = q;
  } // desired trajectory use
  
  // =============================== update jacobian, J_dot, etc ================================================
  using namespace pinocchio;
  Model model;
  //pinocchio::buildModels::manipulator(model);
  pinocchio::urdf::buildModel(urdf_filename,model);
  Data data(model);
  //std::cout << "nv:" << model.nv << std::endl;
  pinocchio::Data::Matrix6x J(6,model.nv);
  J.setZero();
  computeJointJacobian(model,data,q,JOINT_ID,J);
  //std::cout << "J:" << std::endl << J << std::endl; 
  
  computeJointJacobians(model,data,q);  
  computeJointJacobiansTimeVariation(model,data,q,q_dot);  
  
  Matrix6d M;
  M = crba(model, data, q);
  Matrix6d J_inv = J.inverse(); // full rank
  Matrix6d Jt_M_0 = M * J_inv;  // Jt * (J_inv^T M J_inv) = M * J_inv
  //std::cout << "M:" << std::endl << M << std::endl;
  //std::cout << "Jt_M_0:" << std::endl << Jt_M_0 << std::endl; 
  
  Matrix6d J_dot;
  getJointJacobianTimeVariation(model,data,JOINT_ID,LOCAL_WORLD_ALIGNED,J_dot);
  
  //std::cout << "J_dot:" << std::endl << J_dot << std::endl;
  
  std::cout << "q:" << q.transpose() << std::endl;
  std::cout << "q_dot:" << q_dot.transpose() << std::endl; 
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);  
  //std::cout << "a:" << a.transpose() << std::endl; 
  forwardKinematics(model,data,q,q_dot,a);

  // ===================================================================================================
  Eigen::VectorXd q_d = pinocchio::neutral(model);
  q_d << 1.0,1.0,1.0,1.0,1.0,1.0;	 
  // ================================================ generates desired trajectory ================================================
  // reaching desired joint position using a hyperbolic tangent function
  double lambda = 0.01;
  double th = std::tanh(M_PI - lambda * step_); // tanh from 0 - 1
  double ch = std::cosh(M_PI - lambda * step_);
  double sh2 = 1.0/(ch*ch);
  
  /*
  Traj_current_state.topRows(3) = reference_arm_position_ - init_arm_pose_.topRows(3);
  Vector3d reference_arm_euler_;
  toEulerAngle(reference_arm_orientation_, reference_arm_euler_(0),reference_arm_euler_(1),reference_arm_euler_(2));
  Traj_current_state.bottomRows(3) << reference_arm_euler_ - init_arm_pose_.bottomRows(3);
  */
  Traj_current_state = q_d - init_arm_pose_;
  /*
  Traj_current_state.bottomRows(3) =  err_arm_des_orient0.axis() * err_arm_des_orient0.angle();
  if(init_arm_orientation_.coeffs().dot(reference_arm_orientation_.coeffs()) < 0.0)
  {
    reference_arm_orientation_.coeffs() << -reference_arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err0 (reference_arm_orientation_ * init_arm_orientation_.inverse());
  if(quat_rot_err0.coeffs().norm() > 1e-3)
  {
    quat_rot_err0.coeffs() << quat_rot_err0.coeffs()/quat_rot_err0.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient0(quat_rot_err0);
  Traj_current_state.bottomRows(3) << err_arm_des_orient0.axis() * err_arm_des_orient0.angle();
  */
  for (int i = 0; i < 6; i++){
    //take into account also initial/final velocity and acceleration
    desired_arm_pose_[i] = Traj_current_state(i)*0.5*(1.0-th) + init_arm_pose_(i);
    desired_arm_twist_[i] = Traj_current_state(i)*0.5*lambda*sh2;
    desired_arm_accel_[i] = Traj_current_state(i)*lambda*lambda*sh2*th;
  } // can be in cart or in jnt space

  std::cout << "desired_arm_pose_:" << desired_arm_pose_.transpose() << std::endl; 
  std::cout << "desired_arm_twist_:" << desired_arm_twist_.transpose() << std::endl; 
  std::cout << "desired_arm_accel_:" << desired_arm_accel_.transpose() << std::endl; 
  step_++;
  		
  // ==================================== joint based control law ==================================================================
  tau = M_* desired_arm_accel_ - D_ * (q_dot - desired_arm_twist_)- K_ * (q - desired_arm_pose_);
  
  // ================================================ cartesian error calculation ================================================  
  error.topRows(3) = arm_position_ - desired_arm_pose_.topRows(3);
  Vector3d arm_euler_;
  toEulerAngle(arm_orientation_, arm_euler_(0), arm_euler_(1), arm_euler_(2));
  error.bottomRows(3) = arm_euler_ - desired_arm_pose_.bottomRows(3);
  for (int i = 4 ; i < 6 ; i++){ 
  while(error(i) < -M_PI){error(i)+=2*M_PI;} while(error(i) > M_PI){error(i)-=2*M_PI;}}
  error_vel = arm_twist_ - desired_arm_twist_;
  // ================================= cartesian error calculation without desired trajectory generation =========================
  /*
  // error  
  error.topRows(3) = arm_position_ - reference_arm_position_;
  if(reference_arm_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
  {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err (arm_orientation_ * reference_arm_orientation_.inverse());
  if(quat_rot_err.coeffs().norm() > 1e-3)
  {
    quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();
  //error_vel
  error_vel = arm_twist_ - reference_arm_twist_;
  //error_acc
  error_acc.setZero();
  */
  // ================================= (end-effector) force error calculation =====================================================
  error_f = wrench_external_ - reference_wrench_;
  std::cout << "error: " << error.transpose()  << std::endl;
  std::cout << "error_vel: " << error_vel.transpose()  << std::endl;
  //std::cout << "error_f: " << error_f.transpose()  << std::endl;
  
  // ===================================== UDE-based control law ==================================================================
  // J_T is multipled in advance to simplify calculation, which becomes torque of joints
  F = Jt_M_0 * M_.inverse() * (M_ * desired_arm_accel_ - K_*error - D_ * error_vel);// - K_f_ * error_f); 
  //std::cout << "F1 before filtered:" << F.transpose() << std::endl;	  
  F(0) = bqc1.step(F(0)); // 1/(1 - Gf(s)) filtering
  F(1) = bqc2.step(F(1)); 
  F(2) = bqc3.step(F(2));
  F(3) = bqc4.step(F(3));
  F(4) = bqc5.step(F(4));
  F(5) = bqc6.step(F(5));
  //std::cout << "A" << bqc1.biquads[0]->A << "B"<< bqc1.biquads[0]->B <<"wz" << bqc1.biquads[0]->wz << std::endl;
  std::cout << "F1 after filtered: " << F.transpose() << std::endl;	  
  F2 = - Jt_M_0 * arm_twist_;
  //std::cout << "F2 before filtered:" << F2.transpose() << std::endl;	  
  F2(0) = F2(0) * 25; // bqc7.step(F2(0)); // filtering as well
  F2(1) = F2(1) * 25; //bqc8.step(F2(1)); //Gf/(1 - Gf(s)) 
  F2(2) = F2(2) * 25; //bqc9.step(F2(2));
  F2(3) = F2(3) * 25; //bqc10.step(F2(3));
  F2(4) = F2(4) * 25; //bqc11.step(F2(4));
  F2(5) = F2(5) * 25; //bqc12.step(F2(5));
  std::cout << "F2 after filtered:" << F2.transpose() << std::endl;	  
  //std::cout <<"Jt_M_0"<< std::endl << Jt_M_0 << std::endl;

  std::cout << "qd = " << q_d.transpose() << std::endl; 
  // ==================================== calculate C and G matrix =================================================================
  const Eigen::VectorXd & tau_CG = pinocchio::rnea(model,data,q,q_dot,a);// - Jt_M_0 * J_dot * q_dot;
  //Vector6d taux = - Jt_M_0 * J_dot * q_dot; // bingjiluantouyi
  //std::cout << "taux = " << taux.transpose() << std::endl;   
  std::cout << "tau_CG = " << tau_CG.transpose() << std::endl; 
  // inverse dynamics to generate desired Cx_dot + G torque
  //std::cout << "F1:" << F.transpose() << std::endl;	
  //std::cout << "F2:" << F2.transpose() << std::endl;	
  //tau = tau + tau_CG;
  //tau = F + F2;
  //tau = F + tau_CG;
  //Vector6d tau_neg = - J.transpose() *(K_*error + D_ * error_vel); // bingjiluantouyi
  Recieved_Joint_State = false;

}


//!-                     CALLBACKS                       -!//
void Admittance::state_joint_callback(const sensor_msgs::JointState msg){
    // gets Jacobian matrix from joint states
    q(0) = msg.position[10]; while(q(0) < -M_PI){q(0)+=2*M_PI;} while(q(0) > M_PI){q(0)-=2*M_PI;}
    q(1) = msg.position[11]; while(q(1) < -M_PI){q(1)+=2*M_PI;} while(q(1) > M_PI){q(1)-=2*M_PI;}   
    q(2) = msg.position[12]; while(q(2) < -M_PI){q(2)+=2*M_PI;} while(q(2) > M_PI){q(2)-=2*M_PI;}   
    q(3) = msg.position[13]; while(q(3) < -M_PI){q(3)+=2*M_PI;} while(q(3) > M_PI){q(3)-=2*M_PI;}   
    q(4) = msg.position[14]; while(q(4) < -M_PI){q(4)+=2*M_PI;} while(q(4) > M_PI){q(4)-=2*M_PI;}   
    q(5) = msg.position[15]; while(q(5) < -M_PI){q(5)+=2*M_PI;} while(q(5) > M_PI){q(5)-=2*M_PI;}
    
    q_dot(0) = msg.velocity[10];
    q_dot(1) = msg.velocity[11];    
    q_dot(2) = msg.velocity[12];    
    q_dot(3) = msg.velocity[13];    
    q_dot(4) = msg.velocity[14];    
    q_dot(5) = msg.velocity[15];
    Recieved_Joint_State = true;
    //std::cout << "q: " << q.transpose() << std::endl;

    //J(0,0) = - 0.425*sin(th2 + th3)*cos(th5 + th6)*cos(th4) - 0.1993*sin(th5 + th6)*sin(th4) - 0.425*sin(th5 + th6)*cos(th2 + th3) 
    //                  - 0.3922*sin(th5 + th6)*cos(th3) - 0.3922*sin(th3)*cos(th5 + th6)*cos(th4) + 0.1333*cos(th5 + th6)*cos(th4);
    //J(0,1) =  -0.1993*sin(th5 + th6)*sin(th4) - 0.3922*sin(th3)*cos(th5 + th6)*cos(th4) + 0.1333*cos(th5 + th6)*cos(th4);
    //J(0,2) =  -0.1993*sin(th5 + th6)*sin(th4) + 0.1333*cos(th5 + th6)*cos(th4);
    //J(0,3) = -0.1993*cos(th5 + th6);
    //J(0,4) = 0;
    //J(0,5) = 0;
    //J(1,0) = 0.425*sin(th2 + th3)*sin(th5 + th6)*cos(th4) + 0.3922*sin(th5 + th6)*sin(th3)*cos(th4) - 0.1333*sin(th5 + th6)*cos(th4) + 0.1993*sin(th4)*sin(th5)*sin(th6) 
    //                  - 0.1993*sin(th4)*cos(th5)*cos(th6) - 0.425*cos(th2 + th3)*cos(th5 + th6) - 0.3922*cos(th5 + th6)*cos(th3);
    //J(1,1) = 0.3922*sin(th5 + th6)*sin(th3)*cos(th4) - 0.1333*sin(th5 + th6)*cos(th4) - 0.1993*sin(th4)*cos(th5 + th6) - 0.3922*cos(th5 + th6)*cos(th3);
    //J(1,2) = -0.1333*sin(th5 + th6)*cos(th4) - 0.1993*sin(th4)*cos(th5 + th6);
    //J(1,3) = 0.1993*sin(th5 + th6);
    //J(1,4) = 0;
    //J(1,5) = 0; 
    //J(2,0) = (0.425*sin(th2 + th3) + 0.3922*sin(th3) - 0.1333)*sin(th4);
    //J(2,1) = (0.3922*sin(th3) - 0.1333)*sin(th4);
    //J(2,2) = -0.1333*sin(th4);
    //J(2,3) = 0;
    //J(2,4) = 0; 
    //J(2,5) = 0;
    //J(3,0) = 1.0*sin(th4)*cos(th5 + th6);
    //J(3,1) = 1.0*sin(th4)*cos(th5 + th6);
    //J(3,2) = 1.0*sin(th4)*cos(th5 + th6); 
    //J(3,3) = -1.0*sin(th5 + th6);
    //J(3,4) = 0; 
    //J(3,5) = 0;
    //J(4,0) = -1.0*sin(th5 + th6)*sin(th4);
    //J(4,1) = -1.0*sin(th5 + th6)*sin(th4); 
    //J(4,2) = -1.0*sin(th5 + th6)*sin(th4);
    //J(4,3) = -1.0*cos(th5 + th6);
    //J(4,4) = 0; 
    //J(4,5) = 0;
    //J(5,0) = 1.0*cos(th4);
    //J(5,1) = 1.0*cos(th4);
    //J(5,2) = 1.0*cos(th4);
    //J(5,3) = 0;
    //J(5,4) = 1; 
    //J(5,5) = 1;    
    //std::cout << "my jacobian algo" << J << std::endl;
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
    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
    wrench_external_ <<  rotation_ft_base * wrench_ft_frame;
  }
}

void Admittance::reference_state_wrench_callback(
  const geometry_msgs::WrenchStamped msg) {
    reference_wrench_ <<  msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z;
}

//!-               COMMANDING THE ROBOT                  -!//

void Admittance::send_commands_to_robot() {
    joint_effort_msg::JointEfforts arm_joint_cmd;
    //Vector6d limit;
    //limit << 150,150,150,28,28,28;
    //for(int i=0;i<6;i++){
    //  if(tau(i) > limit(i)) tau(i) = limit(i);
    //  if(tau(i) < -limit(i)) tau(i) = -limit(i); 
    //}
    arm_joint_cmd.Joint1Effort = tau(0);
    arm_joint_cmd.Joint2Effort = tau(1);
    arm_joint_cmd.Joint3Effort = tau(2);
    arm_joint_cmd.Joint4Effort = tau(3);
    arm_joint_cmd.Joint5Effort = tau(4);
    arm_joint_cmd.Joint6Effort = tau(5);
    pub_arm_cmd_.publish(arm_joint_cmd);
}

void Admittance::send_commands_to_car() {
  geometry_msgs::Twist car_twist_cmd;

  car_twist_cmd.linear.x  = car_desired_twist_adm_(0);
  car_twist_cmd.linear.y  = 0.0;
  car_twist_cmd.linear.z  = 0.0;
  car_twist_cmd.angular.x = 0.0;
  car_twist_cmd.angular.y = 0.0;
  car_twist_cmd.angular.z = car_desired_twist_adm_(1)+car_desired_twist_adm_(5);
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


