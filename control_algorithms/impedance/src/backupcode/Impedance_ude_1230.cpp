#include "impedance/Impedance.h"
#include "impedance/KDL_Base.h"
#include "kdl_conversions/kdl_msg.h"
#include <math.h>


void Impedance::init(ros::NodeHandle &nh,
        std::string topic_arm_state,
        std::string topic_cart_arm_state,
        std::string topic_arm_command,
        std::string topic_wrench_state,
        std::string topic_car_state,
        std::string topic_car_command,
        std::string topic_reference_cart_arm_state,
        std::string topic_reference_wrench_state,
        std::string topic_reference_car_state,
        std::vector<double> Ka,
        std::vector<double> Kv,
        std::vector<double> Kp,
        std::vector<double> M,
        std::vector<double> D,
        std::vector<double> K,
        std::vector<double> Kf,
        double Wf,
        double Car_K_v,
        double Car_K_w,
        std::vector<double> desired_pose,
        std::string base_link,
        std::string end_link)
{
    //* Subscribers
    sub_arm_state_           = nh_.subscribe(topic_arm_state, 5,
        &Impedance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
    sub_cart_arm_state_           = nh_.subscribe(topic_cart_arm_state, 5,
        &Impedance::cart_state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
    sub_wrench_state_        = nh_.subscribe(topic_wrench_state, 5,
        &Impedance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_car_state_              = nh_.subscribe(topic_car_state, 5, 
      &Impedance::state_car_callback, this,ros::TransportHints().reliable().tcpNoDelay());    

    sub_reference_cart_arm_state_           = nh_.subscribe(topic_reference_cart_arm_state, 5,
        &Impedance::reference_cart_state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
    sub_reference_wrench_state_        = nh_.subscribe(topic_reference_wrench_state, 5,
        &Impedance::reference_state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_reference_car_state_              = nh_.subscribe(topic_reference_car_state, 5, 
      &Impedance::reference_state_car_callback, this,ros::TransportHints().reliable().tcpNoDelay());            
    
    sub_posture_ = nh_.subscribe("/joint_torque_controller/command", 1,
        &Impedance::command,this, ros::TransportHints().reliable().tcpNoDelay());
        
    //* Publishers
    pub_arm_cmd_              = nh_.advertise<joint_effort_msg::JointEfforts>(topic_arm_command, 5);
    pub_car_cmd_              = nh_.advertise<geometry_msgs::Twist>(topic_car_command, 5);    
    
    // KDL
    kdl_base::KDL_Base::init(nh);
    
    // ========================================== feedforward process ==================================================================    
    KDL::Frame baseFrame = KDL::Frame(); 
    //baseFrame.M = baseFrame.M.RPY(0.0, 0.0, 0.0); 
    KDL::FrameVel base_VelFrame = KDL::FrameVel(baseFrame, KDL::Twist( KDL::Vector(0.0,0.0,0.0), KDL::Vector(0.0,0.0,0.0) )); // linear, rotational 

    std::cout << "baseFrame.R:" << std::endl << baseFrame.M.data[0] << "," << baseFrame.M.data[1] << "," << baseFrame.M.data[2] << std::endl << baseFrame.M.data[3] << "," 
    	<< baseFrame.M.data[4] << "," << baseFrame.M.data[5] << std::endl << baseFrame.M.data[6] << "," << baseFrame.M.data[7] << "," << baseFrame.M.data[8] << std::endl 
    	<< "baseFrame.p:" << baseFrame.p.data[0] << "," << baseFrame.p.data[1] << "," << baseFrame.p.data[2] << std::endl; 

    std::cout << "baseVelFrame.R:" << std::endl << base_VelFrame.M.R.data[0] << "," << base_VelFrame.M.R.data[1] << ","   
      	<< base_VelFrame.M.R.data[2] << std::endl << base_VelFrame.M.R.data[3] << "," << base_VelFrame.M.R.data[4] << "," 
	<< base_VelFrame.M.R.data[5] << std::endl << base_VelFrame.M.R.data[6] << "," 
    	<< base_VelFrame.M.R.data[7] << "," << base_VelFrame.M.R.data[8] << std::endl 
    	<< "baseFrame.W:" << base_VelFrame.M.w.data[0] << "," << base_VelFrame.M.w.data[1] << "," << base_VelFrame.M.w.data[2] << std::endl 
    	<< "baseFrame.p:" << base_VelFrame.p.p.data[0] << "," << base_VelFrame.p.p.data[1] << "," << base_VelFrame.p.p.data[2] << std::endl 
    	<< "baseFrame.v:" << base_VelFrame.p.v.data[0] << "," << base_VelFrame.p.v.data[1] << "," << base_VelFrame.p.v.data[2] << std::endl;
    
    //KDL::FrameVel currentFrame = baseFrame * KDL::FrameVel(segment.pose(joints.q(q_nr)),
    //                                  segment.twist(joints.q(q_nr), joints.qdot(q_nr)));
    fk_ = new KdlTreeFk();
    fk_->baseFrame = baseFrame;
    fk_->base_VelFrame = base_VelFrame;
    	
    std::string name_space = nh_.getNamespace();
    std::string robot_description;
    ros::param::search(name_space,"robot_description", robot_description);
    std::cout << "rd: "<< robot_description << std::endl;
    try{
        fk_->loadFromChain(kdl_chain_, "root");
        ROS_INFO_STREAM("Getting robot description file for feedforward!");
    }
    catch (std::runtime_error){
        ROS_ERROR("Could not get robot description file!");
        //exit(0);
    }
    // Constructing map with names of frames we want
    link_names.resize(this->kdl_chain_.getNrOfSegments());
    joints_in.resize(this->kdl_chain_.getNrOfJoints());
    joints_in_vel.resize(this->kdl_chain_.getNrOfJoints());
    for(std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++){
      link_names[i] = kdl_chain_.getSegment(i).getName();
      //std::cout << link_names[i] << std::endl;
    }
    for(size_t i = 0; i < link_names.size(); ++i){
      frames[link_names[i]] = KDL::Frame();
      velframes[link_names[i]] = KDL::FrameVel();
    }
    // init
    joints_in.data << 0.0,0.0,0.0,0.0,0.0,0.0;
    joints_in_vel.q = joints_in;
    joints_in_vel.qdot = joints_in;

    //std::cout << "joints_in:" << joints_in.data.transpose() << std::endl;

        fk_->baseFrame.M.RPY(0.0,0.0,0.0); 
        fk_->base_VelFrame = KDL::FrameVel(fk_->baseFrame, KDL::Twist( KDL::Vector(0.0,0.0,0.0), KDL::Vector(0.0,0.0,0.0)));
        // linear, rotational 
        
        
        try{
          fk_->getPoses(joints_in, frames);
          ROS_INFO_STREAM("Get fk pose calculation succeeded!");
        }
        catch (std::runtime_error e){
          ROS_WARN("Exception caught during FK pos computation: %s", e.what());
        }
    
        try{
          fk_->getVelocities(joints_in_vel, velframes);
          ROS_INFO_STREAM("Get fk vel calculation succeeded!");
        }
        catch (std::runtime_error e){
          ROS_WARN("Exception caught during FK vel computation: %s", e.what());
        }
    
        std::cout << velframes[link_names[link_names.size()-1]].p.p[0] <<
          ","     << velframes[link_names[link_names.size()-1]].p.p[1] <<
          ","     << velframes[link_names[link_names.size()-1]].p.p[2] + 0.4 << std::endl;
        

    // ==================================================================================================================================
    Gravity = KDL::Vector::Zero();
    Gravity(2) = -9.81;
    
    Kp_.resize(this->kdl_chain_.getNrOfJoints());
    Kv_.resize(this->kdl_chain_.getNrOfJoints());
    Ka_.resize(this->kdl_chain_.getNrOfJoints());
    M_.resize(this->kdl_chain_.getNrOfJoints());
    C_.resize(this->kdl_chain_.getNrOfJoints());
    G_.resize(this->kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());
    
    // KDL: Kinematics
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
    // KDL::ChainIkSolverVel_pinv_givens ik_vel_solver_ = KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_);
    // KDL::ChainFkSolverPos_recursive fk_pos_solver_ = KDL::ChainFkSolverPos_recursive(this->kdl_chain_);
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR(this->kdl_chain_, *(fk_pos_solver_.get()), *(ik_vel_solver_.get())));

    // KDL: Dynamics
    id_pos_solver_.reset(new KDL::ChainIdSolver_RNE(this->kdl_chain_, Gravity));
    id_solver_.reset( new KDL::ChainDynParam(this->kdl_chain_, Gravity));
    
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    jnt_to_jac_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));    
    
    // get publishing period
    if (!nh.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
    }

    // Variable init
    Jnt_Pos_Init_State.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Desired_State.resize(this->kdl_chain_.getNrOfJoints());
    CMD_State.resize(this->kdl_chain_.getNrOfJoints());
    Current_State.resize(this->kdl_chain_.getNrOfJoints());

    Jnt_Pos_State.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Vel_State.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Toq_State.resize(this->kdl_chain_.getNrOfJoints());

    Jnt_Toq_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
    Prev_Jnt_Toq_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
    Prev_Jnt_Toq_Cmd_.data << 0.0,0.0,0.0,0.0,0.0,0.0;
    
    // digital filter
    Jnt_Toq_Cmd_t1_.resize(this->kdl_chain_.getNrOfJoints());    
    Jnt_Toq_Cmd_t2_.resize(this->kdl_chain_.getNrOfJoints());
    Filt_Jnt_Toq_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
    Filt_Jnt_Toq_Cmd_t1_.resize(this->kdl_chain_.getNrOfJoints());
    Filt_Jnt_Toq_Cmd_t2_.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Toq_Cmd_t1_.data << 0.0,0.0,0.0,0.0,0.0,0.0;
    Jnt_Toq_Cmd_t2_.data << 0.0,0.0,0.0,0.0,0.0,0.0;
    Filt_Jnt_Toq_Cmd_.data << 0.0,0.0,0.0,0.0,0.0,0.0;
    Filt_Jnt_Toq_Cmd_t1_.data << 0.0,0.0,0.0,0.0,0.0,0.0;
    Filt_Jnt_Toq_Cmd_t2_.data << 0.0,0.0,0.0,0.0,0.0,0.0;
        
    Ext_Wrench = KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
    Ref_Wrench = KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
    //KDL::Wrench wrench = KDL::Wrench(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
    //Ext_Wrenches.back() = wrench;
    
    Cart_ee_Ref_Pose << 0.0,0.0,0.0,0.0,0.0,0.0;
    Cart_ee_Ref_Twist << 0.0,0.0,0.0,0.0,0.0,0.0;
    Cart_ee_Ref_Acc << 0.0,0.0,0.0,0.0,0.0,0.0;
    //desired_pose_ = desired_pose;
    //Desired_Pos_ = KDL::Vector(desired_pose[0], desired_pose[1], desired_pose[2]);
    //Desired_Ori_ = KDL::Rotation::Quaternion(desired_pose[3], desired_pose[4], desired_pose[5],desired_pose[6]);
    //Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);
    
    for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++){
        Kp_(i) = Kp[i];
        Kv_(i) = Kv[i];
        Ka_(i) = Ka[i];
    }

    Impedance_M = M;
    Impedance_D = D;
    Impedance_K = K;
    Impedance_Kf = Kf;

    base_link_ = base_link;
    end_link_ = end_link;
    //std::cout << base_link_ << std::endl;
    
    Recieved_Joint_State = false;
    // Cmd_Flag_            = true;
    // Init_Flag_           = true;
    //Step_                = 0;
    
    // Car control init
    Car_K_v_ = Car_K_v;
    Car_K_w_ = Car_K_w;
    car_desired_twist_adm_.setZero();
    
    // UDE biquad filter init
    Wf_ = Wf;
/*
// ========================================= first order butter UDE ============================================================  
    bq1.set( 2.00000e+00, 0.00000e+00, 0.00000e+00, -1.00000e+00, 0.00000e+00 );
    bqc1.add( &bq1 ); 
    bq2.set( 2.00000e+00, 0.00000e+00, 0.00000e+00, -1.00000e+00, 0.00000e+00 );
    bqc2.add( &bq2 );   
    bq3.set( 2.00000e+00, 0.00000e+00, 0.00000e+00, -1.00000e+00, 0.00000e+00 );
    bqc3.add( &bq3 );   
    bq4.set( 2.00000e+00, 0.00000e+00, 0.00000e+00, -1.00000e+00, 0.00000e+00 );
    bqc4.add( &bq4 );   
    bq5.set( 2.00000e+00, 0.00000e+00, 0.00000e+00, -1.00000e+00, 0.00000e+00 );
    bqc5.add( &bq5 );   
    bq6.set( 2.00000e+00, 0.00000e+00, 0.00000e+00, -1.00000e+00, 0.00000e+00 );
    bqc6.add( &bq6 ); 
   

    bq7.set( 1.00000e+00, 1.00000e+00, 0.00000e+00, -2.00000e+00, 1.00000e+00 );
    bqc7.add( &bq7 ); 
    bq8.set( 1.00000e+00, 1.00000e+00, 0.00000e+00, -2.00000e+00, 1.00000e+00 );
    bqc8.add( &bq8 ); 
    bq9.set( 1.00000e+00, 1.00000e+00, 0.00000e+00, -2.00000e+00, 1.00000e+00 );
    bqc9.add( &bq9 ); 
    bq10.set( 1.00000e+00, 1.00000e+00, 0.00000e+00, -2.00000e+00, 1.00000e+00 );
    bqc10.add( &bq10 ); 
    bq11.set( 1.00000e+00, 1.00000e+00, 0.00000e+00, -2.00000e+00, 1.00000e+00 );
    bqc11.add( &bq11 ); 
    bq12.set( 1.00000e+00, 1.00000e+00, 0.00000e+00, -2.00000e+00, 1.00000e+00 );
    bqc12.add( &bq12 ); 
*/
/*
// ================================================== closeloop UDE ======================================================
//    first order Low pass filter
    bq1.set( 1.00000e+00, Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc1.add( &bq1 ); 
    bq2.set( 1.00000e+00, Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc2.add( &bq2 );   
    bq3.set( 1.00000e+00, Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc3.add( &bq3 );   
    bq4.set( 1.00000e+00, Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc4.add( &bq4 );   
    bq5.set( 1.00000e+00, Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc5.add( &bq5 );   
    bq6.set( 1.00000e+00, Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc6.add( &bq6 ); 
    
    bq7.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc7.add( &bq7 ); 
    bq8.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc8.add( &bq8 ); 
    bq9.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc9.add( &bq9 ); 
    bq10.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc10.add( &bq10 ); 
    bq11.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc11.add( &bq11 ); 
    bq12.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc12.add( &bq12 ); 
    // for s*Gf/(1-Gf) 
    
*/
// ================================================== closeloop UDE ======================================================
//    first order Low pass filter
    bq1.set( 1.00000e+00 + Wf_ , -1 , 0.00000e+00, -1 , 0 );
    bqc1.add( &bq1 ); 
    bq2.set( 1.00000e+00 + Wf_ , -1 , 0.00000e+00, -1 , 0 );
    bqc2.add( &bq2 );   
    bq3.set( 1.00000e+00 + Wf_ , -1 , 0.00000e+00, -1 , 0 );
    bqc3.add( &bq3 );   
    bq4.set( 1.00000e+00 + Wf_ , -1 , 0.00000e+00, -1 , 0 );
    bqc4.add( &bq4 );   
    bq5.set( 1.00000e+00 + Wf_ , -1 , 0.00000e+00, -1 , 0 );
    bqc5.add( &bq5 );   
    bq6.set( 1.00000e+00 + Wf_ , -1 , 0.00000e+00, -1 , 0 );
    bqc6.add( &bq6 ); 
    
    bq7.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc7.add( &bq7 ); 
    bq8.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc8.add( &bq8 ); 
    bq9.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc9.add( &bq9 ); 
    bq10.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc10.add( &bq10 ); 
    bq11.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc11.add( &bq11 ); 
    bq12.set( Wf_, 0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00 );
    bqc12.add( &bq12 ); 
    // for s*Gf/(1-Gf) 
    
/*
//  ========================== wf=50 2 order butter =======================================
//    second order butterworth
    bq1.set( 1.10816e+00, -1.04488e+00, 3.69348e-01, -1.26108e+00, 2.61192e-01 );
    bqc1.add( &bq1 ); 
    bq2.set( 1.10816e+00, -1.04488e+00, 3.69348e-01, -1.26108e+00, 2.61192e-01 );
    bqc2.add( &bq2 );   
    bq3.set( 1.10816e+00, -1.04488e+00, 3.69348e-01, -1.26108e+00, 2.61192e-01 );
    bqc3.add( &bq3 );   
    bq4.set( 1.10816e+00, -1.04488e+00, 3.69348e-01, -1.26108e+00, 2.61192e-01 );
    bqc4.add( &bq4 );   
    bq5.set( 1.10816e+00, -1.04488e+00, 3.69348e-01, -1.26108e+00, 2.61192e-01 );
    bqc5.add( &bq5 );   
    bq6.set( 1.10816e+00, -1.04488e+00, 3.69348e-01, -1.26108e+00, 2.61192e-01 );
    bqc6.add( &bq6 ); 
    bq7.set( 1.08189e-01, 2.16365e-01, 1.08181e-01, -1.26306e+00, 2.61531e-01 );
    bq72.set(1.00000e+00, -9.42826e-01, 3.33325e-01, -9.41064e-01, 3.32873e-01);
    bqc7.add( &bq7 ).add( &bq72 ); 
    bq8.set( 1.08189e-01, 2.16365e-01, 1.08181e-01, -1.26306e+00, 2.61531e-01 );
    bq82.set(1.00000e+00, -9.42826e-01, 3.33325e-01, -9.41064e-01, 3.32873e-01);
    bqc8.add( &bq8 ).add( &bq82 ); 
    bq9.set( 1.08189e-01, 2.16365e-01, 1.08181e-01, -1.26306e+00, 2.61531e-01 );
    bq92.set(1.00000e+00, -9.42826e-01, 3.33325e-01, -9.41064e-01, 3.32873e-01);
    bqc9.add( &bq9 ).add( &bq92 ); 
    bq10.set( 1.08189e-01, 2.16365e-01, 1.08181e-01, -1.26306e+00, 2.61531e-01 );
    bq102.set(1.00000e+00, -9.42826e-01, 3.33325e-01, -9.41064e-01, 3.32873e-01);
    bqc10.add( &bq10 ).add( &bq102 ); 
    bq11.set( 1.08189e-01, 2.16365e-01, 1.08181e-01, -1.26306e+00, 2.61531e-01 );
    bq112.set(1.00000e+00, -9.42826e-01, 3.33325e-01, -9.41064e-01, 3.32873e-01);
    bqc11.add( &bq11 ).add( &bq112 ); 
    bq12.set( 1.08189e-01, 2.16365e-01, 1.08181e-01, -1.26306e+00, 2.61531e-01 );
    bq122.set(1.00000e+00, -9.42826e-01, 3.33325e-01, -9.41064e-01, 3.32873e-01);
    bqc12.add( &bq12 ).add( &bq122 ); 
*/
    ROS_INFO("The BiQuad filter is ready to use.");
    Step = 0;
    RMSE << 0,0,0,0,0,0;
    
    VIC_state = false;
    
    init_secs = ros::Time::now().toSec();

}


// =======================================================================================
// =================================== functions =========================================
// =======================================================================================
bool Impedance::get_rotation_matrix(Matrix6d & rotation_matrix,
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


void Impedance::toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw){
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


// =======================================================================================
// =================================== callbacks =========================================
// =======================================================================================
void Impedance::state_arm_callback(const joint_state_msg::JointState msg)
{
  for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++){
    Jnt_Pos_State(i) = msg.position[i];// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
    Jnt_Vel_State(i) = msg.velocity[i];// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
    Jnt_Toq_State(i) = msg.effort[i];// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
    Recieved_Joint_State = true;
  }
  //std::cout<<Jnt_Pos_State(0)<<","<<Jnt_Pos_State(1)<<","<<Jnt_Pos_State(2)<<","
  //<<Jnt_Pos_State(3)<<","<<Jnt_Pos_State(4)<<","<<Jnt_Pos_State(5)<<std::endl;
  // std::cout<<"Recieved Joint State"<<std::endl;
}


void Impedance::cart_state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  Cart_Pos_State.topRows(3) <<  msg->pose.position.x,// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5),
                    msg->pose.position.y,// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5),
                    msg->pose.position.z;// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
  Quaterniond cart_arm_orientation_;
  cart_arm_orientation_.coeffs() << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
  tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  //double roll, pitch, yaw;
  m.getRPY(Cart_Pos_State(3), Cart_Pos_State(4), Cart_Pos_State(5));
  Cart_Pos_State(3);// += 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
  Cart_Pos_State(4);// += 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
  Cart_Pos_State(5);// += 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
  //std::cout << "  rpy  :" << roll<< ","<< pitch<<"," << yaw <<std::endl;
  //std::cout << "orienta:" << cart_arm_orientation_.coeffs().transpose()<<std::endl;
  //toEulerAngle(cart_arm_orientation_, Cart_Pos_State(3), Cart_Pos_State(4), Cart_Pos_State(5));
  //std::cout << "cartpos:" << Cart_Pos_State.transpose()<<std::endl;
  Cart_Vel_State << msg->twist.linear.x,// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5),
                msg->twist.linear.y,// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5),
                msg->twist.linear.z,// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5),
                msg->twist.angular.x,// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5),
                msg->twist.angular.y,// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5),
                msg->twist.angular.z;// + 0.1 * ((float)std::rand()/RAND_MAX - 0.5);
}


void Impedance::state_wrench_callback(
  const geometry_msgs::WrenchStamped msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (1) {
    wrench_ft_frame <<  msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z;
    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
    wrench_ft_frame <<  rotation_ft_base * wrench_ft_frame;
  }
   //wrench_ft_frame << std::endl;
   Ext_Wrench = KDL::Wrench(KDL::Vector(wrench_ft_frame(0),wrench_ft_frame(1),wrench_ft_frame(2)+1.841),
                            KDL::Vector(wrench_ft_frame(3),wrench_ft_frame(4),wrench_ft_frame(5)));
   //Ext_Wrenches.back() = wrench;
   //wrench_x = msg->force.x;
   //wrench_y = msg->force.y;
   //wrench_z = msg->force.z;
}

    
void Impedance::reference_state_car_callback(const cartesian_state_msgs::PoseTwistConstPtr msg){
  Car_Ref_Pose(0) = msg->pose.position.x;
  Car_Ref_Pose(1) = msg->pose.position.y;
  Car_Ref_Pose(2) = msg->pose.position.z;
  tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(Car_Ref_Pose(3), Car_Ref_Pose(4), Car_Ref_Pose(5));
  Car_Ref_Twist << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
}
    
    
void Impedance::reference_cart_state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg){
  Cart_ee_Ref_Pose(0) = msg->pose.position.x;
  Cart_ee_Ref_Pose(1) = msg->pose.position.y;
  Cart_ee_Ref_Pose(2) = msg->pose.position.z;
  tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(Cart_ee_Ref_Pose(3), Cart_ee_Ref_Pose(4), Cart_ee_Ref_Pose(5));

  Cart_ee_Ref_Twist << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
  Cart_ee_Ref_Acc << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}
    
void Impedance::reference_state_wrench_callback(const geometry_msgs::WrenchStamped msg){
  Ref_Wrench.force(0) = msg.wrench.force.x,
  Ref_Wrench.force(1) = msg.wrench.force	.y;
  Ref_Wrench.force(2) = msg.wrench.force.z;
  Ref_Wrench.torque(0) = msg.wrench.torque.x;
  Ref_Wrench.torque(1) = msg.wrench.torque.y;
  Ref_Wrench.torque(2) = msg.wrench.torque.z;
  //std::cout << Ref_Wrench.force(0) << Ref_Wrench.force(1) <<Ref_Wrench.force(2) << std::endl;
}


void Impedance::state_car_callback(const nav_msgs::OdometryConstPtr msg){
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


// =======================================================================================
// =================================== control ===========================================
// =======================================================================================
void Impedance::compute_impedance(bool flag)
{
    if(flag)
    {
        // ================================================================================
        KDL::JntArrayVel Jnt_Vel;
        Jnt_Vel = KDL::JntArrayVel(Jnt_Pos_State, Jnt_Vel_State);
        /*
        if (wrench_z != 0)
        {
            KDL::Frame End_Pose;
            fk_pos_solver_->JntToCart(Jnt_Pos_State,End_Pose);


            KDL::FrameVel End_Pose_Vel;

            fk_vel_solver_->JntToCart(Jnt_Vel, End_Pose_Vel);

            KDL::Vector pose_p, pose_vel_p;
            pose_p = End_Pose.p;
            pose_vel_p = End_Pose_Vel.p.p;

            // double acc_x = (wrench_x - (Impedance_D[0]*pose_vel_p(0) + Impedance_K[0]*(pose_p(0)-desired_pose_[0])))/Impedance_M[0];
            // double acc_y = (wrench_y - (Impedance_D[1]*pose_vel_p(1) + Impedance_K[1]*(pose_p(1)-desired_pose_[1])))/Impedance_M[1];
            double acc_z = (wrench_z - (Impedance_D[2]*pose_vel_p(2) + Impedance_K[2]*(desired_pose_[2]-pose_p(2))))/Impedance_M[2];

            ros::Rate loop_rate_(200);
            ros::Duration duration = loop_rate_.expectedCycleTime();
            // pos_x = pos_x + 0.01*(pose_vel_p(0) * duration.toSec() + 0.5 * acc_x * duration.toSec() * duration.toSec());
            // pos_y = pos_y + 0.01*(pose_vel_p(1) * duration.toSec() + 0.5 * acc_y * duration.toSec() * duration.toSec());
            pos_z = 10*(pose_vel_p(2) * duration.toSec() + 0.5 * acc_z * duration.toSec() * duration.toSec());
            Desired_Pos_ = KDL::Vector(desired_pose_[0]+pos_x, desired_pose_[1]+pos_y, desired_pose_[2]+pos_z);
            Desired_Ori_ = KDL::Rotation::Quaternion(desired_pose_[3], desired_pose_[4], desired_pose_[5],desired_pose_[6]);
            Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);
        }
        ik_pos_solver_->CartToJnt(Jnt_Pos_State, Desired_Pose_, CMD_State);
        */ /*
        // reaching desired joint position using a hyperbolic tangent function
        double lambda = 0.1;
        double th = tanh(M_PI - lambda*Step_);
        double ch = cosh(M_PI - lambda*Step_);
        double sh2 = 1.0/(ch*ch); */


        // -mpi/2 0 0 
        //  ======================= irrelevant  =============================
        /*
        for(size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
        {
            //take into account also initial/final velocity and acceleration
            Current_State(i) = CMD_State(i) - Jnt_Pos_Init_State(i); //desired jointffx
            Jnt_Desired_State.q(i) = Current_State(i)*0.5*(1.0-th) + Jnt_Pos_Init_State(i);
            Jnt_Desired_State.qdot(i) = Current_State(i)*0.5*lambda*sh2;
            Jnt_Desired_State.qdotdot(i) = Current_State(i)*lambda*lambda*sh2*th;

            Cart_Current_State = cart_desired_pos - Cart_Pos_Init_State;
            Cart_Desired_Pose(i) = Cart_Current_State(i)*0.5*(1.0-th) + Cart_Pos_Init_State(i);
            Cart_Desired_Twist(i) = Cart_Current_State(i)*0.5*lambda*sh2;
            Cart_Desired_Acc(i) = Cart_Current_State(i)*lambda*lambda*sh2*th; 
        }
        // std::cout<<Current_State(0)<<","<<Current_State(1)<<","<<Current_State(2)<<","
        // <<Current_State(3)<<","<<Current_State(4)<<","<<Current_State(5)<<std::endl;
        ++Step_;
        if(Jnt_Desired_State.q == CMD_State)
        {
            Cmd_Flag_ = false;	//reset command flag
            Step_ = 0;
            Jnt_Pos_Init_State = Jnt_Pos_State;
            // ROS_INFO("Posture OK");
        }*/
        
        //  ================================================================
        jnt_to_jac_solver_->JntToJac(Jnt_Pos_State, J_);
    	// computing Inertia, Coriolis and Gravity matrices
        id_solver_->JntToMass(Jnt_Pos_State, M_);
        id_solver_->JntToGravity(Jnt_Pos_State, G_);

        KDL::JntArray Jnt_Desired_Vel_State;
        Jnt_Desired_Vel_State.data = J_.data.inverse() * Cart_ee_Ref_Twist;
        id_solver_->JntToCoriolis(Jnt_Pos_State, Jnt_Desired_Vel_State, C_); // C \dot{x}_d
        
        KDL::JntArray pid_cmd_(this->kdl_chain_.getNrOfJoints());
        // compensation of Coriolis and Gravity
        KDL::JntArray cg_cmd_(this->kdl_chain_.getNrOfJoints());

        jnt_to_jac_dot_solver_->JntToJacDot(Jnt_Vel, J_dot_q_dot_);
        //  std::cout << " J_dot_q_dot_: "<< J_dot_q_dot_.vel.data[0] << std::endl; 

        //Eigen::Matrix<double,6,1> cart_desired_pos;
        //cart_desired_pos << 0.2, 0.4, 0.7, -M_PI/2, 0.0, 0.0;
        for(size_t i=0; i<this->kdl_chain_.getNrOfJoints(); i++){
            // =================================== PID controller ==============================
            //pid_cmd_(i) = Ka_(i)*Jnt_Desired_State.qdotdot(i) + Kv_(i)*(Jnt_Desired_State.qdot(i) - Jnt_Vel_State(i)) + Kp_(i)*(Jnt_Desired_State.q(i) - Jnt_Pos_State(i));

            // =================================== step2: UDE controller without Kf ==============================
            //pid_cmd_(i) = 1/Impedance_M[i] * (Impedance_M[i] * Jnt_Desired_State.qdotdot(i)
            //                                + Impedance_D[i] * (Jnt_Desired_State.qdot(i) - Jnt_Vel_State(i))
            //                                + Impedance_K[i] * (Jnt_Desired_State.q(i) - Jnt_Pos_State(i)));
            // =================================== step5: Cart UDE controller without kf ==============================
            if(i<=2){
            // =================================== step 3: impedance position controller ==============================
            //pid_cmd_(i) = 1/Impedance_M[i] * (Impedance_D[i] * (- Cart_Vel_State(i)) - Impedance_K[i] * (Cart_Pos_State(i) - Cart_ee_Ref_Pose(i)) ) - J_dot_q_dot_.vel.data[i];
            // =================================== step 4: Impedance controller without UDE ==============================
            pid_cmd_(i) =                    ( //Impedance_M[i] *  Cart_ee_Ref_Acc(i)
                                             + Impedance_D[i] * (Cart_ee_Ref_Twist(i) - Cart_Vel_State(i))
                                             + Impedance_K[i] * (Cart_ee_Ref_Pose(i) - Cart_Pos_State(i)) 
                                             - Impedance_Kf[i] * (Ext_Wrench.force(i) - Ref_Wrench.force(i)) );
                                             //- J_dot_q_dot_.vel.data[i];
            cg_cmd_(i) = G_(i) + C_(i) - Ext_Wrench.force(i);
            }
            else{
            // =================================== step 3: impedance position controller ==============================
            //double e = Cart_Pos_State(i) - Cart_ee_Ref_Pose(i);
            //while(e <= -M_PI){e+=2*M_PI;std::cout<< e << ":-1";}
            //while(e > M_PI){e-=2*M_PI;std::cout<< e << ":+1";}
            //pid_cmd_(i) = 1/Impedance_M[i] * (Impedance_D[i] * (- Cart_Vel_State(i)) - Impedance_K[i] * e ) - J_dot_q_dot_.rot.data[i-3];
            // =================================== step 4: Impedance controller without UDE ==============================
            pid_cmd_(i) =                    ( //Impedance_M[i] *  Cart_ee_Ref_Acc(i)
                                             + Impedance_D[i] * (Cart_ee_Ref_Twist(i) - Cart_Vel_State(i))
                                             + Impedance_K[i] * (Cart_ee_Ref_Pose(i) - Cart_Pos_State(i)) 
                                             - Impedance_Kf[i] * (Ext_Wrench.torque(i-3) - Ref_Wrench.torque(i-3)) );
                                             //- J_dot_q_dot_.rot.data[i-3];
            // C vector and G vector force compensation.  
            cg_cmd_(i) = G_(i) + C_(i) - Ext_Wrench.torque(i-3); 
            }             
        }

        //std::cout << "J_dot_Q_dot:" << J_dot_q_dot_.vel << J_dot_q_dot_.rot << std::endl;
        std::cout << "cart_ee_Ref_pos:" << (Cart_ee_Ref_Pose.transpose()) << std::endl;
        std::cout << "Cart_ee_Pos:" << (Cart_Pos_State.transpose()) << std::endl;
        //std::cout << "cart error:" << -(cart_desired_pos.transpose() - Cart_Pos_State.transpose()) << std::endl;
        std::cout << "cart_ee_error:" << -(Cart_ee_Ref_Pose.transpose() - Cart_Pos_State.transpose()) << std::endl;
        std::cout << "cart_vel_ee_error:" << -(Cart_ee_Ref_Twist.transpose() - Cart_Vel_State.transpose()) << std::endl;

        std::cout << "Ext_Wrench: " << Ext_Wrench.force(0) << "," << Ext_Wrench.force(1) << "," << Ext_Wrench.force(2) << "," 
                                  << Ext_Wrench.torque(0) << "," << Ext_Wrench.torque(1) << "," << Ext_Wrench.torque(2) << std::endl;
        
        std::cout << "Ref_Wrench: " << Ref_Wrench.force(0) << "," << Ref_Wrench.force(1) << "," << Ref_Wrench.force(2) << "," 
                                  << Ref_Wrench.torque(0) << "," << Ref_Wrench.torque(1) << "," << Ref_Wrench.torque(2) << std::endl;
        
        
        //std::cout << " J_: "<< J_.data << std::endl;
        //std::cout << " J_q: "<< J_.data * Jnt_Vel_State.data  << std::endl;
        //std::cout << " Vee: "<< Cart_Vel_State << std::endl;
        //std::cout << " Jq - Vee error: "<< J_.data * Jnt_Vel_State.data - Cart_Vel_State << std::endl;
        //std::cout << " q - J^-1 Vee error: "<<  Jnt_Vel_State.data.transpose() - Cart_Vel_State.transpose() * J_.data.inverse().transpose() << std::endl;
        //std::cout << " J_^-1: "<< J_.data.inverse() << std::endl;                   
        //std::cout << "Joint torque before force cal: "<< pid_cmd_.data.transpose() << std::endl;
        // step 6: cart PID controller with kf
        //std::cout << "Joint torque before M*J: "<< pid_cmd_.data.transpose() << std::endl;
        //Jnt_Toq_Cmd_.data = M_.data * J_.data.inverse() * pid_cmd_.data; // step5
        Jnt_Toq_Cmd_.data = J_.data.transpose() * pid_cmd_.data; // step5
        
        Jnt_Toq_Cmd_.data += M_.data * J_.data.transpose() * Cart_ee_Ref_Acc;        
        //std::cout << "pid output:" << std::endl << pid_cmd_.data.transpose() << std::endl;
        //std::cout << "J^-1 M_ * J_.inverse(): " << std::endl<< J_.data.inverse().transpose() * M_.data * J_.data.inverse() << std::endl;
        //std::cout << "M_.data * J_.data.inverse(): " << std::endl<< M_.data * J_.data.inverse() << std::endl;
        //std::cout << "output:" << std::endl << M_.data * J_.data.inverse() * pid_cmd_.data << std::endl;
        //std::cout << "J_.data.transpose(): " << std::endl << J_.data.transpose() << std::endl;
        //std::cout << "output:" << std::endl << J_.data.transpose() * pid_cmd_.data << std::endl;

        // step 5: =============================== UDE based controller ===============================================
        Jnt_Toq_Cmd_.data(0) = bqc1.step(Jnt_Toq_Cmd_.data(0)); // 1/(1 - Gf(s)) filtering
        Jnt_Toq_Cmd_.data(1) = bqc2.step(Jnt_Toq_Cmd_.data(1)); 
        Jnt_Toq_Cmd_.data(2) = bqc3.step(Jnt_Toq_Cmd_.data(2));
        Jnt_Toq_Cmd_.data(3) = bqc4.step(Jnt_Toq_Cmd_.data(3));
        Jnt_Toq_Cmd_.data(4) = bqc5.step(Jnt_Toq_Cmd_.data(4));
        Jnt_Toq_Cmd_.data(5) = bqc6.step(Jnt_Toq_Cmd_.data(5));
        
        KDL::JntArray ude_cmd_(this->kdl_chain_.getNrOfJoints());
        ude_cmd_.data = - M_.data * Jnt_Vel_State.data;   // s Gf(s) / (1 - Gf(s)) filtering Jnt_Vel_State

        ude_cmd_.data(0) = bqc7.step(ude_cmd_.data(0));
        ude_cmd_.data(1) = bqc8.step(ude_cmd_.data(1));
        ude_cmd_.data(2) = bqc9.step(ude_cmd_.data(2));
        ude_cmd_.data(3) = bqc10.step(ude_cmd_.data(3));
        ude_cmd_.data(4) = bqc11.step(ude_cmd_.data(4));
        ude_cmd_.data(5) = bqc12.step(ude_cmd_.data(5));
        //std::cout << " Jnt_Pos_State: "<< Jnt_Pos_State.data << std::endl;
                ude_cmd_.data += Wf_ * M_.data * J_.data.inverse() * (Cart_ee_Ref_Twist); // 1/(1 - Gf(s)) filtering // step 5
        KDL::Add(Jnt_Toq_Cmd_,ude_cmd_,Jnt_Toq_Cmd_);
        // ============================================================================================================        

        
        // step 7: =============================== UDE butterworth controller ===============================================
        /*
        KDL::JntArray ude_cmd_(this->kdl_chain_.getNrOfJoints());
        Filt_Jnt_Toq_Cmd_.data = 0.98947218 * Filt_Jnt_Toq_Cmd_t1_.data - 0.35029345 * Filt_Jnt_Toq_Cmd_t2_.data 
                               + 0.09020532 * Jnt_Toq_Cmd_.data + 0.18041063 * Jnt_Toq_Cmd_t1_.data + 0.09020532 * Jnt_Toq_Cmd_t2_.data;
        
        Filt_Jnt_Toq_Cmd_t2_.data = Filt_Jnt_Toq_Cmd_t1_.data;
        Filt_Jnt_Toq_Cmd_t1_.data = Filt_Jnt_Toq_Cmd_.data;
        Jnt_Toq_Cmd_t2_.data = Jnt_Toq_Cmd_t1_.data;
        Jnt_Toq_Cmd_t1_.data = Jnt_Toq_Cmd_.data;
        
        ude_cmd_.data = M_.data * Jnt_Vel_State.data;   // 1/T * (1 - 1/(Ts+1) )	  
        
        std::cout << "Joint torque: "<< Jnt_Toq_Cmd_.data.transpose() << std::endl;
        std::cout << "prev Joint torque: "<< Prev_Jnt_Toq_Cmd_.data.transpose() << std::endl;
        std::cout << "UDE joint torque: "<< ude_cmd_.data.transpose() << std::endl;
        Jnt_Toq_Cmd_.data = Jnt_Toq_Cmd_.data + Prev_Jnt_Toq_Cmd_.data - ude_cmd_.data; // - M_.data * J_.data.inverse() * xddot;
        */
        
        // step 6: =============================== UDE bitquad controller ===============================================
        /*
        KDL::JntArray ude_cmd_(this->kdl_chain_.getNrOfJoints());
        
        Prev_Jnt_Toq_Cmd_.data(0) = bqc1.step(Prev_Jnt_Toq_Cmd_.data(0)); 
        Prev_Jnt_Toq_Cmd_.data(1) = bqc2.step(Prev_Jnt_Toq_Cmd_.data(1)); 
        Prev_Jnt_Toq_Cmd_.data(2) = bqc3.step(Prev_Jnt_Toq_Cmd_.data(2));
        Prev_Jnt_Toq_Cmd_.data(3) = bqc4.step(Prev_Jnt_Toq_Cmd_.data(3));
        Prev_Jnt_Toq_Cmd_.data(4) = bqc5.step(Prev_Jnt_Toq_Cmd_.data(4));
        Prev_Jnt_Toq_Cmd_.data(5) = bqc6.step(Prev_Jnt_Toq_Cmd_.data(5));
        
        ude_cmd_.data = M_.data * Jnt_Vel_State.data;   // 1/T * (1 - 1/(Ts+1) )	  
        ude_cmd_.data(0) = 25 * (ude_cmd_.data(0) - bqc7.step(ude_cmd_.data(0)));
        ude_cmd_.data(1) = 25 * (ude_cmd_.data(1) - bqc8.step(ude_cmd_.data(1)));
        ude_cmd_.data(2) = 25 * (ude_cmd_.data(2) - bqc9.step(ude_cmd_.data(2)));
        ude_cmd_.data(3) = 25 * (ude_cmd_.data(3) - bqc10.step(ude_cmd_.data(3)));
        ude_cmd_.data(4) = 25 * (ude_cmd_.data(4) - bqc11.step(ude_cmd_.data(4)));
        ude_cmd_.data(5) = 25 * (ude_cmd_.data(5) - bqc12.step(ude_cmd_.data(5)));
        std::cout << "Joint torque: "<< Jnt_Toq_Cmd_.data.transpose() << std::endl;
        std::cout << "prev Joint torque: "<< Prev_Jnt_Toq_Cmd_.data.transpose() << std::endl;
        std::cout << "UDE joint torque: "<< ude_cmd_.data.transpose() << std::endl;
        Jnt_Toq_Cmd_.data = Jnt_Toq_Cmd_.data + Prev_Jnt_Toq_Cmd_.data - ude_cmd_.data; // - M_.data * J_.data.inverse() * xddot;
        
        Prev_Jnt_Toq_Cmd_.data = Jnt_Toq_Cmd_.data;
        */
        // =================================================c and g =====================================================
        KDL::Add(Jnt_Toq_Cmd_,cg_cmd_,Jnt_Toq_Cmd_);
        //std::cout << "Joint torque: "<< Jnt_Toq_Cmd_.data.transpose() << std::endl;
        //std::cout << " CG torque: "<< cg_cmd_.data.transpose() << std::endl;
        // ============================================== friction terms ===================================================
        /*
        for(size_t i=0; i<this->kdl_chain_.getNrOfJoints(); i++){        
            Jnt_Toq_Cmd_.data(i) += 2 + 0.2 * Jnt_Vel_State.data(i);
        }
        */
        //0.2 * Jnt_Vel_State.data + 2
        
        


        // ========================================== feedforward ====================================================
        joints_in = Jnt_Pos_State;
        joints_in_vel.q = Jnt_Pos_State;
        joints_in_vel.qdot = Jnt_Vel_State;
        
        Vector3d car_angle = car_orientation_.matrix().eulerAngles(0,1,2);
        toEulerAngle(car_orientation_, car_angle(0), car_angle(1), car_angle(2));
        
        //KDL::Frame baseFrame = KDL::Frame(); 
        //baseFrame.M = baseFrame.M.RPY(0.0, 0.0, 0.0);
        fk_->baseFrame.M.RPY(car_angle(0), car_angle(1), car_angle(2)); 
        fk_->base_VelFrame = KDL::FrameVel(fk_->baseFrame, KDL::Twist( KDL::Vector(car_twist_(0), car_twist_(1), car_twist_(2)), KDL::Vector(car_twist_(3), car_twist_(4), car_twist_(5))));
        // linear, rotational 
        
        try{
          fk_->getPoses(joints_in, frames);
          //ROS_INFO_STREAM("Get fk pose calculation succeeded!");
        }
        catch (std::runtime_error e){
          ROS_WARN("Exception caught during FK pos computation: %s", e.what());
        }
    
        try{
          fk_->getVelocities(joints_in_vel, velframes);
          //ROS_INFO_STREAM("Get fk vel calculation succeeded!");
        }
        catch (std::runtime_error e){
          ROS_WARN("Exception caught during FK vel computation: %s", e.what());
        }
        // =============================== mobile car control =======================================
        double e1, e2, e3;
        double vr, wr;
        vr = 0.0; wr = 0.0;
        e1 = car_position_(0) - Car_Ref_Pose(0);
        e2 = car_position_(1) - Car_Ref_Pose(1);
        
        //std::cout << "car_pose: " << car_position_.transpose() << ", "<< car_angle.transpose() << std::endl;  
        //std::cout << "car_ref_pose: " << Car_Ref_Pose.transpose() << std::endl;  
        //std::cout << "car_twist: " << car_twist_.transpose() << std::endl;  
        e3 = car_angle(2) - Car_Ref_Pose(5);
        
        car_desired_twist_adm_(0) = - Car_K_v_ * e1 + vr * cos(e3); // e1
        //std::cout << "e3: " << e3 << std::endl;
        car_desired_twist_adm_(5) = - vr * sin(e3)/e3 * e2 - Car_K_w_ * e3 + wr; // e1  
        //std::cout << "v_and_w: " << car_desired_twist_adm_(0) << "," << car_desired_twist_adm_(5) << std::endl;
        
        // ========================================== RMSE Calculation ===============================================
        double e;
        for(size_t i=0; i<this->kdl_chain_.getNrOfJoints(); i++){
          e = Cart_ee_Ref_Pose(i) - Cart_Pos_State(i);
          RMSE(i) = (RMSE(i) * Step + e*e)/(Step+1);
        }
        
        Step++;
        std::cout << "RMSE: " << RMSE.transpose() << std::endl;
        
        send_commands_to_robot();
        
        send_commands_to_car();
        
        Recieved_Joint_State = false;
        
        
        if(Cart_Pos_State(0) > 5.0){
            VIC_state = true;    
        }
        
        if(VIC_state){
            if( Impedance_Kf[1] < 1.0  ){
                Impedance_Kf[1] += 0.05;
            }
            if( Impedance_K[1] >= 51.0 ){
                Impedance_K[1] -= (500.0-50.0)/20.0;
            }
        }
        
        std::cout << "Impedance_Kf: " << Impedance_Kf[0] << ", " << Impedance_Kf[1] << ", " << Impedance_Kf[2] << ", " 
                                      << Impedance_Kf[3] << ", " << Impedance_Kf[4] << ", " << Impedance_Kf[5] << ", " << std::endl;  
        std::cout << "Impedance_K : " << Impedance_K[0] << ", " << Impedance_K[1] << ", " << Impedance_K[2] << ", " 
                                      << Impedance_K[3] << ", " << Impedance_K[4] << ", " << Impedance_K[5] << ", " << std::endl;  
        std::cout << "Impedance_D : " << Impedance_D[0] << ", " << Impedance_D[1] << ", " << Impedance_D[2] << ", " 
                                      << Impedance_D[3] << ", " << Impedance_D[4] << ", " << Impedance_D[5] << ", " << std::endl;  
        
        secs = ros::Time::now().toSec();
        t = secs - init_secs;
        if (t > 5){
            init_secs = secs;
            double c = 0.5;
            for(int i = 0; i<3 ; i++){
                Impedance_D[i] += c;
            }
        }
    }
}


void Impedance::command(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if(msg->data.size() == 0)
        ROS_INFO("Desired configuration must be of dimension %lu", this->kdl_chain_.getNrOfJoints());
    else if(msg->data.size() != this->kdl_chain_.getNrOfJoints())
    {
        ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size());
        return;
    }
    else
    {
        for (unsigned int i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
            CMD_State(i) = msg->data[i];
        std::cout<<"COmmand SET"<<std::endl;
        // Cmd_Flag_ = true;
        // when a new command is set, steps should be reset to avoid jumps in the update
        Step_ = 0;
    }
}


void Impedance::send_commands_to_robot() {
    joint_effort_msg::JointEfforts msg;
    Vector6d limit;
    limit << 150,150,150,28,28,28;
    for(int i=0;i<6;i++){
      if(Jnt_Toq_Cmd_(i) > limit(i)) Jnt_Toq_Cmd_(i) = limit(i);
      if(Jnt_Toq_Cmd_(i) < -limit(i)) Jnt_Toq_Cmd_(i) = -limit(i); 
    }
    msg.Joint1Effort = Jnt_Toq_Cmd_(0);
    msg.Joint2Effort = Jnt_Toq_Cmd_(1);
    msg.Joint3Effort = Jnt_Toq_Cmd_(2);
    msg.Joint4Effort = Jnt_Toq_Cmd_(3);
    msg.Joint5Effort = Jnt_Toq_Cmd_(4);
    msg.Joint6Effort = Jnt_Toq_Cmd_(5);
    // std::cout<<Jnt_Toq_Cmd_(0)<<","<<Jnt_Toq_Cmd_(1)<<","<<Jnt_Toq_Cmd_(2)<<","
    // <<Jnt_Toq_Cmd_(3)<<","<<Jnt_Toq_Cmd_(4)<<","<<Jnt_Toq_Cmd_(5)<<std::endl;
    pub_arm_cmd_.publish(msg);
}


void Impedance::send_commands_to_car() {
  geometry_msgs::Twist car_twist_cmd;
  car_twist_cmd.linear.x  = car_desired_twist_adm_(0);
  car_twist_cmd.linear.y  = 0.0;
  car_twist_cmd.linear.z  = 0.0;
  car_twist_cmd.angular.x = 0.0;
  car_twist_cmd.angular.y = 0.0;
  car_twist_cmd.angular.z = car_desired_twist_adm_(5);
  pub_car_cmd_.publish(car_twist_cmd);
}


void Impedance::run()
{
    ros::Duration(0.1).sleep();
    ROS_INFO("Running the impedance control loop .................");
    ros::Rate loop_rate_(200);
    while (nh_.ok()) {

        compute_impedance(Recieved_Joint_State);

        ros::spinOnce();

        loop_rate_.sleep();
    }
}
