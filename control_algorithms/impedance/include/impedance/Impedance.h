#ifndef IMPEDACE_H
#define IMPEDACE_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "KDL_Base.h"
#include <kdl/chainidsolver_recursive_newton_euler.hpp>//Inverse Dynamics
#include <kdl/chaindynparam.hpp>                        //Inverse Dynamics Params

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include <std_msgs/Float64MultiArray.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include "joint_effort_msg/JointEffort.h"
#include "joint_effort_msg/JointEfforts.h"
#include "joint_state_msg/JointState.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float64MultiArray.h"
#include <fstream>
#include <chrono>
#include <ctime>

#include "nasa_robodyn_controllers_core/KdlTreeFk.h"
#include "nasa_robodyn_controllers_core/KdlTreeParser.h"
#include "BiQuad.h"

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;

class Impedance: public kdl_base::KDL_Base
{
public:
    Impedance() {}
    ~Impedance() {}

    void init(ros::NodeHandle &nh,
        std::string topic_arm_state,
        std::string topic_cart_arm_state,
        std::string topic_arm_command,
        std::string topic_wrench_state,
        std::string topic_car_state,
        std::string topic_car_imu,
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
        std::string end_link);

    void run();

private:

    void compute_impedance(bool flag);

private:
    bool get_rotation_matrix(Matrix6d & rotation_matrix,
                             tf::TransformListener & listener,
                             std::string from_frame,  std::string to_frame);
    void toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw);
    
    void state_arm_callback(const joint_state_msg::JointState msg);

    void cart_state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);

    void state_wrench_callback(const geometry_msgs::WrenchStamped msg);

    void reference_state_car_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
    
    void reference_cart_state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
    
    void reference_state_wrench_callback(const geometry_msgs::WrenchStamped msg);

    void state_car_callback(const nav_msgs::OdometryConstPtr msg);
    
    void imu_car_callback(const sensor_msgs::ImuConstPtr msg);  
    
    void imp_param_update_callback(const std_msgs::Float64MultiArray::ConstPtr msg);

    void command(const std_msgs::Float64MultiArray::ConstPtr &msg);

    void send_commands_to_robot();
    void send_commands_to_car();
    
protected:
    ros::NodeHandle                 nh_;
    // Subscribers:
    ros::Subscriber                 sub_command_; // Interface to external commands
    ros::Subscriber                 sub_arm_state_;
    ros::Subscriber                 sub_cart_arm_state_;
    ros::Subscriber                 sub_wrench_state_;
    ros::Subscriber                 sub_car_state_;
    ros::Subscriber                 sub_car_imu_;    
    
    ros::Subscriber                 sub_reference_cart_arm_state_;
    ros::Subscriber                 sub_reference_wrench_state_;
    ros::Subscriber                 sub_reference_car_state_;
    
    ros::Subscriber                 sub_posture_;
    ros::Subscriber                 sub_imp_param_update_;

    // Publishers:
    ros::Publisher                  pub_arm_cmd_;
    ros::Publisher                  pub_car_cmd_;

    ros::Time                       last_publish_time_;
    double                          publish_rate_;

    // KDL Varaibales:
    KDL::JntArray                   Jnt_Pos_Init_State;
    KDL::JntArrayAcc                Jnt_Desired_State;
    KDL::JntArray                   CMD_State;
    KDL::JntArray                   Current_State;
    KDL::JntArray                   Jnt_Toq_Cmd_;
    KDL::JntArray                   Prev_Jnt_Toq_Cmd_;
    
    KDL::JntArray                   Jnt_Toq_Cmd_t1_;    
    KDL::JntArray                   Jnt_Toq_Cmd_t2_;
    KDL::JntArray                   Filt_Jnt_Toq_Cmd_;
    KDL::JntArray                   Filt_Jnt_Toq_Cmd_t1_;
    KDL::JntArray                   Filt_Jnt_Toq_Cmd_t2_;
    
    
    KDL::JntArray                   Jnt_Pos_State;
    KDL::JntArray                   Jnt_Vel_State;
    KDL::JntArray                   Jnt_Toq_State;


    KDL::Wrench                     Ext_Wrench;
    KDL::Wrench                     Ref_Wrench;
    
    KDL::Rotation                   R_car_;
    KDL::Vector                     Desired_Pos_;
    KDL::Frame                      Desired_Pose_;

    KDL::Vector                     Gravity;

    KDL::JntSpaceInertiaMatrix      M_; //Inertia Matrix
    KDL::JntArray                   C_, G_;
    KDL::JntArray                   Kp_, Kv_, Ka_;
    double                          Wf_;
    double                          Car_K_v_, Car_K_w_;
    
    KDL::Jacobian  		     J_;
    KDL::Twist			     J_dot_q_dot_;
    
    bool                            Recieved_Joint_State;
    bool                            Cmd_Flag_;
    bool                            Init_Flag_;
    uint                            Step_;

    //  Kinematics
    boost::shared_ptr<KDL::ChainFkSolverPos>    fk_pos_solver_;
    boost::shared_ptr<KDL::ChainFkSolverVel>    fk_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverVel>    ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    boost::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver_;
    //  Dynamics
    boost::shared_ptr<KDL::ChainIdSolver>       id_pos_solver_;
    boost::shared_ptr<KDL::ChainDynParam>       id_solver_;

    std::vector<double>     Impedance_M, Impedance_D, Impedance_K, Impedance_Kf;
    std::vector<double>     desired_pose_;

    double                  wrench_x;
    double                  wrench_y;
    double                  wrench_z;
    double                  pos_x;
    double                  pos_y;
    double                  pos_z;
    double                  K_e;
    double                  D_e;
    
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
    BiQuad bq7, bq72;
    BiQuad bq8, bq82;
    BiQuadChain bqc7;
    BiQuadChain bqc8;  
    BiQuad bq9, bq92;
    BiQuad bq10, bq102;
    BiQuadChain bqc9;
    BiQuadChain bqc10;  
    BiQuad bq11, bq112;
    BiQuad bq12, bq122;
    BiQuadChain bqc11;
    BiQuadChain bqc12;
    
    Eigen::Matrix<double,6,1> Cart_Pos_State;
    Eigen::Matrix<double,6,1> Cart_Vel_State;    
    
    std::string   base_link_;
    std::string   end_link_;
    
    // Listeners
    tf::TransformListener listener_ft_;
    tf::TransformListener listener_control_;
    tf::TransformListener listener_arm_;
    
    Vector3d      car_position_;
    Quaterniond   car_orientation_;
    Vector6d      car_twist_;
    Vector6d      car_desired_twist_adm_;
    Vector6d      car_acc_;

    Vector6d Cart_ee_Ref_Acc;
    Vector6d Cart_ee_Ref_Twist;
    Vector6d Cart_ee_Ref_Pose;
    Vector6d Car_Ref_Twist;
    Vector6d Car_Ref_Pose;
    
    KdlTreeFk* fk_;
    std::map<std::string, KDL::Frame> frames;
    std::map<std::string, KDL::FrameVel> velframes;
    std::vector<std::string> link_names;
    
    KDL::JntArray joints_in;
    KDL::JntArrayVel joints_in_vel;
    
    uint Step;
    Vector6d RMSE;
    
    bool VIC_state;
    bool VIC_state2;
    double t;
    double init_secs; 
    double secs;

    std::string filename;

};


#endif
