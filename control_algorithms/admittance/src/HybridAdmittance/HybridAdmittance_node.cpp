#include "ros/ros.h"
#include "hybridadmittance/HybridAdmittance.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_node");

    ros::NodeHandle nh;
    double frequency = 100.0;

    // Parameters
    std::string topic_car_state;
    std::string topic_car_reference_state;
    std::string topic_arm_state;
    std::string topic_arm_command;
    std::string topic_arm_reference_state;
    std::string topic_wrench_state;
    std::string topic_wrench_reference_state;
    std::string topic_car_command;

    std::string base_link;
    std::string end_link;

    std::vector<double> M;
    std::vector<double> D;
    std::vector<double> K;
    std::vector<double> K_f;
    std::vector<double> desired_pose;
    std::vector<double> Dcar;
    double arm_max_vel;
    double arm_max_acc;


    // LOADING PARAMETERS FROM THE ROS SERVER 

    // Topic names
    if (!nh.getParam("topic_car_reference_state", topic_car_reference_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the car reference."); return -1; }
    if (!nh.getParam("topic_car_state", topic_car_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the car."); return -1; }
    if (!nh.getParam("topic_arm_state", topic_arm_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_arm_reference_state", topic_arm_reference_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_arm_command", topic_arm_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm."); return -1; }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state)) { ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor."); return -1; }
    if (!nh.getParam("topic_wrench_reference_state", topic_wrench_reference_state)) { ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor."); return -1; }
    if (!nh.getParam("topic_car_command", topic_car_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm."); return -1; }
    // ADMITTANCE PARAMETERS
    if (!nh.getParam("mass_arm", M)) { ROS_ERROR("Couldn't retrieve the desired mass of the arm."); return -1; }
    if (!nh.getParam("damping_arm", D)) { ROS_ERROR("Couldn't retrieve the desired damping of the coupling."); return -1; }
    if (!nh.getParam("stiffness_coupling", K)) { ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling."); return -1; }
    if (!nh.getParam("wrench_stiffness_coupling", K_f)) { ROS_ERROR("Couldn't retrieve the desired stiffness of the wrench."); return -1; }
    if (!nh.getParam("damping_car", Dcar)) { ROS_ERROR("Couldn't retrieve the desired damping of the coupling."); return -1; }

    if (!nh.getParam("base_link", base_link)) { ROS_ERROR("Couldn't retrieve the base_link."); return -1; }
    if (!nh.getParam("end_link", end_link)) { ROS_ERROR("Couldn't retrieve the end_link."); return -1; } 
    if (!nh.getParam("desired_pose", desired_pose)) { ROS_ERROR("Couldn't retrieve the desired pose of the spring."); return -1; }
    if (!nh.getParam("arm_max_vel", arm_max_vel)) { ROS_ERROR("Couldn't retrieve the max velocity for the arm."); return -1;}
    if (!nh.getParam("arm_max_acc", arm_max_acc)) { ROS_ERROR("Couldn't retrieve the max acceleration for the arm."); return -1;}
    // Constructing the controller
    Admittance admittance(
        nh,
        frequency,
        topic_car_state,
        topic_arm_state,
        topic_arm_reference_state,
        topic_arm_command,
        topic_wrench_state,
        topic_wrench_reference_state,
        topic_car_command,
        M, D, K, K_f,
        desired_pose,
        Dcar,
        base_link,
        end_link,
        arm_max_vel,
        arm_max_acc);

    // Running the controller
    admittance.run();

    return 0;
}
