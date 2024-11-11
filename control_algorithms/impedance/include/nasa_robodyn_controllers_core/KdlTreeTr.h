#ifndef KDL_TREE_TR_H
#define KDL_TREE_TR_H

#include "nasa_robodyn_controllers_core/KdlTreeIk.h"
#include "kdl/articulatedbodyinertia.hpp"
#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <iomanip>

class KdlTreeTr : public KdlTreeIk
{
public:  
    KdlTreeTr();
    ~KdlTreeTr();

    virtual void getJointPositions(const KDL::JntArray& joints_in, const std::vector<std::string>& nodeNames,
                   const std::vector<KDL::Frame>& nodeFrames, KDL::JntArray& joints_out,
                   const std::vector<NodePriority>& nodePriorities);

    inline void setMBar(double mbar = 1e-7) { this->mbar= mbar; }
    inline void setKr(double Kr = 0.00032) {this->Kr = Kr;} //Kr = 0.000032
    inline void setMaxTwist(double maxTwist=0.1) {this->maxTwist = maxTwist;}
    inline void setMaxJointVel(double maxVel=0.015) {this->jointMaxVel = maxVel;}
    virtual void initialize();
    void setJointInertias(const std::map<std::string, double>& inertia_in);

protected:
    //types
    typedef std::map<std::string, KDL::Frame>::iterator FrameMapItr;
    typedef std::pair<KDL::Twist, NodePriority> TwistPriority;
    typedef std::map<std::string, TwistPriority>::iterator TwistMapItr;
    typedef std::pair<Eigen::MatrixXd, Eigen::VectorXd> Task;
    typedef std::multimap<int, std::pair<std::string, int> >::iterator PriorityTaskAxisItr;

    double mbar;
    double Kr;
    double maxTwist;
    double jointMaxVel;

    bool limitJoints(const KDL::JntArray& joints_in, KDL::JntArray& joints_out, KDL::JntArray &delta_joints);
    bool limitTaskTwist(Eigen::VectorXd &twist);
    bool limitTaskTwist(KDL::Twist &twist);
private:

    int numJoints;
    int numConstraints;
    std::vector<std::string> nodeNames;
    std::vector<NodePriority> nodePriorities;
    std::map<std::string, KDL::Frame> currentFrameMap;
    std::map<std::string, KDL::Frame> desiredFrameMap;
    std::map<std::string, TwistPriority > twistMap;
    std::vector<Task> taskList;
    std::multimap<int, std::pair<std::string, int> > priorityTaskAxisMap;


    void createFrameTwistMaps(const std::vector<KDL::Frame>& nodeFrames, const std::vector<NodePriority>& NodePriorities);
    void updateNodeFrameMap(const std::vector<KDL::Frame>& nodeFrames);
    bool findFrameTwist(const KDL::JntArray& joints_in);
    bool isTaskAchieved(const KDL::JntArray& joints_in, std::stringstream &errMsg);
    void findJointInertia(const KDL::JntArray& joints_in, KDL::RotationalInertia &inertia_out);

    void getTaskReconstruction(const KDL::JntArray& joints_in, KDL::JntArray& joint_deltas);
    void getOrderedTasksByPriority(const KDL::JntArray& joints_in);
    void getOrderedTasksByFrame(const KDL::JntArray& joints_in);
    Task getCenterJointsTask(const KDL::JntArray& joints_in);

    void modifyToAvoidSingularities(const Eigen::VectorXd& nm, const double mom, const Eigen::VectorXd& dr, Eigen::VectorXd& drp );

    inline double sign(double num) {return( num < 0. ? -1. : 1.);}
    double cubic(const double &q, const double &pLeft, const double &pRight, const double &ct1, const double &ct2);
    double boundedCubic(const double &q, const double &pLeft, const double &pRight, const double &ct1, const double &ct2);
    int pInv(const Eigen::MatrixXd& jac_in, Eigen::MatrixXd& ijac_out, double &mom);


    Eigen::VectorXd getBiasVector(const Eigen::VectorXd& joints_in, const Eigen::VectorXd &joint_deltas, const Eigen::MatrixXd &ijac);
    Eigen::MatrixXd jacJntWeights;
    Eigen::MatrixXd ijacJntWeights;

    bool velocityLim;
    bool jointLim;
    bool momLim;
};

#endif // KDL_TREE_TR_H
