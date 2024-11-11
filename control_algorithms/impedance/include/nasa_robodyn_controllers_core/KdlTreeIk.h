/**
 * @file KdlTreeIk.h
 * @brief Defines the KdlTreeIk class.
 * @author Ross Taylor
 */
#ifndef KDL_TREE_IK_H
#define KDL_TREE_IK_H

#include "nasa_robodyn_controllers_core/MotionLimiter.h"
#include "nasa_robodyn_controllers_core/KdlTreeUtilities.h"
#include "nasa_robodyn_controllers_core/KdlTreeFk.h"

#include <kdl/treejnttojacsolver.hpp>
#include <kdl/frameacc.hpp>
#include <kdl/jntarrayacc.hpp>

//#include <nasa_r2_common_msgs/PriorityArray.h>

#include <kdl/frames.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <iomanip>

class KdlTreeIk : public KdlTreeUtilities
{
public:
    typedef KDL::Twist NodePriority;
    static const int IGNORE;// = 0;
    static const int LOW;// = 1;
    static const int MEDIUM;// = 128;
    static const int HIGH;// = 254;
    static const int CRITICAL;// = 255;

    KdlTreeIk();
    ~KdlTreeIk();

    inline double getTimeStep() const {return timeStep;}

    /**
     * @brief set time step between trajectory bread crumbs
     * @param timeStep_in time step
     * @return void
     * @exception invalid_argument Time step must be greater than 0
     */
    void setTimeStep(double timeStep_in = 0.01);


    // maximum number of iterations allowed to achieve critical convergience
    void setMaxCriticalIterations(unsigned int maxIter_in = 50)
    {
        critMaxIter = maxIter_in;
    }

    // maximum number of iterations allowed to achieve noncritical convergience
    void setMaxNonCriticalIterations(unsigned int maxIter_in = 5)
    {
        nonCritMaxIter = maxIter_in;
    }

    void setEpsilon(double eps_in=1e-6)
    {
        eps = eps_in;
    }

    inline void setPositionLimiter(boost::shared_ptr<JointNamePositionLimiter> posLimiter) {positionLimiter = posLimiter;}

    void setLambda(double lambda = 0.05)
    {
        lambda_squared = lambda*lambda;
    }

    void setJointInertias(const std::map<std::string, double>& inertia_in);

    void setPriorityTol(const std::map<int, std::pair<double, double> >& priority_tol)
    {
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeIk")<<log4cpp::Priority::INFO<<"updated priority tol map: ";
        for(std::map<int, std::pair<double, double> >::const_iterator itr = priority_tol.begin(); itr != priority_tol.end(); ++itr)
        {
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeIk")<<log4cpp::Priority::INFO<<itr->first<<": ("<<itr->second.first<<","<<itr->second.second<<")";
        }
        priorityTolMap = priority_tol;
    }

    /**
     * @brief getJoints
     * @param joints_in input joints
     * @param nodeNames input frame names
     * @param nodeFrames input frames
     * @param joints_out output joints
     * @param nodePriorities input node priorities (6 * nodeNames.size())
     */
    virtual void getFrames (const KDL::JntArray& joints_in, std::map<std::string, KDL::Frame>& frameMap);
    virtual void getJointPositions(const KDL::JntArray& joints_in, const std::vector<std::string>& nodeNames,
                   const std::vector<KDL::Frame>& nodeFrames, KDL::JntArray& joints_out,
                   const std::vector<NodePriority>& nodePriorities);

    std::auto_ptr<KdlTreeFk> fkPosSolverPtr;

protected:
    virtual void initialize();

    std::vector<std::string> jointNames;
    //std::auto_ptr<KdlTreeFk> fkPosSolverPtr;

    double timeStep;

    unsigned int critMaxIter;
    unsigned int nonCritMaxIter;
    double eps;
    double lambda_squared;

    std::auto_ptr<KDL::TreeJntToJacSolver> jacSolverPtr;

    /** @brief the inertia of each joint in the same position as received from getJointPositions
      */
    KDL::JntArray inertias;

    /** @brief the mapping of joint names to joint inertias
      *         Joint Name : joint inertia
      */
    std::map<std::string, double> inertiaMap;
    double minInertia;
    double maxInertia;

    /** @brief the mapping between priority number and tolerence amount
      *         Priority Num : (Linear Tolerence, Angular Tolerence)
      *                         (m, rad)
      */
    std::map<int, std::pair<double, double> > priorityTolMap;

    boost::shared_ptr<JointNamePositionLimiter> positionLimiter;

private:

    /**
     * @brief getJointVelocities - getJointVelocities.
     */
    void getJointVelocities(const KDL::JntArray& joints_in, const std::vector<std::pair<std::string, KDL::Twist> >& twist_deltas,
                            const std::vector<NodePriority>& nodePriorities, KDL::JntArray& joint_deltas) const;

    bool useDampedLeastSquares;
};

#endif
