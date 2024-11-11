#include <stdexcept>

#include <nasa_robodyn_controllers_core/KdlChainIk.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
//#include "nasa_common_utilities/Logger.h"
/***************************************************************************//**
 *
 * @brief Constructor for KdlChainIk
 * @param _chain
 *
 ******************************************************************************/
KdlChainIk::KdlChainIk(const KDL::Chain& _chain)
    : chain(_chain)
{
    // TODO: get this from somewhere
    std::map<std::string, double> jointMin;
    std::map<std::string, double> jointMax;

    jointMin["/r2/left_leg/joint0"] = -1.5708;
    jointMax["/r2/left_leg/joint0"] = 1.5708;
    jointMin["/r2/left_leg/joint1"] = -1.3875;
    jointMax["/r2/left_leg/joint1"] = 1.1694;
    jointMin["/r2/left_leg/joint2"] = -3.14159;
    jointMax["/r2/left_leg/joint2"] = 3.14159;
    jointMin["/r2/left_leg/joint3"] = -0.0698;
    jointMax["/r2/left_leg/joint3"] = 2.6529;
    jointMin["/r2/left_leg/joint4"] = -3.14159;
    jointMax["/r2/left_leg/joint4"] = 3.14159;
    jointMin["/r2/left_leg/joint5"] = -0.0698;
    jointMax["/r2/left_leg/joint5"] = 2.6529;
    jointMin["/r2/left_leg/joint6"] = -3.14159;
    jointMax["/r2/left_leg/joint6"] = 3.14159;

    jointMin["/r2/right_leg/joint0"] = -1.5708;
    jointMax["/r2/right_leg/joint0"] = 1.5708;
    jointMin["/r2/right_leg/joint1"] = -1.3875;
    jointMax["/r2/right_leg/joint1"] = 1.1694;
    jointMin["/r2/right_leg/joint2"] = -3.14159;
    jointMax["/r2/right_leg/joint2"] = 3.14159;
    jointMin["/r2/right_leg/joint3"] = -0.0698;
    jointMax["/r2/right_leg/joint3"] = 2.6529;
    jointMin["/r2/right_leg/joint4"] = -3.14159;
    jointMax["/r2/right_leg/joint4"] = 3.14159;
    jointMin["/r2/right_leg/joint5"] = -0.0698;
    jointMax["/r2/right_leg/joint5"] = 2.6529;
    jointMin["/r2/right_leg/joint6"] = -3.14159;
    jointMax["/r2/right_leg/joint6"] = 3.14159;

    if (chain.getNrOfJoints() == 0)
    {
        std::stringstream err;
        err << "KdlChainIk given chain with no joints";
        //RCS::Logger::log("gov.nasa.controllers.KdlChainIk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }

    // find joints
    KDL::JntArray jMin(chain.getNrOfJoints()), jMax(chain.getNrOfJoints());
    unsigned int j = 0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
        {
            std::string jointName = chain.getSegment(i).getJoint().getName();
            // limits
            jMin(j) = jointMin[jointName];
            jMax(j) = jointMax[jointName];

            ++j;
        }
    }

    // setup solver
    fkPosSolverPtr.reset(new KDL::ChainFkSolverPos_recursive(chain));
    ikVelSolverPtr.reset(new KDL::ChainIkSolverVel_pinv(chain));
    ikPosSolverPtr.reset(new KDL::ChainIkSolverPos_NR_JL(chain, jMin, jMax, *fkPosSolverPtr, *ikVelSolverPtr));
}

KdlChainIk::~KdlChainIk()
{
}
/***************************************************************************//**
 *
 * @brief Get base Frame
 * @return Result of chain.getSegment(0).getName()
 *
 ******************************************************************************/
const std::string& KdlChainIk::getBaseFrame() const
{
    return (chain.getSegment(0).getName());
}
/***************************************************************************//**
 *
 * @brief Retrieve joint names from Segment
 * @param jointNames Vector string to place joint names into
 *
 ******************************************************************************/
void KdlChainIk::getJointNames(std::vector<std::string>& jointNames) const
{
    jointNames.clear();
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
        {
            std::string jointName = chain.getSegment(i).getJoint().getName();
            jointNames.push_back(jointName);
        }
    }
}
/***************************************************************************//**
 *
 * @brief Retrieve joint positions
 * @param joints_in
 * @param tipPositions
 * @param joints_out
 *
 ******************************************************************************/
void KdlChainIk::getJointPositions(const KDL::JntArray& joints_in,
                                   const std::vector<KDL::Frame>& tipPositions,
                                   std::vector<KDL::JntArray>& joints_out) const
{
    joints_out.clear();
    joints_out.resize(tipPositions.size());

    KDL::JntArray jointPoseIn = joints_in;
    KDL::JntArray jointPoseOut;
    for (unsigned int i = 0; i < tipPositions.size(); ++i)
    {
        int ret = ikPosSolverPtr->CartToJnt(jointPoseIn, tipPositions[i], jointPoseOut);
        if (ret >= 0)
        {
            joints_out[i] = jointPoseOut;
            jointPoseIn = jointPoseOut;
        }
        else
        {
            std::stringstream err;
            err << "Unable to find an IK solution for tipPosition[" << i << "]" << std::endl;
            double x, y, z, w;
            tipPositions[i].M.GetQuaternion(x, y, z, w);
            err << "\t" << tipPositions[i].p.x() << ", " << tipPositions[i].p.y() << ", " <<
                tipPositions[i].p.z() << ", " << x << ", " << y << ", " << z << ", " << w << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlChainIk", log4cpp::Priority::WARN, err.str());
            throw std::runtime_error(err.str());
        }
    }
}

void KdlChainIk::getCurrentPose(const KDL::JntArray& joints_in, KDL::Frame& currPose) const
{
    if (fkPosSolverPtr->JntToCart(joints_in, currPose) < 0)
    {
        std::stringstream err;
        err << "Unable to find an FK solution";
        //RCS::Logger::log("gov.nasa.controllers.KdlChainIk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}
