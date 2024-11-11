
#include "nasa_robodyn_controllers_core/KdlTreeTr.h"

using namespace Eigen;
using namespace KDL;
using namespace std;

KdlTreeTr::KdlTreeTr()
    :KdlTreeIk()
{
    setMBar();
    setKr();
    setMaxTwist();
    setMaxJointVel();
//    mbar           = 1e-6;
//    Kr             = 0.000032;
//    critMaxIter    = 25;
//    numJoints      = 0;
//    numConstraints = 0;
//    maxTwist       = 0.1;
//    jointMaxVel    = 0.015;
//    //jointMaxVel    = 0.009;
//    //jointMaxVel    = 10;
}

KdlTreeTr::~KdlTreeTr()
{

}

void KdlTreeTr::initialize()
{
    KdlTreeIk::initialize();
    nodeNames.clear();
    currentFrameMap.clear();
    twistMap.clear();
    desiredFrameMap.clear();
    taskList.clear();
}

void KdlTreeTr::setJointInertias(const std::map<std::string, double>& inertia_in)
{
    KdlTreeIk::setJointInertias(inertia_in);
    double x;
//    if(maxInertia != 0)
//    {
//        inertias.data = inertias.data*1./maxInertia;
//    }
    jacJntWeights = inertias.data.asDiagonal();
//    ! @todo this is slow and shouldn't be done on the set, it should be done before needed
    pInv(jacJntWeights, ijacJntWeights, x);
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.setJointInertias")<<log4cpp::Priority::DEBUG<<"Min Inertia: "<<minInertia<<", Max Inertia: "<<maxInertia;

}

/** @brief creates empty maps for frames and twist
  */
void KdlTreeTr::createFrameTwistMaps(const vector<Frame>& nodeFrames, const vector<NodePriority>& nodePriorities)
{
    desiredFrameMap.clear();
    currentFrameMap.clear();
    twistMap.clear();
    taskList.clear();
    priorityTaskAxisMap.clear();

    unsigned int i;
    for (i = 0; i < nodeNames.size(); ++i)
    {
        if (tree.getSegment(nodeNames[i]) == tree.getSegments().end())
        {
            std::stringstream err;
            err << "KdlTreeIk node name (" << nodeNames[i] << ") not found in tree" << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
            return;
        }
        desiredFrameMap.insert(std::make_pair(nodeNames[i], nodeFrames[i]));
        currentFrameMap.insert(std::make_pair(nodeNames[i], KDL::Frame()));
        twistMap.insert(std::make_pair(nodeNames[i], std::make_pair(KDL::Twist(), nodePriorities[i])));
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.createFrameTwistMaps")<<log4cpp::Priority::INFO<<"node "<<nodeNames[i] << " added to maps";
        for(int p = 0; p < 6; ++p)
        {
            priorityTaskAxisMap.insert(std::make_pair(nodePriorities[i][p], std::make_pair(nodeNames[i], p) ) );
        }
    }

    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.createFrameTwistMaps")<<log4cpp::Priority::INFO<<"maps resized to "<<nodeNames.size();
}

/** @brief updates the desired frames
  */
void KdlTreeTr::updateNodeFrameMap(const vector<Frame>& nodeFrames)
{
    for (unsigned int i = 0; i < nodeNames.size(); ++i)
    {
        if (tree.getSegment(nodeNames[i]) == tree.getSegments().end())
        {
            std::stringstream err;
            err << "in updateFrameMaps() node name (" << nodeNames[i] << ") not found in tree" << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }
        desiredFrameMap[nodeNames[i]] = nodeFrames[i];
        // RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.updateNodeFrameMap")<<log4cpp::Priority::DEBUG<<"updated "<<nodeNames[i]<<" to : "
        //         <<"("<<desiredFrameMap[nodeNames[i]].p.x()
        //         <<","<<desiredFrameMap[nodeNames[i]].p.y()
        //         <<","<<desiredFrameMap[nodeNames[i]].p.z()
        //         <<","<<desiredFrameMap[nodeNames[i]].M.GetRot().x()
        //         <<","<<desiredFrameMap[nodeNames[i]].M.GetRot().y()
        //         <<","<<desiredFrameMap[nodeNames[i]].M.GetRot().z()<<")";
    }
}

/**
  * @brief fills frame and twist maps
  * @param jointIn    the input joint positions
  * @returns true if solution has converged, false otherwise;
  */
bool KdlTreeTr::findFrameTwist(const JntArray& jointIn)
{
    //frameMap.clear();
    fkPosSolverPtr->getPoses(jointIn, currentFrameMap);

    bool converged = true;
    //bool achieved = true;

    for (FrameMapItr nodeFrameItr = desiredFrameMap.begin(); nodeFrameItr != desiredFrameMap.end(); ++nodeFrameItr)
    {
        //! find corresponding current frame for desired frame
        FrameMapItr frameItr = currentFrameMap.find(nodeFrameItr->first);
        if (frameItr == currentFrameMap.end())
        {
            std::stringstream err;
            err << "node name (" << frameItr->first << ") not found in frame map" << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.findFrameTwist", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }
        //! find the difference between the desired frame and the current frame
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG << "Current pose:"
        //                                                                      << frameItr->second.p.x() << ", " <<frameItr->second.p.y() << ", " << frameItr->second.p.z() << ", "
        //                                                                      << frameItr->second.M.GetRot().x() << ", " << frameItr->second.M.GetRot().y() << ", " << frameItr->second.M.GetRot().z();

        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG << "Desired pose:"
        //                                                                      << nodeFrameItr->second.p.x() << ", " <<nodeFrameItr->second.p.y() << ", " << nodeFrameItr->second.p.z() << ", "
        //                                                                      << nodeFrameItr->second.M.GetRot().x() << ", " << nodeFrameItr->second.M.GetRot().y() << ", " << nodeFrameItr->second.M.GetRot().z();

        Twist twist = diff(frameItr->second, nodeFrameItr->second);
        limitTaskTwist(twist);

        //! save in twistMap
        TwistMapItr twistItr = twistMap.find(nodeFrameItr->first);
        if (twistItr == twistMap.end())
        {
            std::stringstream err;
            err << "node name (" << nodeFrameItr->first << ") not found in twist map" << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.findFrameTwist", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }

        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<"Twist for "<<nodeFrameItr->first<<":";
//        for(int i =0; i<3; ++i)
//        {
//            if(isnan(twist.vel[i]))
//            {
//                throw std::runtime_error("nan in twist vel found");
//            }
//            if(priorityTolMap.find((unsigned int)twistItr->second.second.vel[i])!= priorityTolMap.end())
//            {
//                RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<"tolerence: "<<priorityTolMap.at(twistItr->second.second.vel[i]).first;
//                if(fabs(twistItr->second.first.vel[i]-twist.vel[i]) > eps)
//                {
//                    //converged = false;  UNUSED VARIABLE
//                }
//                if(fabs(twist.vel[i]) > priorityTolMap.at(twistItr->second.second.vel[i]).first)
//                {
//                    achieved = false;
//                }
//            }
//            RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<"new twist: "<<twist.vel[i]<<", "<<fabs(twistItr->second.first.vel[i]-twist.vel[i]);
//            RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<"priority: "<<twist.vel[i]<<", "<<twistItr->second.second.vel[i];
//        }
//        for(int i =0; i<3; ++i)
//        {
//            if(isnan(twist.rot[i]))
//            {
//                throw std::runtime_error("nan in twist ang found");
//            }
//            if(priorityTolMap.find((unsigned int)twistItr->second.second.rot[i]) != priorityTolMap.end())
//            {
//                RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<"tolerence: "<<priorityTolMap.at(twistItr->second.second.rot[i]).second;
//                if(fabs(twistItr->second.first.vel[i]-twist.rot[i]) > eps)
//                {
//                    //converged = false;  UNUSED VARIABLE
//                }
//                if(fabs(twist.rot[i]) > priorityTolMap.at(twistItr->second.second.rot[i]).second)
//                {
//                    achieved = false;
//                }
//            }
//            RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<"new twist: "<<twist.rot[i]<<", "<<fabs(twistItr->second.first.rot[i]-twist.rot[i]);
//            RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<"priority: "<<twist.rot[i]<<", "<<twistItr->second.second.rot[i];
//        }
        for (int i = 0; i < 6; ++i)
        {
            if (isnan(twist[i]))
            {
                throw std::runtime_error("nan in twist found");
            }

//            if(fabs(twistItr->second.first[i]-twist[i]) > eps )
//                converged = false;

            if(twistItr->second.second[i] <= LOW && !(fabs(twist[i]) < eps || fabs(twistItr->second.first[i]-twist[i]) < eps ))
            {
                converged = false;
            }
//            else if(twistItr->second.second[i] >= MEDIUM && !converged)
//            {
//                achieved = false;
//            }

            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<twist[i]<<", "<<fabs(twistItr->second.first[i]-twist[i]);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.findFrameTwist")<<log4cpp::Priority::DEBUG<<twist[i]<<", "<<twistItr->second.second[i];
        }

        twistItr->second.first = twist;
    }

    //return (achieved || converged);
    return converged;
//    return achieved;
}


bool KdlTreeTr::limitTaskTwist(KDL::Twist &twist)
{
    bool changed = false;
    for (int i = 0; i < 6; ++i)
    {
        if (twist[i] > maxTwist)
        {
            twist[i] = maxTwist;
            changed  = true;
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitTaskTwist")<<log4cpp::Priority::DEBUG<<i<<", "<<twist[i]<<", limited to "<<maxTwist;
        }
        if (twist[i] < -maxTwist)
        {
            twist[i] = -maxTwist;
            changed  = true;
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitTaskTwist")<<log4cpp::Priority::DEBUG<<i<<", "<<twist[i]<<", limited to "<<-maxTwist;
        }
    }
    return changed;
}

bool KdlTreeTr::limitTaskTwist(VectorXd &twist)
{
    bool changed = false;
    for (int i = 0; i < 6; ++i)
    {
        if (twist[i] > maxTwist)
        {
            twist[i] = maxTwist;
            changed  = true;
        }
        if (twist[i] < -maxTwist)
        {
            twist[i] = -maxTwist;
            changed  = true;
        }
    }
    return changed;
}

/**
 * @brief getJoints
 * @param jointIn input joints
 * @param nodeNames input frame names
 * @param nodeFrames input frames
 * @param jointsOut output joints
 * @param nodePriorities input node priorities (6 * nodeNames.size())
 */
void KdlTreeTr::getJointPositions(const JntArray& jointIn, const vector<string>& nodeNames,
                                  const vector<Frame>& nodeFrames, JntArray& jointsOut,
                                  const vector<NodePriority>& nodePriorities)
{
    if (this->nodeNames != nodeNames || this->nodePriorities != nodePriorities)
    {
        this->nodeNames      = nodeNames;
        this->nodePriorities = nodePriorities;
        createFrameTwistMaps(nodeFrames, nodePriorities);
    }

    updateNodeFrameMap(nodeFrames);

    numJoints = jointIn.rows();

    KDL::JntArray deltaJoints(numJoints);
    jointsOut = jointIn;
    unsigned int i;
    for (i = 0; i < critMaxIter; ++i)
    {
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getJointPositions")<<log4cpp::Priority::INFO<<"entering loop "<<i;
        if (findFrameTwist(jointsOut))
        {
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getJointPositions")<<log4cpp::Priority::INFO<<"converged "<<i;
            break;
        }

        //! A move is neccesssary, find it
        getTaskReconstruction(jointsOut, deltaJoints);

        //! limitJoints
        limitJoints(jointIn, jointsOut, deltaJoints);
    }

    //! @todo output the maximum difference between desired pose and reconstructed pose and pass that back
    //!       or add an additional "accuracy" input for each task which will cause this to throw if it is not achieved
    std::stringstream err;
    err << "Unable to find a good TR solution.. "<<std::endl;
    if (!isTaskAchieved(jointsOut, err))
    {
        if (momLim)
        {
            err << "mom below limit"<<std::endl;
        }
        if (velocityLim)
        {
            err << "joint velocity limit reached"<<std::endl;
        }
        if (jointLim)
        {
            err << "joint limit reached"<<std::endl;
        }
        // RL HAX:
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.getJointPositions", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
}

bool KdlTreeTr::isTaskAchieved(const KDL::JntArray& jointIn, stringstream &errMsg)
{
    fkPosSolverPtr->getPoses(jointIn, currentFrameMap);

    bool achieved = true;

    for (FrameMapItr nodeFrameItr = desiredFrameMap.begin(); nodeFrameItr != desiredFrameMap.end(); ++nodeFrameItr)
    {
        //! find corresponding current frame for desired frame
        FrameMapItr frameItr = currentFrameMap.find(nodeFrameItr->first);
        if (frameItr == currentFrameMap.end())
        {
            std::stringstream err;
            err << "node name (" << frameItr->first << ") not found in frame map" << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.isTaskAchieved", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }
        //! find the difference between the desired frame and the current frame
        Twist twist = diff(frameItr->second, nodeFrameItr->second);
        limitTaskTwist(twist);

        //! save in twistMap
        TwistMapItr twistItr = twistMap.find(nodeFrameItr->first);
        if (twistItr == twistMap.end())
        {
            std::stringstream err;
            err << "node name (" << nodeFrameItr->first << ") not found in twist map" << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.isTaskAchieved", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }

        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.isTaskAchieved")<<log4cpp::Priority::DEBUG<<"Twist for "<<nodeFrameItr->first<<":";
        for (int i = 0; i < 3; ++i)
        {
            if (isnan(twist.vel[i]))
            {
                throw std::runtime_error("nan in twist vel found");
            }
            if (priorityTolMap.find((unsigned int)twistItr->second.second.vel[i])!= priorityTolMap.end())
            {
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.isTaskAchieved")<<log4cpp::Priority::DEBUG<<"tolerence: "<<priorityTolMap.at(twistItr->second.second.vel[i]).first;
                if (fabs(twist.vel[i]) > priorityTolMap.at(twistItr->second.second.vel[i]).first)
                {
                    errMsg<< nodeFrameItr->first <<" exceeds tolerence on axis "<<i<<". Value: "<<abs(twist.vel[i])<<", Tol: "<<priorityTolMap.at(twistItr->second.second.vel[i]).second<<std::endl;
                    achieved = false;
                }
            }
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.isTaskAchieved")<<log4cpp::Priority::DEBUG<<"new twist: "<<twist.vel[i]<<", "<<fabs(twistItr->second.first.vel[i]-twist.vel[i]);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.isTaskAchieved")<<log4cpp::Priority::DEBUG<<"priority: "<<twist.vel[i]<<", "<<twistItr->second.second.vel[i];
        }
        for (int i = 0; i < 3; ++i)
        {
            if (isnan(twist.rot[i]))
            {
                throw std::runtime_error("nan in twist ang found");
            }
            if (priorityTolMap.find((unsigned int)twistItr->second.second.rot[i]) != priorityTolMap.end())
            {
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.isTaskAchieved")<<log4cpp::Priority::DEBUG<<"tolerence: "<<priorityTolMap.at(twistItr->second.second.rot[i]).second;
                if (fabs(twist.rot[i]) > priorityTolMap.at(twistItr->second.second.rot[i]).second)
                {
                    errMsg<< nodeFrameItr->first <<" exceeds tolerence on axis "<<i<<". Value: "<<abs(twist.rot[i])<<", Tol: "<<priorityTolMap.at(twistItr->second.second.rot[i]).second<<std::endl;
                    achieved = false;
                }
            }
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.isTaskAchieved")<<log4cpp::Priority::DEBUG<<"new twist: "<<twist.rot[i]<<", "<<fabs(twistItr->second.first.rot[i]-twist.rot[i]);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.isTaskAchieved")<<log4cpp::Priority::DEBUG<<"priority: "<<twist.rot[i]<<", "<<twistItr->second.second.rot[i];
        }

        twistItr->second.first = twist;
    }

    return achieved;
}

/**
  * @brief performs a task based decomposition (tasks defined by twist_deltas and twist_priorities)
  *        and reconstructs the task to avoid algorithmic and kinematic singularities
  * @param jointIn input joint positions
  * @param twist_deltas the desired displacement
  * @param joint_deltas the output joint positions
  * @param twist_priorities input twist priorities ( 0 - do not care)
  */
void KdlTreeTr::getTaskReconstruction(const KDL::JntArray& jointIn, KDL::JntArray& joint_deltas)
{
    MatrixXd J;
    MatrixXd Jh;
    MatrixXd Jhi;
    VectorXd dr;
    VectorXd drh;
    VectorXd drhp;
    MatrixXd N = MatrixXd::Identity(numJoints, numJoints);
    VectorXd dq = VectorXd::Zero(numJoints);
    //MatrixXd A = MatrixXd::Identity(numJoints, numJoints);
    //MatrixXd Ai = MatrixXd::Identity(numJoints, numJoints);
    MatrixXd L;
    MatrixXd Li;
    double   mom;
    //double   x;

    //getOrderedTasksByFrame(jointIn);
    getOrderedTasksByPriority(jointIn);

    //! @todo add optimization to push null space solution toward the middle of the joint's range
    // taskJacobians.push_back(getCenterJointsTask(jointIn));

    for(unsigned int i = 0; i < taskList.size(); ++i)
    {
        mom            = 0;
        J              = taskList[i].first;
        dr             = taskList[i].second;
        numConstraints = dr.rows();

        // Kinetic energy minimizing jacobian
        //pInv(jacJntWeights, ijacJntWeights, mom);
        Jh  = J * N;
        L   = Jh * ijacJntWeights * Jh.transpose();
        pInv(L, Li, mom);
        Jhi = ijacJntWeights*Jh.transpose()*Li;
        N   = N - Jhi*Jh;
        drh = dr - J*dq;

        // Velocity minimizing jacobian
//        Jh = J * N;
//        pInv(Jh,Jhi, mom);
//        N = N - Jhi*Jh;
//        drh = dr - J*dq;

        if (mom != 0)
        {
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getTaskReconstruction")<<log4cpp::Priority::DEBUG<<"mom:"<<mom;
            modifyToAvoidSingularities(getBiasVector(jointIn.data, dq, Jhi), mom, drh, drhp);
            dq= dq + Jhi*drhp;
        }
        else
        {
            std::stringstream err;
            err << "Unable to find a good TR solution -- MOM is zero."<< std::endl;
            err << "This only happens if:"<<std::endl;
            err << "\ta) You are attempting to control the same axis twice in the same trajectory"<<std::endl;
            err << "\tb) You are attempting to control an axis which can not move"<<std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.getTaskReconstruction", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }
    }

    joint_deltas.data = dq;
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getTaskReconstruction")<<log4cpp::Priority::DEBUG<<"dq:"<<joint_deltas.data;

//    ////////////////////////////////////////////////////////////////////////////////////////////
//    //show me dq
//    RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getTaskReconstruction")<<log4cpp::Priority::DEBUG<<"dq: \n\r"<<dq;
//    RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getTaskReconstruction")<<log4cpp::Priority::DEBUG<<"joint deltas: \n\r"<<joint_deltas.data;
//    ////////////////////////////////////////////////////////////////////////////////////////////
}

/**
  * @brief  creates a sorted vector of Tasks (jacobian and dr) based on the priority value given.
  *         It combines tasks in different frames which all have the same priority number.
  *         They are then sorted by highest priority first.
  * @todo   This is not very efficient because the matrices and vectors get resized at every iteration,
  *         really, it only needs to be resized when we create the maps....
  * @todo   Remove items that have a priority of IGNORE
  * @param  jointIn   the current joint positions
  * @returns    a sorted vector of Tasks
  */
void KdlTreeTr::getOrderedTasksByPriority(const JntArray& jointIn)
{
    //get jacobians and store them in a map
    std::map<std::string, KDL::Jacobian> nameJacobianMap;
    Jacobian jac(numJoints);
    for (TwistMapItr twistIt = twistMap.begin(); twistIt != twistMap.end(); ++twistIt)
    {
        int ret = jacSolverPtr->JntToJac(jointIn, jac, twistIt->first);
        if (ret < 0)
        {
            std::stringstream err;
            err<<"unable to find Jacobian for Twist: "<< twistIt->first<<", probably due to incorrectly initialized jacobian size"<<std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByPriority", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }
        nameJacobianMap[twistIt->first] = jac;
    }
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByPriority")<<log4cpp::Priority::DEBUG<<"solved for jacobians";
    //create taskMap which has jacobian and twist for each axis
    taskList.clear();
    for (PriorityTaskAxisItr ptaItr = priorityTaskAxisMap.begin(); ptaItr != priorityTaskAxisMap.end(); ++ptaItr)
    {
        int priorityNum = ptaItr->first;
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByPriority")<<log4cpp::Priority::DEBUG<<"Finding task for priority #"<<priorityNum;
        if (priorityNum == IGNORE)
            continue;
        Task mTask;
        int priorityCount = priorityTaskAxisMap.count(priorityNum);
        mTask.first.resize(priorityCount, numJoints);
        mTask.second.resize(priorityCount);
        for (int i = 0; i < priorityCount; ++i)
        {
            string nodeName = ptaItr->second.first;
            int    axis     = ptaItr->second.second;
            mTask.first.block(i, 0, 1, numJoints) = nameJacobianMap[nodeName].data.block(axis, 0, 1, numJoints);
            mTask.second[i] = twistMap[nodeName].first[axis];
            ++ptaItr;
        }
        taskList.push_back(mTask);
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByPriority")<<log4cpp::Priority::DEBUG<<"task #"<<priorityNum<<":\n\r"<<mTask.second;
        if (priorityCount > 0)
        {
            --ptaItr;
        }


    }
    //reverse order so highest number comes first
    //std::reverse(taskList.begin(), taskList.end());
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByPriority")<<log4cpp::Priority::DEBUG<<"Found "<<taskList.size()<<" tasks";
}

/**
  * @brief  creates a sorted vector of Tasks (jacobian and dr) based on the order they are provided.
  *         This is much more efficient since the jacobian matrix and dr vector need to be resized just once.
  * @param  jointIn   The current joint positions
  * @returns    a vector of sorted Tasks
  */
void KdlTreeTr::getOrderedTasksByFrame(const JntArray& jointIn)
{
    TwistMapItr twistMapItr;
    KDL::RotationalInertia ri;
    //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByFrame", log4cpp::Priority::DEBUG, "Ordered task list by frame:");
    if (taskList.size() != nodeNames.size())
    {
        taskList.resize(nodeNames.size());
    }
    for (unsigned int i = 0; i < nodeNames.size(); ++i)
    {
        if ((twistMapItr = twistMap.find(nodeNames[i])) == twistMap.end())
        {
            std::stringstream err;
            err << "could not find twist for frame: " << nodeNames[i] << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByFrame", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }
        TwistPriority twistPriority = twistMapItr->second;

        numConstraints = 0;
        for (int j = 0; j < 6; ++j)
        {
            if (twistPriority.second[j] != IGNORE)
            {
                ++numConstraints;
            }
        }

        if (numConstraints != 0)
        {
            MatrixXd J = MatrixXd::Zero(numConstraints, numJoints);
            VectorXd dr(numConstraints);
            Jacobian jac(numJoints);

            //get jacobian for twist
            int ret = jacSolverPtr->JntToJac(jointIn, jac, twistMapItr->first);
            if (ret < 0)
            {
                std::stringstream err;
                err << "unable to find a Jacobian for Twist: " <<twistMapItr->first << std::endl;
                err << "jointIn : "<<jointIn.data;
                //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByFrame", log4cpp::Priority::ERROR, err.str());
                throw std::runtime_error(err.str());
            }

            // count number of constraints
            // put jacobian in big matrix and put the twist in the big t
            int jacIndex = 0;
            for (int j = 0; j < 6; ++j)
            {
                if (twistPriority.second[j] != IGNORE)
                {
                    J.block(jacIndex, 0, 1, numJoints) = jac.data.block(j, 0, 1, numJoints);
                    dr[jacIndex] = twistPriority.first[j];
                    ++jacIndex;
                }
            }
            taskList[i] = std::make_pair(J, dr);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getOrderedTasksByFrame")<<log4cpp::Priority::DEBUG<<"task "<<nodeNames[i]<<":\n\r"<<dr;
        }

    }
    return;
}

KdlTreeTr::Task KdlTreeTr::getCenterJointsTask(const JntArray& jointIn)
{

    std::map<std::string, double>::const_iterator minIt;
    std::map<std::string, double>::const_iterator maxIt;
    JntArray q(numJoints);
    //! find the midpoint of all joints
    std::pair<double, double> limits;
    for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex)
    {
        if (positionLimiter->getLimits(jointNames.at(jointIndex), limits))
        {
            q(jointIndex) = (limits.first + limits.second)/2.;
        }
        else
        {
            std::stringstream err;
            err << "in getCenterJointsTask() unable to find the min/max for joint: " << jointNames.at(jointIndex);
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }
    }
    std::map<std::string, KDL::Frame> centerFrameMap;
    fkPosSolverPtr->getPoses(q,centerFrameMap);
    std::map<std::string, KDL::Frame> currentFrameMap;
    fkPosSolverPtr->getPoses(jointIn, currentFrameMap);

    MatrixXd bigJac(6*centerFrameMap.size(), numJoints);
    VectorXd bigTwist(6*centerFrameMap.size());
    int jacIndex = 0;

    for (FrameMapItr frameItr = currentFrameMap.begin(); frameItr != currentFrameMap.end(); ++frameItr)
    {
        FrameMapItr nodeFrameItr;
        if ((nodeFrameItr = centerFrameMap.find(frameItr->first))!= centerFrameMap.end())
        {
            //! find the twist bewteen the current frame and the center frame
            Twist twist = diff(frameItr->second, nodeFrameItr->second);
            for (int twistIdx = 0; twistIdx < 6; ++twistIdx)
            {
                //! @todo hunt down this nan problem
                if(isnan(twist[twistIdx]))
                {
                    twist[twistIdx] = 0;
                }
            }

            //! solve for the jacobian
            Jacobian jac;
            int ret = jacSolverPtr->JntToJac(jointIn, jac, frameItr->first);
            if (ret < 0)
            {
                std::stringstream err;
                err << "in getCenterJointsTask() unable to find a Jacobian for Twist: " << frameItr->first << std::endl;
                //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr", log4cpp::Priority::ERROR, err.str());
                throw std::runtime_error(err.str());
            }

            //! build big Jacobian and big Twist
            for (unsigned int i = 0; i< 6; ++i)
            {
                bigJac.block(jacIndex, 0, 1, numJoints) = jac.data.block(i, 0, 1, numJoints);
                bigTwist[jacIndex] = twist[i];
                ++jacIndex;
            }

        }
        else
        {
            std::stringstream err;
            err << "in CenterJointsTask() unable to find the center frame for joint: " << frameItr->first;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
        }

    }

    return Task(bigJac, bigTwist);
}


/**
  * @param nm: bias vector (vector perpendicular to the object to be avoided in task space)
  * @param mom: the measure of manipulability for this task
  * @param dr: the desired task deltas
  * @param drp: the modified task deltas
  */
void KdlTreeTr::modifyToAvoidSingularities(const VectorXd& nm, const double mom, const VectorXd& dr, VectorXd& drp)
{
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"entered modifyToAvoidSingularities(const VectorXd& nm, const double mom, const VectorXd& dr, VectorXd& drp)";
    MatrixXd Im(numConstraints,numConstraints);
    MatrixXd Mtemp;
    double   k1;
    double   k2;
    VectorXd drp_temp;
    momLim = false;

    //k1 = 0.5 * (1 + sign(dr.dot(nm))) * boundedCubic(mom,1,0,mbar,2*mbar);
    k1 = 0.5 * (1 - sign(dr.dot(nm))) * boundedCubic(mom,1,0,mbar,2*mbar);
    //k2 = Kr * boundedCubic(mom, 1, 0, 0.5*mbar, mbar);
    k2 = Kr * boundedCubic(mom, 1, 0, 0.5*mbar, mbar);

    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"k1: "<<k1;
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"k2: "<<k2;
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"mbar: "<<mbar;
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"boundedCubic k1: "<<boundedCubic(mom,1,0,mbar,2*mbar);
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"boundedCubic k2: "<<boundedCubic(mom, 1, 0, eps, mbar);
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"task size: "<<taskList.size();
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::INFO<<"mom: "<<mom;
    if(k1 > 0 || k2 > 0)
    {
        momLim = true;
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::INFO<<"k1: "<<k1;
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::INFO<<"k2: "<<k2;
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::INFO<<"nm\n"<<nm;
        //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::INFO<<"mom: "<<mom;
    }

    Im.setIdentity(numConstraints,numConstraints);

    Mtemp = Im - k1* (nm * nm.transpose());                  // Mtemp = Im - k1*[nm * nm^T]

    drp_temp = Mtemp * dr;                  // Im - k1*[ nm * nm^T] * dr

    drp = drp_temp + k2 * nm;               // drp = (Im - k1*[nm * nm^T]) * dr + k2*nm

//    if(k2 > 0 && k1 > 0)
//    {
//        cin.ignore();
//    }

    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.modifyToAvoidSingularities")<<log4cpp::Priority::DEBUG<<"drp: "<<drp;
//    if(mom<=mbar)
//    {
//        cin.ignore();
//    }
}

/**
  * @brief  limits joint angles to be within valid range
  * @param q    The input joint angles
  * @param dq   The desired joint changes
  * @param qp   The modified joint angles to avoid limits
  */
bool KdlTreeTr::limitJoints(const JntArray &jointIn, JntArray& jointsOut, JntArray& deltaJoints)
{
    //limit joints
    bool limited = false;
    velocityLim  = false;
    jointLim     = false;
    std::map<std::string, double>::const_iterator it;
    for (unsigned int jointIndex = 0; jointIndex < jointsOut.rows(); ++jointIndex)
    {
        double newJointPos = jointsOut(jointIndex)+deltaJoints(jointIndex);

        if ((newJointPos - jointIn(jointIndex))  > jointMaxVel)
        {
            velocityLim = true;
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<jointNames.at(jointIndex)<<": "<<newJointPos - jointIn(jointIndex)<<" near max vel "<<jointMaxVel;
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"current delta: "<<deltaJoints(jointIndex);
            deltaJoints(jointIndex)= jointIn(jointIndex) + jointMaxVel - jointsOut(jointIndex);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"new delta: "<<deltaJoints(jointIndex);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<"new velocity: "<<jointsOut(jointIndex)+deltaJoints(jointIndex) - jointIn(jointIndex);
            //cin.ignore();
        }
        if ((newJointPos - jointIn(jointIndex))  < - jointMaxVel)
        {
            velocityLim = true;
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<jointNames.at(jointIndex)<<": "<<newJointPos - jointIn(jointIndex)<<" near min vel "<<-jointMaxVel;
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"current delta: "<<deltaJoints(jointIndex);

            deltaJoints(jointIndex) = jointIn(jointIndex) - jointMaxVel - jointsOut(jointIndex);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"new delta: "<<deltaJoints(jointIndex);
            //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<"new velocity: "<< jointsOut(jointIndex)+deltaJoints(jointIndex) - jointIn(jointIndex);
            //cin.ignore();

        }

        std::pair<double, double> limits;
        if (positionLimiter->getLimits(jointNames.at(jointIndex), limits))
        {
            /// this will be deprecated by RDEV-1318 so hardcoding a buffer for now
            double jointLimBuffer = 0.087; // ~5 degrees
            if (newJointPos <= limits.first + jointLimBuffer && sign(deltaJoints(jointIndex))<0 && deltaJoints(jointIndex) != 0)
            {
                jointLim = true;
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<jointNames.at(jointIndex)<<": "<<jointsOut(jointIndex)<<" near min "<<it->second;
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"current delta: "<<deltaJoints(jointIndex);
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"limit multiplier: "<<boundedCubic(jointsOut(jointIndex), 0, 1, it->second, it->second+jointLimBuffer)<<" buffer "<<jointLimBuffer;
                //! if we are near min and the delta is negative (moving toward min), then apply limiting function
                //deltaJoints(jointIndex) *= 0.5*(1. - sign(deltaJoints(jointIndex)))*boundedCubic(jointsOut(jointIndex), 0, 1, it->second, it->second+jointLimBuffer);
                deltaJoints(jointIndex) *= boundedCubic(newJointPos, 0, 1, limits.first, limits.first+jointLimBuffer);
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"new delta: "<<deltaJoints(jointIndex);
                limited = true;
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<jointNames.at(jointIndex)<<": "<<jointsOut(jointIndex)<<" near min "<<it->second<<" new value: "<<jointsOut(jointIndex) + deltaJoints(jointIndex);
            }

            if (newJointPos >= limits.second-jointLimBuffer && sign(deltaJoints(jointIndex))>0  && deltaJoints(jointIndex) != 0)
            {
                jointLim = true;
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<jointNames.at(jointIndex)<<": "<<jointsOut(jointIndex)<<" near max "<<it->second;
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"current delta: "<<deltaJoints(jointIndex);
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"limit multiplier: "<<boundedCubic(jointsOut(jointIndex), 1, 0, it->second-jointLimBuffer, it->second)<<" buffer "<<jointLimBuffer;
                //! if we are near max and the delta is positive (moving toward max), then apply limiting function
                //deltaJoints(jointIndex) *= 0.5*(1. + sign(deltaJoints(jointIndex)))*boundedCubic(jointsOut(jointIndex), 1, 0, it->second-jointLimBuffer, it->second);
                deltaJoints(jointIndex) *= boundedCubic(newJointPos, 1, 0, limits.second-jointLimBuffer, limits.second);
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"new delta: "<<deltaJoints(jointIndex);
                limited = true;
                //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::INFO<<jointNames.at(jointIndex)<<": "<<jointsOut(jointIndex)<<" near min "<<it->second<<" new value: "<<jointsOut(jointIndex) + deltaJoints(jointIndex);
            }

        }

        jointsOut(jointIndex) += deltaJoints(jointIndex);

    }

    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.limitJoints")<<log4cpp::Priority::DEBUG<<"jointsOut\n"<<jointsOut.data;
    return limited;
}

VectorXd KdlTreeTr::getBiasVector(const VectorXd& jointIn, const VectorXd& joint_deltas, const MatrixXd& ijac)
{
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getBiasVector")<<log4cpp::Priority::DEBUG<<"entered getBiasVector(const VectorXd& jointIn, const VectorXd& joint_deltas, const MatrixXd& ijac)";
    VectorXd bias   = VectorXd::Zero(numJoints);
    VectorXd output = VectorXd::Zero(numConstraints);
    //std::cout<<"numJoints: "<<numJoints<<std::endl;
    // std::cout<<"numConstraints: "<<numConstraints<<std::endl;
    std::map<std::string, double>::const_iterator it;
    std::pair<double, double> limits;
    for (int jointIndex = 0; jointIndex < numJoints; ++jointIndex)
    {
        limits.first  = 0;
        limits.second = 0;
        positionLimiter->getLimits(jointNames.at(jointIndex), limits);

        //bias(jointIndex) = jointIn(jointIndex)+joint_deltas(jointIndex) - (min+max)/2.;
        //cout<<"bias: "<<bias(jointIndex)<<std::endl;
        bias(jointIndex) =  (limits.first+limits.second)*0.5 - (jointIn(jointIndex)+joint_deltas(jointIndex));
    }

    output = bias.transpose() * ijac;
    if(output.norm() != 0)
        output.normalize();

    ///////////////////////////////////////////////////////////////////////////////////////
    //show me bias vector
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.getBiasVector")<<log4cpp::Priority::DEBUG<<"output bias vector: \n\r"<<output;
    //////////////////////////////////////////////////////////////////////////////////////
    return output;
}

/**
  * @brief  shape function
  *
  *	Shape_04: Cubic.
  *	Y(x) = a3*t^3 + a2*t^2 + a1*t + a0;
  *	Y(ct1) = pLeft
  *	Y(ct2) = pRight
  *	D(Y)(ct1) = 0
  *	D(Y)(ct2) = 0
  */
double KdlTreeTr::cubic(const double& q, const double& pLeft, const double& pRight, const double& ct1, const double& ct2)
{
    double t1;
    double t12;
    double t17;
    double t2;
    double t24;
    double t25;
    double t30;
    double t31;
    double t36;
    double t7;

    t1  = q*q;
    t2  = t1*q;
    t7  = t1*pRight;
    t12 = t1*pLeft;
    t17 = ct1*ct2;
    t24 = ct1*ct1;
    t25 = t24*ct1;
    t30 = ct2*ct2;
    t31 = t30*ct2;
    t36 = 2.0*t2*pRight-2.0*t2*pLeft-3.0*t7*ct1-3.0*t7*ct2+3.0*t12*ct1+3.0*t12*
          ct2+6.0*t17*q*pRight-6.0*t17*q*pLeft+t25*pRight-3.0*t24*pRight*ct2-pLeft*t31+
          3.0*pLeft*ct1*t30;
    return(t36/(-t31-3.0*ct2*t24+t25+3.0*ct1*t30));
}

/**
  *	Shape_04_bounded: Cubic Bounded.
  *	         _
  *	        |   pLeft                                 x <= ct1
  *	        |
  *	Y(x) = -|   a3*t^3 + a2*t^2 + a1*t + a0    ct1 <  x < ct2
  *	        |
  *	        |_  pRight                         ct2 <= x
  *
  *	D(Y)(ct1) = 0
  *	D(Y)(ct2) = 0
  */
double KdlTreeTr::boundedCubic(const double& q, const double& pLeft, const double& pRight, const double& ct1, const double& ct2)
{
    if (q <= ct1)
    {
        return pLeft;
    }
    else if (ct2 <= q)
    {
        return pRight;
    }
    else
    {
        return cubic(q, pLeft, pRight, ct1, ct2);
    }
}


/**
  * @brief calculate the pseudoinverse of a matrix
  * @param jacIn       the input matrix
  * @param ijacOut     the inverted matrix
  * @param mom          the measure of manipulability of the matrix
  */
int KdlTreeTr::pInv(const MatrixXd& jacIn, MatrixXd& ijacOut, double &mom)
{
    int    numConstraints = jacIn.rows();
    int    numJoints      = jacIn.cols();
    double momcheck;

    ijacOut.resize(numJoints, numConstraints);

    VectorXd tmp(numJoints);
    VectorXd S(VectorXd::Zero(numJoints));
    MatrixXd U(MatrixXd::Identity(numConstraints, numJoints));
    MatrixXd V(MatrixXd::Identity(numJoints, numJoints));
    if (KDL::svd_eigen_HH(jacIn, U, S, V, tmp, critMaxIter) < 0)
    {
        std::stringstream err;
        err << "in pInv() SVD calculation failed" << std::endl;
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeTr", log4cpp::Priority::ERROR, err.str());
        return -1;
    }
    tmp      = VectorXd::Zero(numJoints);
    mom      = 1;
    momcheck = 1;
    //mom = numJoints/numConstraints;
    for (int i = 0; i < min(numJoints, numConstraints); ++i)
    {
        if (S(i) != 0)
        {
            double is = 1/S(i);
            tmp(i)    = is;
        }
        //mom *= pow(S(i), (double)1/numConstraints);
        mom *= S(i);
    }
    //mom = mom*min(numJoints, numConstraints);
    ijacOut  = V * tmp.asDiagonal()*U.adjoint();
    mom      = pow(mom, (double)1/numConstraints);
    //momcheck = S(min(numJoints, numConstraints)-1)/S(0);
    momcheck = S.minCoeff()*S.maxCoeff();
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //show me inverse
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.pInv")<<log4cpp::Priority::DEBUG<<"ijac: \n\r"<<ijacOut;
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.pInv")<<log4cpp::Priority::DEBUG<<"mom: "<<mom;
    //RCS::Logger::getCategory("gov.nasa.controllers.KdlTreeTr.pInv")<<log4cpp::Priority::DEBUG<<"momcehck: "<<momcheck;
    /////////////////////////////////////////////////////////////////////////////////////////////////
    mom = momcheck;
    return 0;
}

