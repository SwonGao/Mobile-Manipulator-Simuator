#include <nasa_robodyn_controllers_core/KdlTreeIk.h>

const int KdlTreeIk::IGNORE   = 0; //nasa_r2_common_msgs::PriorityArray::IGNORE;
const int KdlTreeIk::LOW      = 4; //nasa_r2_common_msgs::PriorityArray::LOW;
const int KdlTreeIk::MEDIUM   = 3; //nasa_r2_common_msgs::PriorityArray::MEDIUM;
const int KdlTreeIk::HIGH     = 2; //nasa_r2_common_msgs::PriorityArray::HIGH;
const int KdlTreeIk::CRITICAL = 1; //nasa_r2_common_msgs::PriorityArray::CRITICAL;

KdlTreeIk::KdlTreeIk()
    : KdlTreeUtilities()
    , inertias(1)
    , useDampedLeastSquares(true)
{
    setTimeStep();
    setMaxCriticalIterations();
    setMaxNonCriticalIterations();
    setEpsilon();
    setLambda();
}

KdlTreeIk::~KdlTreeIk()
{
}

void KdlTreeIk::initialize()
{
    KdlTreeUtilities::getJointNames(jointNames);

    // setup solver
    fkPosSolverPtr.reset(new KdlTreeFk());
    fkPosSolverPtr->setTree(tree);

    jacSolverPtr.reset(new KDL::TreeJntToJacSolver(tree));
    inertias.resize(jointNames.size());
}

void KdlTreeIk::setTimeStep(double timeStepIn)
{
    if (timeStepIn > 0)
    {
        timeStep = timeStepIn;
    }
    else
    {
        std::stringstream err;
        err << "KdlTreeIk::setTimeStep() - Time step must be greater than 0";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeIk", log4cpp::Priority::ERROR, err.str());
        throw std::invalid_argument(err.str());
    }
}

void KdlTreeIk::getFrames(const KDL::JntArray& joints_in, std::map<std::string, KDL::Frame>& frameMap)
{
    fkPosSolverPtr->getPoses(joints_in, frameMap);
}
void KdlTreeIk::setJointInertias(const std::map<std::string, double> &inertiaIn)
{
    // if the inertia map has changed, repopulate the inertias
    if (inertiaIn != inertiaMap)
    {
        inertiaMap = inertiaIn;
        minInertia = 1.;
        maxInertia = 1.;
        std::map<std::string, double>::iterator itr;
        for (itr = inertiaMap.begin(); itr != inertiaMap.end(); ++itr)
        {
            if (minInertia > itr->second)
            {
                minInertia = itr->second;
            }
            if (maxInertia < itr->second)
            {
                maxInertia = itr->second;
            }
        }
        for(unsigned int i = 0; i < jointNames.size(); ++i)
        {
            itr = inertiaMap.find(jointNames[i]);
            if (itr != inertiaMap.end())
            {
                inertias(i) = itr->second;
            }
            else
            {
                inertias(i) = minInertia;
            }
        }
    }
}

void KdlTreeIk::getJointPositions(const KDL::JntArray& jointsIn, const std::vector<std::string>& nodeNames,
                                  const std::vector<KDL::Frame>& nodeFrames, KDL::JntArray& joints_out,
                                  const std::vector<NodePriority>& nodePriorities)
{
    // create frame and twist maps
    std::map<std::string, KDL::Frame> frameMap;
    std::vector<std::pair<std::string, KDL::Twist> > twistVec;
    for (unsigned int i = 0; i < nodeNames.size(); ++i)
    {
        if (tree.getSegment(nodeNames[i]) == tree.getSegments().end())
        {
            std::stringstream err;
            err << "KdlTreeIk node name (" << nodeNames[i] << ") not found in tree" << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeIk", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
            return;
        }
        frameMap.insert(std::make_pair(nodeNames[i], KDL::Frame()));
        twistVec.push_back(std::make_pair(nodeNames[i], KDL::Twist()));
    }

    KDL::JntArray delta_joints(jointsIn.rows());
    joints_out = jointsIn;
    unsigned int iter;
    for (iter = 0; iter < critMaxIter; ++iter)
    {
        //get the current poses of the nodes (frames) we are controlling
        fkPosSolverPtr->getPoses(joints_out, frameMap);

        bool twistZero = true;
        for (unsigned int nodeIndex = 0; nodeIndex < nodeNames.size(); ++nodeIndex)
        {
            //find the difference between the current frame pose (frameMap) and the deisred frame pose (nodeFrames)
            KDL::Twist twist = KDL::diff(frameMap.at(nodeNames[nodeIndex]), nodeFrames.at(nodeIndex));
            //save in twistVec
            twistVec[nodeIndex].second = twist;

            //determine if a move is necessary (twistZero = true, no movement)
            for (unsigned int axisIndex = 0; axisIndex < 3; ++axisIndex)
            {
                if (!KDL::Equal(twist.vel[axisIndex],0.,eps))
                {
                    if (nodePriorities[nodeIndex].vel[axisIndex] == KdlTreeIk::CRITICAL || iter < nonCritMaxIter)
                    {
                        twistZero = false;
                    }
                }
            }
            for (unsigned int axisIndex = 0; axisIndex < 3; ++axisIndex)
            {
                if (!KDL::Equal(twist.rot[axisIndex],0.,eps))
                {
                    if (nodePriorities[nodeIndex].rot[axisIndex] == KdlTreeIk::CRITICAL || iter < nonCritMaxIter)
                    {
                        twistZero = false;
                    }
                }
            }
        }

        if (twistZero)
        {
            break;
        }

        //a move is necessary, find the velocity
        getJointVelocities(joints_out, twistVec, nodePriorities, delta_joints);

        std::map<std::string, double>::const_iterator it;
        for (unsigned int jointIndex = 0; jointIndex < joints_out.rows(); ++jointIndex)
        {
            // deprecated joint limiting (leaving commented until RDEV-1375)
//            if ((it = jointMin.find(jointNames.at(jointIndex))) != jointMin.end() // min set
//                    && joints_out(jointIndex) <= it->second // min violated
//                    && delta_joints(jointIndex) < 0.) // moving toward min
//            {
//                if (jointLimBuffer > 0.)
//                    delta_joints(jointIndex) *= (1. + (joints_out(jointIndex) - it->second) / jointLimBuffer);
//                else
//                    delta_joints(jointIndex) = 0.;
//            }
//            else if ((it = jointMax.find(jointNames.at(jointIndex))) != jointMax.end() // max set
//                     && joints_out(jointIndex) >= it->second // max violated
//                     && delta_joints(jointIndex) > 0.) // moving toward max
//            {
//                if (jointLimBuffer > 0.)
//                    delta_joints(jointIndex) *= (1. - (joints_out(jointIndex) - it->second) / jointLimBuffer);
//                else
//                    delta_joints(jointIndex) = 0.;
//            }

            joints_out(jointIndex) += delta_joints(jointIndex);

            if (positionLimiter->hasLimits(jointNames.at(jointIndex)))
            {
                positionLimiter->limit(jointNames.at(jointIndex), joints_out(jointIndex));
            }
        }
    }

    if (iter >= critMaxIter)
    {
        std::stringstream err;
        err << "Unable to find an IK solution" << std::endl;
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeIk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
}

void KdlTreeIk::getJointVelocities(const KDL::JntArray& jointsIn, const std::vector<std::pair<std::string, KDL::Twist> >& twist_deltas,
                                   const std::vector<NodePriority>& nodePriorities, KDL::JntArray& joint_deltas) const
{
    // get the jacobian
    unsigned int numConstraints = 0;
    for (std::vector<NodePriority>::const_iterator it = nodePriorities.begin(); it != nodePriorities.end(); ++it)
    {
        for (int i = 0; i < 6; ++i)
        {
            if (it->operator[](i) != IGNORE)
            {
                ++numConstraints;
            }

        }

    }
    unsigned int    numJoints = jointsIn.rows();
    KDL::Jacobian   jac(numJoints);
    Eigen::MatrixXd bigJac(numConstraints, numJoints);
    Eigen::VectorXd bigTwist(numConstraints);
    MatrixXd        Wy            = MatrixXd::Identity(numConstraints, numConstraints);
    unsigned int    priorityIndex = 0;
    unsigned int    jacIndex      = 0;
    for (std::vector<std::pair<std::string, KDL::Twist> >::const_iterator twistIt = twist_deltas.begin(); twistIt != twist_deltas.end(); ++twistIt)
    {
        KDL::Twist nPriority = nodePriorities[priorityIndex];
        //get jacobian for twist
        int ret = jacSolverPtr->JntToJac(jointsIn, jac, twistIt->first);
        if (ret < 0)
        {
            std::stringstream err;
            err << "Unable to find a Jacobian for Twist: " << twistIt->first << std::endl;
            //RCS::Logger::log("gov.nasa.controllers.KdlTreeIk", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
            return;
        }

        // put jacobian in big matrix and put the twist in the big t
        for (unsigned int i = 0; i < 6; ++i)
        {
            if (nPriority[i] != IGNORE)
            {
                bigJac.block(jacIndex, 0, 1, numJoints) = jac.data.block(i, 0, 1, numJoints);
                bigTwist[jacIndex]                      = twistIt->second[i];
                Wy(jacIndex, jacIndex)                  = (256-static_cast<double>(nPriority[i]))/255.;
                ++jacIndex;
            }

        }
        ++priorityIndex;
    }

    // get weighted jacobian
    bigJac = Wy * bigJac;

    // get SVD
    VectorXd tmp(numJoints);
    VectorXd S(VectorXd::Zero(numJoints));
    MatrixXd U(MatrixXd::Identity(numConstraints, numJoints));
    MatrixXd V(MatrixXd::Identity(numJoints, numJoints));
    if (KDL::svd_eigen_HH(bigJac, U, S, V, tmp, critMaxIter) < 0)
    {
        std::stringstream err;
        err << "SVD calculation failed" << std::endl;
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeIk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }

    // get weighted twists
    bigTwist = Wy * bigTwist;

    //first we calculate Ut*v_in
    double sum;
    for (unsigned int i = 0; i < numJoints; ++i)
    {
        sum = 0.0;
        for (unsigned int j = 0; j < numConstraints; ++j)
        {
            sum += U(j,i) * bigTwist(j);
        }

        if (useDampedLeastSquares)
        {
            // Damped Least Squares
            tmp[i] = sum * S[i] / (S[i]*S[i] + lambda_squared);
        }
        else
        {
            // truncated PseudoInverse
            tmp[i] = sum * (fabs(S[i]) < eps ? 0.0 : 1.0 / S[i]);
        }
    }
    //tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
    //it with V to get qdot_out
    for (unsigned int i = 0; i < numJoints; ++i)
    {
        sum = 0.0;
        for (unsigned int j = 0; j < numJoints; ++j)
        {
            sum += V(i,j) * tmp[j];
        }
        //Put the result in qdot_out
        joint_deltas(i)=sum;
    }
}
