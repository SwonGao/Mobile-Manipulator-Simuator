#include "nasa_robodyn_controllers_core/MobileTreeIk.h"

const int MobileTreeIk::BASE = -1;

MobileTreeIk::MobileTreeIk()
    : KdlTreeTr(), mobileJoints(6)
{
}

MobileTreeIk::~MobileTreeIk()
{
}

void MobileTreeIk::setBases(const std::vector<std::string>& basesIn)
{
    if (basesIn.size() < 1)
    {
        std::stringstream err;
        err << "MobileTreeIk setBases must get at least one base" << std::endl;
        //RCS::Logger::log("gov.nasa.controllers.MobileTreeIk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }

    for (unsigned int i = 0; i < basesIn.size(); ++i)
    {
        if (!hasSegment(basesIn[i]))
        {
            std::stringstream err;
            err << "MobileTreeIk setBases couldn't find segment " << basesIn[i];
            //RCS::Logger::log("gov.nasa.controllers.MobileTreeIk", log4cpp::Priority::ERROR, err.str());
            throw std::runtime_error(err.str());
            return;
        }
    }

    bases = basesIn;
}

void MobileTreeIk::resetMobileJoints()
{
    mobileJoints = KDL::JntArray(6);
}

void MobileTreeIk::getJointPositions(const KDL::JntArray& jointsIn, const std::vector<std::string>& nodeNames,
                                     const std::vector<KDL::Frame>& nodeFrames, KDL::JntArray& jointsOut,
                                     const std::vector<KdlTreeIk::NodePriority>& nodePriorities)
{
    // add mobile joints
    KDL::JntArray allJoints(jointsIn.rows() + mobileJoints.rows());
    allJoints.data.segment(mobileJoints.rows(), jointsIn.rows()) = jointsIn.data;
    for (unsigned int i = 0; i < mobileJoints.rows(); ++i)
    {
        allJoints.data[i] = mobileJoints(i);
    }

    std::vector<KDL::Frame> frames = nodeFrames;

    // add bases to end
    for (unsigned int i = 0; i < bases.size(); ++i)
    {
        KDL::Frame baseFrame;
        fkPosSolverPtr->getPose(allJoints, bases[i], baseFrame);
        frames.insert(frames.begin(), baseFrame);
    }

    std::vector<KdlTreeIk::NodePriority> priorities = nodePriorities;
    // add bases to nodes and priorities
    std::vector<std::string> nodes = nodeNames;
    for (unsigned int i = 0; i < bases.size(); ++i)
    {
        nodes.insert(nodes.begin(), bases[i]);
        KDL::Twist priority;
        for (unsigned int j = 0; j < mobileJoints.rows(); ++j)
        {
            priority[j] = MobileTreeIk::BASE;
        }
        priorities.insert(priorities.begin(), priority);
    }

    KDL::JntArray allJoints_out(allJoints.rows());
    KdlTreeTr::getJointPositions(allJoints, nodes, frames, allJoints_out, priorities);

    // extract out joints that matter
    for (unsigned int i = 0; i < jointsOut.rows(); ++i)
    {
        jointsOut(i) = allJoints_out(i+mobileJoints.rows());
    }
    for (unsigned int i = 0; i < mobileJoints.rows(); ++i)
    {
        mobileJoints(i) = allJoints_out(i);
    }
}

void MobileTreeIk::initialize()
{
    robotModelBase = KdlTreeTr::getBaseName();
    bases.push_back(robotModelBase);

    KDL::Tree newTree("world_base");
    KDL::Chain baseChain;

    // add mobile joints
    resetMobileJoints();
    baseChain.addSegment(KDL::Segment("baseX",     KDL::Joint("baseX", KDL::Joint::TransX)));
    baseChain.addSegment(KDL::Segment("baseY",     KDL::Joint("baseY", KDL::Joint::TransY)));
    baseChain.addSegment(KDL::Segment("baseZ",     KDL::Joint("baseZ", KDL::Joint::TransZ)));
    baseChain.addSegment(KDL::Segment("baseRoll",  KDL::Joint("baseRoll", KDL::Joint::RotX)));
    baseChain.addSegment(KDL::Segment("basePitch", KDL::Joint("basePitch", KDL::Joint::RotY)));
    baseChain.addSegment(KDL::Segment("baseYaw",   KDL::Joint("baseYaw", KDL::Joint::RotZ)));
    baseChain.addSegment(KDL::Segment(robotModelBase));

    newTree.addChain(baseChain, "world_base");
    newTree.addTree(tree, robotModelBase);
    tree = newTree;

    KdlTreeTr::initialize();
}

void MobileTreeIk::getJointNames(std::vector<std::string>& jointNames) const
{
    jointNames.resize(tree.getNrOfJoints() - mobileJoints.rows());
    const KDL::SegmentMap& segments = tree.getSegments();
    for (KDL::SegmentMap::const_iterator segIt = segments.begin(); segIt != segments.end(); ++segIt)
    {
        if (segIt->second.segment.getJoint().getType() != KDL::Joint::None)
        {
            // is a joint
            // don't add first 6
            if (segIt->second.q_nr > mobileJoints.rows()-1)
            {
                jointNames.at(segIt->second.q_nr - mobileJoints.rows()) = segIt->second.segment.getJoint().getName();
            }
        }
    }
}

void MobileTreeIk::getFrames (const KDL::JntArray& joints_in, std::map<std::string, KDL::Frame>& frameMap)
{
    KDL::JntArray mobile_joints_in(joints_in.rows() + mobileJoints.rows());

    for (unsigned int i = 0; i < joints_in.rows(); ++i)
    {
        mobile_joints_in(i+mobileJoints.rows()) = joints_in(i);
    }
    for (unsigned int i = 0; i < mobileJoints.rows(); ++i)
    {
        mobile_joints_in(i) = mobileJoints(i);
    }

    KdlTreeIk::getFrames(mobile_joints_in, frameMap);
}

void MobileTreeIk::getMobileJoints(std::vector<double>& mobileJointValues) const
{
    mobileJointValues.resize(6);
    for (size_t i = 0; i < 6; ++i)
        mobileJointValues[i] = mobileJoints(i);
}

void MobileTreeIk::setMobileJoints(const std::vector<double>& mobileJointValues)
{
    if (mobileJointValues.size() != 6)
    {
        std::string err("Expected vector of size 6 for setMobileJoints");
        //RCS::Logger::log("gov.nasa.controllers.MobileTreeIk", log4cpp::Priority::ERROR, err);
        throw std::runtime_error(err);
    }

    for (size_t i = 0; i < mobileJointValues.size(); ++i)
        mobileJoints(i) = mobileJointValues[i];
}

void MobileTreeIk::doFK(const KDL::JntArray& joints_in, std::map<std::string, KDL::Frame>& frameMap)
{
    // add mobile joints
    KDL::JntArray allJoints(joints_in.rows() + mobileJoints.rows());
    allJoints.data.segment(mobileJoints.rows(), joints_in.rows()) = joints_in.data;
    for (unsigned int i = 0; i < mobileJoints.rows(); ++i)
        allJoints.data[i] = mobileJoints(i);

    fkPosSolverPtr->getPoses(allJoints, frameMap);
}
