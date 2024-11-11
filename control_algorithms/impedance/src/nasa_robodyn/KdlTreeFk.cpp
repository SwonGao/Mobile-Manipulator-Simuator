#include <nasa_robodyn_controllers_core/KdlTreeFk.h>

KdlTreeFk::KdlTreeFk()
{
}

KdlTreeFk::~KdlTreeFk()
{
}

void KdlTreeFk::getPose(const KDL::JntArray& joints, const std::string &name, KDL::Frame& frame)
{
    if (fkSolver->JntToCart(joints, frame, name) < 0)
    {
        std::stringstream err;
        err << "KdlTreeFk::getPose() failed to get pose";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeFk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
}

void KdlTreeFk::getPoses(const KDL::JntArray& joints, std::map<std::string, KDL::Frame>& frames)
{
    //std::cout << "in" << (int) joints.rows() << "vs" << (int) tree.getNrOfJoints();
    if (joints.rows() != tree.getNrOfJoints())
    {

        std::stringstream err;
        err << "getPoses() joints not properly sized";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeFk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
    KDL::Frame baseFrame = this->baseFrame;
    if (frames.empty())
    {
        recursiveGetPoses(true, baseFrame, tree.getRootSegment(), joints, frames);
    }
    else
    {
        //  make sure all of the listed frames are available
        // if non existant frames are requested, this is the only way to find out
        for (std::map<std::string, KDL::Frame>::const_iterator it = frames.begin(); it != frames.end(); ++it)
        {
            if (tree.getSegment(it->first) == tree.getSegments().end())
            {
                std::stringstream err;
                err << "KdlTreeFk::getPoses() requested an unavailable pose";
                //RCS::Logger::log("gov.nasa.controllers.KdlTreeFk", log4cpp::Priority::ERROR, err.str());
                throw std::runtime_error(err.str());
                return;
            }
        }
        recursiveGetPoses(false, baseFrame, tree.getRootSegment(), joints, frames);
    }
}

void KdlTreeFk::getVelocities(const KDL::JntArrayVel& joints, std::map<std::string, KDL::FrameVel>& frames)
{
    if (joints.q.rows() != tree.getNrOfJoints() || joints.qdot.rows() != tree.getNrOfJoints())
    {
        std::stringstream err;
        err << "getVelocities() joints not properly sized";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeFk", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
    KDL::FrameVel baseFrame = this->base_VelFrame;
    if (frames.empty())
    {
        recursiveGetVels(true, baseFrame, tree.getRootSegment(), joints, frames);
    }
    else
    {
        /// make sure all of the listed frames are available
        // if non existant frames are requested, this is the only way to find out
        for (std::map<std::string, KDL::FrameVel>::const_iterator it = frames.begin(); it != frames.end(); ++it)
        {
            if (tree.getSegment(it->first) == tree.getSegments().end())
            {
                std::stringstream err;
                err << "KdlTreeFk::getPoses() requested an unavailable pose";
                //RCS::Logger::log("gov.nasa.controllers.KdlTreeFk", log4cpp::Priority::ERROR, err.str());
                throw std::runtime_error(err.str());
                return;
            }
        }
        recursiveGetVels(false, baseFrame, tree.getRootSegment(), joints, frames);
    }
}

void KdlTreeFk::initialize()
{
    fkSolver.reset(new KDL::TreeFkSolverPos_recursive(tree));
}

void KdlTreeFk::recursiveGetPoses(bool getAll, const KDL::Frame &baseFrame,
                                  const KDL::SegmentMap::const_iterator &it, const KDL::JntArray &joints, std::map<std::string, KDL::Frame> &frames)
{
    /// get frame for this segment and store as needed
    const std::string&      segmentName  = it->first;
    const KDL::TreeElement& element      = it->second;
    const KDL::Segment&     segment      = element.segment;
    KDL::Frame              currentFrame = baseFrame * (segment.pose(joints(element.q_nr)));
    if (getAll)
    {
        frames[segmentName] = currentFrame;
    }
    else
    {
        std::map<std::string, KDL::Frame>::iterator frameIt = frames.find(segmentName);
        if (frameIt != frames.end())
        {
            frameIt->second = currentFrame;
        }
    }

    /// recurse
    const std::vector<KDL::SegmentMap::const_iterator>& children = element.children;
    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIt = children.begin();
            childIt != children.end(); ++childIt)
    {
        recursiveGetPoses(getAll, currentFrame, *childIt, joints, frames);
    }
}

void KdlTreeFk::recursiveGetVels(bool getAll, const KDL::FrameVel &baseFrame,
                                 const KDL::SegmentMap::const_iterator &it, const KDL::JntArrayVel &joints, std::map<std::string, KDL::FrameVel> &frames)
{
    /// get FrameVel for this segment and store as needed
    const std::string& segmentName  = it->first;
    const KDL::TreeElement& element = it->second;
    const KDL::Segment& segment     = element.segment;
    unsigned int q_nr               = element.q_nr;
    KDL::FrameVel currentFrame      = baseFrame * KDL::FrameVel(segment.pose(joints.q(q_nr)),
                                        segment.twist(joints.q(q_nr), joints.qdot(q_nr)));
    if (getAll){
        frames[segmentName] = currentFrame;
    }
    else{
        std::map<std::string, KDL::FrameVel>::iterator frameIt = frames.find(segmentName);
        if (frameIt != frames.end()){
            frameIt->second = currentFrame;
        }
    }

    /// recurse
    const std::vector<KDL::SegmentMap::const_iterator>& children = element.children;
    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIt = children.begin();
            childIt != children.end(); ++childIt)
    {
        recursiveGetVels(getAll, currentFrame, *childIt, joints, frames);
    }
}

