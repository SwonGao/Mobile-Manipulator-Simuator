#include <nasa_robodyn_controllers_core/KdlTreeUtilities.h>

KdlTreeUtilities::KdlTreeUtilities()
{
}

KdlTreeUtilities::~KdlTreeUtilities()
{
}

void KdlTreeUtilities::getChainJointNames(const KDL::Chain& chain,
        std::vector<std::string>& jointNames) const
{
    jointNames.resize(chain.getNrOfJoints());
    const std::vector<KDL::Segment>&   segments = chain.segments;
    std::vector<std::string>::iterator nameIt   = jointNames.begin();
    for (unsigned int i = 0; i < segments.size(); ++i)
    {
        const KDL::Joint& joint = segments[i].getJoint();
        if (joint.getType() != KDL::Joint::None)
        {
            //is a joint
            *nameIt = joint.getName();
            ++nameIt;
        }
    }
}

void KdlTreeUtilities::getChainJointNames(const std::string& toolFrame,
        std::vector<std::string>& jointNames) const
{
    getChainJointNames(getBaseName(), toolFrame, jointNames);
}

void KdlTreeUtilities::getChainJointNames(const std::string& baseFrame, const std::string& toolFrame,
        std::vector<std::string>& jointNames) const
{
    KDL::Chain chain;
    if (!getChain(baseFrame, toolFrame, chain))
    {
        jointNames.clear();
        return;
    }
    getChainJointNames(chain, jointNames);
}

void KdlTreeUtilities::getJointNames(std::vector<std::string>& jointNames) const
{
    jointNames.resize(tree.getNrOfJoints());
    const KDL::SegmentMap& segments = tree.getSegments();
    for (KDL::SegmentMap::const_iterator segIt = segments.begin(); segIt != segments.end(); ++segIt)
    {
        const KDL::TreeElement& element = segIt->second;
        const KDL::Joint&       joint   = element.segment.getJoint();
        if (joint.getType() != KDL::Joint::None)
        {
            // is a joint
            jointNames[element.q_nr] = joint.getName();
        }
    }
}


 bool KdlTreeUtilities::getChain(const std::string& baseFrame, const std::string& toolFrame, KDL::Chain &chain) const
 {
     if(baseFrame == toolFrame)
     {
         chain = KDL::Chain();
         return false;
     }
     return (tree.getChain(baseFrame, toolFrame, chain));
 }
/** @brief adds a segment to the tree
  * @param segment      The segment to add
  * @param hookName     The reference (parent) frame to add the segment to
  * @throws std::runtime_error  if a segment with the same name has already been added
  * @throws std::runtime_error  if hookName does not exist in the current tree
  */
void KdlTreeUtilities::addSegment(const KDL::Segment &segment, const std::string &hookName )
{
    if(hasSegment(segment.getName()))
    {
        std::stringstream err;
        err << "KdlTreeUtilities::addSegment could not add " << segment.getName() << ". A segment with that name already exists in the tree.";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeUtilities", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }
    if(!hasSegment(hookName))
    {
        std::stringstream err;
        err << "KdlTreeUtilities::addSegment could not add " << segment.getName() << " to "<<hookName<<"--"<<hookName<<" does not exist in the tree.";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeUtilities", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }

    tree.addSegment(segment, hookName);
}

/** @brief removes a segment and all its children from the tree
  * @param segName      The name of the segment to remove
  * @returns false      If segName is not in the current tree
  */
bool KdlTreeUtilities::removeSegment(const std::string &segName)
{
    KDL::Tree t(tree.getRootSegment()->first);
    bool success = this->removeSegRecursive(t, tree.getRootSegment(), tree.getRootSegment()->first, segName);
    tree = t;
    return success;
}

bool KdlTreeUtilities::removeSegRecursive(KDL::Tree &tree, KDL::SegmentMap::const_iterator root, const std::string& hook_name, const std::string& seg_remove_name)
{
    //get iterator for root-segment
    KDL::SegmentMap::const_iterator child;
    //try to add all of root's children
    for (unsigned int i = 0; i < root->second.children.size(); i++)
    {
        child = root->second.children[i];
        if(child->second.segment.getName()!=seg_remove_name)
        {
            //Try to add the child
            if (tree.addSegment(child->second.segment, hook_name))
            {
                //if child is added, add all the child's children
                if (!(this->removeSegRecursive(tree, child, child->first, seg_remove_name)))
                    //if it didn't work, return false
                    return false;
            }
            else
            {
                std::stringstream err;
                err << "KdlTreeUtilities::removeSegRecursive could not add child" << child->second.segment.getName();
                //RCS::Logger::log("gov.nasa.controllers.KdlTreeUtilities", log4cpp::Priority::ERROR, err.str());
                throw std::runtime_error(err.str());
            }
        }
    }
    return true;
}
