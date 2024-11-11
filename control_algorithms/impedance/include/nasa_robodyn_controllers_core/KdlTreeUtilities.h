#ifndef KDL_TREE_UTILITIES_H
#define KDL_TREE_UTILITIES_H

#include <kdl/jntarray.hpp>
#include <nasa_robodyn_controllers_core/KdlTreeParser.h>
#include <iostream>

class KdlTreeUtilities : public KdlTreeParser
{
public:
    KdlTreeUtilities();
    ~KdlTreeUtilities();

    /**
     * @brief get base of tree. Virtual so derived classes can publish a different base if desired.
     */
    inline virtual std::string getBaseName() const
    {
        return tree.getRootSegment()->first;
    }

    /**
     * @brief get joint names in tree. Virtual so derived classes can manipulate the published names if desired.
     */
    virtual void getJointNames(std::vector<std::string>& jointNames) const;
    /**
     * @brief get joint count of tree. Virtual so derived classes can publish a different number if desired.
     */
    inline virtual unsigned int getJointCount() const
    {
        return tree.getNrOfJoints();
    }

    /**
     * @brief get joint names in chain.
     */
    void getChainJointNames(const KDL::Chain& chain,
                       std::vector<std::string>& jointNames) const;

    /**
     * @brief get joint names in chain from base to toolFrame. Virtual so derived classes can manipulate the published names if desired.
     */
    virtual void getChainJointNames(const std::string& toolFrame,
                       std::vector<std::string>& jointNames) const;

    /**
     * @brief get joint names in chain from baseFrame to toolFrame. Virtual so derived classes can manipulate the published names if desired.
     */
    virtual void getChainJointNames(const std::string& baseFrame, const std::string& toolFrame,
                       std::vector<std::string>& jointNames) const;
    

    bool hasSegment(const std::string& segName) const
    {
        return getTree().getSegment(segName) != getTree().getSegments().end();
    }

    /**
     * @brief get chain from baseFrame to toolFrame. Virtual so derived classes can manipulate the chain if desired.
     */
    virtual bool getChain(const std::string& baseFrame, const std::string& toolFrame, KDL::Chain &chain) const;

    virtual void addSegment(const KDL::Segment& segment, const std::string &hookName);

    virtual bool removeSegment(const std::string& segName);

    bool removeSegRecursive(KDL::Tree &tree, KDL::SegmentMap::const_iterator root, const std::string& hook_name, const std::string& seg_remove_name);

protected:

private:

};

#endif
