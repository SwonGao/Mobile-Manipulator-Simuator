/**
 * @file KdlTreeFk.h
 * @brief Defines the KdlTreeFk class.
 * @author Ross Taylor
 */
#ifndef KDL_TREE_FK_H
#define KDL_TREE_FK_H

#include <kdl/treefksolver.hpp>
#include <memory>
#include <kdl/jntarrayvel.hpp>
#include <kdl/framevel.hpp>

#include "nasa_robodyn_controllers_core/KdlTreeUtilities.h"
#include <kdl/treefksolverpos_recursive.hpp>
//#include "nasa_common_utilities/Logger.h"

class KdlTreeFk : public KdlTreeUtilities
{
public:
    KdlTreeFk();
    ~KdlTreeFk();

    void getPose(const KDL::JntArray& joints, const std::string &name, KDL::Frame& frame);
    /**
     * @brief getPoses
     * @param joints input joints
     * @param frames output frames
     * @throws std::runtime_error - requested an unavailable pose
     * @details if frames is empty, all segments are returned, otherwise the segments named are returned
     */
    void getPoses(const KDL::JntArray& joints, std::map<std::string, KDL::Frame>& frames);
    /**
     * @brief getVelocities
     * @param joints input joints
     * @param frames output frames
     * @throws std::runtime_error - requested an unavailable pose
     * @details if frames is empty, all segments are returned, otherwise the segments named are returned
     */
    void getVelocities(const KDL::JntArrayVel& joints, std::map<std::string, KDL::FrameVel>& frames);
    
    KDL::Frame baseFrame;
    KDL::FrameVel base_VelFrame;
    
protected:
    virtual void initialize();

    void recursiveGetPoses(bool getAll, const KDL::Frame& baseFrame, const KDL::SegmentMap::const_iterator& it,
                           const KDL::JntArray& joints, std::map<std::string, KDL::Frame>& frames);

    void recursiveGetVels(bool getAll, const KDL::FrameVel& baseFrame, const KDL::SegmentMap::const_iterator& it,
                          const KDL::JntArrayVel& joints, std::map<std::string, KDL::FrameVel>& frames);

private:
    std::auto_ptr<KDL::TreeFkSolverPos> fkSolver;
};

#endif
