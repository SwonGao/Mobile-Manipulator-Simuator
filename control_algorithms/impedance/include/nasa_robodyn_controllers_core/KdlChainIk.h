#ifndef KDL_CHAIN_IK_H
#define KDL_CHAIN_IK_H

#include <map>
#include <vector>
#include <memory>

#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>

class KdlChainIk
{
public:
    KdlChainIk(const KDL::Chain& _chain);
    ~KdlChainIk();

    const std::string& getBaseFrame() const;
    void getJointNames(std::vector<std::string>&) const;
    void getJointPositions(const KDL::JntArray& joints_in,
                           const std::vector<KDL::Frame>& tipPositions,
                           std::vector<KDL::JntArray>& joints_out) const;
    void getCurrentPose(const KDL::JntArray& joints_in, KDL::Frame& currPose) const;

private:
    KDL::Chain chain;

    std::auto_ptr<KDL::ChainFkSolverPos> fkPosSolverPtr;
    std::auto_ptr<KDL::ChainIkSolverVel> ikVelSolverPtr;
    std::auto_ptr<KDL::ChainIkSolverPos> ikPosSolverPtr;
};

#endif
