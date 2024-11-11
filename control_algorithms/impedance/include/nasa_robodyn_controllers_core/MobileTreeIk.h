/**
 * @file MobileTreeIk.h
 * @brief Defines the MobileTreeIk class.
 * @author Ross Taylor
 */
#ifndef MOBILE_TREE_IK_H
#define MOBILE_TREE_IK_H

#include "nasa_robodyn_controllers_core/KdlTreeTr.h"

class MobileTreeIk : public KdlTreeTr
{
public:
    static const int BASE;

    MobileTreeIk();
    ~MobileTreeIk();

    void setBases(const std::vector<std::string>& bases_in);

    void resetMobileJoints();

    /**
     * @brief getJoints
     * @param joints_in input joints
     * @param nodeNames input frame names
     * @param nodeFrames input frames
     * @param joints_out output joints
     * @param nodePriorities input node priorities (6 * nodeNames.size())
     */
    virtual void getJointPositions(const KDL::JntArray& joints_in, const std::vector<std::string>& nodeNames,
                   const std::vector<KDL::Frame>& nodeFrames, KDL::JntArray& joints_out,
                   const std::vector<NodePriority>& nodePriorities);

    inline virtual std::string getBaseName() const {return robotModelBase;}
    virtual void getJointNames(std::vector<std::string>& jointNames) const;
    inline virtual unsigned int getJointCount() const { return tree.getNrOfJoints() - 6; }
//    void setJointInertias(const KDL::JntArray &inertia_in);
    virtual void getFrames (const KDL::JntArray& joints_in, std::map<std::string, KDL::Frame>& frameMap);
     void setPriorityTol(const std::map<int, std::pair<double, double> >& priority_tol)
     {
         KdlTreeTr::setPriorityTol(priority_tol);
         priorityTolMap[MobileTreeIk::BASE] = std::make_pair(eps, eps);
     }
	 
	 // Added by Ryan Luna
	 void getMobileJoints(std::vector<double>& mobileJointValues) const;
     void setMobileJoints(const std::vector<double>& mobileJointValues);
     void doFK(const KDL::JntArray& joints_in, std::map<std::string, KDL::Frame>& frameMap);

protected:
    virtual void initialize();

private:
    KDL::JntArray mobileJoints;
    std::vector<std::string> bases;
    std::string robotModelBase;

};

#endif
