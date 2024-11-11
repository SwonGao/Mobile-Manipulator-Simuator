/**
 * @file MotionLimiter.h
 * @brief Defines the motion limiter classes.
 * @author Ross Taylor
 * @date Aug 30, 2012
 */

#ifndef MOTION_LIMITER_H
#define MOTION_LIMITER_H

#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <stdexcept>
//#include "nasa_common_utilities/Logger.h"

/**
 * @class JointMotionLimitHelper
 * @brief Helper for maintaining joint limits
 */
class JointMotionLimitHelper
{
public:
    JointMotionLimitHelper(bool positiveOnly_in = true);
    JointMotionLimitHelper(const JointMotionLimitHelper& tocopy);
    virtual ~JointMotionLimitHelper();

    /**
     * @brief set limit for each joint
     * @param limArray_in limit array for joints
     * @return void
     * @details sets an array of joint limits.
     */
    void setLimits(const std::vector<double>& limArray_in);

    /**
     * @brief set limit for all joints
     * @param lim is limit to set
     * @return void
     */
    void setLimit(double lim);

    /**
     * @brief getLimit get limit
     * @throws std:out_of_range if index is invalid
     */
    double getLimit(unsigned int index) const;
    inline const std::vector<double>& getLimits() const {return limArray;}
    inline bool getPositiveOnly() const {return positiveOnly;}

private:
    std::vector<double> limArray;
    bool positiveOnly;
};

/**
 * @class JointPositionLimiter
 * @brief wrapper around JointMotionLimiter for positions
 */
class JointPositionLimiter
{
public:
    JointPositionLimiter() : maxLimiter(false), minLimiter(false) {}
    JointPositionLimiter(const JointPositionLimiter& tocopy);
    virtual ~JointPositionLimiter(){}

    void setLimits(const std::vector<double>& minLimits, const std::vector<double>& maxLimits);
    inline double getMaxPosition(unsigned int index) const {return maxLimiter.getLimit(index);}
    inline const std::vector<double>& getMaxPositions() const {return maxLimiter.getLimits();}
    inline double getMinPosition(unsigned int index) const {return minLimiter.getLimit(index);}
    inline const std::vector<double>& getMinPositions() const {return minLimiter.getLimits();}

private:
    JointMotionLimitHelper maxLimiter;
    JointMotionLimitHelper minLimiter;
};

/**
 * @class JointVelocityLimiter
 * @brief wrapper around JointMotionLimiter for velocities
 */
class JointVelocityLimiter
{
public:
    JointVelocityLimiter() {}
    JointVelocityLimiter(const JointVelocityLimiter& tocopy);
    virtual ~JointVelocityLimiter(){}

    inline void setVelocityLimits(const std::vector<double>& limArray_in) {velMotionLimiter.setLimits(limArray_in);}
    inline void setVelocityLimit(double lim) {velMotionLimiter.setLimit(lim);}
    inline double getVelocityLimit(unsigned int index) const {return velMotionLimiter.getLimit(index);}
    inline const std::vector<double>& getVelocityLimits() const {return velMotionLimiter.getLimits();}

private:
    JointMotionLimitHelper velMotionLimiter;
};

class JointMotionLimiter : public JointVelocityLimiter
{
public:
    JointMotionLimiter() {}
    JointMotionLimiter(const JointMotionLimiter& tocopy);
    virtual ~JointMotionLimiter() {}

    void setLimits(const std::vector<double>& velLimits, const std::vector<double>& accLimits);

    inline void setAccelerationLimit(double lim) {accMotionLimiter.setLimit(lim);}
    inline double getAccelerationLimit(unsigned int index) const {return accMotionLimiter.getLimit(index);}
    inline const std::vector<double>& getAccelerationLimits() const {return accMotionLimiter.getLimits();}

private:
    JointMotionLimitHelper accMotionLimiter;

    // override with private so this can't be called; this helps guarantee vel/acc limits maintain the appropriate lengths
    inline void setVelocityLimits(const std::vector<double>& limArray_in) {JointVelocityLimiter::setVelocityLimits(limArray_in);}
};

/**
 * @class JointNamePositionLimiter
 * @brief wrapper to add position limiting by joint name to a JointPositionLimiter
 */
class JointNamePositionLimiter
{
public:
    JointNamePositionLimiter() {}
    virtual ~JointNamePositionLimiter() {}

    void setJointPositionLimiter(boost::shared_ptr<JointPositionLimiter> positionLimiter_in)
    {
        positionLimiter = positionLimiter_in;
    }

    /**
     * @brief set limits based on joint_name
     * @param jointNames vector of joint names
     * @param limArray_in vector of joint limits
     * @return void
     * @exception logic error jointNames and limArray_in have different lengths
     */
    void setLimits(const std::vector<std::string>& jointNames, const std::vector<double>& minPositionArray_in, const std::vector<double>& maxPositionArray_in);

    /**
     * @brief reorder limits based on joint_name
     * @param jointNames vector of joint names
     * @return void
     * @details reorders the array of of jointlimits to match the order in jointNames
     */
    void reorderLimits(const std::vector<std::string>& jointNames);

    inline bool hasLimits(const std::string& jointName) const {return jointLimitIndices.find(jointName) != jointLimitIndices.end();}
    double getMinPosition(const std::string& jointName) const;
    double getMaxPosition(const std::string& jointName) const;
    bool getLimits(const std::string& jointName, std::pair<double, double>& limits) const;

    bool limit(const std::string& jointName, double& value) const;

private:
    JointNamePositionLimiter(const JointNamePositionLimiter&) {}
    boost::shared_ptr<JointPositionLimiter> positionLimiter;
    // store index into limits array
    std::map<std::string, unsigned int> jointLimitIndices;
    // store joints vector for setLimits to save cycles if it hasn't changed
    std::vector<std::string> joints;
};

/**
 * @class JointNameMotionLimiter
 * @brief wrapper to add motion limiting by joint name to a JointMotionLimiter
 */
class JointNameMotionLimiter
{
public:
    JointNameMotionLimiter() {}
    virtual ~JointNameMotionLimiter() {}

    void setJointMotionLimiter(boost::shared_ptr<JointMotionLimiter> motionLimiter_in)
    {
        motionLimiter = motionLimiter_in;
    }

    /**
     * @brief set limits based on joint_name
     * @param jointNames vector of joint names
     * @param limArray_in vector of joint limits
     * @return void
     * @exception logic error jointNames and limArray_in have different lengths
     */
    void setLimits(const std::vector<std::string>& jointNames, const std::vector<double>& velLimitArray_in, const std::vector<double>& accLimitArray_in);
    /**
     * @brief set all joint limits for jointNames to the values
     * @param velLim velocity limit
     * @param accLim acceleration limit
     * @return void
     */
    void setLimits(const std::vector<std::string>& jointNames, double velLim, double accLim);

    /**
     * @brief reorder Motion limits based on joint_name
     * @param jointNames vector of joint names
     * @return void
     * @details reorders the array of of jointlimits to match the order in jointNames
     */
    void reorderLimits(const std::vector<std::string>& jointNames);

    double getVelocityLimit(const std::string& jointName) const;
    double getAccelerationLimit(const std::string& jointName) const;

private:
    JointNameMotionLimiter(const JointNamePositionLimiter&) {}
    boost::shared_ptr<JointMotionLimiter> motionLimiter;
    // store index into limits array
    std::map<std::string, unsigned int> jointLimitIndices;
    // store joints vector for setLimits to save cycles if it hasn't changed
    std::vector<std::string> joints;
};

/**
 * @class CartesianMotionLimitHelper
 * @brief Helper for maintaining Cartesian Motion limits
 */
class CartesianMotionLimitHelper
{
public:
    CartesianMotionLimitHelper();
    CartesianMotionLimitHelper(const CartesianMotionLimitHelper& tocopy);
    virtual ~CartesianMotionLimitHelper();

    /**
     * @brief set limits
     * @param linLimit_in maximum linear Motion
     * @param rotLimit_in maximum roational Motion
     * @return void
     * @exception invalid_argument limits must not be less than 0
     */
    void setLimits(double linLimit_in = .10, double rotLimit_in = 0.50);

    inline double getLinearLimit() const {return linLimit;}
    inline double getRotationalLimit() const {return rotLimit;}

private:
    double linLimit;
    double rotLimit;
};

/**
 * @class CartesianVelocityLimiter
 * @brief wrapper for CartesianMotionLimiter for velocity
 */
class CartesianVelocityLimiter
{
public:
    CartesianVelocityLimiter() {}
    CartesianVelocityLimiter(const CartesianVelocityLimiter& tocopy);
    virtual ~CartesianVelocityLimiter() {}
    inline void setVelocityLimits(double linLimit_in, double rotLimit_in) {motionLimiter.setLimits(linLimit_in, rotLimit_in);}
    inline double getLinearVelocityLimit() const {return motionLimiter.getLinearLimit();}
    inline double getRotationalVelocityLimit() const {return motionLimiter.getRotationalLimit();}

private:
    CartesianMotionLimitHelper motionLimiter;
};

class CartesianMotionLimiter : public CartesianVelocityLimiter
{
public:
    CartesianMotionLimiter() {}
    CartesianMotionLimiter(const CartesianMotionLimiter& tocopy);
    virtual ~CartesianMotionLimiter() {}

    inline void setAccelerationLimits(double linLimit_in, double rotLimit_in) {accMotionLimiter.setLimits(linLimit_in, rotLimit_in);}
    inline double getLinearAccelerationLimit() const {return accMotionLimiter.getLinearLimit();}
    inline double getRotationalAccelerationLimit() const {return accMotionLimiter.getRotationalLimit();}

private:
    CartesianMotionLimitHelper accMotionLimiter;
};

#endif
