#include <nasa_robodyn_controllers_core/MotionLimiter.h>
#include <sstream>

JointMotionLimitHelper::JointMotionLimitHelper(bool positiveOnly_in)
    : positiveOnly(positiveOnly_in)
{
}

JointMotionLimitHelper::JointMotionLimitHelper(const JointMotionLimitHelper& toCopy)
{
    positiveOnly = toCopy.getPositiveOnly();
    limArray     = toCopy.getLimits();
}

JointMotionLimitHelper::~JointMotionLimitHelper()
{
}

void JointMotionLimitHelper::setLimit(double lim)
{
    if (positiveOnly && lim < 0.)
    {
        //RCS::Logger::getCategory("gov.nasa.controllers.JointMotionLimitHelper") << log4cpp::Priority::ERROR << "setLimit lim must not be < 0";
        throw std::invalid_argument("JointMotionLimitHelper::setLimit lim must not be < 0");
    }

    for (unsigned int i = 0; i < limArray.size(); ++i)
    {
        limArray[i] = lim;
    }
}

void JointMotionLimitHelper::setLimits(const std::vector<double>& limArray_in)
{
    limArray.resize(limArray_in.size());
    for (unsigned int i = 0; i < limArray_in.size(); ++i)
    {
        if (positiveOnly && limArray_in[i] < 0.)
        {
            //RCS::Logger::getCategory("gov.nasa.controllers.JointMotionLimitHelper") << log4cpp::Priority::ERROR << "setLimits lim must not be < 0";
            throw std::invalid_argument("JointMotionLimitHelper::setLimits lim must not be < 0");
        }

        limArray[i] = limArray_in[i];
    }
}

double JointMotionLimitHelper::getLimit(unsigned int index) const
{
    if (limArray.size() > index)
    {
        return limArray[index];
    }
    else
    {
        //RCS::Logger::getCategory("gov.nasa.controllers.JointMotionLimitHelper") << log4cpp::Priority::ERROR << "getLimit index out of range";
        throw std::out_of_range("JointMotionLimitHelper::getLimit index out of range");
    }
}

JointPositionLimiter::JointPositionLimiter(const JointPositionLimiter& toCopy)
{
    setLimits(toCopy.getMinPositions(), toCopy.getMaxPositions());
}

void JointPositionLimiter::setLimits(const std::vector<double>& minLimits, const std::vector<double>& maxLimits)
{
    if (minLimits.size() != maxLimits.size())
    {
        //RCS::Logger::getCategory("gov.nasa.controllers.JointPositionLimiter") << log4cpp::Priority::ERROR << "setPositionLimits min/max length mismatch";
        throw std::out_of_range("JointPositionLimiter::setPositionLimits min/max length mismatch");
    }
    minLimiter.setLimits(minLimits);
    maxLimiter.setLimits(maxLimits);
}

JointVelocityLimiter::JointVelocityLimiter(const JointVelocityLimiter& toCopy)
{
    setVelocityLimits(toCopy.getVelocityLimits());
}

JointMotionLimiter::JointMotionLimiter(const JointMotionLimiter& toCopy)
    : JointVelocityLimiter()
{
    setLimits(toCopy.getVelocityLimits(), toCopy.getAccelerationLimits());
}

void JointMotionLimiter::setLimits(const std::vector<double>& velLimits, const std::vector<double>& accLimits)
{
    if (velLimits.size() != accLimits.size())
    {
        //RCS::Logger::getCategory("gov.nasa.controllers.JointMotionLimiter") << log4cpp::Priority::ERROR << "setLimits vel/acc length mismatch";
        throw std::out_of_range("JointMotionLimiter::setLimits vel/acc length mismatch");
    }
    setVelocityLimits(velLimits);
    accMotionLimiter.setLimits(accLimits);
}

void JointNamePositionLimiter::setLimits(const std::vector<std::string>& jointNames,
        const std::vector<double>& minPositionArray_in, const std::vector<double>& maxPositionArray_in)
{
    if (jointNames.size() != minPositionArray_in.size() || jointNames.size() != maxPositionArray_in.size())
    {
        // length mismatch
        //RCS::Logger::getCategory("gov.nasa.controllers.JointNamePositionLimiter") << log4cpp::Priority::DEBUG << "setLimits() - jointNames and limit arrays have different lengths";
    }

    if (jointNames != joints)
    {
        jointLimitIndices.clear();
        for (unsigned int i = 0; i < jointNames.size(); ++i)
        {
            jointLimitIndices[jointNames[i]] = i;
        }
        joints = jointNames;
    }

    positionLimiter->setLimits(minPositionArray_in, maxPositionArray_in);
}

void JointNamePositionLimiter::reorderLimits(const std::vector<std::string>& jointNames)
{
    //! do we need to do anything?
    if (joints == jointNames)
    {
        return;
    }

    //! first collect the limits specified by jointNames
    joints.resize(jointLimitIndices.size());
    std::vector<double> newMinPositions(jointLimitIndices.size());
    std::vector<double> newMaxPositions(jointLimitIndices.size());
    std::map<std::string, unsigned int> oldIndices = jointLimitIndices;
    jointLimitIndices.clear();
    unsigned int newIndex = 0;
    for (unsigned int i = 0; i < jointNames.size(); ++i)
    {
        std::map<std::string, unsigned int>::iterator it = oldIndices.find(jointNames[i]);
        if (it == oldIndices.end())
        {
            // not there
            std::stringstream err;
            err << "reorderLimits() - no limit available for " << jointNames[i];
            //RCS::Logger::getCategory("gov.nasa.controllers.JointNamePositionLimiter") << log4cpp::Priority::ERROR << err.str();
            throw std::invalid_argument(err.str());
        }
        else
        {
            joints[newIndex]                 = jointNames[i];
            jointLimitIndices[jointNames[i]] = newIndex;
            newMinPositions[newIndex]        = positionLimiter->getMinPosition(it->second);
            newMaxPositions[newIndex]        = positionLimiter->getMaxPosition(it->second);
            oldIndices.erase(it);
            ++newIndex;
        }
    }

    //! add any others that were in the map
    for (std::map<std::string, unsigned int>::iterator it = oldIndices.begin(); it != oldIndices.end(); ++it)
    {
        joints[newIndex]             = it->first;
        jointLimitIndices[it->first] = newIndex;
        newMinPositions[newIndex]    = positionLimiter->getMinPosition(it->second);
        newMaxPositions[newIndex]    = positionLimiter->getMaxPosition(it->second);
        ++newIndex;
    }

    positionLimiter->setLimits(newMinPositions, newMaxPositions);
}

double JointNamePositionLimiter::getMinPosition(const std::string& jointName) const
{
    std::map<std::string, unsigned int>::const_iterator mapIt = jointLimitIndices.find(jointName);
    if (mapIt != jointLimitIndices.end())
    {
        return positionLimiter->getMinPosition(mapIt->second);
    }
    else
    {
        throw std::runtime_error((std::string("JointNamePositionLimiter::getMinPosition: no limit available for ") + jointName).c_str());
        return 0.;
    }
}

double JointNamePositionLimiter::getMaxPosition(const std::string& jointName) const
{
    std::map<std::string, unsigned int>::const_iterator mapIt = jointLimitIndices.find(jointName);
    if (mapIt != jointLimitIndices.end())
    {
        return positionLimiter->getMaxPosition(mapIt->second);
    }
    else
    {
        throw std::runtime_error((std::string("JointNamePositionLimiter::getMaxPosition: no limit available for ") + jointName).c_str());
        return 0.;
    }
}

bool JointNamePositionLimiter::getLimits(const std::string& jointName, std::pair<double, double>& limits) const
{
    std::map<std::string, unsigned int>::const_iterator mapIt = jointLimitIndices.find(jointName);
    if (mapIt != jointLimitIndices.end())
    {
        limits.first  = positionLimiter->getMinPosition(mapIt->second);
        limits.second = positionLimiter->getMaxPosition(mapIt->second);
        return true;
    }
    else
    {
        return false;
    }
}

bool JointNamePositionLimiter::limit(const std::string &jointName, double &value) const
{
    std::map<std::string, unsigned int>::const_iterator mapIt = jointLimitIndices.find(jointName);
    if (mapIt != jointLimitIndices.end())
    {
        if (value < positionLimiter->getMinPosition(mapIt->second))
        {
            value = positionLimiter->getMinPosition(mapIt->second);
            return true;
        }
        if (value > positionLimiter->getMaxPosition(mapIt->second))
        {
            value = positionLimiter->getMaxPosition(mapIt->second);
            return true;
        }
    }
    else
    {
        throw std::runtime_error((std::string("JointNamePositionLimiter::limit: no limits available for ") + jointName).c_str());
        return false;
    }

    return false;
}

void JointNameMotionLimiter::setLimits(const std::vector<std::string>& jointNames,
                                       const std::vector<double>& velLimitArrayIn, const std::vector<double>& accLimitArrayIn)
{
    if (jointNames.size() != velLimitArrayIn.size() || jointNames.size() != accLimitArrayIn.size())
    {
        // length mismatch
        std::stringstream err;
        err << "JointNameMotionLimiter::setLimits() - jointNames and limit arrays have different lengths";
        //RCS::Logger::log("gov.nasa.controllers.JointNameMotionLimiter", log4cpp::Priority::DEBUG, err.str());
    }

    if (jointNames != joints)
    {
        jointLimitIndices.clear();
        for (unsigned int i = 0; i < jointNames.size(); ++i)
        {
            jointLimitIndices[jointNames[i]] = i;
        }
        joints = jointNames;
    }

    motionLimiter->setLimits(velLimitArrayIn, accLimitArrayIn);
}

void JointNameMotionLimiter::setLimits(const std::vector<std::string>& jointNames, double velLim, double accLim)
{
    std::vector<double> velLims(jointNames.size(), velLim);
    std::vector<double> accLims(jointNames.size(), accLim);
    setLimits(jointNames, velLims, accLims);
}

void JointNameMotionLimiter::reorderLimits(const std::vector<std::string>& jointNames)
{
    //! do we need to do anything?
    if (joints == jointNames)
    {
        return;
    }

    //! first collect the limits specified by jointNames
    joints.resize(jointLimitIndices.size());
    std::vector<double> newVelocityLimits(jointLimitIndices.size());
    std::vector<double> newAccelerationLimits(jointLimitIndices.size());
    std::map<std::string, unsigned int> oldIndices = jointLimitIndices;
    jointLimitIndices.clear();
    unsigned int newIndex = 0;
    for (unsigned int i = 0; i < jointNames.size(); ++i)
    {
        std::map<std::string, unsigned int>::iterator it = oldIndices.find(jointNames[i]);
        if (it == oldIndices.end())
        {
            // not there
            std::stringstream err;
            err << "reorderLimits() - no limit available for " << jointNames[i];
            //RCS::Logger::getCategory("gov.nasa.controllers.JointNameMotionLimiter") << log4cpp::Priority::ERROR << err.str();
            throw std::invalid_argument(err.str());
        }
        else
        {
            joints[newIndex]                 = jointNames[i];
            jointLimitIndices[jointNames[i]] = newIndex;
            newVelocityLimits[newIndex]      = motionLimiter->getVelocityLimit(it->second);
            newAccelerationLimits[newIndex]  = motionLimiter->getAccelerationLimit(it->second);
            oldIndices.erase(it);
            ++newIndex;
        }
    }

    //! add any others that were in the map
    for (std::map<std::string, unsigned int>::iterator it = oldIndices.begin(); it != oldIndices.end(); ++it)
    {
        joints[newIndex]                = it->first;
        jointLimitIndices[it->first]    = newIndex;
        newVelocityLimits[newIndex]     = motionLimiter->getVelocityLimit(it->second);
        newAccelerationLimits[newIndex] = motionLimiter->getAccelerationLimit(it->second);
        ++newIndex;
    }

    motionLimiter->setLimits(newVelocityLimits, newAccelerationLimits);
}

double JointNameMotionLimiter::getVelocityLimit(const std::string& jointName) const
{
    std::map<std::string, unsigned int>::const_iterator mapIt = jointLimitIndices.find(jointName);
    if (mapIt != jointLimitIndices.end())
    {
        return motionLimiter->getVelocityLimit(mapIt->second);
    }
    else
    {
        throw std::runtime_error((std::string("JointNameMotionLimiter::getVelocityLimit: no limit available for ") + jointName).c_str());
        return 0.;
    }
}

double JointNameMotionLimiter::getAccelerationLimit(const std::string& jointName) const
{
    std::map<std::string, unsigned int>::const_iterator mapIt = jointLimitIndices.find(jointName);
    if (mapIt != jointLimitIndices.end())
    {
        return motionLimiter->getAccelerationLimit(mapIt->second);
    }
    else
    {
        throw std::runtime_error((std::string("JointNameMotionLimiter::getAccelerationLimit: no limit available for ") + jointName).c_str());
        return 0.;
    }
}

CartesianMotionLimitHelper::CartesianMotionLimitHelper()
{
    setLimits();
}

CartesianMotionLimitHelper::CartesianMotionLimitHelper(const CartesianMotionLimitHelper &toCopy)
{
    linLimit = toCopy.getLinearLimit();
    rotLimit = toCopy.getRotationalLimit();
}

CartesianMotionLimitHelper::~CartesianMotionLimitHelper()
{
}

void CartesianMotionLimitHelper::setLimits(double linLimitIn, double rotLimitIn)
{
    if (linLimitIn >= 0 && rotLimitIn >= 0)
    {
        linLimit = linLimitIn;
        rotLimit = rotLimitIn;
    }
    else
    {
        std::stringstream err;
        err << "CartesianMotionLimitHelper::setLimits() - limits must not be less than 0";
        //RCS::Logger::log("gov.nasa.controllers.CartesianMotionLimitHelper", log4cpp::Priority::ERROR, err.str());
        throw std::invalid_argument(err.str());
    }
}

CartesianVelocityLimiter::CartesianVelocityLimiter(const CartesianVelocityLimiter& toCopy)
{
    setVelocityLimits(toCopy.getLinearVelocityLimit(), toCopy.getRotationalVelocityLimit());
}

CartesianMotionLimiter::CartesianMotionLimiter(const CartesianMotionLimiter& toCopy)
    : CartesianVelocityLimiter(toCopy)
{
    setAccelerationLimits(toCopy.getLinearAccelerationLimit(), toCopy.getRotationalAccelerationLimit());
}

