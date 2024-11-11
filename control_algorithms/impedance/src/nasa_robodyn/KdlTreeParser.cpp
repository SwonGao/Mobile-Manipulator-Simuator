#include "nasa_robodyn_controllers_core/KdlTreeParser.h"

KdlTreeParser::KdlTreeParser()
{
}

KdlTreeParser::~KdlTreeParser()
{
}

void KdlTreeParser::loadFromFile(const std::string &fileName)
{
    if (!kdl_parser::treeFromFile(fileName, tree))
    {
        std::stringstream err;
        err << "KdlTreeParser::loadFromFile could not load file " << fileName;
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeParser", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }

    initialize();
}

void KdlTreeParser::loadFromParam(const std::string &paramName)
{
    if (!kdl_parser::treeFromParam(paramName, tree))
    {
        std::stringstream err;
        err << "KdlTreeParser::loadFromParam could not load parameter " << paramName;
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeParser", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
    initialize();
}

void KdlTreeParser::loadFromModel(const urdf::Model model)
{
    if (!kdl_parser::treeFromUrdfModel(model, tree))
    {
        std::stringstream err;
        err << "KdlTreeParser::loadFromModel could not load model";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeParser", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }

    initialize();
}

void KdlTreeParser::loadFromChain(const KDL::Chain chain, const std::string& rootname){
    //std::cout << "6666" << std::endl;
    //std::cout << chain.getNrOfJoints() << std::endl;
    //std::cout << "6666" << std::endl;
    //std::cout << tree.getNrOfSegments() << std::endl;
    //for(std::size_t i = 0; i < chain.getNrOfSegments(); i++){
    //  std::cout << chain.getSegment(i).getName() << std::endl;
    //}
    if (!tree.addChain(chain, rootname)){
        std::stringstream err;
        err << "KdlTreeParser::loadFromChain could not load chain";
        //RCS::Logger::log("gov.nasa.controllers.KdlTreeParser", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
        return;
    }
    //std::cout << tree.getNrOfJoints();
    initialize();
}

