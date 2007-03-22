#ifndef _LoadModel_h_
#define _LoadModel_h_
// LoadModel.h
// author: Frank C. Anderson, Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim
{
class Model;

OSIMSIMULATION_API Model* LoadModel(const std::string &aModelLibraryName, const std::string &aModelFileName=""); 
}

#endif // __LoadModel_h__
