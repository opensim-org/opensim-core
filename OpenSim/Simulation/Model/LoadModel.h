#ifndef _LoadModel_h_
#define _LoadModel_h_
// LoadModel.h
// author: Frank C. Anderson, Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include "Model.h"


RDSIMULATION_API void LoadOpenSimLibraries(int argc,char **argv);
RDSIMULATION_API OpenSim::Model* LoadModel(const std::string &aModelLibraryName); 
RDSIMULATION_API OpenSim::Model* LoadModel(int argc,char **argv);


#endif // __Investigation_h__


