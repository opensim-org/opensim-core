#ifndef _LoadModel_h_
#define _LoadModel_h_
// LoadModel.h
// author: Frank C. Anderson, Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include "AbstractModel.h"


OSIMSIMULATION_API void LoadOpenSimLibraries(int argc,char **argv);
OSIMSIMULATION_API OpenSim::AbstractModel* LoadModel(const std::string &aModelLibraryName, const std::string &aModelFileName=""); 


#endif // __Investigation_h__


