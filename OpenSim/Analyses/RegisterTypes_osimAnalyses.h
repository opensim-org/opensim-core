#ifndef _RegisterTypes_Analyses_h_
#define _RegisterTypes_Analyses_h_
// RegisterTypes_Analyses.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "osimAnalysesDLL.h"


extern "C" {

OSIMANALYSES_API void RegisterTypes_Analyses(); 

}

class suAnalysesInstantiator 
{ 
public: 
        suAnalysesInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    

#endif // __RegisterTypes_Analyses_h__


