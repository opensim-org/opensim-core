#ifndef _RegisterTypes_osimSimmKinematicsEngine_h_
#define _RegisterTypes_osimSimmKinematicsEngine_h_
// RegisterTypes_osimSimmKinematicsEngine.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "osimSimmKinematicsEngineDLL.h"


extern "C" {

OSIMSIMMKINEMATICSENGINE_API void RegisterTypes_osimSimmKinematicsEngine(); 

}

/**
 * A class intended for the purpose of automatically registering classes defined in this simulation Dll
 */
class osimSimmKinematicsEngineInstantiator 
{ 
public: 
        osimSimmKinematicsEngineInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    

#endif // __RegisterTypes_osimSimmKinematicsEngine_h__


