#ifndef _RegisterTypes_osimSdfastEngine_h_
#define _RegisterTypes_osimSdfastEngine_h_
// RegisterTypes_SdfastEngine.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "osimSdfastEngineDLL.h"


extern "C" {

OSIMSDFASTENGINE_API void RegisterTypes_SdfastEngine(); 

}

/**
 * A class intended for the purpose of automatically registering classes defined in this simulation Dll
 */
class osimSdfastEngineInstantiator 
{ 
public: 
        osimSdfastEngineInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    

#endif // __RegisterTypes_SdfastEngine_h__


