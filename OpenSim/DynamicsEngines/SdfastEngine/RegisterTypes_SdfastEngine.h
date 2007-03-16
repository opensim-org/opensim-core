#ifndef _RegisterTypes_SdfastEngine_h_
#define _RegisterTypes_SdfastEngine_h_
// RegisterTypes_SdfastEngine.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "SdfastEngineDLL.h"


extern "C" {

SDFAST_ENGINE_API void RegisterTypes_SdfastEngine(); 

}

/**
 * A class intended for the purpose of automatically registering classes defined in this simulation Dll
 */
class SdfastEngineInstantiator 
{ 
public: 
        SdfastEngineInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    

#endif // __RegisterTypes_SdfastEngine_h__


