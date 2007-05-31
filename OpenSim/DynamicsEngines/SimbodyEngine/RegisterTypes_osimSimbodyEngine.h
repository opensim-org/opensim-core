#ifndef _RegisterTypes_osimSimbodyEngine_h_
#define _RegisterTypes_osimSimbodyEngine_h_
// RegisterTypes_SimbodyEngine.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "osimSimbodyEngineDLL.h"


extern "C" {

OSIMSIMBODYENGINE_API void RegisterTypes_SimbodyEngine(); 

}

/**
 * A class intended for the purpose of automatically registering classes defined in this simulation Dll
 */
class osimSimbodyEngineInstantiator 
{ 
public: 
        osimSimbodyEngineInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    

#endif // __RegisterTypes_SimbodyEngine_h__


