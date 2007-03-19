#ifndef _RegisterTypes_Actuators_h_
#define _RegisterTypes_Actuators_h_
// RegisterTypes.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "osimActuatorsDLL.h"


extern "C" {

OSIMACTUATORS_API void RegisterTypes_Actuators(); 

}

class osimActuatorsInstantiator
{
public:
       osimActuatorsInstantiator();
private:
       void registerDllClasses();
};

#endif // __RegisterTypes_Actuators_h__


