#ifndef _RegisterTypes_Actuators_h_
#define _RegisterTypes_Actuators_h_
// RegisterTypes.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Actuators/rdActuatorsDLL.h>


extern "C" {

RDACTUATORS_API void RegisterTypes_Actuators(); 

}

class rdActuatorsInstantiator
{
public:
       rdActuatorsInstantiator();
private:
       void registerDllClasses();
};

#endif // __RegisterTypes_Actuators_h__


