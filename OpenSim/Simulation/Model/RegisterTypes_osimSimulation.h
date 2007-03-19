#ifndef _RegisterTypes_osimSimulation_h_
#define _RegisterTypes_osimSimulation_h_
// RegisterTypes_osimSimulation.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Simulation/osimSimulationDLL.h>


extern "C" {

OSIMSIMULATION_API void RegisterTypes_osimSimulation(); 

}

/**
 * A class intended for the purpose of automatically registering classes defined in this simulation Dll
 */
class osimSimulationInstantiator 
{ 
public: 
        osimSimulationInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    

#endif // __RegisterTypes_osimSimulation_h__


