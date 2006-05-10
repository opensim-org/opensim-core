#ifndef _RegisterTypes_rdSimulation_h_
#define _RegisterTypes_rdSimulation_h_
// RegisterTypes_rdSimulation.h
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Simulation/rdSimulationDLL.h>


extern "C" {

RDSIMULATION_API void RegisterTypes_rdSimulation(); 

}

/**
 * A class intended for the purpose of automatically registering classes defined in this simulation Dll
 */
class rdSimulationInstantiator 
{ 
public: 
        rdSimulationInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    

#endif // __RegisterTypes_rdSimulation_h__


