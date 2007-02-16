#ifndef __IKTaskSet_h__
#define __IKTaskSet_h__

#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Set.h>
#include "IKTask.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 */

class RDSIMULATION_API IKTaskSet : public Set<IKTask>
{
public:
	IKTaskSet() { setType("IKTaskSet"); }
	IKTaskSet(const IKTaskSet &aIKTaskSet) : Set<IKTask>(aIKTaskSet) { }
#ifndef SWIG
	IKTaskSet& operator=(const IKTaskSet &aIKTaskSet) { Set<IKTask>::operator=(aIKTaskSet); return *this; }
#endif
//=============================================================================
};	// END of class IKTaskSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTaskSet_h__
