#ifndef __IKTask_h__
#define __IKTask_h__

#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyDbl.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @author Eran Guendelman
 * @version 1.0
 */

class RDSIMULATION_API IKTask : public Object
{
protected:
	PropertyDbl _weightProp;
	double &_weight;

public:
	IKTask();
	IKTask(const IKTask &aIKTask);
	virtual Object* copy() const = 0;

#ifndef SWIG
	IKTask& operator=(const IKTask &aIKTask);
#endif

	double getWeight() { return _weight; }
	void setValue(double aWeight) { _weight = aWeight; }

private:
	void setupProperties();
//=============================================================================
};	// END of class IKTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTask_h__
