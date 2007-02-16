#ifndef __IKCoordinateTask_h__
#define __IKCoordinateTask_h__

#include <OpenSim/Simulation/rdSimulationDLL.h>
#include "IKTask.h"
#include <OpenSim/Tools/PropertyBool.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 */

class RDSIMULATION_API IKCoordinateTask : public IKTask
{
protected:
	PropertyBool _fromFileProp;
	bool &_fromFile;

public:
	IKCoordinateTask();
	IKCoordinateTask(const IKCoordinateTask &aIKCoordinateTask);
	virtual Object* copy() const;

#ifndef SWIG
	IKCoordinateTask& operator=(const IKCoordinateTask &aIKCoordinateTask);
#endif

	bool getFromFile() { return _fromFile; }
	void setFromFile(bool aFromFile) { _fromFile = aFromFile; }

private:
	void setupProperties();
//=============================================================================
};	// END of class IKCoordinateTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKCoordinateTask_h__
