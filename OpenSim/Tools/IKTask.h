#ifndef __IKTask_h__
#define __IKTask_h__

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyDbl.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @author Eran Guendelman
 * @version 1.0
 */

class OSIMTOOLS_API IKTask : public Object
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

private:
	void setupProperties();
//=============================================================================
};	// END of class IKTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTask_h__
