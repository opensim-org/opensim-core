#ifndef __IKCoordinateTask_h__
#define __IKCoordinateTask_h__

#include "osimToolsDLL.h"
#include "IKTask.h"
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 */

class OSIMTOOLS_API IKCoordinateTask : public IKTask
{
protected:
	PropertyBool _fromFileProp;
	bool &_fromFile;

	PropertyDbl _valueProp;
	double &_value;

public:
	IKCoordinateTask();
	IKCoordinateTask(const IKCoordinateTask &aIKCoordinateTask);
	virtual Object* copy() const;

#ifndef SWIG
	IKCoordinateTask& operator=(const IKCoordinateTask &aIKCoordinateTask);
#endif

	bool getFromFile() { return _fromFile; }
	bool getValueUseDefault() const { return _valueProp.getUseDefault(); }
	double getValue() const { return _value; }

private:
	void setupProperties();
//=============================================================================
};	// END of class IKCoordinateTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKCoordinateTask_h__
