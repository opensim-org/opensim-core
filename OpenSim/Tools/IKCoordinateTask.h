#ifndef __IKCoordinateTask_h__
#define __IKCoordinateTask_h__

#include "osimToolsDLL.h"
#include "IKTask.h"
#include <OpenSim/Common/PropertyStr.h>
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
	OPENSIM_DECLARE_DERIVED(IKCoordinateTask, IKTask);
public:
	enum ValueType { DefaultValue, ManualValue, FromFile };

protected:
	PropertyStr _valueTypeProp;
	std::string &_valueType;

	PropertyDbl _valueProp;
	double &_value;

public:
	IKCoordinateTask();
	IKCoordinateTask(const IKCoordinateTask &aIKCoordinateTask);
	virtual Object* copy() const;

#ifndef SWIG
	IKCoordinateTask& operator=(const IKCoordinateTask &aIKCoordinateTask);
#endif

	void setValueType(ValueType type) { _valueType = ValueTypeToString(type); }
	ValueType getValueType() const { return StringToValueType(_valueType); }

	double getValue() const { return _value; }
	void setValue(double value) { _value = value; }

	static std::string ValueTypeToString(ValueType type);
	static ValueType StringToValueType(const std::string &str);

private:
	void setupProperties();
//=============================================================================
};	// END of class IKCoordinateTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKCoordinateTask_h__
