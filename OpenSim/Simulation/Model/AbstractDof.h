#ifndef __AbstractDof_h__
#define __AbstractDof_h__

// AbstractDof.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>

namespace OpenSim {

class AbstractDynamicsEngine;
class AbstractJoint;
class AbstractCoordinate;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a SIMM dof (a "potential"
 * degree of freedom in a joint).
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractDof : public Object  
{

//=============================================================================
// DATA
//=============================================================================
public:
	enum DofType
	{
		Translational,
		Rotational
	};

protected:
	PropertyObjPtr<Function> _functionProp;
	Function *&_function;

	PropertyStr _coordinateNameProp;
	std::string& _coordinateName;

	const AbstractCoordinate *_coordinate;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractDof();
	AbstractDof(const AbstractDof &aDof);
	virtual ~AbstractDof();
	virtual Object* copy() const = 0;

#ifndef SWIG
	AbstractDof& operator=(const AbstractDof &aDof);
#endif
   void copyData(const AbstractDof &aDof);

	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractJoint* aJoint);
	virtual void getAxis(double axis[3]) const = 0;
	virtual const double* getAxisPtr(void) const = 0;
	virtual double getValue() = 0;
	virtual DofType getMotionType() const = 0;
	virtual const AbstractCoordinate* getCoordinate() const { return _coordinate; }
	virtual Function* getFunction() const;

	virtual void peteTest();

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class AbstractDof
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractDof_h__


