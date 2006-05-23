#ifndef _SimmDof_h_
#define _SimmDof_h_

// SimmDof.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/Function.h>
#include "SimmCoordinate.h"
#include "Constant.h"

namespace OpenSim { 

class SimmJoint;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM DOF (a joint element). This is a base class.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmDof : public Object  
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
#ifndef SWIG
	typedef struct
	{
		std::string name;           /* name as dof appears in SD/FAST code */
		std::string constraintName; /* if constrained, additional name */
		double initialValue;        /* value of dof at start of simulation */
		bool constrained;           /* is this dof constrained by a kinematic function? */
		bool fixed;                 /* is this dof fixed (prescribed in SD/FAST)? */
		int stateNumber;            /* element of state vector that holds this dof */
		int errorNumber;            /* if constrained, index into error array */
		int joint;                  /* SD/FAST joint which contains this dof */
		int axis;                   /* axis in SD/FAST joint which corresponds to dof */
		double conversion;          /* for printing (RTOD for rots, 1.0 for trans) */
		double conversionSign;      /* set to -1 if DOF has negative-slope function */
	} sdfastDofInfo;

	sdfastDofInfo _sdfastInfo;
#endif
protected:
	PropertyObjArray _functionsProp;
	ArrayPtrs<Function> &_functions;

	PropertyStr _coordinateNameProp;
	std::string& _coordinateName;

	const SimmCoordinate *_coordinate;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmDof(void);
	SimmDof(DOMElement *aElement);
	SimmDof(const SimmDof &aDof);
	virtual ~SimmDof(void);
	virtual Object* copy(void) const = 0;
	virtual Object* copy(DOMElement *aElement) const = 0;

#ifndef SWIG
	SimmDof& operator=(const SimmDof &aDof);
#endif
   void SimmDof::copyData(const SimmDof &aDof);

	virtual void setup(SimmKinematicsEngine* aEngine, SimmJoint* aJoint);
	virtual void getAxis(double axis[3]) const = 0;
	virtual const double* getAxisPtr(void) const = 0;
	virtual double getValue(void) = 0;
	virtual DofType getDofType(void) const = 0;
	const SimmCoordinate* getCoordinate(void) const { return _coordinate; }
	Function* getFunction(void) const;

	virtual void peteTest(void);

protected:

private:
	void setNull(void);
	void setupProperties(void);

//=============================================================================
};	// END of class SimmDof

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmDof_h__


