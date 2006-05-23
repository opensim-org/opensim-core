#ifndef _SimmRotationDof_h_
#define _SimmRotationDof_h_

// SimmRotationDof.h
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
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include "SimmDof.h"

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM rotational DOF.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API SimmRotationDof : public SimmDof  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyDblArray _axisProp;
	Array<double> &_axis;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmRotationDof();
	SimmRotationDof(DOMElement *aElement);
	SimmRotationDof(const SimmRotationDof &aDof);
	virtual ~SimmRotationDof();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmRotationDof& operator=(const SimmRotationDof &aDof);
#endif
   void SimmRotationDof::copyData(const SimmRotationDof &aDof);

	virtual void getAxis(double axis[3]) const;
	const Array<double>& getAxis() const { return _axis; }
	virtual const double* getAxisPtr() const { return &_axis[0]; }
	virtual double getValue();
	virtual DofType getDofType() const { return DofType::Rotational; }

	virtual void peteTest();

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmDof

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmRotationDof_h__


