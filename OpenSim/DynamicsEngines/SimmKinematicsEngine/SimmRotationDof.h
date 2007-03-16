#ifndef __SimmRotationDof_h__
#define __SimmRotationDof_h__

// SimmRotationDof.h
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
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/Storage.h>
#include "AbstractDof.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM rotational DOF.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmRotationDof : public AbstractDof  
{

//=============================================================================
// DATA
//=============================================================================
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
	SimmRotationDof(const SimmRotationDof &aDof);
	virtual ~SimmRotationDof();
	virtual Object* copy() const;

#ifndef SWIG
	SimmRotationDof& operator=(const SimmRotationDof &aDof);
#endif
   void copyData(const SimmRotationDof &aDof);

	virtual void getAxis(double rAxis[3]) const;
	const Array<double>& getAxis() const { return _axis; }
	virtual const double* getAxisPtr() const { return &_axis[0]; }
	virtual double getValue();
	virtual DofType getMotionType() const { return Rotational; }

	virtual void peteTest();

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmRotationDof
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmRotationDof_h__


