#ifndef __SSimmTranslationDof_h__
#define __SimmTranslationDof_h__

// SimmTranslationDof.h
// Author: Peter Loan, Frank C. Anderson
/*
 * Copyright (c) 2006-2007, Stanford University. All rights reserved. 
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
#include "osimSimmKinematicsEngineDLL.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractDof.h>

namespace OpenSim {

#define TX_NAME "tx"
#define TY_NAME "ty"
#define TZ_NAME "tz"

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM translational DOF.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMMKINEMATICSENGINE_API SimmTranslationDof : public AbstractDof  
{
public:
	enum AxisIndex
	{
		xTranslation = 0,
		yTranslation,
		zTranslation
	};

//=============================================================================
// DATA
//=============================================================================
protected:
	double _axis[3];
	AxisIndex _axisIndex;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmTranslationDof();
	SimmTranslationDof(const SimmTranslationDof &aDof);
	virtual ~SimmTranslationDof();
	virtual Object* copy() const;
	virtual void updateFromXMLNode();

#ifndef SWIG
	SimmTranslationDof& operator=(const SimmTranslationDof &aDof);
#endif
   void copyData(const SimmTranslationDof &aDof);

	virtual void setAxis(const double rAxis[3]);
	virtual void getAxis(double rAxis[3]) const;
	virtual const double* getAxisPtr() const { return &_axis[0]; }
	virtual double getValue();
	virtual DofType getMotionType() const { return Translational; }
	void getTranslation(double rVec[4]);
	AxisIndex getAxisIndex() const { return _axisIndex; }

protected:

private:
	void setNull();
//=============================================================================
};	// END of class SimmTranslationDof
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmTranslationDof_h__


