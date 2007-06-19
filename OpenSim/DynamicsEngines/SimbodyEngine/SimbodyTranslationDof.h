#ifndef __SimbodyTranslationDof_h__
#define __SimbodyTranslationDof_h__

// SimbodyTranslationDof.h
// Author: Peter Loan, Frank C. Anderson
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
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
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractDof.h>
#include <SimTKsimbody.h>

namespace OpenSim {

#define TX_NAME "tx"
#define TY_NAME "ty"
#define TZ_NAME "tz"

//=============================================================================
//=============================================================================
/**
 * A class expressing a translational DOF.
 *
 * @author Peter Loan, Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API SimbodyTranslationDof : public AbstractDof  
{
public:
	enum AxisIndex {
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
	SimbodyTranslationDof();
	SimbodyTranslationDof(const SimbodyTranslationDof &aDof);
	virtual ~SimbodyTranslationDof();
	virtual Object* copy() const;
	virtual void updateFromXMLNode();

#ifndef SWIG
	SimbodyTranslationDof& operator=(const SimbodyTranslationDof &aDof);
#endif
   void copyData(const SimbodyTranslationDof &aDof);

	virtual void setAxis(const	double axis[3]);
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
};	// END of class SimbodyTranslationDof
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodyTranslationDof_h__


