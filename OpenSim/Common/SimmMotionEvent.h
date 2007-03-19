#ifndef __SimmMotionEvent_h__
#define __SimmMotionEvent_h__

// SimmMotionEvent.h
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
#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing an event in a SIMM motion.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API SimmMotionEvent : public Object  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _timeProp;
	double& _time;

	PropertyDblArray _colorProp;
	Array<double> _color;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMotionEvent();
	SimmMotionEvent(const SimmMotionEvent &aEvent);
	virtual ~SimmMotionEvent();
	virtual Object* copy() const;
	SimmMotionEvent& operator=(const SimmMotionEvent &aEvent);

	void copyData(const SimmMotionEvent &aEvent);
	void setName(std::string aName) { _name = aName; }
	void setTime(double aTime) { _time = aTime; }
	double getTime() const { return _time; }
	void setColor(double* aColor);
	const double* getColor() const { return &_color[0]; }

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmMotionEvent
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMotionEvent_h__


