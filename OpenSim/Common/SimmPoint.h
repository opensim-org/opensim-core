#ifndef __SimmPoint_h__
#define __SimmPoint_h__

// SimmPoint.h
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
#include "osimCommonDLL.h"
#include "Object.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a 3D point.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API SimmPoint : public Object
{

//=============================================================================
// DATA
//=============================================================================
private:
	double _location[3];

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmPoint();
	SimmPoint(double coords[3]);
	SimmPoint(const SimmPoint& aPoint);
	virtual ~SimmPoint();
	virtual Object* copy() const;
#ifndef SWIG
	SimmPoint& operator=(const SimmPoint &aPoint);
	SimmPoint& operator+=(const SimmPoint &aPoint);
	SimmPoint& operator/=(double factor);
#endif
	void set(double x, double y, double z);
	double* get() { return _location; }
	void scale(double aScaleFactor);
	bool isVisible() const;
	void peteTest() const;

private:
	void setNull();

//=============================================================================
};	// END of class SimmPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmPoint_h__


