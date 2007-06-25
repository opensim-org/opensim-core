#ifndef __MarkerFrame_h__
#define __MarkerFrame_h__

// MarkerFrame.h
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
#include "ArrayPtrs.h"
#include "SimmPoint.h"
#include "Units.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a frame of marker data from a TRC/TRB file.
 *
 * @author Peter Loan
 * @version 1.0
 */
	class OSIMCOMMON_API MarkerFrame : public Object
{

//=============================================================================
// DATA
//=============================================================================
private:
	int _numMarkers;
	int _frameNumber;
	double _frameTime;
	Units _units;
	ArrayPtrs<SimmPoint> _markers;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MarkerFrame();
	MarkerFrame(int aNumMarkers, int aFrameNumber, double aTime, Units& aUnits);
	MarkerFrame(const MarkerFrame& aFrame);
	virtual ~MarkerFrame();
	void addMarker(double aCoords[3]);
	SimmPoint& getMarker(int aIndex) { return *_markers[aIndex]; }
	int getFrameNumber() const { return _frameNumber; }
	void setFrameNumber(int aNumber) { _frameNumber = aNumber; }
	double getFrameTime() const { return _frameTime; }
	void scale(double aScaleFactor);

private:
	void setNull();

//=============================================================================
};	// END of class MarkerFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerFrame_h__


