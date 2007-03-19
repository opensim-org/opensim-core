// MarkerFrame.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "MarkerFrame.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MarkerFrame::MarkerFrame() :
	_numMarkers(0),
	_frameNumber(-1),
	_units()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor taking all the header information
 *
 * @param aNumMarkers the number of markers in the frame
 * @param aFrameNumber the frame number
 * @param aTime the time of the frame
 * @param aUnits the units of the XYZ marker coordinates
 */
MarkerFrame::MarkerFrame(int aNumMarkers, int aFrameNumber, double aTime, Units& aUnits) :
	_numMarkers(aNumMarkers),
	_frameNumber(aFrameNumber),
	_frameTime(aTime),
	_units(aUnits)
{
	setNull();
}

/**
 * Copy constructor.
 */
MarkerFrame::MarkerFrame(const MarkerFrame& aFrame) :
   Object(aFrame)
{
	setNull();

	_numMarkers = aFrame._numMarkers;
	_frameNumber = aFrame._frameNumber;
	_frameTime = aFrame._frameTime;
	_units = aFrame._units;
	_markers = aFrame._markers;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerFrame::~MarkerFrame()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this MarkerFrame to their null values.
 */
void MarkerFrame::setNull()
{
	setType("MarkerFrame");
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Add a marker to the frame
 *
 * @param aCoords the XYZ coordinates of the marker
 */
void MarkerFrame::addMarker(double aCoords[3])
{
	SimmPoint* pt = new SimmPoint(aCoords);
	_markers.append(pt);
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the XYZ coordinates of all markers in the frame
 *
 * @param aScaleFactor the scale factor
 */
void MarkerFrame::scale(double aScaleFactor)
{
	for (int i = 0; i < _numMarkers; i++)
	{
		double* pt = _markers[i]->get();
		for (int j = 0; j < 3; j++)
			pt[j] *= aScaleFactor;
	}
}

void MarkerFrame::peteTest() const
{
	cout << "      MarkerFrame " << _frameNumber << ": " << endl;
	cout << "         numMarkers: " << _numMarkers << endl;
	cout << "         frameTime: " << _frameTime << endl;
	cout << "         units: " << _units.getLabel() << endl;

	for (int i = 0; i < _numMarkers; i++)
		cout << "         marker " << i << ": " << _markers[i]->get()[0] << ", " << _markers[i]->get()[1] << ", " << _markers[i]->get()[2] << endl;
}
