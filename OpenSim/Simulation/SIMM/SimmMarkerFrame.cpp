// SimmMarkerFrame.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmMarkerFrame.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMarkerFrame::SimmMarkerFrame() :
	_numMarkers(0),
	_frameNumber(-1),
	_markers(NULL),
	_units()
{
}

SimmMarkerFrame::SimmMarkerFrame(int aNumMarkers, int aFrameNumber, double aTime, SimmUnits& aUnits) :
	_numMarkers(aNumMarkers),
	_frameNumber(aFrameNumber),
	_frameTime(aTime),
	_markers(NULL),
	_units(aUnits)
{
}

/**
 * Copy constructor.
 */
SimmMarkerFrame::SimmMarkerFrame(const SimmMarkerFrame& aFrame)
{
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
SimmMarkerFrame::~SimmMarkerFrame()
{
}

void SimmMarkerFrame::addMarker(double coords[3])
{
	SimmPoint* pt = new SimmPoint(coords);
	_markers.append(pt);
}

void SimmMarkerFrame::scale(double aScaleFactor)
{
	for (int i = 0; i < _numMarkers; i++)
	{
		double* pt = _markers[i]->get();
		for (int j = 0; j < 3; j++)
			pt[j] *= aScaleFactor;
	}
}

void SimmMarkerFrame::peteTest() const
{
	cout << "      MarkerFrame " << _frameNumber << ": " << endl;
	cout << "         numMarkers: " << _numMarkers << endl;
	cout << "         frameTime: " << _frameTime << endl;
	cout << "         units: " << _units.getLabel() << endl;

	for (int i = 0; i < _numMarkers; i++)
		cout << "         marker " << i << ": " << _markers[i]->get()[0] << ", " << _markers[i]->get()[1] << ", " << _markers[i]->get()[2] << endl;
}
