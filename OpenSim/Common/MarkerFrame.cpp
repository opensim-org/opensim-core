// MarkerFrame.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
void MarkerFrame::addMarker(const SimTK::Vec3& aCoords)
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
		SimTK::Vec3& pt = _markers[i]->get();
		pt *= aScaleFactor;
	}
}
