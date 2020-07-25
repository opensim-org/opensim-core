/* -------------------------------------------------------------------------- *
 *                         OpenSim:  MarkerFrame.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
    setAuthors("Peter Loan");
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
    //SimmPoint* pt = new SimmPoint(aCoords);
    _markers.push_back(aCoords);
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
        //SimTK::Vec3& pt = _markers[i]->get();
        _markers[i] *= aScaleFactor;
    }
}
