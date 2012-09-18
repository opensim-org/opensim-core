#ifndef __MarkerFrame_h__
#define __MarkerFrame_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  MarkerFrame.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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


// INCLUDE
#include <iostream>
#include <string>
#include "osimCommonDLL.h"
#include "Object.h"
#include "ArrayPtrs.h"
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
class OSIMCOMMON_API MarkerFrame : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerFrame, Object);

//=============================================================================
// DATA
//=============================================================================
private:
	int _numMarkers;
	int _frameNumber;
	double _frameTime;
	Units _units;
	SimTK::Array_<SimTK::Vec3> _markers;

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

	void addMarker(const SimTK::Vec3& aCoords);
	SimTK::Vec3 getMarker(int aIndex) const { return _markers[aIndex]; }
    SimTK::Vec3& updMarker(int aIndex) { return _markers[aIndex]; }
	int getFrameNumber() const { return _frameNumber; }
	void setFrameNumber(int aNumber) { _frameNumber = aNumber; }
	double getFrameTime() const { return _frameTime; }
	void scale(double aScaleFactor);

	const SimTK::Array_<SimTK::Vec3>& getMarkers() const { return _markers;}

private:
	void setNull();

//=============================================================================
};	// END of class MarkerFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerFrame_h__


