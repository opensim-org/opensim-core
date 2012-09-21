#ifndef __PathWrapPoint_h__
#define __PathWrapPoint_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PathWrapPoint.h                          *
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
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Simulation/Model/PathPoint.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class WrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle via point, which is a muscle point that
 * is active only for a specified range of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PathWrapPoint : public PathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrapPoint, PathPoint);

//=============================================================================
// DATA
//=============================================================================
private:
	Array<SimTK::Vec3> _wrapPath; // points defining muscle path on surface of wrap object
   double _wrapPathLength; // length of _wrapPath

	WrapObject* _wrapObject; // the wrap object this point is on

protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PathWrapPoint();
	PathWrapPoint(const PathWrapPoint &aPoint);
	virtual ~PathWrapPoint();

	void copyData(const PathWrapPoint &aPoint);

#ifndef SWIG
	PathWrapPoint& operator=(const PathWrapPoint &aPoint);
#endif

	Array<SimTK::Vec3>& getWrapPath() { return _wrapPath; }
	double getWrapLength() const { return _wrapPathLength; }
	void setWrapLength(double aLength) { _wrapPathLength = aLength; }
	virtual WrapObject* getWrapObject() const { return _wrapObject; }
	void setWrapObject(WrapObject* aWrapObject) { _wrapObject = aWrapObject; }

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class PathWrapPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PathWrapPoint_h__


