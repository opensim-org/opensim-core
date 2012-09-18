#ifndef __MarkerSet_h__
#define __MarkerSet_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  MarkerSet.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Peter Loan                                         *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include "Marker.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Model;
class ScaleSet;
class Body;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of markers for inverse kinematics.
 *
 * @authors Ayman Habib, Peter Loan
 * @version 1.0
 */

class OSIMSIMULATION_API MarkerSet : public Set<Marker> {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerSet, Set<Marker>);

private:
	void setNull();
public:
	MarkerSet();
	MarkerSet(const std::string& aMarkersFileName) SWIG_DECLARE_EXCEPTION;
	MarkerSet(const MarkerSet& aMarkerSet);
	~MarkerSet(void);

    void connectMarkersToModel(Model& aModel);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	MarkerSet& operator=(const MarkerSet &aMarkerSet);
#endif
	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------
	void getMarkerNames(Array<std::string>& aMarkerNamesArray);
	void scale(const ScaleSet& aScaleSet);
	/** Add a prefix to marker names for all markers in the set**/
	void addNamePrefix(const std::string& prefix);
	Marker* addMarker( const std::string& aName, const double aOffset[3], OpenSim::Body& aBody);
//=============================================================================
};	// END of class MarkerSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerSet_h__
