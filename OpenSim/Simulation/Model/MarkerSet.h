#ifndef __MarkerSet_h__
#define __MarkerSet_h__

// MarkerSet.h
// Author: Ayman Habib, Peter Loan
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
