#ifndef __MarkerSet_h__
#define __MarkerSet_h__

// MarkerSet.h
// Author: Ayman Habib, Peter Loan
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include "AbstractMarker.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class AbstractModel;
class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of markers for inverse kinematics.
 *
 * @authors Ayman Habib, Peter Loan
 * @version 1.0
 */

class OSIMSIMULATION_API MarkerSet :	public Set<AbstractMarker>
{
private:
	void setNull();
public:
	MarkerSet();
	MarkerSet(const std::string& aMarkersFileName);
	MarkerSet(const MarkerSet& aMarkerSet);
	~MarkerSet(void);
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
//=============================================================================
};	// END of class MarkerSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerSet_h__
