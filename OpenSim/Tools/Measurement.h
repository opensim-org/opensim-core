#ifndef __Measurement_h__
#define __Measurement_h__

// Measurement.h
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


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include "osimToolsDLL.h"
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Scale.h>
#include <OpenSim/Common/ScaleSet.h>
#include "MarkerPairSet.h"
#include <OpenSim/Simulation/Model/BodyScaleSet.h>

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a measurement (the distance between one or more pairs
 * of markers, used to scale a model).
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API Measurement : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(Measurement, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyObj _markerPairSetProp;
	MarkerPairSet &_markerPairSet;

	PropertyObj _bodyScaleSetProp;
	BodyScaleSet &_bodyScaleSet;

	PropertyBool _applyProp;
	bool &_apply;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Measurement();
	Measurement(const Measurement &aMeasurement);
	virtual ~Measurement();

#ifndef SWIG
	Measurement& operator=(const Measurement &aMeasurement);
#endif
   void copyData(const Measurement &aMeasurement);

	BodyScaleSet &getBodyScaleSet() { return _bodyScaleSet; }

	MarkerPairSet& getMarkerPairSet() { return _markerPairSet; }
	int getNumMarkerPairs() const { return _markerPairSet.getSize(); }
	const MarkerPair& getMarkerPair(int aIndex) const { return _markerPairSet[aIndex]; }

	bool getApply() const { return _apply; }
	void setApply(bool aApply) { 
		_apply = aApply;
		_applyProp.setValueIsDefault(false);
	}

	void applyScaleFactor(double aFactor, ScaleSet& aScaleSet);

	/* Register types to be used when reading a Measurement object from xml file. */
	static void registerTypes();

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Measurement
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Measurement_h__


