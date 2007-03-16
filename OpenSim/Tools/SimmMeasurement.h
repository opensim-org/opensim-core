#ifndef __SimmMeasurement_h__
#define __SimmMeasurement_h__

// SimmMeasurement.h
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


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/Scale.h>
#include <OpenSim/Tools/ScaleSet.h>
#include "SimmMarkerPairSet.h"
#include "BodyScaleSet.h"

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
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
class RDSIMULATION_API SimmMeasurement : public Object  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyObj _markerPairSetProp;
	SimmMarkerPairSet &_markerPairSet;

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
	SimmMeasurement();
	SimmMeasurement(const SimmMeasurement &aMeasurement);
	virtual ~SimmMeasurement();
	virtual Object* copy() const;

#ifndef SWIG
	SimmMeasurement& operator=(const SimmMeasurement &aMeasurement);
#endif
   void copyData(const SimmMeasurement &aMeasurement);

	int getNumMarkerPairs() const { return _markerPairSet.getSize(); }
	const SimmMarkerPair& getMarkerPair(int aIndex) const { return *_markerPairSet[aIndex]; }
	bool getApply() const { return _apply; }
	void applyScaleFactor(double aFactor, ScaleSet& aScaleSet);

	/* Register types to be used when reading a SimmMeasurement object from xml file. */
	static void registerTypes();

	void peteTest() const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmMeasurement
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmMeasurement_h__


