#ifndef _SimmMeasurement_h_
#define _SimmMeasurement_h_

// SimmMeasurement.h
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


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/Scale.h>
#include <OpenSim/Tools/ScaleSet.h>

#include "SimmMarkerPairSet.h"
#include "BodyScaleSet.h"

//=============================================================================
//=============================================================================
/**
 * A class implementing a measurement (the distance between one or more pairs
 * of markers, used to scale a model).
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API SimmMeasurement : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

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
	SimmMeasurement(DOMElement *aElement);
	SimmMeasurement(const SimmMeasurement &aMeasurement);
	virtual ~SimmMeasurement();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmMeasurement& operator=(const SimmMeasurement &aMeasurement);
#endif
   void SimmMeasurement::copyData(const SimmMeasurement &aMeasurement);

	int getNumMarkerPairs() const { return _markerPairSet.getSize(); }
	const SimmMarkerPair& getMarkerPair(int aIndex) const { return *_markerPairSet[aIndex]; }
	bool getApply() const { return _apply; }
	void setApply(bool aApply) { _apply = aApply; }

	void applyScaleFactor(double aFactor, ScaleSet& aScaleSet);

	/* Register types to be used when reading a SimmMeasurement object from xml file. */
	static void registerTypes();

	/** Programmatically add a SimmMarkerPair to SimmMeasurement */
	void addMarkerPair(SimmMarkerPair *aMarkerPair)
	{
		_markerPairSet.append(aMarkerPair);
	}

	/** Programmatically add a BodyScale to SimmMeasurement */
	void addBodyScale(BodyScale *aBodyScale)
	{
		_bodyScaleSet.append(aBodyScale);
	}

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmMeasurement

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmMeasurement_h__


