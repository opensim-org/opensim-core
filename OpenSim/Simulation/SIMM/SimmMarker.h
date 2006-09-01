#ifndef _SimmMarker_h_
#define _SimmMarker_h_

// SimmMarker.h
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
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/VisibleObject.h>

namespace OpenSim { 

class SimmKinematicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM [mocap] marker.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmMarker : public Object
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyDblArray _attachmentProp;
	Array<double> &_attachment;

	PropertyDbl _weightProp;
	double &_weight;

	PropertyBool _fixedProp;
	bool &_fixed;

	// The bodyName property is used only for markers that are part of a
	// SimmMarkerSet, not for ones that are part of a SimmModel.
	PropertyStr _bodyNameProp;
	std::string &_bodyName;

	// Support for Display
	PropertyObj		_displayerProp;
	VisibleObject	&_displayer;

	/** A temporary kluge until the default mechanism is working */
	static Geometry *_defaultGeometry;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmMarker();
	SimmMarker(DOMElement *aElement);
	SimmMarker(const SimmMarker &aMarker);
	virtual ~SimmMarker();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmMarker& operator=(const SimmMarker &aMarker);
#endif
	void copyData(const SimmMarker &aMarker);
	void updateFromMarker(const SimmMarker &aMarker);

	void getOffset(double *aOffset) const;
	const double* getOffset() const { return &_attachment[0]; }
	void setOffset(double pt[3]);
	bool getFixed() const { return _fixed; }
	double getWeight() const { return _weight; }
	const std::string* getBodyName() const;
	void setup(SimmKinematicsEngine* aEngine);
	void scale(Array<double>& aScaleFactors);

	void writeSIMM(std::ofstream& out) const;

	void peteTest() const;
	virtual VisibleObject* getDisplayer() { return &_displayer; };
protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmMarker

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmMarker_h__


