#ifndef __Marker_h__
#define __Marker_h__

// VisibleMarker.h
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
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/Geometry.h>
#include "AbstractMarker.h"

namespace OpenSim {

class AbstractBody;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM [mocap] marker.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API Marker : public AbstractMarker
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyDblArray _offsetProp;
	Array<double> &_offset;

	PropertyBool _fixedProp;
	bool &_fixed;

	// The bodyName property is used only for markers that are part of a
	// MarkerSet, not for ones that are part of a model.
	PropertyStr _bodyNameProp;
	std::string &_bodyName;

	// Body that the marker is attached to
	AbstractBody* _body;

	// Support for Display
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	/** A temporary kluge until the default mechanism is working */
	static Geometry *_defaultGeometry;
	bool _virtual;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Marker();
	Marker(const Marker &aMarker);
	virtual ~Marker();
	virtual Object* copy() const;

#ifndef SWIG
	Marker& operator=(const Marker &aMarker);
#endif
	void copyData(const Marker &aMarker);

	virtual void updateFromMarker(const AbstractMarker &aMarker);
	virtual void getOffset(double *rOffset) const;
	virtual const double* getOffset() const { return &_offset[0]; }
	virtual bool setOffset(Array<double>& aOffset);
	virtual bool setOffset(const double aPoint[3]);
	virtual bool getOffsetUseDefault() const { return _offsetProp.getUseDefault(); }
	virtual bool getFixed() const { return _fixed; }
	virtual bool setFixed(bool aFixed);
	virtual bool getFixedUseDefault() const { return _fixedProp.getUseDefault(); }
	virtual const std::string* getBodyName() const;
	virtual bool setBodyName(const std::string& aName);
	virtual bool getBodyNameUseDefault() const { return _bodyNameProp.getUseDefault(); }
	virtual bool setBodyNameUseDefault(bool aValue);
	virtual AbstractBody* getBody() const { return _body; }
	virtual void setBody(AbstractBody* aBody);
	virtual void scale(const Array<double>& aScaleFactors);
	virtual void setup(AbstractDynamicsEngine* aEngine);
	virtual void updateGeometry();

	virtual VisibleObject* getDisplayer() { return &_displayer; }
	virtual void removeSelfFromDisplay();
	const bool isVirtual()
	{
		return _virtual;
	}
	void setVirtual(bool aTrueFalse)
	{
		_virtual=aTrueFalse;
	}
	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Marker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Marker_h__


