#ifndef __Marker_h__
#define __Marker_h__

// Marker.h
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

	virtual VisibleObject* getDisplayer() const { return &_displayer; }
	virtual void removeSelfFromDisplay();
	const bool isVirtual()
	{
		return _virtual;
	}
	void setVirtual(bool aTrueFalse)
	{
		_virtual=aTrueFalse;
	}

	OPENSIM_DECLARE_DERIVED(Marker, AbstractMarker);
private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Marker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Marker_h__


