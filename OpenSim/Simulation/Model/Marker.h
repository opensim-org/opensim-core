#ifndef __Marker_h__
#define __Marker_h__
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Marker.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ayman Habib                                         *
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
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include "SimTKcommon.h"

namespace OpenSim {

class Body;
class Model;
class VisibleObject;


//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM [mocap] marker.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API Marker : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(Marker, Object);

class Body;

//=============================================================================
// DATA
//=============================================================================
private:

protected:
    const Model* _model;

	PropertyDblVec3 _offsetProp;
	SimTK::Vec3 &_offset;

	PropertyBool _fixedProp;
	bool &_fixed;

	// The bodyName property is used only for markers that are part of a
	// MarkerSet, not for ones that are part of a model.
	PropertyStr _bodyNameProp;
	std::string &_bodyName;

	// Body that the marker is attached to
	OpenSim::Body* _body;

	// Support for Display
	VisibleObject _displayer;

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

    static void deleteMarker(Marker* aMarker) { if (aMarker) delete aMarker; }

#ifndef SWIG
	Marker& operator=(const Marker &aMarker);
#endif
	void copyData(const Marker &aMarker);

	virtual void updateFromMarker(const Marker &aMarker);
	virtual void getOffset(SimTK::Vec3& rOffset) const;
	virtual const SimTK::Vec3& getOffset() const { return _offset; }
	virtual void getOffset(double rOffset[]) const;
	virtual bool setOffset(const SimTK::Vec3& aOffset);
	virtual bool setOffset(const double aOffset[3]);
	virtual bool getOffsetUseDefault() const { return _offsetProp.getValueIsDefault(); }
	virtual bool getFixed() const { return _fixed; }
	virtual bool setFixed(bool aFixed);
	virtual bool getFixedUseDefault() const { return _fixedProp.getValueIsDefault(); }
	virtual const std::string& getBodyName() const;
	virtual bool setBodyName(const std::string& aName);
	virtual bool getBodyNameUseDefault() const { return _bodyNameProp.getValueIsDefault(); }
	virtual bool setBodyNameUseDefault(bool aValue);
	virtual OpenSim::Body& getBody() const { return *_body; }
	virtual void changeBody( OpenSim::Body& aBody );
	virtual void changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody );
	virtual void scale(const SimTK::Vec3& aScaleFactors);
	virtual void connectMarkerToModel(const Model& aModel);
	virtual void updateGeometry();

	virtual const VisibleObject* getDisplayer() const { return &_displayer; }
	virtual VisibleObject*	updDisplayer() { return &_displayer; };

	virtual void removeSelfFromDisplay();
	const bool isVirtual()
	{
		return _virtual;
	}
	void setVirtual(bool aTrueFalse)
	{
		_virtual=aTrueFalse;
	}
private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Marker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Marker_h__


