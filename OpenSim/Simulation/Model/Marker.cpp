/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Marker.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Marker.h"
#include "Model.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "BodySet.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

Geometry *Marker::_defaultGeometry = AnalyticSphere::createSphere(0.01);
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Marker::Marker() :
    Object(),
    _offset(_offsetProp.getValueDblVec()),
    _fixed(_fixedProp.getValueBool()),
    _bodyName(_bodyNameProp.getValueStr())
{
    setNull();
    setupProperties();
    _displayer.setOwner(this);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Marker::~Marker()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarker Marker to be copied.
 */
Marker::Marker(const Marker &aMarker) :
    Object(aMarker),
    _offset(_offsetProp.getValueDblVec()),
    _fixed(_fixedProp.getValueBool()),
    _bodyName(_bodyNameProp.getValueStr())
{
    setNull();
    setupProperties();
    copyData(aMarker);
    _displayer.setOwner(this);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Marker to another.
 *
 * @param aMarker Marker to be copied.
 */
void Marker::copyData(const Marker &aMarker)
{
    _offset = aMarker._offset;
    _fixed = aMarker._fixed;
    _bodyName = aMarker._bodyName;
    _body = NULL;
    _displayer = aMarker._displayer;
    _virtual = aMarker._virtual;
}

//_____________________________________________________________________________
/**
 * Set the data members of this Marker to their null values.
 */
void Marker::setNull()
{
    setVirtual(true);
    _body = 0;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Marker::setupProperties()
{
    _bodyNameProp.setComment("Body segment in the model on which the marker resides.");
    _bodyNameProp.setName("body");
    _propertySet.append(&_bodyNameProp);

    _offsetProp.setComment("Location of a marker on the body segment.");
    const SimTK::Vec3 defaultAttachment(0.0);
    _offsetProp.setName("location");
    _offsetProp.setValue(defaultAttachment);
    //_offsetProp.setAllowableListSize(3);
    _propertySet.append(&_offsetProp);

    _fixedProp.setComment("Flag (true or false) specifying whether or not a marker "
                          "should be kept fixed in the marker placement step.  i.e. If false, the marker is allowed to move.");
    _fixedProp.setName("fixed");
    _fixedProp.setValue(false);
    _propertySet.append(&_fixedProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this Marker.
 */
void Marker::connectMarkerToModel(const Model& aModel)
{

    _model = &aModel;

    if (_bodyName != "")
    {
        OpenSim::Body *oldBody = _body;
        try {
            _body = &const_cast<Model*>(&aModel)->updBodySet().get(_bodyName);
        }
        catch (const Exception&) {
            string errorMessage = "Error: Body " + _bodyName + " referenced in marker " + getName() +
                                  " does not exist in model " +	aModel.getName();
            throw Exception(errorMessage);
        }
        // If marker jumped between bodies we need to fix display stuff
        if (oldBody && oldBody->getName() != _body->getName()) {
            oldBody->updDisplayer()->removeDependent(&_displayer);
        }
        // Todo_AYMAN: make code below safe to call multiple times (since this
        // method may be called multiple times for a marker). Should also try
        // to handle case where a marker is deleted (e.g. in
        // SimbodyEngine::updateMarkerSet) because then may end up with
        // stale pointers.
        VisibleObject* ownerBodyDisplayer;
        if (_body && (ownerBodyDisplayer = _body->updDisplayer())) {
            if(! ownerBodyDisplayer->hasDependent(&_displayer)) {	// Only if first time to be encountered
                ownerBodyDisplayer->addDependent(&_displayer);
            }
        }
        _displayer.setOwner(this);
        updateGeometry();

    }
    else
    {
        string errorMessage = "Error: No body name specified for marker " + getName() + " in model " +
                              aModel.getName();
        throw Exception(errorMessage);
    }
}
//_____________________________________________________________________________
/**
 * Remove self from the list of displayable objects and free resources
 */
void Marker::removeSelfFromDisplay()
{
    VisibleObject* ownerBodyDisplayer;
    if (_body && (ownerBodyDisplayer = _body->updDisplayer())) {
        if (ownerBodyDisplayer->hasDependent(&_displayer)) {
            ownerBodyDisplayer->removeDependent(&_displayer);
        }
    }
}
//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Marker& Marker::operator=(const Marker &aMarker)
{
    // BASE CLASS
    Object::operator=(aMarker);

    copyData(aMarker);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Update an existing marker with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aMarker marker to update from
 */
void Marker::updateFromMarker(const Marker &aMarker)
{
    if (!aMarker.getOffsetUseDefault())
    {
        const Vec3& off = aMarker.getOffset();
        setOffset(off);
        _offsetProp.setValueIsDefault(false);
    }

    if (!aMarker.getFixedUseDefault())
    {
        _fixed = aMarker.getFixed();
        _fixedProp.setValueIsDefault(false);
    }

    if (!aMarker.getBodyNameUseDefault())
    {
        _bodyName = aMarker.getBodyName();
        _bodyNameProp.setValueIsDefault(false);
    }
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the marker's XYZ offset from the body it's attached to.
 *
 * @param rOffset XYZ offset is returned here.
 */
void Marker::getOffset(SimTK::Vec3& rOffset) const
{
    rOffset = _offset;
}

//_____________________________________________________________________________
/**
 * Get the marker's XYZ offset from the body it's attached to.
 *
 * @param rOffset XYZ offset is returned here.
 */
void Marker::getOffset(double rOffset[]) const
{
    rOffset[0] = _offset[0];
    rOffset[1] = _offset[1];
    rOffset[2] = _offset[2];
}

//_____________________________________________________________________________
/**
 * Set the marker's XYZ offset from the body it's attached to.
 *
 * @param aOffset XYZ offset to set to.
 * @return Whether or not the offset was set.
 */
bool Marker::setOffset(const SimTK::Vec3& aOffset)
{
    _offset = aOffset;

    updateGeometry();
    return true;
}

//_____________________________________________________________________________
/**
 * Set the marker's XYZ offset from the body it's attached to.
 *
 * @param aOffset XYZ offset to set to.
 * @return Whether or not the offset was set.
 */
bool Marker::setOffset(const double aOffset[3])
{
    _offset = aOffset;

    updateGeometry();
    return true;
}

//_____________________________________________________________________________
/**
 * Set the marker's fixed status.
 *
 * @param aFixed boolean value to set to.
 * @return Whether or not the fixed status was set.
 */
bool Marker::setFixed(bool aFixed)
{
    _fixed = aFixed;
    return true;
}

//_____________________________________________________________________________
/**
 * Set the 'body name' field, which is used when the marker is added to
 * an existing model.
 *
 * @param aName name of body
 * @return Whether or not the body name was set.
 */
bool Marker::setBodyName(const string& aName)
{
    _bodyName = aName;

    return true;
}

//_____________________________________________________________________________
/**
 * Get the 'body name' field, which is used when the marker is added to
 * an existing model.
 *
 * @return Pointer to the body name.
 */
const string& Marker::getBodyName() const
{
    //if (_bodyNameProp.getValueIsDefault())
    //	return NULL;

    return _bodyName;
}

//_____________________________________________________________________________
/**
 * Set the 'body name' field to use or not use the default value.
 *
 * @return Whether or not the flag was set properly.
 */
bool Marker::setBodyNameUseDefault(bool aValue)
{
    _bodyNameProp.setValueIsDefault(aValue);

    return true;
}

//_____________________________________________________________________________
/**
 * Change the body that this marker is attached to. It assumes that the body is
 * already set, so that connectMarkerToModel() needs to be called to update
 * dependent information.
 *
 * @param aBody Reference to the body.
 */
void Marker::changeBody( OpenSim::Body& aBody)
{

    if (&aBody == _body)
        return;

    setBodyName(aBody.getName());
    connectMarkerToModel(aBody.getModel());
}

//_____________________________________________________________________________
/**
 * Change the body that this marker is attached to. It assumes that the body is
 * already set, so that connectMarkerToModel() needs to be called to update
 * dependent information.
 *
 * @param s State.
 * @param aBody Reference to the body.
 */
void Marker::changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody)
{

    if (&aBody == _body || !_body)
        return;

    // Preserve location means to switch bodies without changing
    // the location of the marker in the inertial reference frame.
    aBody.getModel().getSimbodyEngine().transformPosition(s, *_body, _offset, aBody, _offset);

    setBodyName(aBody.getName());
    connectMarkerToModel(aBody.getModel());
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the marker.
 *
 * @param aScaleFactors XYZ scale factors.
 */
void Marker::scale(const SimTK::Vec3& aScaleFactors)
{
    for (int i = 0; i < 3; i++)
        _offset[i] *= aScaleFactors[i];
}

//_____________________________________________________________________________
/**
 * Update the geometry to correspond to position changes
 */
void Marker::updateGeometry()
{
    Transform position;
    position.setP(getOffset());
    updDisplayer()->setTransform(position);

}
