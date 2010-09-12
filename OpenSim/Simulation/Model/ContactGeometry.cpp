// ContactGeometry.cpp
// Author: Peter Eastman
/*
 * Copyright (c)  2009 Stanford University. All rights reserved. 
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

#include "ContactGeometry.h"
#include "BodySet.h"
#include "Model.h"

using SimTK::Vec3;
using SimTK::Rotation;

namespace OpenSim {

ContactGeometry::ContactGeometry() :
    _bodyName(_bodyNameProp.getValueStr()),
    _locationInBody(_locationInBodyProp.getValueDblVec()),
    _orientationInBody(_orientationInBodyProp.getValueDblVec())
{
	setNull();
	setupProperties();
}

ContactGeometry::ContactGeometry(const Vec3& location, const Vec3& orientation, Body& body) :
    _bodyName(_bodyNameProp.getValueStr()),
    _locationInBody(_locationInBodyProp.getValueDblVec()),
    _orientationInBody(_orientationInBodyProp.getValueDblVec())
{
	setNull();
	setupProperties();
    _body = &body;
    _bodyName = body.getName();
    _locationInBody = location;
    _orientationInBody = orientation;
}

ContactGeometry::ContactGeometry(const ContactGeometry& geom) : ModelComponent(geom),
    _bodyName(_bodyNameProp.getValueStr()),
    _locationInBody(_locationInBodyProp.getValueDblVec()),
    _orientationInBody(_orientationInBodyProp.getValueDblVec())
{
	setNull();
	setupProperties();
    copyData(geom);
}

ContactGeometry::~ContactGeometry()
{
}

void ContactGeometry::setNull()
{
    setType("ContactGeometry");
    _body = NULL;
}

void ContactGeometry::setupProperties()
{
	_bodyNameProp.setName("body_name");
	_propertySet.append(&_bodyNameProp);

	// Location in body
	_locationInBodyProp.setName("location");
    _locationInBodyProp.setValue(Vec3(0));
	_propertySet.append(&_locationInBodyProp);

	// Orientation in body
	_orientationInBodyProp.setName("orientation");
	_orientationInBodyProp.setValue(Vec3(0));
	_propertySet.append(&_orientationInBodyProp);
}

void ContactGeometry::copyData(const ContactGeometry& geom)
{
    _body = geom._body;
    _bodyName = geom._bodyName;
    _locationInBody = geom._locationInBody;
    _orientationInBody = geom._orientationInBody;
}

const Vec3& ContactGeometry::getLocation() const
{
    return _locationInBody;
}

void ContactGeometry::setLocation(const Vec3& location)
{
    _locationInBody = location;
}

const Vec3& ContactGeometry::getOrientation() const
{
    return _orientationInBody;
}

void ContactGeometry::setOrientation(const Vec3& orientation)
{
    _orientationInBody = orientation;
}

SimTK::Transform ContactGeometry::getTransform()
{
    return SimTK::Transform(Rotation(SimTK::BodyRotationSequence,
        _orientationInBody[0], SimTK::XAxis,
        _orientationInBody[1], SimTK::YAxis,
        _orientationInBody[2], SimTK::ZAxis), _locationInBody);
}

Body& ContactGeometry::getBody()
{
    return *_body;
}

const Body& ContactGeometry::getBody() const
{
    return *_body;
}

void ContactGeometry::setBody(Body& body)
{
    _body = &body;
    _bodyName = body.getName();
}

const std::string& ContactGeometry::getBodyName()
{
    return _bodyName;
}

void ContactGeometry::setBodyName(const std::string& name)
{
    _bodyName = name;
    _body = NULL;
}

void ContactGeometry::setup(Model& aModel)
{
    try {
    	_body = &aModel.updBodySet().get(_bodyName);
		_model = &aModel;
    }
	catch (...)
    {
        std::string errorMessage = "Invalid body (" + _bodyName + ") specified in contact geometry " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_body->updDisplayer()->addDependent(updDisplayer());
	_displayer.setTransform(getTransform());
	_displayer.setOwner(this);
}

void ContactGeometry::scale(const ScaleSet& aScaleSet)
{
    throw Exception("ContactGeometry::Scale is not implemented");
}

} // end of namespace OpenSim
