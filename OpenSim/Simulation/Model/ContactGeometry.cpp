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

//=============================================================================
// CONSTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
ContactGeometry::ContactGeometry() : ModelComponent()
{
	setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convienience constructor.
ContactGeometry::ContactGeometry(const Vec3& location, const Vec3& orientation, Body& body) : ModelComponent()
{
	setNull();
    constructProperties();

    _body = &body;
    set_body_name(body.getName());
    set_location(location);
    set_orientation(orientation);
}



void ContactGeometry::setNull()
{
    _body = NULL;
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ContactGeometry::constructProperties()
{
    constructProperty_body_name("Unassigned");
    constructProperty_location(Vec3(0));
    constructProperty_orientation(Vec3(0));
    constructProperty_display_preference(1);

    Array<double> defaultColor(1.0, 3); //color default to 0, 1, 1
	defaultColor[0] = 0.0; 
    constructProperty_color(defaultColor);
}

const Vec3& ContactGeometry::getLocation() const
{
    return get_location();
}

void ContactGeometry::setLocation(const Vec3& location)
{
    set_location(location);
}

const Vec3& ContactGeometry::getOrientation() const
{
    return get_orientation();
}

void ContactGeometry::setOrientation(const Vec3& orientation)
{
    set_orientation(orientation);
}

SimTK::Transform ContactGeometry::getTransform()
{
    return SimTK::Transform(Rotation(SimTK::BodyRotationSequence,
        get_orientation()[0], SimTK::XAxis,
        get_orientation()[1], SimTK::YAxis,
        get_orientation()[2], SimTK::ZAxis), get_location());
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
    set_body_name(body.getName());
}

const std::string& ContactGeometry::getBodyName()
{
    return get_body_name();
}

void ContactGeometry::setBodyName(const std::string& name)
{
    set_body_name(name);
    _body = NULL;
}

const int ContactGeometry::getDisplayPreference()
{
    return get_display_preference();
}

void ContactGeometry::setDisplayPreference(const int dispPref)
{
    set_display_preference(dispPref);
}

void ContactGeometry::connectToModel(Model& aModel)
{
    try {
    	_body = &aModel.updBodySet().get(get_body_name());
		_model = &aModel;
    }
	catch (...)
    {
        std::string errorMessage = "Invalid body (" + get_body_name() + ") specified in contact geometry " + getName();
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
