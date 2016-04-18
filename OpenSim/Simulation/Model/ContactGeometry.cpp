/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactGeometry.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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

#include "ContactGeometry.h"
// TODO #include "BodySet.h"
// TODO #include "Model.h"
#include "PhysicalOffsetFrame.h"
#include <OpenSim/Common/ScaleSet.h>

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Rotation;

// TODO test ContactSphere visualization.
// TODO test updateFromXML.

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
    constructInfrastructure();
}

//_____________________________________________________________________________
// Convenience constructor.
ContactGeometry::ContactGeometry(const Vec3& location, const Vec3& orientation, 
    PhysicalFrame& frame) : ContactGeometry()
{
    // This hack is here because `addComponent()` calls a pure virtual function
    // (getConcreteClassName), and we are not supposed to do that in
    // constructors. We can avoid the call to getConcreteClassName() by setting
    // the name here.
    // TODO
    setName("ContactGeometry");

    Rotation rotation(SimTK::BodyRotationSequence,
            orientation[0], SimTK::XAxis,
            orientation[1], SimTK::YAxis,
            orientation[2], SimTK::ZAxis);
    SimTK::Transform transform(rotation, location);
    auto* offsetFrame = new PhysicalOffsetFrame(frame.getName() + "_offset",
                                                frame, transform);
    // TODO PhysicalOffsetFrame offsetFrame(frame.getName() + "_offset",
    // TODO                                             frame, transform);
    addComponent(offsetFrame);
    // TODO int cix = append_components(offsetFrame);

    offsetFrame->setParentFrame(frame);
    updConnector(0).connect(*offsetFrame);
    // TODO upd_components(cix).updConnector<PhysicalFrame>("parent").setConnecteeName(frame.getName());
//    static_cast<PhysicalOffsetFrame&>(upd_components(cix)).setParentFrame(frame);
    // TODO updConnector<PhysicalFrame>("frame").connect(get_components(cix));

    //// TODO create an intermediate offset frame.
    //set_body_name(body.getName());
    //set_location(location);
    //set_orientation(orientation);
}

void ContactGeometry::setNull()
{
    setAuthors("Peter Eastman");
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ContactGeometry::constructProperties()
{
    // TODO constructProperty_body_name("Unassigned");
    // TODO constructProperty_location(Vec3(0));
    // TODO constructProperty_orientation(Vec3(0));
    constructProperty_display_preference(1);

    Array<double> defaultColor(1.0, 3); //color default to 0, 1, 1
    defaultColor[0] = 0.0; 
    constructProperty_color(defaultColor);
}

void ContactGeometry::constructConnectors()
{
    constructConnector<PhysicalFrame>("frame");
}

/* TODO
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
*/

SimTK::Transform ContactGeometry::getTransform() const
{
    return getFrame().findTransformInBaseFrame();
    // TODO return SimTK::Transform(Rotation(SimTK::BodyRotationSequence,
    // TODO     get_orientation()[0], SimTK::XAxis,
    // TODO     get_orientation()[1], SimTK::YAxis,
    // TODO     get_orientation()[2], SimTK::ZAxis), get_location());
}

const PhysicalFrame& ContactGeometry::getFrame() const
{
    return getConnector<PhysicalFrame>("frame").getConnectee();
}

void ContactGeometry::setFrame(PhysicalFrame& frame)
{
    updConnector<PhysicalFrame>("frame").setConnecteeName(
            frame.getRelativePathName(*this));
}

const std::string& ContactGeometry::getFrameName() const
{
    return getConnector<PhysicalFrame>("frame").getConnecteeName();
}

void ContactGeometry::setFrameName(const std::string& name)
{
    updConnector<PhysicalFrame>("frame").setConnecteeName(name);
}

const int ContactGeometry::getDisplayPreference()
{
    return get_display_preference();
}

void ContactGeometry::setDisplayPreference(const int dispPref)
{
    set_display_preference(dispPref);
}

// TODO PhysicalFrame& ContactGeometry::updBody()
// TODO {
// TODO     return updFrame();
// TODO }

const PhysicalFrame& ContactGeometry::getBody() const
{ return getFrame(); }

void ContactGeometry::setBody(PhysicalFrame& frame)
{ setFrame(frame); }

const std::string& ContactGeometry::getBodyName() const
{ return getFrameName(); }

void ContactGeometry::setBodyName(const std::string& name)
{ setFrameName(name); }

// TODO void ContactGeometry::extendConnectToModel(Model& aModel)
// TODO {
// TODO     Super::extendConnectToModel(aModel);
// TODO 
// TODO     //TODO use Connectors!
// TODO     try {
// TODO         _body =
// TODO             static_cast<PhysicalFrame*>(&updModel().updComponent(get_body_name()));
// TODO     }
// TODO     catch (...)
// TODO     {
// TODO         std::string errorMessage = "Invalid body (" + get_body_name() + ") specified in contact geometry " + getName();
// TODO         throw (Exception(errorMessage.c_str()));
// TODO     }
// TODO }

void ContactGeometry::scale(const ScaleSet& aScaleSet)
{
    throw Exception("ContactGeometry::Scale is not implemented");
}

void ContactGeometry::updateFromXMLNode(SimTK::Xml::Element& node,
                                        int versionNumber) {
    Super::updateFromXMLNode(node, versionNumber);
}

void ContactGeometry::setFrameWithOffset(const SimTK::Vec3& location,
        const SimTK::Vec3& orientation,
        PhysicalFrame& frame,
        ContactGeometry& geom) {

    Rotation rotation(SimTK::BodyRotationSequence,
            orientation[0], SimTK::XAxis,
            orientation[1], SimTK::YAxis,
            orientation[2], SimTK::ZAxis);
    SimTK::Transform transform(rotation, location);
    auto* offsetFrame = new PhysicalOffsetFrame(frame.getName() + "_offset",
                                                frame, transform);
    geom.addComponent(offsetFrame);

    offsetFrame->setParentFrame(frame);
    geom.updConnector<PhysicalFrame>("frame").connect(*offsetFrame);
}












