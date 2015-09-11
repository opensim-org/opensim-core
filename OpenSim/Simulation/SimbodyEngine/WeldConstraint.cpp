/* -------------------------------------------------------------------------- *
 *                        OpenSim:  WeldConstraint.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
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
#include "WeldConstraint.h"
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
WeldConstraint::~WeldConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WeldConstraint::WeldConstraint() :
    Constraint()
{
    setNull();
    constructInfrastructure();
}

WeldConstraint::WeldConstraint( const std::string &name,
                                const std::string& frame1Name,
                                const std::string& frame2Name) : WeldConstraint()
{
    setName(name);
    updConnector<PhysicalFrame>("frame1").set_connectee_name(frame1Name);
    updConnector<PhysicalFrame>("frame2").set_connectee_name(frame2Name);
}

WeldConstraint::WeldConstraint(const std::string &name,
    const PhysicalFrame& frame1,
    const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
    const PhysicalFrame& frame2,
    const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2) :
    WeldConstraint(name, 
        frame1, Transform(Rotation(BodyRotationSequence,
            orientationInFrame1[0], XAxis,
            orientationInFrame1[1], YAxis,
            orientationInFrame1[2], ZAxis), locationInFrame1),
        frame2, Transform(Rotation(BodyRotationSequence,
            orientationInFrame2[0], XAxis,
            orientationInFrame2[1], YAxis,
            orientationInFrame2[2], ZAxis), locationInFrame2) )
{}

WeldConstraint::WeldConstraint(const std::string &name,
    const PhysicalFrame& frame1, const SimTK::Transform& transformInFrame1,
    const PhysicalFrame& frame2, const SimTK::Transform& transformInFrame2)
    : WeldConstraint()
{
    setName(name);

    PhysicalOffsetFrame frame1Offset(frame1.getName() + "_offset",
                                     frame1, transformInFrame1);

    PhysicalOffsetFrame frame2Offset(frame2.getName() + "_offset", 
                                    frame2, transformInFrame2);

    // Append the offset frames to the Joints internal list of frames
    append_frames(frame1Offset);
    append_frames(frame2Offset);

    updConnector<PhysicalFrame>("frame1").set_connectee_name(
        getName() + "/" + frame1Offset.getName() );
    updConnector<PhysicalFrame>("frame2").set_connectee_name(
        getName() + "/" + frame2Offset.getName() );
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this WeldConstraint to their null values.
 */
void WeldConstraint::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/*
 * Construct WeldConstraint's properties
 */
void WeldConstraint::constructProperties()
{
    //Default frames list is empty
    constructProperty_frames();
}

void WeldConstraint::constructConnectors()
{
    constructConnector<PhysicalFrame>("frame1");
    constructConnector<PhysicalFrame>("frame2");
}

void WeldConstraint::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    for (int i = 0; i < getProperty_frames().size(); ++i) {
        addComponent(&upd_frames(i));
    }
}

void WeldConstraint::
    extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystemAfterSubcomponents(system);

    // Get underlying mobilized bodies
    const PhysicalFrame& f1 = 
        getConnector<PhysicalFrame>("frame1").getConnectee();
    const PhysicalFrame& f2 = 
        getConnector<PhysicalFrame>("frame2").getConnectee();

    SimTK::MobilizedBody b1 = f1.getMobilizedBody();
    SimTK::MobilizedBody b2 = f2.getMobilizedBody();
    // Build the transforms
    SimTK::Transform inb1 = f1.findTransformInBaseFrame();
    SimTK::Transform inb2 = f2.findTransformInBaseFrame();

    // Now create a Simbody Constraint::Weld
    SimTK::Constraint::Weld simtkWeld(b1, inb1, b2, inb2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    assignConstraintIndex(simtkWeld.getConstraintIndex());
}

void WeldConstraint::
    setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
    // make sure we are at the position stage
    getSystem().realize(s, SimTK::Stage::Position);

    const PhysicalFrame& frame1 = 
        getConnector<PhysicalFrame>("frame1").getConnectee();
    const PhysicalFrame& frame2 =
        getConnector<PhysicalFrame>("frame2").getConnectee();
    
    // For external forces we assume point position vector is defined 
    // wrt foot (i.e., frame2) because we are passing it in from a
    // prescribed force.
    // We must also get that point position vector wrt ground (i.e., frame1)
    Vec3 spoint = frame2.findLocationInAnotherFrame(s, point, frame1);

    SimTK::Transform in1(frame1.getGroundTransform(s).R(), spoint);
    SimTK::Transform in2(frame2.getGroundTransform(s).R(), point);

    // Add new internal PhysicalOffSetFrames that the Constraint can update 
    // without affecting any other components, ONLY if they don't exist already
    if (_internalOffset1.empty()) {
        _internalOffset1 = new PhysicalOffsetFrame(frame1, in1);
        _internalOffset1->setName("internal_" + frame1.getName());
        updProperty_frames().adoptAndAppendValue(_internalOffset1.get());
        updConnector<PhysicalFrame>("frame1").connect(*_internalOffset1);
    }
    else { // otherwise if is already "wired" up so just update
        _internalOffset1->setOffsetTransform(in1);
    }

    if (_internalOffset2.empty()) {
        _internalOffset2 = new PhysicalOffsetFrame(frame2, in2);
        _internalOffset2->setName("internal_" + frame2.getName());
        updProperty_frames().adoptAndAppendValue(_internalOffset2.get());
        updConnector<PhysicalFrame>("frame2").connect(*_internalOffset2);
    }
    else {
        _internalOffset2->setOffsetTransform(in2);
    }
}

void WeldConstraint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if (documentVersion < XMLDocument::getLatestVersion()) {
        if (documentVersion < 30505) {
            // replace old properties with latest use of PhysicalOffsetFrames properties
            SimTK::Xml::element_iterator body1Element =
                aNode.element_begin("body_1");
            SimTK::Xml::element_iterator body2Element =
                aNode.element_begin("body_2");
            SimTK::Xml::element_iterator locBody1Elt =
                aNode.element_begin("location_body_1");
            SimTK::Xml::element_iterator orientBody1Elt =
                aNode.element_begin("orientation_body_1");
            SimTK::Xml::element_iterator locBody2Elt =
                aNode.element_begin("location_body_2");
            SimTK::Xml::element_iterator orientBody2Elt =
                aNode.element_begin("orientation_body_2");

            // The names of the two PhysicalFrames this bushing connects
            std::string frame1Name("");
            std::string frame2Name("");

            if (body1Element != aNode.element_end()) {
                body1Element->getValueAs<std::string>(frame1Name);
            }

            if (body2Element != aNode.element_end()) {
                body2Element->getValueAs<std::string>(frame2Name);
            }

            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_",
                "frame1", frame1Name);
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_",
                "frame2", frame2Name);

            Vec3 locationInFrame1(0);
            Vec3 orientationInFrame1(0);
            Vec3 locationInFrame2(0);
            Vec3 orientationInFrame2(0);

            if (locBody1Elt != aNode.element_end()) {
                locBody1Elt->getValueAs<Vec3>(locationInFrame1);
            }
            if (orientBody1Elt != aNode.element_end()) {
                orientBody1Elt->getValueAs<Vec3>(orientationInFrame1);
            }
            if (locBody2Elt != aNode.element_end()) {
                locBody2Elt->getValueAs<Vec3>(locationInFrame2);
            }
            if (orientBody2Elt != aNode.element_end()) {
                orientBody2Elt->getValueAs<Vec3>(orientationInFrame2);
            }

            // Avoid collision by prefixing the joint name to the connectee_name
            // as to enforce a local search for the correct local offset
            std::string pName = aNode.getOptionalAttributeValueAs<std::string>("name", "");

            // now append updated frames to the property list if they are not
            // identity transforms.
            if ((locationInFrame1.norm() > 0.0) ||
                (orientationInFrame1.norm() > 0.0)) {
                XMLDocument::addPhysicalOffsetFrame(aNode, frame1Name + "_offset",
                    frame1Name, locationInFrame1, orientationInFrame1);
                body1Element->setValue(pName + "/" + frame1Name + "_offset");
            }

            // again for the offset frame on the child
            if ((locationInFrame2.norm() > 0.0) ||
                (orientationInFrame2.norm() > 0.0)) {
                XMLDocument::addPhysicalOffsetFrame(aNode, frame2Name + "_offset",
                    frame2Name, locationInFrame2, orientationInFrame2);
                body2Element->setValue(pName + "/" + frame2Name + "_offset");
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}
