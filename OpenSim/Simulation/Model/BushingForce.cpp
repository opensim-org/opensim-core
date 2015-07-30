/* -------------------------------------------------------------------------- *
 *                         OpenSim:  BushingForce.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "BushingForce.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
BushingForce::BushingForce()
{
    setNull();
    constructProperties();
}

/* Construct a BusingForce by supplying the Offset frames that the BushingForce
   acts between */
BushingForce::BushingForce(const PhysicalOffsetFrame& frame1,
    const PhysicalOffsetFrame& frame2,
    const SimTK::Vec3& transStiffness,
    const SimTK::Vec3& rotStiffness,
    const SimTK::Vec3& transDamping,
    const SimTK::Vec3& rotDamping)
{
    set_offset_frame1(frame1);
    set_offset_frame2(frame2);
    set_rotational_stiffness(rotStiffness);
    set_translational_stiffness(transStiffness);
    set_rotational_damping(rotDamping);
    set_translational_damping(transDamping);
}

/* Convenience construction that creates the Offsets on the Physical Frames (e.g.
   Bodies that the BushingForce acts between. */
BushingForce::BushingForce(const PhysicalFrame& frame1,
    const SimTK::Vec3& point1,
    const SimTK::Vec3& orientation1,
    const PhysicalFrame& frame2,
    const SimTK::Vec3& point2,
    const SimTK::Vec3& orientation2,
    const SimTK::Vec3& transStiffness,
    const SimTK::Vec3& rotStiffness,
    const SimTK::Vec3& transDamping,
    const SimTK::Vec3& rotDamping)
{
    setNull();
    constructProperties();

    upd_offset_frame1().updConnector<PhysicalFrame>("parent")
        .set_connected_to_name(frame1.getName());
    Rotation rotation1(BodyRotationSequence,
        orientation1[0], XAxis,
        orientation1[1], YAxis,
        orientation1[2], ZAxis);
    upd_offset_frame1().setOffsetTransform(Transform(rotation1, point1));

    upd_offset_frame2().updConnector<PhysicalFrame>("parent")
        .set_connected_to_name(frame2.getName());
    Rotation rotation2(BodyRotationSequence,
        orientation2[0], XAxis,
        orientation2[1], YAxis,
        orientation2[2], ZAxis);
    upd_offset_frame2().setOffsetTransform(Transform(rotation2, point2));

    set_rotational_stiffness(rotStiffness);
    set_translational_stiffness(transStiffness);
    set_rotational_damping(rotDamping);
    set_translational_damping(transDamping);
}

/* Convenience construction that creates the Offsets on the PhysicalFrames (e.g.
   Bodies) that the BushingForce acts between. In this case, PhysicalFrames are 
   identified by name. */
BushingForce::BushingForce(const string&    frame1Name,
    const Vec3&      point1,
    const Vec3&      orientation1,
    const string&    frame2Name,
    const Vec3&      point2,
    const Vec3&      orientation2,
    const Vec3&      transStiffness,
    const Vec3&      rotStiffness,
    const Vec3&      transDamping,
    const Vec3&      rotDamping)
{
    setNull();
    constructProperties();

    upd_offset_frame1().updConnector<PhysicalFrame>("parent")
        .set_connected_to_name(frame1Name);
    Rotation rotation1(BodyRotationSequence,
        orientation1[0], XAxis,
        orientation1[1], YAxis,
        orientation1[2], ZAxis);
    upd_offset_frame1().setOffsetTransform(Transform(rotation1, point1));

    upd_offset_frame2().updConnector<PhysicalFrame>("parent")
        .set_connected_to_name(frame2Name);
    Rotation rotation2(BodyRotationSequence,
        orientation2[0], XAxis,
        orientation2[1], YAxis,
        orientation2[2], ZAxis);
    upd_offset_frame2().setOffsetTransform(Transform(rotation2, point2));


    set_rotational_stiffness(rotStiffness);
    set_translational_stiffness(transStiffness);
    set_rotational_damping(rotDamping);
    set_translational_damping(transDamping);
}

//_____________________________________________________________________________
// Set the data members of this BushingForce to their null values.
void BushingForce::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void BushingForce::constructProperties()
{
    //Default frames
    PhysicalOffsetFrame frame1;
    PhysicalOffsetFrame frame2;
    frame1.setName("offset_frame1");
    frame2.setName("offset_frame2");

    constructProperty_offset_frame1(frame1);
    constructProperty_offset_frame2(frame2);

    // default bushing material properties
    constructProperty_rotational_stiffness(Vec3(0));
    constructProperty_translational_stiffness(Vec3(0));
    constructProperty_rotational_damping(Vec3(0));
    constructProperty_translational_damping(Vec3(0));
}

void BushingForce::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    bool converting = false;
    if (documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion < 30503){
            // replace old properties with latest use of PhysicalOffsetFrames properties
            SimTK::Xml::element_iterator body1Element = aNode.element_begin("body_1");
            SimTK::Xml::element_iterator body2Element = aNode.element_begin("body_2");
            SimTK::Xml::element_iterator locBody1Elt = aNode.element_begin("location_body_1");
            SimTK::Xml::element_iterator orientBody1Elt = aNode.element_begin("orientation_body_1");
            SimTK::Xml::element_iterator locBody2Elt = aNode.element_begin("location_body_2");
            SimTK::Xml::element_iterator orientBody2Elt = aNode.element_begin("orientation_body_2");

            // If default constructed then elements not serialized since they are default
            // values. Check that we have associated elements, then extract their values.
            if (body1Element != aNode.element_end()){
                std::string frame1_name("");
                body1Element->getValueAs<std::string>(frame1_name);
                upd_offset_frame1().updConnector(0).set_connected_to_name(frame1_name);
            }
            if (body2Element != aNode.element_end()){
                std::string frame2_name("");
                body2Element->getValueAs<std::string>(frame2_name);
                upd_offset_frame2().updConnector(0).set_connected_to_name(frame2_name);
            }
            if (locBody1Elt != aNode.element_end()){
                Vec3 location;
                locBody1Elt->getValueAs<Vec3>(location);
                upd_offset_frame1().set_translation(location);
            }
            if (orientBody1Elt != aNode.element_end()){
                Vec3 orientation;
                orientBody1Elt->getValueAs<Vec3>(orientation);
                upd_offset_frame1().set_orientation(orientation);
            }
            if (locBody2Elt != aNode.element_end()){
                Vec3 location;
                locBody2Elt->getValueAs<Vec3>(location);
                upd_offset_frame2().set_translation(location);
            }
            if (orientBody2Elt != aNode.element_end()){
                Vec3 orientation;
                orientBody2Elt->getValueAs<Vec3>(orientation);
                upd_offset_frame2().set_orientation(orientation);
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

void BushingForce::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    //mark the two PhysicalOffsetFrames as subcomponents 
    addComponent(&upd_offset_frame1());
    addComponent(&upd_offset_frame2());
}

// Add underly Simbody elements to the System after subcomponents
void BushingForce::
    extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystemAfterSubcomponents(system);

    const SimTK::Vec3& rotStiffness         = get_rotational_stiffness();
    const SimTK::Vec3& transStiffness       = get_translational_stiffness();
    const SimTK::Vec3& rotDamping           = get_rotational_damping();
    const SimTK::Vec3& transDamping         = get_translational_damping();

    // Get underlying mobilized bodies
    const SimTK::MobilizedBody& b1 = get_offset_frame1().getMobilizedBody();
    const SimTK::MobilizedBody& b2 = get_offset_frame2().getMobilizedBody();

    Vec6 stiffness(rotStiffness[0], rotStiffness[1], rotStiffness[2], 
                   transStiffness[0], transStiffness[1], transStiffness[2]);
    Vec6 damping(rotDamping[0], rotDamping[1], rotDamping[2], 
                 transDamping[0], transDamping[1], transDamping[2]);

    // Now create a Simbody Force::LinearBushing
    SimTK::Force::LinearBushing simtkForce
        (_model->updForceSubsystem(), b1, get_offset_frame1().getOffsetTransform(),
                                      b2, get_offset_frame2().getOffsetTransform(), 
                                      stiffness, damping );
    
    // Beyond the const Component get the index so we can access the 
    // SimTK::Force later.
    BushingForce* mutableThis = const_cast<BushingForce *>(this);
    mutableThis->_index = simtkForce.getForceIndex();
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
// The following methods set properties of the bushing Force.


/* Potential energy is computed by underlying SimTK::Force. */
double BushingForce::computePotentialEnergy(const SimTK::State& s) const
{
    return _model->getForceSubsystem().getForce(_index)
                                      .calcPotentialEnergyContribution(s);
}

//=============================================================================
// Reporting
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> BushingForce::getRecordLabels() const 
{
    const string& frame1Name = get_offset_frame1().getName();
    const string& frame2Name = get_offset_frame2().getName();

    OpenSim::Array<std::string> labels("");
    labels.append(getName()+"."+frame1Name+".force.X");
    labels.append(getName()+"."+frame1Name+".force.Y");
    labels.append(getName()+"."+frame1Name+".force.Z");
    labels.append(getName()+"."+frame1Name+".torque.X");
    labels.append(getName()+"."+frame1Name+".torque.Y");
    labels.append(getName()+"."+frame1Name+".torque.Z");
    labels.append(getName()+"."+frame2Name+".force.X");
    labels.append(getName()+"."+frame2Name+".force.Y");
    labels.append(getName()+"."+frame2Name+".force.Z");
    labels.append(getName()+"."+frame2Name+".torque.X");
    labels.append(getName()+"."+frame2Name+".torque.Y");
    labels.append(getName()+"."+frame2Name+".torque.Z");

    return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> BushingForce::
getRecordValues(const SimTK::State& state) const 
{
    const string& frame1Name = get_offset_frame1().getName();
    const string& frame2Name = get_offset_frame2().getName();

    OpenSim::Array<double> values(1);

    const SimTK::Force::LinearBushing &simtkSpring = 
        (SimTK::Force::LinearBushing &)(_model->getForceSubsystem().getForce(_index));

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    //get the net force added to the system contributed by the bushing
    simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
    SimTK::Vec3 forces = bodyForces(get_offset_frame1().getMobilizedBodyIndex())[1];
    SimTK::Vec3 torques = bodyForces(get_offset_frame1().getMobilizedBodyIndex())[0];
    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    forces = bodyForces(get_offset_frame2().getMobilizedBodyIndex())[1];
    torques = bodyForces(get_offset_frame2().getMobilizedBodyIndex())[0];

    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    return values;
}
