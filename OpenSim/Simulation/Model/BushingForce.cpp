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
    constructInfrastructure();
}

/* Convenience construction that creates the Offsets on the Physical Frames (e.g.
   Bodies that the BushingForce acts between. */
BushingForce::BushingForce( const std::string& frame1_name,
                            const std::string& frame2_name,
                            const SimTK::Vec3& transStiffness,
                            const SimTK::Vec3& rotStiffness,
                            const SimTK::Vec3& transDamping,
                            const SimTK::Vec3& rotDamping )
{
    setNull();
    constructInfrastructure();

    updConnector<PhysicalFrame>("frame1").set_connected_to_name(frame1_name);
    updConnector<PhysicalFrame>("frame2").set_connected_to_name(frame2_name);

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
    // default bushing material properties
    constructProperty_rotational_stiffness(Vec3(0));
    constructProperty_translational_stiffness(Vec3(0));
    constructProperty_rotational_damping(Vec3(0));
    constructProperty_translational_damping(Vec3(0));

    //Default frames list is empty
    constructProperty_frames();
}

//_____________________________________________________________________________
/*
* Construct Structural Connectors
*/
void BushingForce::constructConnectors() {
    constructConnector<PhysicalFrame>("frame1");
    constructConnector<PhysicalFrame>("frame2");
}

void BushingForce::updateFromXMLNode( SimTK::Xml::Element& aNode,
                                      int versionNumber)
{
    int documentVersion = versionNumber;
    bool converting = false;
    if (documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion < 30503){
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
            std::string frame1_name("");
            std::string frame2_name("");

            // Create two new PhysicalOffsetFrames
            PhysicalOffsetFrame frame1_offset;
            PhysicalOffsetFrame frame2_offset;
            frame1_offset.setName("frame1_offset");
            frame2_offset.setName("frame2_offset");

            // If default constructed then elements not serialized since they are default
            // values. Check that we have associated elements, then extract their values.
            if (body1Element != aNode.element_end()){
                body1Element->getValueAs<std::string>(frame1_name);
                frame1_offset.updConnector(0).set_connected_to_name(frame1_name);
            }
            if (body2Element != aNode.element_end()){
                body2Element->getValueAs<std::string>(frame2_name);
                frame2_offset.updConnector(0).set_connected_to_name(frame2_name);
            }
            if (locBody1Elt != aNode.element_end()){
                Vec3 location;
                locBody1Elt->getValueAs<Vec3>(location);
                frame1_offset.set_translation(location);
            }
            if (orientBody1Elt != aNode.element_end()){
                Vec3 orientation;
                orientBody1Elt->getValueAs<Vec3>(orientation);
                frame1_offset.set_orientation(orientation);
            }
            if (locBody2Elt != aNode.element_end()){
                Vec3 location;
                locBody2Elt->getValueAs<Vec3>(location);
                frame2_offset.set_translation(location);
            }
            if (orientBody2Elt != aNode.element_end()){
                Vec3 orientation;
                orientBody2Elt->getValueAs<Vec3>(orientation);
                frame2_offset.set_orientation(orientation);
            }

            // now append updated frames to the property list if they are not
            // identity transforms.
            if ((frame1_offset.get_translation().norm() > 0.0) &&
                (frame1_offset.get_orientation().norm() > 0.0)) {
                append_frames(frame1_offset);
                updConnector(0).set_connected_to_name(frame1_offset.getName());
            }
            else { // connect directly to the frame (body) that was identified by name
                updConnector(0).set_connected_to_name(frame1_name);
            }
            // again for frame2
            if ((frame2_offset.get_translation().norm() > 0.0) &&
                (frame2_offset.get_orientation().norm() > 0.0)) {
                append_frames(frame2_offset);
                updConnector(0).set_connected_to_name(frame2_offset.getName());
            }
            else { // connect directly to the frame (body) that was identified by name
                updConnector(1).set_connected_to_name(frame2_name);
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

void BushingForce::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    //mark frames in property list as subcomponents
    for (int i = 0; i < updProperty_frames().size(); ++i){
        addComponent(&upd_frames(i));
    }
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

    // get connected frames
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    // Get underlying mobilized bodies
    const SimTK::MobilizedBody& b1 = frame1.getMobilizedBody();
    const SimTK::MobilizedBody& b2 = frame2.getMobilizedBody();

    Vec6 stiffness(rotStiffness[0], rotStiffness[1], rotStiffness[2], 
                   transStiffness[0], transStiffness[1], transStiffness[2]);
    Vec6 damping(rotDamping[0], rotDamping[1], rotDamping[2], 
                 transDamping[0], transDamping[1], transDamping[2]);

    // Now create a Simbody Force::LinearBushing
    SimTK::Force::LinearBushing simtkForce
        (_model->updForceSubsystem(), b1, frame1.findTransformInBaseFrame(),
                                      b2, frame2.findTransformInBaseFrame(),
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
/*
 * Provide names of the quantities (column labels) of the force value(s) to be 
 * reported.
 */
OpenSim::Array<std::string> BushingForce::getRecordLabels() const 
{
    const string& frame1Name = getConnectee<PhysicalFrame>("frame1").getName();
    const string& frame2Name = getConnectee<PhysicalFrame>("frame2").getName();

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
/*
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> BushingForce::
getRecordValues(const SimTK::State& state) const 
{
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    const string& frame1Name = frame1.getName();
    const string& frame2Name = frame2.getName();

    OpenSim::Array<double> values(1);

    const SimTK::Force::LinearBushing &simtkSpring = 
        (SimTK::Force::LinearBushing &)(_model->getForceSubsystem().getForce(_index));

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    //get the net force added to the system contributed by the bushing
    simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
    SimTK::Vec3 forces = bodyForces[frame1.getMobilizedBodyIndex()][1];
    SimTK::Vec3 torques = bodyForces[frame1.getMobilizedBodyIndex()][0];
    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    forces = bodyForces[frame2.getMobilizedBodyIndex()][1];
    torques = bodyForces[frame2.getMobilizedBodyIndex()][0];

    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    return values;
}
