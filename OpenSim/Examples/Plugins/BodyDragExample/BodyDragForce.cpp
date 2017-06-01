/* -------------------------------------------------------------------------- *
 *                           BodyDragForce.cpp                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "BodyDragForce.h"

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
 * Default constructor
 */
BodyDragForce::BodyDragForce() : Force()
{
    setNull();
    constructProperties();
}



//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this BodyDragForce to their null values.
 */
void BodyDragForce::setNull()
{
    // no internal data members to initialize.
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BodyDragForce::constructProperties()
{
    constructProperty_body_name("Unassigned");
    constructProperty_coefficient(0.5);
    constructProperty_exponent(2.0);

    // Here are some examples of other constructing other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //constructProperty_string_property("defaultString");
    //constructProperty_int_property(10);
    //constructProperty_bool_property(true);
    //constructProperty_double_property(1.5);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this BodyDragForce.
 */
void BodyDragForce::connectToModel(Model& aModel)
{
    string errorMessage;

    // Base class
    Super::connectToModel(aModel);

    // Look up the body and report an error if it is not found 
    if (!aModel.updBodySet().contains(get_body_name())) {
        errorMessage = "Invalid bodyName (" + get_body_name() + ") specified in Force " + getName();
        throw (Exception(errorMessage.c_str()));
    }
}


//=============================================================================
// COMPUTATION
//=============================================================================

void BodyDragForce::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    if(!_model) return;     // some minor error checking

    SimTK::Vec3 bodyCoMPosBody, bodyCoMPosGround, bodyCoMVelGround, bodyCoMVelGroundRaisedPower, dragForceGround, dragForceBody, oppVelSign;
    BodySet &bs = _model->updBodySet();                                     // get body set
    const Ground &ground = _model->getGround(); // get ground body
    Body &aBody = bs.get(get_body_name());                                      // get the body to apply the force to

    // get CoM position of body in the BODY coordinate system
    bodyCoMPosBody = aBody.getMassCenter();
    // get CoM position of body in the GROUND coordinate system
    bodyCoMPosGround = aBody.getPositionInGround(s);
    // get CoM velocity of body in the GROUND coordinate system
    bodyCoMVelGround = aBody.findStationVelocityInGround(s, bodyCoMPosBody);

    for (int i=0; i<3;i++)
    {
        if (bodyCoMVelGround[i]>0) oppVelSign[i] = -1;                                      // get opposite sign of CoM velocity (GROUND coordinate system)
        if (bodyCoMVelGround[i]<0) oppVelSign[i] = 1;
        if (bodyCoMVelGround[i]==0) oppVelSign[i] = 0;

        dragForceGround[i] = oppVelSign[i] * get_coefficient() * std::pow(bodyCoMVelGround[i], get_exponent()); // calculate drag force in the GROUND coordinate system
    }

    // transform drag force into the BODY coordinate system
    dragForceBody = ground.expressVectorInAnotherFrame(s,
                                                       dragForceGround,
                                                       aBody);

    // Apply drag force to the body
    // ------------------------------
    // applyForceToPoint requires the force application point to be in the inertial (ground) frame
    // and the force vector itself to be in the body frame
    applyForceToPoint(s, aBody, bodyCoMPosGround, dragForceBody, bodyForces);




    // Debuging info
    // --------------
    int deb = 0;
    if (deb)
    {
        cout << "Time = " << s.getTime() << endl;
        cout << aBody.getName() << " CoM position (body frame) = " << bodyCoMPosBody << endl;
        cout << aBody.getName() << " CoM position (ground frame) = " << bodyCoMPosGround << endl;
        cout << aBody.getName() << " CoM velocity (ground frame) = " << bodyCoMVelGround << endl;
        cout << aBody.getName() << " CoM velocity opposite sign (ground frame) = " << oppVelSign << endl;
        cout << aBody.getName() << " CoM velocity^" << get_exponent() << " (ground frame) = " << bodyCoMVelGroundRaisedPower << endl;
        cout << "Drag coefficient = " << get_coefficient() << "\tDrag exponent = " << get_exponent() << endl;
        cout << "dragForce (ground) = " << dragForceGround << endl;
        cout << "dragForce (body frame) = " << dragForceBody << endl;
        auto res = system("pause");
    }

    return;
}

/** Potential energy function */
double BodyDragForce::computePotentialEnergy(const SimTK::State& s) const
{
    return 0;
}


//=============================================================================
// REPORTING
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> BodyDragForce::getRecordLabels() const 
{
    OpenSim::Array<std::string> labels("");
    labels.append(getName()+"."+get_body_name()+".force.X");
    labels.append(getName()+"."+get_body_name()+".force.Y");
    labels.append(getName()+"."+get_body_name()+".force.Z");
    return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> BodyDragForce::getRecordValues(const SimTK::State& s) const 
{
    OpenSim::Array<double> values(3);

    SimTK::Vec3 bodyCoMPosBody, bodyCoMPosGround, bodyCoMVelGround, bodyCoMVelGroundRaisedPower, dragForceGround, dragForceBody, oppVelSign;
    BodySet &bs = _model->updBodySet();                                     // get body set
    const Ground &ground = _model->getGround();              // get ground body
    Body &aBody = bs.get(get_body_name());                                      // get the body to apply the force to

    // get CoM position of body in the BODY coordinate system
    bodyCoMPosBody = aBody.getMassCenter();
    // get CoM position of body in the GROUND coordinate system
    bodyCoMPosGround = aBody.getPositionInGround(s);
    // get CoM velocity of body in the GROUND coordinate system
    bodyCoMVelGround = aBody.findStationVelocityInGround(s, bodyCoMPosBody);

    for (int i=0; i<3;i++)
    {
        if (bodyCoMVelGround[i]>0) oppVelSign[i] = -1;                                      // get opposite sign of CoM velocity (GROUND coordinate system)
        if (bodyCoMVelGround[i]<0) oppVelSign[i] = 1;
        if (bodyCoMVelGround[i]==0) oppVelSign[i] = 0;

        dragForceGround[i] = oppVelSign[i] * get_coefficient() * pow(bodyCoMVelGround[i], get_exponent());  // calculate drag force in the GROUND coordinate system
        values.append(dragForceGround[i]);
    }

    return values;
}
