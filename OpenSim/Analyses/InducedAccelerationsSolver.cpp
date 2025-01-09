/* -------------------------------------------------------------------------- *
 *               OpenSim:  InducedAccelerationsSolver.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include "InducedAccelerationsSolver.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define CENTER_OF_MASS_NAME string("center_of_mass")

//=============================================================================
// CONSTRUCTOR
//=============================================================================
InducedAccelerationsSolver::InducedAccelerationsSolver(const Model& model) :
    Solver(model)
{
    setAuthors("Ajay Seth");
    _modelCopy = model;
    _modelCopy.initSystem();
    _forceThreshold = 1.0; // 1N
}

//=============================================================================
// SOLVE
//=============================================================================
/* Solve for induced accelerations (udot_f) for any applied force expressed
   in terms of individual mobility and body forces */
const SimTK::Vector& InducedAccelerationsSolver::solve(const SimTK::State& s,
        const SimTK::Vector& appliedMobilityForces, 
        const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces,
        SimTK::Vector_<SimTK::SpatialVec>* constraintReactions)
{
    SimTK::State& s_solver = _modelCopy.updWorkingState();
    return s_solver.getUDot();
}

/* Solve for the induced accelerations (udot_f) for a Force in the model 
   identified by its name. */
const SimTK::Vector& InducedAccelerationsSolver::solve(const SimTK::State& s,
                const string& forceName,
                bool computeActuatorPotentialOnly,
                SimTK::Vector_<SimTK::SpatialVec>* constraintReactions)
{
    int nu = _modelCopy.getNumSpeeds();
    double aT = s.getTime();

    SimTK::State& s_solver = _modelCopy.updWorkingState();

    //_modelCopy.initStateWithoutRecreatingSystem(s_solver);
    // Just need to set current time and kinematics to determine state of constraints
    s_solver.setTime(aT);
    s_solver.updQ()=s.getQ();
    s_solver.updU()=s.getU();

    // Check the external forces and determine if contact constraints should be applied at this time
    // and turn constraint on if it should be.
    Array<bool> constraintOn = applyContactConstraintAccordingToExternalForces(s_solver);

    // Hang on to a state that has the right flags for contact constraints turned on/off
    _modelCopy.setPropertiesFromState(s_solver);
    // Use this state for the remainder of this step (record)
    s_solver = _modelCopy.getMultibodySystem().realizeTopology();
    // DO NOT recreate the system, will lose location of constraint
    _modelCopy.initStateWithoutRecreatingSystem(s_solver);

    //cout << "Solving for contributor: " << _contributors[c] << endl;
    // Need to be at the dynamics stage to disable a force
    s_solver.setTime(aT);
    _modelCopy.getMultibodySystem().realize(s_solver, SimTK::Stage::Dynamics);
        
    if(forceName == "total"){
        // Set gravity ON
        _modelCopy.getGravityForce().enable(s_solver);

        //Use same conditions on constraints
        s_solver.updU() = s.getU();
        s_solver.updZ() = s.getZ();

        //Make sure all the actuators are on!
        for(int f=0; f<_modelCopy.getActuators().getSize(); f++){
            _modelCopy.updActuators().get(f).setAppliesForce(s_solver, true);
        }

        // Get to  the point where we can evaluate unilateral constraint conditions
        _modelCopy.getMultibodySystem().realize(s_solver, SimTK::Stage::Acceleration);

        /* *********************************** ERROR CHECKING *******************************
        SimTK::Vec3 pcom =_modelCopy.getMultibodySystem().getMatterSubsystem().calcSystemMassCenterLocationInGround(s_solver);
        SimTK::Vec3 vcom =_modelCopy.getMultibodySystem().getMatterSubsystem().calcSystemMassCenterVelocityInGround(s_solver);
        SimTK::Vec3 acom =_modelCopy.getMultibodySystem().getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s_solver);

        SimTK::Matrix M;
        _modelCopy.getMultibodySystem().getMatterSubsystem().calcM(s_solver, M);
        cout << "mass matrix: " << M << endl;

        SimTK::Inertia sysInertia = _modelCopy.getMultibodySystem().getMatterSubsystem().calcSystemCentralInertiaInGround(s_solver);
        cout << "system inertia: " << sysInertia << endl;

        SimTK::SpatialVec sysMomentum =_modelCopy.getMultibodySystem().getMatterSubsystem().calcSystemMomentumAboutGroundOrigin(s_solver);
        cout << "system momentum: " << sysMomentum << endl;

        const SimTK::Vector &appliedMobilityForces = _modelCopy.getMultibodySystem().getMobilityForces(s_solver, SimTK::Stage::Dynamics);
        appliedMobilityForces.dump("All Applied Mobility Forces");
        
        // Get all applied body forces like those from contact
        const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces = _modelCopy.getMultibodySystem().getRigidBodyForces(s_solver, SimTK::Stage::Dynamics);
        appliedBodyForces.dump("All Applied Body Forces");

        SimTK::Vector ucUdot;
        SimTK::Vector_<SimTK::SpatialVec> ucA_GB;
        _modelCopy.getMultibodySystem().getMatterSubsystem().calcAccelerationIgnoringConstraints(s_solver, appliedMobilityForces, appliedBodyForces, ucUdot, ucA_GB) ;
        ucUdot.dump("Udots Ignoring Constraints");
        ucA_GB.dump("Body Accelerations");

        SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces(_constraintSet.getSize(), SimTK::SpatialVec(SimTK::Vec3(0)));
        SimTK::Vector constraintMobilityForces(0);

        int nc = _modelCopy.getMultibodySystem().getMatterSubsystem().getNumConstraints();
        for (SimTK::ConstraintIndex cx(0); cx < nc; ++cx) {
            if (!_modelCopy.getMultibodySystem().getMatterSubsystem().isConstraintDisabled(s_solver, cx)){
                cout << "Constraint " << cx << " enabled!" << endl;
            }
        }
        //int nMults = _modelCopy.getMultibodySystem().getMatterSubsystem().getTotalMultAlloc();

        for(int i=0; i<constraintOn.getSize(); i++) {
            if(constraintOn[i])
                _constraintSet[i].calcConstraintForces(s_solver, constraintBodyForces, constraintMobilityForces);
        }
        constraintBodyForces.dump("Constraint Body Forces");
        constraintMobilityForces.dump("Constraint Mobility Forces");
        // ******************************* end ERROR CHECKING *******************************/
    
        for(int i=0; i<constraintOn.getSize(); i++) {
            _replacementConstraints[i].setIsEnforced(s_solver, constraintOn[i]);
            // Make sure we stay at Dynamics so each constraint can evaluate its conditions
            _modelCopy.getMultibodySystem().realize(s_solver, SimTK::Stage::Acceleration);
        }

        // This should also push changes to defaults for unilateral conditions
        _modelCopy.setPropertiesFromState(s_solver);

    }
    else if(forceName == "gravity"){
        // Set gravity ON
        _modelCopy.updForceSubsystem().setForceIsDisabled(s_solver, _modelCopy.getGravityForce().getForceIndex(), false);

        // zero velocity
        s_solver.setU(SimTK::Vector(nu,0.0));

        // disable other forces
        for(int f=0; f<_modelCopy.getForceSet().getSize(); f++){
            _modelCopy.updForceSet()[f].setAppliesForce(s_solver, false);
        }
    }
    else if(forceName == "velocity"){       
        // Set gravity off
        _modelCopy.updForceSubsystem().setForceIsDisabled(s_solver, _modelCopy.getGravityForce().getForceIndex(), true);

        // non-zero velocity
        s_solver.updU() = s.getU();
            
        // zero actuator forces
        for(int f=0; f<_modelCopy.getActuators().getSize(); f++){
            _modelCopy.updActuators().get(f).setAppliesForce(s_solver, false);
        }
        // Set the configuration (gen. coords and speeds) of the model.
        _modelCopy.getMultibodySystem().realize(s_solver, SimTK::Stage::Velocity);
    }
    else{ //The rest are actuators      
        // Set gravity OFF
        _modelCopy.updForceSubsystem().setForceIsDisabled(s_solver, _modelCopy.getGravityForce().getForceIndex(), true);

        // zero actuator forces
        for(int f=0; f<_modelCopy.getActuators().getSize(); f++){
            _modelCopy.updActuators().get(f).setAppliesForce(s_solver, false);
        }

        // zero velocity
        SimTK::Vector U(nu,0.0);
        s_solver.setU(U);
        s_solver.updZ() = s.getZ();
        // light up the one Force who's contribution we are looking for
        int ai = _modelCopy.getForceSet().getIndex(forceName);
        if(ai<0){
            log_warn("Force '{}' not found in model '{}'.", forceName,
                    _modelCopy.getName());
        }
        Force &force = _modelCopy.getForceSet().get(ai);
        force.setAppliesForce(s_solver, true);

        ScalarActuator *actuator = dynamic_cast<ScalarActuator*>(&force);
        if(actuator){
            if(computeActuatorPotentialOnly){
                actuator->overrideActuation(s_solver, true);
                actuator->setOverrideActuation(s_solver, 1.0);
            }
        }

        // Set the configuration (gen. coords and speeds) of the model.
        _modelCopy.getMultibodySystem().realize(s_solver, SimTK::Stage::Model);
        _modelCopy.getMultibodySystem().realize(s_solver, SimTK::Stage::Velocity);

    }// End of if to select contributor 

    // cout << "Constraint 0 is of "<< _constraintSet[0].getConcreteClassName() << " and should be " << constraintOn[0] << " and is actually " <<  (_constraintSet[0].isDisabled(s_solver) ? "off" : "on") << endl;
    // cout << "Constraint 1 is of "<< _constraintSet[1].getConcreteClassName() << " and should be " << constraintOn[1] << " and is actually " <<  (_constraintSet[1].isDisabled(s_solver) ? "off" : "on") << endl;

    // After setting the state of the model and applying forces
    // Compute the derivative of the multibody system (speeds and accelerations)
    _modelCopy.getMultibodySystem().realize(s_solver, SimTK::Stage::Acceleration);

    // Sanity check that constraints hasn't totally changed the configuration of the model
    // double error = (s.getQ()-s_solver.getQ()).norm();

    // Report reaction forces for debugging
    /*
    SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces(_constraintSet.getSize());
    SimTK::Vector mobilityForces(0);

    for(int i=0; i<constraintOn.getSize(); i++) {
        if(constraintOn[i])
            _constraintSet.get(i).calcConstraintForces(s_solver, constraintBodyForces, mobilityForces);
    }*/

    return s_solver.getUDot();
}

const SimTK::State& InducedAccelerationsSolver::
    getSolvedState(const SimTK::State& s) const
{
    const SimTK::State& s_solver = _modelCopy.getWorkingState();

    // check that state of the model hasn't changed since the solve
    if((s.getTime() == s_solver.getTime()) &&
        (s.getNY() == s_solver.getNY())) {
        // ensure the solver stage is, in fact, acceleration; if not,
        // no solver was performed or it was unsuccessful
        if(s_solver.getSystemStage() >= SimTK::Stage::Acceleration){
            return s_solver;
        } else {
            throw Exception("InducedAccelerationsSolver::"
                "Cannot access solver state without executing 'solve' first.");
        }
    }
    else{
        throw Exception("InducedAccelerationsSolver::"
                "Cannot access solver state when model state has changed.");
    }
}



double InducedAccelerationsSolver::
    getInducedCoordinateAcceleration(const SimTK::State& s,
        const string& coordName)
{
    const SimTK::State& s_solver = getSolvedState(s);

    const Coordinate* coord = NULL; 
    int ind = _modelCopy.getCoordinateSet().getIndex(coordName);
    if(ind < 0){
        std::string msg = "InducedAccelerationsSolver::";
        msg = msg + "cannot find coordinate '" + coordName + "'.";
        throw Exception(msg);
    }

    coord = &_modelCopy.getCoordinateSet()[ind];
    return coord->getAccelerationValue(s_solver);
}

const SimTK::SpatialVec& InducedAccelerationsSolver::
    getInducedBodyAcceleration(const SimTK::State& s,
        const string& bodyName)
{
    const SimTK::State& s_solver = getSolvedState(s);

    const Body* body = NULL; 
    int ind = _modelCopy.getBodySet().getIndex(bodyName);
    if(ind < 0){
        std::string msg = "InducedAccelerationsSolver::";
        msg = msg + "cannot find body '" + bodyName + "'.";
        throw Exception(msg);
    }

    body = &_modelCopy.getBodySet()[ind];

    return body->getMobilizedBody().getBodyAcceleration(s_solver);
}


SimTK::Vec3 InducedAccelerationsSolver::
    getInducedMassCenterAcceleration(const SimTK::State& s)
{
    const SimTK::State& s_solver = getSolvedState(s);
    return _modelCopy.getMatterSubsystem()
        .calcSystemMassCenterAccelerationInGround(s_solver);
}



Array<bool> InducedAccelerationsSolver::
    applyContactConstraintAccordingToExternalForces(SimTK::State &s)
{
    Array<bool> constraintOn(false, _replacementConstraints.getSize());
    double t = s.getTime();

    for(int i=0; i<_forcesToReplace.getSize(); i++){
        ExternalForce* exf = dynamic_cast<ExternalForce*>(&_forcesToReplace[i]);
        SimTK::Vec3 point, force, gpoint;

        force = exf->getForceAtTime(t);
        
        // If the applied force is "significant" replace it with a constraint
        if (force.norm() > _forceThreshold){
            // get the point of contact from applied external force
            point = exf->getPointAtTime(t);
            // point should be expressed in the "applied to" body for consistency across all constraints
            if(exf->getPointExpressedInBodyName() != exf->getAppliedToBodyName()){
                const PhysicalFrame* appliedToBody =
                        getModel().findComponent<PhysicalFrame>(
                                exf->getAppliedToBodyName());
                const PhysicalFrame* expressedInBody =
                        getModel().findComponent<PhysicalFrame>(
                                exf->getPointExpressedInBodyName());

                OPENSIM_THROW_IF_FRMOBJ(appliedToBody == nullptr, Exception,
                        "ExternalForce's appliedToBody " +
                                exf->getAppliedToBodyName() + " not found.");
                OPENSIM_THROW_IF_FRMOBJ(expressedInBody == nullptr, Exception,
                        "ExternalForce's pointExpressedInBodyName " +
                                exf->getPointExpressedInBodyName() +
                                " not found.");

                getModel().getMultibodySystem().realize(s, SimTK::Stage::Velocity);
                point = expressedInBody->findStationLocationInAnotherFrame(
                        s, point, *appliedToBody);
            }

            _replacementConstraints[i].setContactPointForInducedAccelerations(s, point);

            // turn on the constraint
            _replacementConstraints[i].setIsEnforced(s, true);
            // return the state of the constraint
            constraintOn[i] = true;

        }
        else{
            // turn off the constraint
            _replacementConstraints[i].setIsEnforced(s, false);
            // return the state of the constraint
            constraintOn[i] = false;
        }
    }

    return constraintOn;
}
