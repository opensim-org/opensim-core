/* -------------------------------------------------------------------------- *
 *                  OpenSim:  RollingOnSurfaceConstraint.cpp                  *
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
#include <iostream>
#include <math.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "RollingOnSurfaceConstraint.h"

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
RollingOnSurfaceConstraint::~RollingOnSurfaceConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
RollingOnSurfaceConstraint::RollingOnSurfaceConstraint() :
    UnilateralConstraint()
{
    setNull();
    constructProperties();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this RollingOnSurfaceConstraint to their null values.
 */
void RollingOnSurfaceConstraint::setNull()
{
    setAuthors("Ajay Seth");
    _defaultUnilateralConditions = std::vector<bool>(4, false);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void RollingOnSurfaceConstraint::constructProperties()
{
    // Body 1 name
    constructProperty_rolling_body("");

    // Body 2 name
    constructProperty_surface_body("ground");

    // Surface parameters
    constructProperty_surface_normal(Vec3(0, 1.0, 0));

    constructProperty_surface_height(0.0);
    
    constructProperty_friction_coefficient(0.5);

    constructProperty_contact_radius(0.01);
}

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this RollingOnSurfaceConstraint.
 */
void RollingOnSurfaceConstraint::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    string errorMessage;

    std::string rollingBodyName = get_rolling_body();
    std::string surfaceBodyName = get_surface_body();
    
        // Look up the two bodies being constrained together by name in the
    // dynamics engine and might as well keep a pointer to them
    _rollingBody = &_model->updBodySet().get(rollingBodyName);
    _surfaceBody = &_model->updBodySet().get(surfaceBodyName);

    if (!_rollingBody) {
        errorMessage = "Invalid RollingBody (" + rollingBodyName + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    if (!_surfaceBody) {
        errorMessage = "Invalid SurfaceBody (" + surfaceBodyName + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
}

void RollingOnSurfaceConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    // Get underlying mobilized bodies
    SimTK::MobilizedBody roller = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_rollingBody->getMobilizedBodyIndex());
    SimTK::MobilizedBody surface = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_surfaceBody->getMobilizedBodyIndex());
    
    // Add a ficticious massless body to be the "Case" reference body coincident with surface for the no-slip constraint
    SimTK::MobilizedBody::Weld  cb(surface, SimTK::Body::Massless());

    // Constrain the roller to the surface
    SimTK::Constraint::PointInPlane contactY(surface, SimTK::UnitVec3(get_surface_normal()), get_surface_height(), roller,Vec3(0));
    SimTK::Constraint::ConstantAngle contactTorqueAboutY(surface, SimTK::UnitVec3(1,0,0), roller, SimTK::UnitVec3(0,0,1));
    // Constrain the roller to roll on surface and not slide 
    SimTK::Constraint::NoSlip1D contactPointXdir(cb, SimTK::Vec3(0), SimTK::UnitVec3(1,0,0), surface, roller);
    SimTK::Constraint::NoSlip1D contactPointZdir(cb, SimTK::Vec3(0), SimTK::UnitVec3(0,0,1), surface, roller);

     // Beyond the const Component get the index so we can access the SimTK::Constraint later
    RollingOnSurfaceConstraint* mutableThis = const_cast<RollingOnSurfaceConstraint *>(this);
    // Make sure that there is nothing in the list of constraint indices
    mutableThis->_indices.clear();
    // Get the index so we can access the SimTK::Constraint later
    mutableThis->_indices.push_back(contactY.getConstraintIndex());
    mutableThis->_indices.push_back(contactTorqueAboutY.getConstraintIndex());
    mutableThis->_indices.push_back(contactPointXdir.getConstraintIndex());
    mutableThis->_indices.push_back(contactPointZdir.getConstraintIndex());

    mutableThis->_numConstraintEquations = (int)_indices.size();

    // For compound constraints, the bodies and/or mobilities involved must be characterized by
    // the first "master" constraint, which dictates the behavior of the other constraints
    // For example, enabling and disabling. This enables a compound constraint to be treated
    // like a single constraint for the purpose of enabling/disabling and getting output
    mutableThis->_index = _indices[0];
}

void RollingOnSurfaceConstraint::extendInitStateFromProperties(SimTK::State& state) const
{
    Super::extendInitStateFromProperties(state);

    // All constraints treated the same as default behavior at initilization
    for(int i=0; i < _numConstraintEquations; i++){
        SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
        // initialize the status of the constraint
        if(_defaultUnilateralConditions[i]){
            simConstraint.enable(state);
        }
        else{
            simConstraint.disable(state);
        }
    }
}

void RollingOnSurfaceConstraint::setPropertiesFromState(const SimTK::State& state)
{
    Super::setPropertiesFromState(state);

    set_isDisabled(isDisabled(state));
    for(int i=0; i < _numConstraintEquations; i++){
        SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
        // initialize the status of the constraint
        _defaultUnilateralConditions[i] = !simConstraint.isDisabled(state); 
    }
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the weld constraint */
void RollingOnSurfaceConstraint::setRollingBodyByName(std::string aBodyName)
{
    set_rolling_body(aBodyName);
}

void RollingOnSurfaceConstraint::setSurfaceBodyByName(std::string aBodyName)
{
    set_surface_body(aBodyName);
}

/** Set the point of contact on the rolling body that will be in contact with the surface */
void RollingOnSurfaceConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
    // Get the individual underlying constraints
    SimTK::Constraint::PointInPlane &contactY = (SimTK::Constraint::PointInPlane &)_model->updMatterSubsystem().updConstraint(_indices[0]);
    SimTK::Constraint::ConstantAngle &contactTorqueAboutY = (SimTK::Constraint::ConstantAngle &)_model->updMatterSubsystem().updConstraint(_indices[1]);
    SimTK::Constraint::NoSlip1D &contactPointXdir = (SimTK::Constraint::NoSlip1D &)_model->updMatterSubsystem().updConstraint(_indices[2]);
    SimTK::Constraint::NoSlip1D &contactPointZdir = (SimTK::Constraint::NoSlip1D &)_model->updMatterSubsystem().updConstraint(_indices[3]);

    // The contact point coordinates in the surface body frame 
    Vec3 spoint;
    Vec3 surfaceNormal = get_surface_normal();

    // make sure we are at the position stage
    _model->getMultibodySystem().realize(s, SimTK::Stage::Position);

    // For external forces we assume w.r.t. ground
    _model->getSimbodyEngine().transformPosition(s, *_rollingBody, point, *_surfaceBody, spoint);

    // The contact point coordinates in the surface body frame 
    contactY.setDefaultPlaneNormal(UnitVec3(surfaceNormal));
    contactY.setDefaultPlaneHeight(~spoint*surfaceNormal);
    // And the point in the follower (roller) frame
    contactY.setDefaultFollowerPoint(point);

    // Assume that torsion constraint is always normal to surface so find corresponding vector
    // in the rolling body
    Vec3 normalInRoller(0);
    Vec3 baseAxisInRoller(0);
    UnitVec3 surfaceBase = contactTorqueAboutY.getDefaultBaseAxis();
    _model->getSimbodyEngine().transform(s, *_surfaceBody, surfaceNormal, *_rollingBody, normalInRoller);
    _model->getSimbodyEngine().transform(s, *_surfaceBody, surfaceBase, *_rollingBody, baseAxisInRoller);

    contactTorqueAboutY.setDefaultFollowerAxis(UnitVec3(baseAxisInRoller%normalInRoller));

    // Set the point of no slip for this instant
    contactPointXdir.setDefaultContactPoint(spoint);
    contactPointZdir.setDefaultContactPoint(spoint);
}

std::vector<bool> RollingOnSurfaceConstraint::unilateralConditionsSatisfied(const SimTK::State &state)
{
    std::vector<bool> conditionsSatisfied(4,false);
    int mp, mv, ma;
    SimTK::Vector lambda;

    // The reaction forces necessary for resolving the unilateral conditions
    double normalForce = 0;
    double normalTorque = 0;
    SimTK::Vec3 rollForce(0);
    double tangentialForce = 0;

    // Get the individual underlying constraints
    SimTK::Constraint&contactY = _model->updMatterSubsystem().updConstraint(_indices[0]);
    SimTK::Constraint&contactTorqueAboutY = _model->updMatterSubsystem().updConstraint( _indices[1]);
    SimTK::Constraint&contactPointXdir = _model->updMatterSubsystem().updConstraint(_indices[2]);
    SimTK::Constraint&contactPointZdir = _model->updMatterSubsystem().updConstraint(_indices[3]);

    // Constraint conditions only matter if the constraint is enabled.
    if(!contactY.isDisabled(state)){
        // Obtain the surface constraint force in the normal direction.
        contactY.getNumConstraintEquationsInUse(state, mp, mv, ma);
        lambda = contactY.getMultipliersAsVector(state);
        normalForce = -lambda[0];

        // Obtain the surface constraint torque about the surface normal.
        if(!contactTorqueAboutY.isDisabled(state)){
            contactTorqueAboutY.getNumConstraintEquationsInUse(state, mp, mv, ma);
            lambda = contactTorqueAboutY.getMultipliersAsVector(state);
            normalTorque = lambda[0];
        }

        // Obtain the rolling force components.
        if(!contactPointXdir.isDisabled(state)){
            contactPointXdir.getNumConstraintEquationsInUse(state, mp, mv, ma);
            lambda = contactPointXdir.getMultipliersAsVector(state);
            rollForce[0] = lambda[0];
        }
        if(!contactPointZdir.isDisabled(state)){
            contactPointZdir.getNumConstraintEquationsInUse(state, mp, mv, ma);
            lambda = contactPointZdir.getMultipliersAsVector(state);
            rollForce[2] = lambda[0];
        }
    }

    // get the magnitude of the rolling force 
    tangentialForce = rollForce.norm();
    
    // All internal constraints depend on the normal force: no force no constraint
    // That is what makes this constraint unilateral
    if(normalForce > 0.0){
        conditionsSatisfied[0] = true;

        // Now check that rolling forces are not exceeding normal force according to Coulomb friction
        if(tangentialForce <= get_friction_coefficient()*normalForce){
            conditionsSatisfied[2] = true;
            conditionsSatisfied[3] = true;

            // If we are not slipping then constraint will not allow rotation of roller about normal
            // as long the required torque does not exceed the torque capacity of the tangential force.
            if( normalTorque <= get_contact_radius()*get_friction_coefficient()*normalForce )
                conditionsSatisfied[1] = true;
        }
    }
    
    // Cache the conditions until the next reevaluation
    _defaultUnilateralConditions = conditionsSatisfied;

    return conditionsSatisfied;
}

//-----------------------------------------------------------------------------
// DISABLE
//-----------------------------------------------------------------------------

//_____________________________________________________________________________
/**
 * Get whether or not the RollingOnSurfaceConstraint is disabled.
 * Simbody multibody system instance is realized every time the isDisabled
 * changes, BUT multiple sets to the same value have no cost.
 *
 * @param isDisabled If true the constraint is disabled; if false the constraint is enabled.
 */
bool RollingOnSurfaceConstraint::isDisabled(const SimTK::State &state) const
{
    // The parent constraint in is the plane constraint, so check its value
    return _model->updMatterSubsystem().updConstraint(_indices[0]).isDisabled(state);
}

//_____________________________________________________________________________
/**
 * Set whether or not the RollingOnSurfaceConstraint is disabled.
 * Since the constraint is composed of multiple constraints, this method can  
 * disable the constraints, but enabling is not guaranteed. For example, if
 * the unilateral conditions are violated the constraint will be disabled.
 *
 * @param isDisabled If true the constraint is disabled; if false the constraint is enabled.
 */
bool RollingOnSurfaceConstraint::setDisabled(SimTK::State& state, bool isDisabled)
{
    // All constraints treated the same as default behavior i.e. at initilization
    std::vector<bool> shouldBeOn(_numConstraintEquations, !isDisabled);

    // If dynamics has been realized, then this is an attempt to enable/disable the constraint
    // during a computation and not an initialization, in which case we must check the 
    // unilateral conditions for each constraint
    if(state.getSystemStage() > Stage::Dynamics)
        shouldBeOn = unilateralConditionsSatisfied(state);

    return setDisabled(state, isDisabled, shouldBeOn);
}


bool RollingOnSurfaceConstraint::setDisabled(SimTK::State& state, bool isDisabled, std::vector<bool> shouldBeOn)
{

    for(int i=0; i < _numConstraintEquations; i++){
        SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
        bool isConstraintOn = !simConstraint.isDisabled(state);

        // Check if we already have the correct enabling of the constraint then do nothing 
        if(shouldBeOn[i] == isConstraintOn)
            continue;

        // Otherwise we have to change the status of the constraint
        if(shouldBeOn[i]){
            simConstraint.enable(state);
        }
        else{
            simConstraint.disable(state);
        }
    }

    //Update the property accordingly
    set_isDisabled(isDisabled);

    // Return whether or not constraint is in the state the caller wanted
    // The first constraint is the "master" so its state is what we care about
    return isDisabled == _model->updMatterSubsystem().updConstraint(_indices[0]).isDisabled(state);
}


//-----------------------------------------------------------------------------
// FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Ask the RollingOnSurfaceConstraint for the forces it is imposing on the system
 * Simbody multibody system must be realized to at least position
 * Returns: the bodyForces on those bodies being constrained (constrainedBodies)
 *              a SpatialVec (6 components) describing resulting torque and force
 *          mobilityForces acting along constrained mobilities  
 *
 * @param state State of model
 * @param bodyForcesInAncestor is a Vector of SpatialVecs contain constraint forces
 * @param mobilityForces is a Vector of forces that act along the constrained
 *         mobilitities associated with this constraint
 */
void RollingOnSurfaceConstraint::calcConstraintForces(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
                                      SimTK::Vector& mobilityForces) const
{
    SimTK::Vector_<SimTK::SpatialVec> bfs;
    SimTK::Vector mfs;

    for(int i=0; i < _numConstraintEquations; i++){
        
        SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
        int ncb = simConstraint.getNumConstrainedBodies();
        int mp, mv, ma;

        if(!simConstraint.isDisabled(state)){
            simConstraint.getNumConstraintEquationsInUse(state, mp, mv, ma);
            SimTK::Vector multipliers = simConstraint.getMultipliersAsVector(state);
            simConstraint.calcConstraintForcesFromMultipliers(state, multipliers, bfs, mfs);
            
            int sbi = -1;
            int rbi = -1;
            int anc = simConstraint.getAncestorMobilizedBody().getMobilizedBodyIndex();
            
            for(int j=0; j< ncb; j++){
                if (_surfaceBody->getMobilizedBodyIndex() == simConstraint.getMobilizedBodyFromConstrainedBody(ConstrainedBodyIndex(j)).getMobilizedBodyIndex())
                    sbi = j;
                if (_rollingBody->getMobilizedBodyIndex() == simConstraint.getMobilizedBodyFromConstrainedBody(ConstrainedBodyIndex(j)).getMobilizedBodyIndex())
                    rbi = j;
            }

            /*
            cout << "Constraint " << i << "  forces:" << endl;
            cout << " Surf body index: " << sbi << " Expressed in: " << anc << endl;
            cout << " Roll body index: " << rbi << " Expressed in: " << anc << endl;
            bfs.dump(" Constrint body forces:");
            */
            bodyForcesInAncestor[0] += bfs[sbi];
            bodyForcesInAncestor[1] += bfs[rbi];
            mobilityForces += mfs;
        }
    }
}
