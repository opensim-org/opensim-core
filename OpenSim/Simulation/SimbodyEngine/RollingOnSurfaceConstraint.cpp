/* -------------------------------------------------------------------------- *
 *                  OpenSim:  RollingOnSurfaceConstraint.cpp                  *
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
#include "RollingOnSurfaceConstraint.h"
#include "simbody/internal/SimbodyMatterSubsystem.h"
#include "simbody/internal/Constraint_PointInPlane.h"
#include "simbody/internal/MobilizedBody_Weld.h"

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
    _rollingFrame.reset(nullptr);
    _surfaceFrame.reset(nullptr);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void RollingOnSurfaceConstraint::constructProperties()
{
    // Surface parameters
    constructProperty_surface_normal(Vec3(0, 1.0, 0));

    constructProperty_surface_height(0.0);
    
    constructProperty_friction_coefficient(0.5);

    constructProperty_contact_radius(0.01);
}

void RollingOnSurfaceConstraint::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
    _rollingFrame = &getSocket<PhysicalFrame>("rolling_body").getConnectee();
    _surfaceFrame = &getSocket<PhysicalFrame>("surface_body").getConnectee();
}

void RollingOnSurfaceConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    // Get underlying mobilized bodies
    SimTK::MobilizedBody roller = _rollingFrame->getMobilizedBody();
    SimTK::MobilizedBody surface = _surfaceFrame->getMobilizedBody();
    
    // Add a fictitious massless body to be the "Case" reference body coincident with surface for the no-slip constraint
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
    assignConstraintIndex(_indices[0]);
}

void RollingOnSurfaceConstraint::extendInitStateFromProperties(SimTK::State& state) const
{
    Super::extendInitStateFromProperties(state);

    // All constraints treated the same as default behavior at initialization
    for(int i=0; i < _numConstraintEquations; i++){
        SimTK::Constraint& simConstraint = 
            updSystem().updMatterSubsystem().updConstraint(_indices[i]);
        // initialize the status of the constraint
        if(_defaultUnilateralConditions[i]){
            simConstraint.enable(state);
        }
        else{
            simConstraint.disable(state);
        }
    }
}

void
RollingOnSurfaceConstraint::
extendSetPropertiesFromState(const SimTK::State& state) {
    Super::extendSetPropertiesFromState(state);

    set_isEnforced(isEnforced(state));
    for(int i=0; i < _numConstraintEquations; i++){
        SimTK::Constraint& simConstraint = 
            updSystem().updMatterSubsystem().updConstraint(_indices[i]);
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
void RollingOnSurfaceConstraint::setRollingBodyByName(const std::string& aBodyName)
{
    updSocket<PhysicalFrame>("rolling_body").setConnecteePath(aBodyName);
}

void RollingOnSurfaceConstraint::setSurfaceBodyByName(const std::string& aBodyName)
{
    updSocket<PhysicalFrame>("surface_body").setConnecteePath(aBodyName);
}

/** Set the point of contact on the rolling body that will be in contact with the surface */
void RollingOnSurfaceConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
    // Get the individual underlying constraints
    SimTK::Constraint::PointInPlane &contactY = (SimTK::Constraint::PointInPlane &)
        updSystem().updMatterSubsystem().updConstraint(_indices[0]);
    SimTK::Constraint::ConstantAngle &contactTorqueAboutY = (SimTK::Constraint::ConstantAngle &)
        updSystem().updMatterSubsystem().updConstraint(_indices[1]);
    SimTK::Constraint::NoSlip1D &contactPointXdir = (SimTK::Constraint::NoSlip1D &)
        updSystem().updMatterSubsystem().updConstraint(_indices[2]);
    SimTK::Constraint::NoSlip1D &contactPointZdir = (SimTK::Constraint::NoSlip1D &)
        updSystem().updMatterSubsystem().updConstraint(_indices[3]);

    // The contact point coordinates in the surface body frame 
    Vec3 surfaceNormal = get_surface_normal();

    // make sure we are at the position stage
    getSystem().realize(s, SimTK::Stage::Position);

    // For external forces we assume w.r.t. ground
    Vec3 spoint = _rollingFrame->findStationLocationInAnotherFrame(s, point, *_surfaceFrame);

    // The contact point coordinates in the surface body frame 
    contactY.setDefaultPlaneNormal(UnitVec3(surfaceNormal));
    contactY.setDefaultPlaneHeight(~spoint*surfaceNormal);
    // And the point in the follower (roller) frame
    contactY.setDefaultFollowerPoint(point);

    // Assume that torsion constraint is always normal to surface.
    // Find corresponding vector in the rolling body
    Vec3 normalInRoller = 
        _surfaceFrame->expressVectorInAnotherFrame(s, surfaceNormal, *_rollingFrame);

    const UnitVec3& surfaceBase = contactTorqueAboutY.getDefaultBaseAxis();
    Vec3 baseAxisInRoller =
        _surfaceFrame->expressVectorInAnotherFrame(s, surfaceBase, *_rollingFrame);

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
    SimTK::Constraint&contactY = updSystem().updMatterSubsystem().updConstraint(_indices[0]);
    SimTK::Constraint&contactTorqueAboutY = updSystem().updMatterSubsystem().updConstraint(_indices[1]);
    SimTK::Constraint&contactPointXdir = updSystem().updMatterSubsystem().updConstraint(_indices[2]);
    SimTK::Constraint&contactPointZdir = updSystem().updMatterSubsystem().updConstraint(_indices[3]);

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

bool RollingOnSurfaceConstraint::isEnforced(const SimTK::State &state) const {
    // The parent constraint in is the plane constraint, so check its value
    return !updSystem().
           updMatterSubsystem().
           updConstraint(_indices[0]).
           isDisabled(state);
}

bool RollingOnSurfaceConstraint::setIsEnforced(SimTK::State& state,
                                               bool isEnforced) {
    // All constraints treated the same as default behavior i.e. at
    // initialization
    std::vector<bool> shouldBeOn(_numConstraintEquations, isEnforced);

    // If dynamics has been realized, then this is an attempt to enforce/disable
    //  the constraint during a computation and not an initialization, in which
    // case we must check the unilateral conditions for each constraint
    if(state.getSystemStage() > Stage::Dynamics)
        shouldBeOn = unilateralConditionsSatisfied(state);

    return setIsEnforced(state, isEnforced, shouldBeOn);
}

bool RollingOnSurfaceConstraint::setIsEnforced(SimTK::State& state,
                                               bool isEnforced,
                                               std::vector<bool> shouldBeOn) {
    for(int i=0; i < _numConstraintEquations; i++){
        SimTK::Constraint& simConstraint =
            updSystem().updMatterSubsystem().updConstraint(_indices[i]);
        bool isConstraintOn = !simConstraint.isDisabled(state);

        // Check if we already have the correct enabling of the constraint then
        // do nothing 
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
    set_isEnforced(isEnforced);

    // Return whether or not constraint is in the state the caller wanted
    // The first constraint is the "master" so its state is what we care about
    return isEnforced != updSystem().
                         updMatterSubsystem().
                         updConstraint(_indices[0]).
                         isDisabled(state);
}

//-----------------------------------------------------------------------------
// FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void RollingOnSurfaceConstraint::calcConstraintForces(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
                                      SimTK::Vector& mobilityForces) const
{
    SimTK::Vector_<SimTK::SpatialVec> bfs;
    SimTK::Vector mfs;

    for(int i=0; i < _numConstraintEquations; i++){
        
        SimTK::Constraint& simConstraint = updSystem().updMatterSubsystem().updConstraint(_indices[i]);
        int ncb = simConstraint.getNumConstrainedBodies();
        int mp, mv, ma;

        if(!simConstraint.isDisabled(state)){
            simConstraint.getNumConstraintEquationsInUse(state, mp, mv, ma);
            SimTK::Vector multipliers = simConstraint.getMultipliersAsVector(state);
            simConstraint.calcConstraintForcesFromMultipliers(state, multipliers, bfs, mfs);
            
            int sbi = -1;
            int rbi = -1;
            //int anc = simConstraint.getAncestorMobilizedBody().getMobilizedBodyIndex();
            
            for(int j=0; j< ncb; j++){
                if (_surfaceFrame->getMobilizedBodyIndex() == simConstraint.getMobilizedBodyFromConstrainedBody(ConstrainedBodyIndex(j)).getMobilizedBodyIndex())
                    sbi = j;
                if (_rollingFrame->getMobilizedBodyIndex() == simConstraint.getMobilizedBodyFromConstrainedBody(ConstrainedBodyIndex(j)).getMobilizedBodyIndex())
                    rbi = j;
            }

            /*
            cout << "Constraint " << i << "  forces:" << endl;
            cout << " Surf body index: " << sbi << " Expressed in: " << anc << endl;
            cout << " Roll body index: " << rbi << " Expressed in: " << anc << endl;
            bfs.dump(" Constraint body forces:");
            */
            bodyForcesInAncestor[0] += bfs[sbi];
            bodyForcesInAncestor[1] += bfs[rbi];
            mobilityForces += mfs;
        }
    }
}

void RollingOnSurfaceConstraint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if (documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion<30500){
            // replace old properties with latest use of Sockets
            SimTK::Xml::element_iterator body1Element = aNode.element_begin("rolling_body");
            SimTK::Xml::element_iterator body2Element = aNode.element_begin("surface_body");
            std::string body1_name(""), body2_name("");
            // If default constructed then elements not serialized since they
            // are default values. Check that we have associated elements, then
            // extract their values.
            // Constraints in pre-4.0 models are necessarily 1 level deep
            // (model, constraints), and Bodies are necessarily 1 level deep.
            // Here we create the correct relative path (accounting for sets
            // being components).
            if (body1Element != aNode.element_end()) {
                body1Element->getValueAs<std::string>(body1_name);
                body1_name = XMLDocument::updateConnecteePath30517("bodyset",
                                                                   body1_name);
            }
            if (body2Element != aNode.element_end()) {
                body2Element->getValueAs<std::string>(body2_name);
                body2_name = XMLDocument::updateConnecteePath30517("bodyset",
                                                                   body2_name);
            }
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_",
                    "rolling_body", body1_name);
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_",
                    "surface_body", body2_name);
        }
    }

    Super::updateFromXMLNode(aNode, versionNumber);
}

bool
RollingOnSurfaceConstraint::
setIsEnforcedWithCachedUnilateralConditions(bool isEnforced,
                                            SimTK::State& state) {
    return setIsEnforced(state, isEnforced, _defaultUnilateralConditions);
}
