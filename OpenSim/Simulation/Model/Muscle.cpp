/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Muscle.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
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
#include "Muscle.h"


#include "Model.h"
#include <OpenSim/Common/XMLDocument.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const Vec3 DefaultMuscleColor(.8, .1, .1); // Red for backward compatibility

//=============================================================================
// CONSTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
Muscle::Muscle()
{
    constructProperties();
}

//_____________________________________________________________________________
// Override default implementation by object to intercept and fix the XML node
// underneath the model to match current version.
void Muscle::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if ( versionNumber < XMLDocument::getLatestVersion()) {
        log_debug("Updating Muscle object to latest format...");

        if (versionNumber <= 20301){
            SimTK::Xml::element_iterator pathIter =
                                            aNode.element_begin("GeometryPath");
            if (pathIter != aNode.element_end()) {
                XMLDocument::renameChildNode(*pathIter, "MusclePointSet", "PathPointSet");
                XMLDocument::renameChildNode(*pathIter, "MuscleWrapSet", "PathWrapSet");
            } else { // There was no GeometryPath, just MusclePointSet
                SimTK::Xml::element_iterator musclePointSetIter = aNode.element_begin("MusclePointSet");
                bool pathPointSetFound=false;
                if (musclePointSetIter != aNode.element_end()){
                    XMLDocument::renameChildNode(aNode, "MusclePointSet", "PathPointSet");
                    pathPointSetFound=true;
                }
                bool pathWrapSetFound=false;
                SimTK::Xml::element_iterator muscleWrapSetIter = aNode.element_begin("MuscleWrapSet");
                if (muscleWrapSetIter != aNode.element_end()){
                    XMLDocument::renameChildNode(aNode, "MuscleWrapSet", "PathWrapSet");
                    pathWrapSetFound=true;
                }
                // Now create a "GeometryPath" node and move MusclePointSet & MuscleWrapSet under it
                SimTK::Xml::Element myPathElement("GeometryPath");
                SimTK::Xml::Node moveNode;
                if (pathPointSetFound) {
                    SimTK::Xml::element_iterator  pathPointSetIter = aNode.element_begin("PathPointSet");
                    moveNode = aNode.removeNode(pathPointSetIter);
                    myPathElement.insertNodeAfter(myPathElement.element_end(),moveNode);
                }
                if (pathWrapSetFound) {
                    SimTK::Xml::element_iterator  pathWrapSetIter = aNode.element_begin("PathWrapSet");
                    moveNode = aNode.removeNode(pathWrapSetIter);
                    myPathElement.insertNodeAfter(myPathElement.element_end(),moveNode);
                }
                aNode.insertNodeAfter(aNode.element_end(), myPathElement);
            }
            XMLDocument::renameChildNode(aNode, "pennation_angle", "pennation_angle_at_optimal");
        }
        if (versionNumber < 30513) {
            SimTK::Xml::element_iterator minControlElt =
                aNode.element_begin("min_control");
            double minControl = 0;
            if (minControlElt != aNode.element_end()) {
                minControlElt->getValueAs<double>(minControl);
                // If the min_control value is 0, then remove the min_control
                // property in XML since it is likely a result of a mistake. In
                // previous versions, the min_control property was no updated
                // to reflect the Muscle's min_activation. Removing it allows
                // the Muscle to use the appropriate default specified by the
                // derived concrete Muscle.
                if (SimTK::isNumericallyEqual(minControl, 0.0)) {
                    aNode.removeNode(minControlElt);
                    minControlElt->clearOrphan();
                }
            }
            SimTK::Xml::element_iterator maxControlElt =
                aNode.element_begin("max_control");
            double maxControl = 0;
            if (maxControlElt != aNode.element_end()) {
                maxControlElt->getValueAs<double>(maxControl);
                // allow Muscle to use its default
                if (SimTK::isNumericallyEqual(maxControl, 1.0)) {
                    aNode.removeNode(maxControlElt);
                    maxControlElt->clearOrphan();
                }
            }
        }
        if (versionNumber < 30516) {
            // Find GeometryPath node and insert <default_color>
            SimTK::Xml::element_iterator  geomPathIter = aNode.element_begin("GeometryPath");
            if (geomPathIter != aNode.element_end()) {
                SimTK::Xml::element_iterator  defaultColorIter = geomPathIter->element_begin("default_color");
                if (defaultColorIter == geomPathIter->element_end()) {
                    SimTK::Xml::Element myDefaultColorEement("default_color");
                    myDefaultColorEement.setValue(".8 .1 .1"); // DefaultMuscleColor
                    geomPathIter->appendNode(myDefaultColorEement);
                }
            }
        }

    }
    // Call base class now assuming aNode has been corrected for current version
    Super::updateFromXMLNode(aNode, versionNumber);
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Muscle::constructProperties()
{
    constructProperty_max_isometric_force(1000.0);
    constructProperty_optimal_fiber_length(0.1);
    constructProperty_tendon_slack_length(0.2);
    constructProperty_pennation_angle_at_optimal(0.0);
    constructProperty_max_contraction_velocity(10.0);
    constructProperty_ignore_tendon_compliance(false);
    constructProperty_ignore_activation_dynamics(false);

    // By default the min and max controls on muscle are 0.0 and 1.0
    setMinControl(0.0);
    setMaxControl(1.0);
    updPath().setDefaultColor(DefaultMuscleColor);
}


//--------------------------------------------------------------------------
// MUSCLE PARAMETERS GETTERS AND SETTERS
//--------------------------------------------------------------------------
double Muscle::getMaxIsometricForce() const
{   return get_max_isometric_force(); }

double Muscle::getOptimalFiberLength() const
{   return get_optimal_fiber_length(); }

double Muscle::getTendonSlackLength() const
{   return get_tendon_slack_length(); }

double Muscle::getPennationAngleAtOptimalFiberLength() const
{   return get_pennation_angle_at_optimal(); }

double Muscle::getMaxContractionVelocity() const
{   return get_max_contraction_velocity(); }

void Muscle::setMaxIsometricForce(double aMaxIsometricForce)
{   set_max_isometric_force(aMaxIsometricForce); }

void Muscle::setOptimalFiberLength(double aOptimalFiberLength)
{   set_optimal_fiber_length(aOptimalFiberLength); }

void Muscle::setTendonSlackLength(double aTendonSlackLength)
{   set_tendon_slack_length(aTendonSlackLength); }

void Muscle::setPennationAngleAtOptimalFiberLength(double aPennationAngle)
{   set_pennation_angle_at_optimal(aPennationAngle); }

void Muscle::setMaxContractionVelocity(double aMaxContractionVelocity)
{   set_max_contraction_velocity(aMaxContractionVelocity); }


//=============================================================================
// ModelComponent Interface Implementation
//=============================================================================
void Muscle::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    _muscleWidth = getOptimalFiberLength()
                    * sin(getPennationAngleAtOptimalFiberLength());

    _maxIsometricForce = getMaxIsometricForce();
    _optimalFiberLength = getOptimalFiberLength();
    _pennationAngleAtOptimal = getPennationAngleAtOptimalFiberLength();
    _tendonSlackLength = getTendonSlackLength();
}

// Add Muscle's contributions to the underlying system
 void Muscle::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    addModelingOption("ignore_tendon_compliance", 1);
    addModelingOption("ignore_activation_dynamics", 1);

    // Cache the calculated values for this muscle categorized by their realization stage

    //Matt.Millard: changed length info to the velocity stage, as I need to know
    //              both the position and velocity of the multibody system and
    //              the muscles path before solving for the fiber length and
    //              velocity in the reduced model.
    this->_lengthInfoCV = addCacheVariable("lengthInfo", MuscleLengthInfo(), SimTK::Stage::Velocity);
    this->_velInfoCV = addCacheVariable("velInfo", FiberVelocityInfo(), SimTK::Stage::Velocity);
    this->_dynamicsInfoCV = addCacheVariable("dynamicsInfo", MuscleDynamicsInfo(), SimTK::Stage::Dynamics);
    this->_potentialEnergyInfoCV = addCacheVariable("potentialEnergyInfo", MusclePotentialEnergyInfo(), SimTK::Stage::Velocity);
 }

void Muscle::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    set_ignore_tendon_compliance(getIgnoreTendonCompliance(state));
    set_ignore_activation_dynamics(getIgnoreActivationDynamics(state));
}

void  Muscle::extendInitStateFromProperties(SimTK::State& state) const {
    Super::extendInitStateFromProperties(state);

    setIgnoreTendonCompliance(state,
        get_ignore_tendon_compliance());
    setIgnoreActivationDynamics(state,
        get_ignore_activation_dynamics());
}

// Get/set runtime flag to ignore tendon compliance when computing muscle
// dynamics.
bool Muscle::getIgnoreTendonCompliance(const SimTK::State& s) const
{
    return (getModelingOption(s, "ignore_tendon_compliance") > 0);
}

void Muscle::setIgnoreTendonCompliance(SimTK::State& s, bool ignore) const
{
    setModelingOption(s, "ignore_tendon_compliance", int(ignore));
}


/* get/set flag to activation dynamics when computing muscle dynamics  */
bool Muscle::getIgnoreActivationDynamics(const SimTK::State& s) const
{
    return (getModelingOption(s, "ignore_activation_dynamics") > 0);
}

void Muscle::setIgnoreActivationDynamics(SimTK::State& s, bool ignore) const
{
    setModelingOption(s, "ignore_activation_dynamics", int(ignore));
}



//=============================================================================
// GET values of interest from calculations
//=============================================================================
//_____________________________________________________________________________
//**
// * get the excitation value for this Muscle
// */
double Muscle::getExcitation( const SimTK::State& s) const {
    return( getControl(s) );
}


/* get the activation level of the muscle, which modulates the active force of the muscle
    and has a normalized (0 to 1) value */
double Muscle::getActivation(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).activation;
}

/* get the current working fiber length (m) for the muscle */
double Muscle::getFiberLength(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).fiberLength;
}

/* get the current pennation angle (radians) between the fiber and tendon at the current fiber length  */
double Muscle::getPennationAngle(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).pennationAngle;
}

/* get the cosine of the current pennation angle (radians) between the fiber and tendon at the current fiber length  */
double Muscle::getCosPennationAngle(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).cosPennationAngle;
}

/* get the current tendon length (m)  given the current joint angles and fiber length */
double Muscle::getTendonLength(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).tendonLength;
}

/* get the current normalized fiber length (fiber_length/optimal_fiber_length) */
double Muscle::getNormalizedFiberLength(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).normFiberLength;
}

/* get the current fiber length (m) projected (*cos(pennationAngle)) onto the tendon direction */
double Muscle::getFiberLengthAlongTendon(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).fiberLength * getMuscleLengthInfo(s).cosPennationAngle;
}

/* get the current tendon strain (delta_l/lo is dimensionless)  */
double Muscle::getTendonStrain(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).tendonStrain;
}

/* the potential energy (J) stored in the fiber due to its parallel elastic element */
double Muscle::getFiberPotentialEnergy(const SimTK::State& s) const
{
    return getMusclePotentialEnergyInfo(s).fiberPotentialEnergy;
}

/* the potential energy (J) stored in the tendon */
double Muscle::getTendonPotentialEnergy(const SimTK::State& s) const
{
    return getMusclePotentialEnergyInfo(s).tendonPotentialEnergy;
}

/* the total potential energy (J) stored in the muscle */
double Muscle::getMusclePotentialEnergy(const SimTK::State& s) const
{
    return getMusclePotentialEnergyInfo(s).musclePotentialEnergy;
}

/* get the passive fiber (parallel elastic element) force multiplier */
double Muscle::getPassiveForceMultiplier(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).fiberPassiveForceLengthMultiplier;
}

/* get the active fiber (contractile element) force multiplier due to current fiber length */
double Muscle::getActiveForceLengthMultiplier(const SimTK::State& s) const
{
    return getMuscleLengthInfo(s).fiberActiveForceLengthMultiplier;
}

/* get current fiber velocity (m/s) positive is lengthening */
double Muscle::getFiberVelocity(const SimTK::State& s) const
{
    return getFiberVelocityInfo(s).fiberVelocity;
}

/* get normalized fiber velocity (fiber_length/s / max_contraction_velocity) */
double Muscle::getNormalizedFiberVelocity(const SimTK::State& s) const
{
    return getFiberVelocityInfo(s).normFiberVelocity;
}

/* get the current fiber velocity (m/s) projected onto the tendon direction */
double Muscle::getFiberVelocityAlongTendon(const SimTK::State& s) const
{
    return getFiberVelocityInfo(s).fiberVelocityAlongTendon;
}

/* get the tendon velocity (m/s) */
double Muscle::getTendonVelocity(const SimTK::State& s) const
{
    return getFiberVelocityInfo(s).tendonVelocity;
}

/* get the dimensionless factor resulting from the fiber's force-velocity curve */
double Muscle::getForceVelocityMultiplier(const SimTK::State& s) const
{
    return getFiberVelocityInfo(s).fiberForceVelocityMultiplier;
}

/* get pennation angular velocity (radians/s) */
double Muscle::getPennationAngularVelocity(const SimTK::State& s) const
{
    return getFiberVelocityInfo(s).pennationAngularVelocity;
}

/* get the current fiber force (N)*/
double Muscle::getFiberForce(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).fiberForce;
}

/* get the current fiber force (N) applied to the tendon */
double Muscle::getFiberForceAlongTendon(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).fiberForceAlongTendon;
}


/* get the current active fiber force (N) due to activation*force_length*force_velocity relationships */
double Muscle::getActiveFiberForce(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).activeFiberForce;
}

/* get the total force applied by all passive elements in the fiber (N) */
double Muscle::getPassiveFiberForce(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).passiveFiberForce;
}

/* get the current active fiber force (N) projected onto the tendon direction */
double Muscle::getActiveFiberForceAlongTendon(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).activeFiberForce * getMuscleLengthInfo(s).cosPennationAngle;
}

/* get the total force applied by all passive elements in the fiber (N)
   projected onto the tendon direction */
double Muscle::getPassiveFiberForceAlongTendon(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).passiveFiberForce * getMuscleLengthInfo(s).cosPennationAngle;
}

/* get the current tendon force (N) applied to bones */
double Muscle::getTendonForce(const SimTK::State& s) const
{
    return getMaxIsometricForce() * getMuscleDynamicsInfo(s).normTendonForce;
}

/* get the current fiber stiffness (N/m) defined as the partial derivative
    of fiber force w.r.t. fiber length */
double Muscle::getFiberStiffness(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).fiberStiffness;
}

/* get the current fiber stiffness (N/m) defined as the partial derivative
    of fiber force along the tendon w.r.t. small changes in fiber length
    along the tendon*/
double Muscle::getFiberStiffnessAlongTendon(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).fiberStiffnessAlongTendon;
}


/* get the current tendon stiffness (N/m) defined as the partial derivative
    of tendon force w.r.t. tendon length */
double Muscle::getTendonStiffness(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).tendonStiffness;
}

/* get the current muscle stiffness (N/m) defined as the partial derivative
    of muscle force w.r.t. muscle length */
double Muscle::getMuscleStiffness(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).muscleStiffness;
}

/* get the current fiber power (W) */
double Muscle::getFiberActivePower(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).fiberActivePower;
}

/* get the current fiber active power (W) */
double Muscle::getFiberPassivePower(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).fiberPassivePower;
}

/* get the current tendon power (W) */
double Muscle::getTendonPower(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).tendonPower;
}

/* get the current muscle power (W) */
double Muscle::getMusclePower(const SimTK::State& s) const
{
    return getMuscleDynamicsInfo(s).musclePower;
}


void Muscle::setExcitation(SimTK::State& s, double excitation) const
{
    setControls(SimTK::Vector(1, excitation), _model->updControls(s));
}

/* Access to muscle calculation data structures */
const Muscle::MuscleLengthInfo& Muscle::getMuscleLengthInfo(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _lengthInfoCV)) {
        return getCacheVariableValue(s, _lengthInfoCV);
    }

    MuscleLengthInfo& umli = updCacheVariableValue(s, _lengthInfoCV);
    calcMuscleLengthInfo(s, umli);
    markCacheVariableValid(s, _lengthInfoCV);
    return umli;
}

Muscle::MuscleLengthInfo& Muscle::updMuscleLengthInfo(const SimTK::State& s) const
{
    return updCacheVariableValue(s, _lengthInfoCV);
}

const Muscle::FiberVelocityInfo& Muscle::
getFiberVelocityInfo(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _velInfoCV)) {
        return getCacheVariableValue(s, _velInfoCV);
    }

    FiberVelocityInfo& ufvi = updCacheVariableValue(s, _velInfoCV);
    calcFiberVelocityInfo(s, ufvi);
    markCacheVariableValid(s, _velInfoCV);
    return ufvi;
}

Muscle::FiberVelocityInfo& Muscle::
updFiberVelocityInfo(const SimTK::State& s) const
{
    return updCacheVariableValue(s, _velInfoCV);
}

const Muscle::MuscleDynamicsInfo& Muscle::
getMuscleDynamicsInfo(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _dynamicsInfoCV)) {
        return getCacheVariableValue(s, _dynamicsInfoCV);
    }

    MuscleDynamicsInfo& umdi = updCacheVariableValue(s, _dynamicsInfoCV);
    calcMuscleDynamicsInfo(s, umdi);
    markCacheVariableValid(s, _dynamicsInfoCV);
    return umdi;
}
Muscle::MuscleDynamicsInfo& Muscle::
updMuscleDynamicsInfo(const SimTK::State& s) const
{
    return updCacheVariableValue(s, _dynamicsInfoCV);
}

const Muscle::MusclePotentialEnergyInfo& Muscle::
getMusclePotentialEnergyInfo(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _potentialEnergyInfoCV)) {
        return getCacheVariableValue(s, _potentialEnergyInfoCV);
    }

    MusclePotentialEnergyInfo& umpei = updCacheVariableValue(s, _potentialEnergyInfoCV);
    calcMusclePotentialEnergyInfo(s, umpei);
    markCacheVariableValid(s, _potentialEnergyInfoCV);
    return umpei;
}

Muscle::MusclePotentialEnergyInfo& Muscle::
updMusclePotentialEnergyInfo(const SimTK::State& s) const
{
    return updCacheVariableValue(s, _potentialEnergyInfoCV);
}



//_____________________________________________________________________________
/**
 * Get the stress in this muscle actuator.  It is calculated as the force
 * divided by the maximum isometric force (which is proportional to its area).
 */
double Muscle::getStress(const SimTK::State& s) const
{
    return getActuation(s) / getMaxIsometricForce();
}


//=============================================================================
// CALCULATIONS
//=============================================================================
/* calculate muscle's position related values such fiber and tendon lengths,
    normalized lengths, pennation angle, etc... */
void Muscle::calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
{
    throw Exception("ERROR- "+getConcreteClassName()
        + "::calcMuscleLengthInfo() NOT IMPLEMENTED.");
}

/* calculate muscle's velocity related values such fiber and tendon velocities,
    normalized velocities, pennation angular velocity, etc... */
void Muscle::calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
    throw Exception("ERROR- "+getConcreteClassName()
        + "::calcFiberVelocityInfo() NOT IMPLEMENTED.");
}

/* calculate muscle's active and passive force-length, force-velocity,
    tendon force, relationships and their related values */
void Muscle::calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
    throw Exception("ERROR- "+getConcreteClassName()
        + "::calcMuscleDynamicsInfo() NOT IMPLEMENTED.");
}

/* calculate muscle's fiber and tendon potential energy */
void Muscle::calcMusclePotentialEnergyInfo(const SimTK::State& s,
    MusclePotentialEnergyInfo& mpei) const
{
    throw Exception("ERROR- "+getConcreteClassName()
        + "::calcMusclePotentialEnergyInfo() NOT IMPLEMENTED.");
}


//=============================================================================
// Required by CMC and Static Optimization
//=============================================================================
double Muscle::calcInextensibleTendonActiveFiberForce(SimTK::State& s,
                                                  double activation) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);
    return getMaxIsometricForce()*activation*
        mli.fiberActiveForceLengthMultiplier*fvi.fiberForceVelocityMultiplier
        *mli.cosPennationAngle;
}

//=============================================================================
// FORCE APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the muscle's force at its points of attachment to the bodies.
 */
void Muscle::computeForce(const SimTK::State& s,
                          SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                          SimTK::Vector& generalizedForces) const
{
    // This calls compute actuation.
    Super::computeForce(s, bodyForces, generalizedForces);

    if (getDebugLevel() < 0) return;
    // NOTE: Actuation could be negative, in particular during CMC, when the
    // optimizer is computing gradients, but in those cases the actuation will
    // be overridden and will not be computed by the muscle.
    if (!isActuationOverridden(s) && (getActuation(s) < -SimTK::SqrtEps)) {
        string msg = getConcreteClassName()
            + "::computeForce, muscle "+ getName() + " force < 0";
        log_debug("{}  at time = ", msg, s.getTime());
        //throw Exception(msg);
    }
}

double Muscle::computePotentialEnergy(const SimTK::State& s) const
{
    const MusclePotentialEnergyInfo& mpei = getMusclePotentialEnergyInfo(s);
    return mpei.musclePotentialEnergy;
}

SimTK::Vec3 Muscle::computePathColor(const SimTK::State& state) const {
    const double activation =
        SimTK::clamp(0., getActivation(state), 1.);
    const SimTK::Vec3 color(activation, 0, 1-activation); // blue to red
    return color;
}


void Muscle::updateGeometry(const SimTK::State& s)
{
    updGeometryPath().updateGeometry(s);
}


//_____________________________________________________________________________
/**
 * Utility function to calculate the current pennation angle in a
 * muscle. Pennation angle increases as muscle fibers shorten. The implicit
 * modeling assumption is that muscles have constant width.
 *
 * @param s State
 * @return Current pennation angle (in radians).
 */
/*
 double Muscle::calcPennationAngle(const SimTK::State &s) const
{
    double aFiberLength = getFiberLength(s);
    if (aFiberLength < SimTK::Eps){
        cout << "Muscle::calcPennationAngle() ERROR- fiber length is zero." << endl;
        return SimTK::NaN;
    }

    double value = _muscleWidth/aFiberLength;

    if(value >= 1.0){
        cout << "Muscle::calcPennationAngle() ERROR- pennation at 90 degrees." << endl;
        return SimTK_PI/2.0;
    }
   else
      return asin(value);
}
*/
