/* -------------------------------------------------------------------------- *
 *                     OpenSim:  CoordinateLimitForce.cpp                     *
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
#include "CoordinateLimitForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
CoordinateLimitForce::CoordinateLimitForce()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
CoordinateLimitForce::CoordinateLimitForce
   (const string& coordName, double q_upper, 
    double K_upper, double q_lower, double K_lower, double damping, double dq, 
    bool computeDissipationEnergy) : Force()
{
    setNull();
    constructProperties();

    set_coordinate(coordName);
    set_upper_stiffness(K_upper);
    set_upper_limit(q_upper);
    set_lower_stiffness(K_lower);
    set_lower_limit(q_lower);
    set_damping(damping);
    set_transition(dq);

    set_compute_dissipation_energy(computeDissipationEnergy);

    setName(coordName + "_LimitForce");
}

//_____________________________________________________________________________
// Set the data members of this actuator to their null values. Note that we
// also use this after copy construction or copy assignment; these should be
// calculated at extendConnectToModel().
void CoordinateLimitForce::setNull()
{
    setAuthors("Ajay Seth");
    _upStep.reset();
    _loStep.reset();
    
    // Scaling for coordinate values in m or degrees (rotational) 
    _w = SimTK::NaN;

    // Coordinate limits in internal (SI) units (m or rad)
    _qup = SimTK::NaN;
    _qlow = SimTK::NaN;
    // Constant stiffnesses in internal (SI) N/m or Nm/rad
    _Kup = SimTK::NaN;;
    _Klow = SimTK::NaN;

    // Damping in internal (SI) units of N/(m/s) or Nm/(rad/s)
    _damp = SimTK::NaN;

    _coord = NULL;
}
//_____________________________________________________________________________
// Allocate and initialize properties.
void CoordinateLimitForce::constructProperties()
{
    constructProperty_coordinate("UNASSIGNED");
    constructProperty_upper_stiffness(1.0);
    constructProperty_upper_limit(0.0);
    constructProperty_lower_stiffness(1.0);
    constructProperty_lower_limit(0.0);
    constructProperty_damping(0.001);
    constructProperty_transition(0.1);
    constructProperty_compute_dissipation_energy(false);
}


//=============================================================================
// GET AND SET CoordinateLimitForce Stiffness and Damping parameters
//=============================================================================
void CoordinateLimitForce::setUpperStiffness(double aUpperStiffness)
{
    set_upper_stiffness(aUpperStiffness);
}

void CoordinateLimitForce::setUpperLimit(double aUpperLimit)
{
    set_upper_limit(aUpperLimit);
}

void CoordinateLimitForce::setLowerStiffness(double aLowerStiffness)
{
    set_lower_stiffness(aLowerStiffness);
}

void CoordinateLimitForce::setLowerLimit(double aLowerLimit)
{
    set_lower_limit(aLowerLimit);
}

void CoordinateLimitForce::setDamping(double aDamping)
{
    set_damping(aDamping);
}

void CoordinateLimitForce::setTransition(double aTransition)
{
    set_transition(aTransition);
}

void CoordinateLimitForce::setComputeDissipationEnergy(bool flag)
{
    set_compute_dissipation_energy(flag);
}

//_____________________________________________________________________________
/**
 * Get the parameters.
 */
double CoordinateLimitForce::getUpperStiffness() const
{
    return get_upper_stiffness();
}

double CoordinateLimitForce::getUpperLimit() const
{
    return get_upper_limit();
}

double CoordinateLimitForce::getLowerStiffness() const
{
    return get_lower_stiffness();
}
double CoordinateLimitForce::getLowerLimit() const
{
    return get_lower_limit();
}

double CoordinateLimitForce::getDamping() const
{
    return get_damping();
}

double CoordinateLimitForce::getTransition() const
{
    return get_transition();
}

bool CoordinateLimitForce::isComputingDissipationEnergy() const
{
    return get_compute_dissipation_energy();
}

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this CoordinateLimitForce.
 */
void CoordinateLimitForce::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    string errorMessage;

    const string& coordName = get_coordinate();
    const double& upperStiffness = get_upper_stiffness();
    const double& upperLimit = get_upper_limit();
    const double& lowerStiffness = get_lower_stiffness();
    const double& lowerLimit = get_lower_limit();
    const double& transition = get_transition();
    const double& damping = get_damping();

    // Look up the coordinate
    if (!_model->updCoordinateSet().contains(coordName)) {
        errorMessage = "CoordinateLimitForce: Invalid coordinate (" + coordName + ") specified in Actuator " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    _coord = &_model->updCoordinateSet().get(coordName);

    // scaling for units
    _w = (_coord->getMotionType() == Coordinate::Rotational) ? SimTK_DEGREE_TO_RADIAN : 1.0;

    _qup = _w*upperLimit;
    _qlow = _w*lowerLimit;
    _Kup = upperStiffness/_w;
    _Klow = lowerStiffness/_w;
    _damp = damping/_w;

    // Define the transition from no stiffness to the upperStiffness as coordinate increases 
    // beyond the upper limit
    _upStep.reset(
            new SimTK::Function::Step(0.0, _Kup, _qup, _qup+_w*transition));
    // Define the transition from lowerStiffness to zero as coordinate increases to the lower limit
    _loStep.reset(
            new SimTK::Function::Step(_Klow, 0.0, _qlow-_w*transition, _qlow));
}


/** Create the underlying Force that is part of the multibody system. */
void CoordinateLimitForce::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    this->_dissipationPowerCV =
        addCacheVariable("dissipationPower", 0.0, SimTK::Stage::Dynamics);

    if(isComputingDissipationEnergy()){
        addStateVariable("dissipatedEnergy");
    }
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and apply the mobility force corresponding to the passive limit force
 *
 */
void CoordinateLimitForce::computeForce( const SimTK::State& s, 
                               SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                               SimTK::Vector& mobilityForces) const
{
    applyGeneralizedForce(s, *_coord, calcLimitForce(s), mobilityForces);
}

double CoordinateLimitForce::calcLimitForce( const SimTK::State& s) const
{
    double q = _coord->getValue(s);
    SimTK::Vector qv(1,q);
    double K_up = _upStep->calcValue(qv);
    double K_low = _loStep->calcValue(qv);

    double qdot = _coord->getSpeedValue(s);
    double f_up = -K_up*(q - _qup);
    double f_low = K_low*(_qlow - q);

    // dividing the stiffness by the constant yields the transition function that can 
    // also be applied to damping
    double f_damp = -_damp*(K_up/_Kup + K_low/_Klow)*qdot;

    // dissipative power is negative power but is already implied by "dissipation"
    // so negate power so that dissipation power is a positive number
    double dissPower = -qdot * f_damp;
    setCacheVariableValue(s, _dissipationPowerCV, dissPower);

    double f_limit = f_up + f_low + f_damp;

    return f_limit;
}

// Potential energy stored in the limit spring
double CoordinateLimitForce::computePotentialEnergy(const SimTK::State& s) const
{
    double q = _coord->getValue(s);
    SimTK::Vector qv(1,q);

    double K=0;
    double delta = 0;
    if(q > _qup){ // against upper limit
        K = _Kup;
        delta = q-_qup;
    }
    else if(q < _qlow){ // against lower limit
        K = _Klow;
        delta = (_qlow-q);
    }
    else{
        // no limits being hit
        return 0.0;
    }
    
    const double &trans = _w*get_transition();

    if(delta >= trans){
        // = 5/14*K*trans^2 - 1/2*K*trans^2 + 1/2*K*(delta)^2
        return K*((-2.0/14.0)*trans*trans + 0.5*(delta)*(delta));
    }
    else{
        double x = delta/trans;
        // This is the integral of K(x)*x*dx evaluated at
        // x = delta/trans, where 
        // K(x) = K*(10*x^3-15*x^4+6*x^5) is the definition of an
        // S curve continuous step function defined in Simbody
        // SimTK/include/SimTKcommon/Scalar.h
        return K*x*x*x*(2.0-2.5*x+(6.0/7.0)*x*x) * (delta*delta);
    }
}
// power dissipated by the damping term of the coordinate limit force
double CoordinateLimitForce::getPowerDissipation(const SimTK::State& s) const
{
    return  getCacheVariableValue(s, _dissipationPowerCV);
}

// energy dissipated by the damping term of the coordinate limit force
double CoordinateLimitForce::getDissipatedEnergy(const SimTK::State& s) const
{
    if(isComputingDissipationEnergy()){
        return getStateVariableValue(s, "dissipatedEnergy");
    } else {
        throw Exception("CoordinateLimitForce::getDissipatedEnergy() compute_dissipation_energy set to false.");
        return SimTK::NaN;
    }
}

void CoordinateLimitForce::
    computeStateVariableDerivatives(const SimTK::State& s) const
{
    if (appliesForce(s) && isComputingDissipationEnergy()){
        setStateVariableDerivativeValue(s, "dissipatedEnergy", 
            getPowerDissipation(s));
    }
}


/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
Array<std::string> CoordinateLimitForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName());
    labels.append("PotentialEnergy");
    return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
Array<double> CoordinateLimitForce::getRecordValues(const SimTK::State& state) const {
    OpenSim::Array<double> values(0.0, 0, 2);
    values.append(calcLimitForce(state));
    values.append(computePotentialEnergy(state));
    return values;
}
