/* -------------------------------------------------------------------------- *
 *                 OpenSim:  Millard2012EquilibriumMuscle.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard, Tom Uchida, Ajay Seth                          *
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
#include "Millard2012EquilibriumMuscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <iostream>
#include <OpenSim/Common/Exception.h>
#include <SimTKcommon/internal/ExceptionMacros.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string Millard2012EquilibriumMuscle::
    STATE_ACTIVATION_NAME = "activation";
const string Millard2012EquilibriumMuscle::
    STATE_FIBER_LENGTH_NAME = "fiber_length";
const double MIN_NONZERO_DAMPING_COEFFICIENT = 0.001;

//==============================================================================
// PROPERTIES
//==============================================================================
void Millard2012EquilibriumMuscle::setNull()
{   setAuthors("Matthew Millard, Tom Uchida, Ajay Seth"); }

void Millard2012EquilibriumMuscle::constructProperties()
{
    constructProperty_fiber_damping(0.1); //damped model used by default
    constructProperty_default_activation(0.05);
    constructProperty_default_fiber_length(getOptimalFiberLength());

    constructProperty_activation_time_constant(0.010);
    constructProperty_deactivation_time_constant(0.040);
    constructProperty_minimum_activation(0.01);

    constructProperty_ActiveForceLengthCurve(ActiveForceLengthCurve());
    constructProperty_ForceVelocityCurve(ForceVelocityCurve());
    constructProperty_FiberForceLengthCurve(FiberForceLengthCurve());
    constructProperty_TendonForceLengthCurve(TendonForceLengthCurve());
}

void Millard2012EquilibriumMuscle::buildMuscle()
{
    try {
        // Fiber velocity becomes unstable as pennation angle approaches Pi/2
        // because the kinematic equations for fiber velocity approach a
        // singularity as phi -> Pi/2.
        //      lceAT = lce*cos(phi)
        //      dlceAT = dlcedt*cos(phi) - lce*sin(phi)*dphidt
        //      dlcedt = (dlceAT + lce*sin(phi)*dphidt) / cos(phi)
        double maxPennationAngle = acos(0.1);
        penMdl = MuscleFixedWidthPennationModel(getOptimalFiberLength(),
                                        getPennationAngleAtOptimalFiberLength(),
                                        maxPennationAngle);

        // Ensure object names are up-to-date
        const std::string& aName = getName();

        ActiveForceLengthCurve& falCurve = upd_ActiveForceLengthCurve();
        falCurve.setName(aName+"_ActiveForceLengthCurve");

        ForceVelocityCurve& fvCurve = upd_ForceVelocityCurve();
        fvCurve.setName(aName+"_ForceVelocityCurve");

        FiberForceLengthCurve& fpeCurve = upd_FiberForceLengthCurve();
        fpeCurve.setName(aName+"_FiberForceLengthCurve");

        TendonForceLengthCurve& fseCurve = upd_TendonForceLengthCurve();
        fseCurve.setName(aName+"_TendonForceLengthCurve");

        // Include fiber damping in the model only if the damping coefficient is
        // larger than MIN_NONZERO_DAMPING_COEFFICIENT. This is done to ensure
        // we remain sufficiently far from the numerical singularity at beta=0.
        use_fiber_damping = (getFiberDamping() >=
                             MIN_NONZERO_DAMPING_COEFFICIENT);

        // To initialize, we need to create an inverse force-velocity curve
        double conSlopeAtVmax   = fvCurve.getConcentricSlopeAtVmax();
        double conSlopeNearVmax = fvCurve.getConcentricSlopeNearVmax();
        double isometricSlope   = fvCurve.getIsometricSlope();
        double eccSlopeAtVmax   = fvCurve.getEccentricSlopeAtVmax();
        double eccSlopeNearVmax = fvCurve.getEccentricSlopeNearVmax();
        double conCurviness     = fvCurve.getConcentricCurviness();
        double eccCurviness     = fvCurve.getEccentricCurviness();
        double eccForceMax      = fvCurve.
                                  getMaxEccentricVelocityForceMultiplier();

        // A few parameters may need to be adjusted to avoid singularities
        // (e.g., if an elastic tendon is used with no fiber damping).
        if(!get_ignore_tendon_compliance() && !use_fiber_damping) {
			set_minimum_activation(clamp(0.01, get_minimum_activation(), 1));

            if(falCurve.getMinValue() < 0.1) {
                falCurve.setMinValue(0.1);
            }
            if(cos(penMdl.getMaximumPennationAngle()) < SimTK::SignificantReal)
            {
                penMdl.setMaximumPennationAngle(maxPennationAngle);
            }
            if(conSlopeAtVmax < 0.1 || eccSlopeAtVmax < 0.1) {
                fvCurve.setCurveShape(0.1, conSlopeNearVmax, isometricSlope,
                                      0.1, eccSlopeNearVmax, eccForceMax);
            }

        } else { //singularity-free model
			set_minimum_activation(clamp(0, get_minimum_activation(), 1));
            falCurve.setMinValue(0.0);
            fvCurve.setCurveShape(0.0, conSlopeNearVmax, isometricSlope,
                                  0.0, eccSlopeNearVmax, eccForceMax);
        }

        if(conSlopeAtVmax < 0.1 || eccSlopeAtVmax < 0.1) {
            conSlopeAtVmax = 0.1;
            eccSlopeAtVmax = 0.1;
        }
        fvInvCurve = ForceVelocityInverseCurve(conSlopeAtVmax,
                                               conSlopeNearVmax,
                                               isometricSlope,
                                               eccSlopeAtVmax,
                                               eccSlopeNearVmax,
                                               eccForceMax,
                                               conCurviness,
                                               eccCurviness);

        // Ensure all sub-objects are up-to-date
        penMdl.ensureModelUpToDate();
        falCurve.ensureCurveUpToDate();
        fvCurve.ensureCurveUpToDate();
        fvInvCurve.ensureCurveUpToDate();
        fpeCurve.ensureCurveUpToDate();
        fseCurve.ensureCurveUpToDate();

        // Compute the minimum active fiber length (in meters)
        double minActiveFiberLength = falCurve.getMinActiveFiberLength()
                                      * getOptimalFiberLength();
 
        // Minimum pennated fiber length (in meters)
        double minPennatedFiberLength = penMdl.getMinimumFiberLength();
        m_minimumFiberLength = max(minActiveFiberLength,minPennatedFiberLength);

        // Minimum fiber length along the tendon
        double phi = penMdl.calcPennationAngle(m_minimumFiberLength);
        m_minimumFiberLengthAlongTendon =
            penMdl.calcFiberLengthAlongTendon(m_minimumFiberLength,cos(phi));

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in " + getName()
                          + "::buildMuscle()\n" + x.what();
        throw OpenSim::Exception(msg);
    }
    setObjectIsUpToDateWithProperties();
}

void Millard2012EquilibriumMuscle::ensureMuscleUpToDate()
{
    if(!isObjectUpToDateWithProperties()) {
        buildMuscle();
    }
}

//==============================================================================
// CONSTRUCTORS
//==============================================================================
Millard2012EquilibriumMuscle::Millard2012EquilibriumMuscle()
{
    setNull();
    constructProperties();
    ensureMuscleUpToDate();
}

Millard2012EquilibriumMuscle::Millard2012EquilibriumMuscle(
const std::string &aName, double aMaxIsometricForce, double aOptimalFiberLength,
double aTendonSlackLength, double aPennationAngle)
{
    setNull();
    constructProperties();

    setName(aName);
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);

    ensureMuscleUpToDate();
}

//==============================================================================
// GET METHODS
//==============================================================================
bool Millard2012EquilibriumMuscle::getUseFiberDamping() const
{   return use_fiber_damping; }
double Millard2012EquilibriumMuscle::getFiberDamping() const
{   return get_fiber_damping(); }
double Millard2012EquilibriumMuscle::getDefaultActivation() const
{   return get_default_activation(); }
double Millard2012EquilibriumMuscle::getDefaultFiberLength() const
{   return clampFiberLength(get_default_fiber_length()); }
double Millard2012EquilibriumMuscle::getActivationTimeConstant() const
{   return get_activation_time_constant(); }
double Millard2012EquilibriumMuscle::getDeactivationTimeConstant() const
{   return get_deactivation_time_constant(); }
double Millard2012EquilibriumMuscle::getMinimumActivation() const
{   return get_minimum_activation(); }

const ActiveForceLengthCurve& Millard2012EquilibriumMuscle::
getActiveForceLengthCurve() const
{   return get_ActiveForceLengthCurve(); }

const ForceVelocityCurve& Millard2012EquilibriumMuscle::
getForceVelocityCurve() const
{   return get_ForceVelocityCurve(); }

const FiberForceLengthCurve& Millard2012EquilibriumMuscle::
getFiberForceLengthCurve() const
{   return get_FiberForceLengthCurve(); }

const TendonForceLengthCurve& Millard2012EquilibriumMuscle::
getTendonForceLengthCurve() const
{   return get_TendonForceLengthCurve(); }

const MuscleFixedWidthPennationModel& Millard2012EquilibriumMuscle::
getPennationModel() const
{   return penMdl; }

double Millard2012EquilibriumMuscle::getMaximumPennationAngle() const
{   return penMdl.getMaximumPennationAngle(); }
double Millard2012EquilibriumMuscle::getMinimumFiberLength() const
{   return m_minimumFiberLength; }
double Millard2012EquilibriumMuscle::getMinimumFiberLengthAlongTendon() const
{   return m_minimumFiberLengthAlongTendon; }

double Millard2012EquilibriumMuscle::
getTendonForceMultiplier(SimTK::State& s) const
{   return getMuscleDynamicsInfo(s).normTendonForce; }

double Millard2012EquilibriumMuscle::
getFiberStiffnessAlongTendon(const SimTK::State& s) const
{ return getMuscleDynamicsInfo(s).fiberStiffnessAlongTendon; }

double Millard2012EquilibriumMuscle::
getFiberVelocity(const SimTK::State& s) const
{   return getFiberVelocityInfo(s).fiberVelocity; }

double Millard2012EquilibriumMuscle::
getActivationDerivative(const SimTK::State& s) const
{
    double activationDerivative = 0.0;

    if(!get_ignore_activation_dynamics()) {
        double u = getExcitation(s);
        double a = getActivation(s);
        activationDerivative = calcActivationDerivative(a,u);
    }
    return activationDerivative;
}

Array<std::string> Millard2012EquilibriumMuscle::getStateVariableNames() const
{
    Array<std::string> stateVariableNames =
        ModelComponent::getStateVariableNames();

    for(int i=0; i<stateVariableNames.getSize(); ++i) {
        stateVariableNames[i] = getName()+"."+stateVariableNames[i];
    }
    return stateVariableNames;
}

SimTK::SystemYIndex Millard2012EquilibriumMuscle::
getStateVariableSystemIndex(const std::string &stateVariableName) const
{
    unsigned start = (unsigned)stateVariableName.find(".");
    unsigned end   = (unsigned)stateVariableName.length();

    if(start==end) {
        return ModelComponent::getStateVariableSystemIndex(stateVariableName);
    } else {
        string localName = stateVariableName.substr(++start, end-start);
        return ModelComponent::getStateVariableSystemIndex(localName);
    }
}

//==============================================================================
// SET METHODS
//==============================================================================
void Millard2012EquilibriumMuscle::
setMuscleConfiguration(bool ignoreTendonCompliance,
                       bool ignoreActivationDynamics,
                       double dampingCoefficient)
{
    set_ignore_tendon_compliance(ignoreTendonCompliance);
    set_ignore_activation_dynamics(ignoreActivationDynamics);
    setFiberDamping(dampingCoefficient);
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::setFiberDamping(double dampingCoefficient)
{
    if(dampingCoefficient < MIN_NONZERO_DAMPING_COEFFICIENT) {
        set_fiber_damping(0.0);
        use_fiber_damping = false;
    } else {
    set_fiber_damping(dampingCoefficient);
        use_fiber_damping = true;
    }
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::setDefaultActivation(double activation)
{
    set_default_activation(clampActivation(activation));
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
setActivation(SimTK::State& s, double activation) const
{   
    if(!get_ignore_activation_dynamics()) {
        setStateVariable(s, STATE_ACTIVATION_NAME, clampActivation(activation));
        markCacheVariableInvalid(s,"velInfo");
        markCacheVariableInvalid(s,"dynamicsInfo");
    }
}

void Millard2012EquilibriumMuscle::setDefaultFiberLength(double fiberLength)
{
    set_default_fiber_length(clampFiberLength(fiberLength));
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
setActivationTimeConstant(double activationTimeConstant)
{
    set_activation_time_constant(max(0.0, activationTimeConstant));
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
setDeactivationTimeConstant(double deactivationTimeConstant)
{
    set_deactivation_time_constant(max(0.0, deactivationTimeConstant));
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
setMinimumActivation(double minimumActivation)
{
    set_minimum_activation(min(1.0, max(0.0, minimumActivation)));
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::setActiveForceLengthCurve(
ActiveForceLengthCurve& aActiveForceLengthCurve)
{
    set_ActiveForceLengthCurve(aActiveForceLengthCurve);
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::setForceVelocityCurve(
ForceVelocityCurve& aForceVelocityCurve)
{
    set_ForceVelocityCurve(aForceVelocityCurve);
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::setFiberForceLengthCurve(
FiberForceLengthCurve& aFiberForceLengthCurve)
{
    set_FiberForceLengthCurve(aFiberForceLengthCurve);
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::setTendonForceLengthCurve(
TendonForceLengthCurve& aTendonForceLengthCurve)
{
    set_TendonForceLengthCurve(aTendonForceLengthCurve);
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
setFiberLength(SimTK::State& s, double fiberLength) const
{
    if(!get_ignore_tendon_compliance()) {
        setStateVariable(s, STATE_FIBER_LENGTH_NAME,
                         clampFiberLength(fiberLength));
        markCacheVariableInvalid(s,"lengthInfo");
        markCacheVariableInvalid(s,"velInfo");
        markCacheVariableInvalid(s,"dynamicsInfo");
    }
}

//==============================================================================
// MUSCLE.H INTERFACE
//==============================================================================
double Millard2012EquilibriumMuscle::
computeActuation(const SimTK::State& s) const
{
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setForce(s, mdi.tendonForce);
    return mdi.tendonForce;
}

//==============================================================================
// OTHER PUBLIC METHODS
//==============================================================================
void Millard2012EquilibriumMuscle::
computeInitialFiberEquilibrium(SimTK::State& s) const
{
    if(get_ignore_tendon_compliance()) {                    // rigid tendon
        return;
    }

    // Elastic tendon initialization routine.
    try {
        // Initialize activation as specified by the user.
        double clampedActivation = clampActivation(get_default_activation());
        setActivation(s,clampedActivation);

        // Initialize the multibody system to the initial state vector.
        setFiberLength(s, getOptimalFiberLength());
        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

        // Compute an initial muscle state that develops the desired force and
        // shares the muscle stretch between the muscle fiber and the tendon
        // according to their relative stiffnesses.
        int flag_status      = -1;
        double solnErr       = SimTK::NaN;
        int iterations       = 0;
        double fiberLength   = SimTK::NaN;
        double fiberVelocity = SimTK::NaN;
        double tendonForce   = SimTK::NaN;

        // tol is the desired tolerance in Newtons.
        double tol = 1e-8*getMaxIsometricForce();
        if(tol < SimTK::SignificantReal*10) {
            tol = SimTK::SignificantReal*10;
        }
        int maxIter = 200;
        double pathLength = getLength(s);
        double pathLengtheningSpeed = getLengtheningSpeed(s);

        SimTK::Vector soln;
        soln = estimateMuscleFiberState(clampedActivation, pathLength,
                                        pathLengtheningSpeed, tol, maxIter);
        flag_status   = (int)soln[0];
        solnErr       = soln[1];
        iterations    = (int)soln[2];
        fiberLength   = soln[3];
        fiberVelocity = soln[4];
        tendonForce   = soln[5];

        switch(flag_status) {
            case 0: //converged
            {
                setForce(s,tendonForce);
                setFiberLength(s,fiberLength);

            }break;

            case 1: //lower bound on fiber length was reached
            {
                setForce(s,tendonForce);
                setFiberLength(s,fiberLength);
                printf("\n\nMillard2012EquilibriumMuscle Initialization:"
                       "%s is at its minimum length of %f\n",
                       getName().c_str(), getMinimumFiberLength());
            }break;

            case 2: //maximum number of iterations reached
            {
                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());

                char msgBuffer[1000];
                int n = sprintf(msgBuffer,
                    "WARNING: No suitable initial conditions found for %s by "
                    "computeInitialFiberEquilibrium.\n"
                    "Continuing with an initial fiber force of 0 and an "
                    "initial length of %f.\n"
                    "Here is a report from the routine:\n\n"
                    "   Solution Error:    %f > tol (%f)\n"
                    "   Newton Iterations: %d of max. iterations (%d)\n"
                    "Verify that the initial activation is valid and that the "
                    "length of the musculotendon actuator\n"
                    "doesn't produce a pennation angle of 90 degrees or a "
                    "fiber length less than zero:\n"
                    "   Activation:      %f\n"
                    "   Actuator length: %f\n\n",
                    getName().c_str(),
                    penMdl.getOptimalFiberLength(),
                    abs(solnErr), tol,
                    iterations, maxIter,
                    clampedActivation,
                    fiberLength);

                    cerr << msgBuffer << endl;
            }break;

            default:
                printf("\n\nWARNING: invalid error flag returned from "
                       "initialization routine for %s."
                       "Setting tendon force to 0.0 and fiber length to the "
                       "optimal fiber length.",
                       getName().c_str());
                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());
        }

    } catch (const std::exception& e) {
        // If the initialization routine fails in some unexpected way, tell the
        // user and continue with some valid initial conditions.
        cerr << "\n\nWARNING: Millard2012EquilibriumMuscle initialization "
                "exception caught:" << endl;
        cerr << e.what() << endl;
        cerr << "Continuing with initial tendon force of 0 and a fiber length "
                "equal to the optimal fiber length.\n\n" << endl;
        setForce(s,0);
        setFiberLength(s,getOptimalFiberLength());
    }
}

void Millard2012EquilibriumMuscle::
computeFiberEquilibriumAtZeroVelocity(SimTK::State& s) const
{
    if(get_ignore_tendon_compliance()) {                    // rigid tendon
        return;
    }

    // Elastic tendon initialization routine.
    try {
        // Initialize activation as specified by the user.
        double clampedActivation = clampActivation(get_default_activation());
        setActivation(s,clampedActivation);

        // Initialize the multibody system to the initial state vector.
        setFiberLength(s, getOptimalFiberLength());
        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

        // Compute the fiber length where the fiber and tendon are in static
        // equilibrium. Fiber and tendon velocity are set to zero.
        int flag_status      = -1;
        double solnErr       = SimTK::NaN;
        int iterations       = 0;
        double fiberLength   = SimTK::NaN;
        double fiberVelocity = 0.0;
        double tendonForce   = SimTK::NaN;

        // tol is the desired tolerance in Newtons.
        double tol = 1e-8*getMaxIsometricForce();
        if(tol < SimTK::SignificantReal*10) {
            tol = SimTK::SignificantReal*10;
        }
        int maxIter = 200;
        double pathLength = getLength(s);
        double pathLengtheningSpeed = 0.0;

        SimTK::Vector soln;
        soln = estimateMuscleFiberState(clampedActivation, pathLength,
                                        pathLengtheningSpeed, tol, maxIter,
                                        true);
        flag_status   = (int)soln[0];
        solnErr       = soln[1];
        iterations    = (int)soln[2];
        fiberLength   = soln[3];
        fiberVelocity = soln[4];
        tendonForce   = soln[5];

        switch(flag_status) {
            case 0: //converged
            {
                setForce(s,tendonForce);
                setFiberLength(s,fiberLength);

            }break;

            case 1: //lower bound on fiber length was reached
            {
                setForce(s,tendonForce);
                setFiberLength(s,fiberLength);
                printf("\n\nMillard2012EquilibriumMuscle static solution:"
                       "%s is at its minimum length of %f\n",
                       getName().c_str(), getMinimumFiberLength());
            }break;

            case 2: //maximum number of iterations reached
            {
                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());

                char msgBuffer[1000];
                int n = sprintf(msgBuffer,
                    "WARNING: No suitable static solution found for %s by "
                    "computeFiberEquilibriumAtZeroVelocity().\n"
                    "Continuing with an initial fiber force of 0 and an "
                    "initial length of %f.\n"
                    "Here is a report from the routine:\n\n"
                    "   Solution Error:    %f > tol (%f)\n"
                    "   Newton Iterations: %d of max. iterations (%d)\n"
                    "Verify that the default activation is valid and that the "
                    "length of the musculotendon actuator\n"
                    "doesn't produce a pennation angle of 90 degrees or a "
                    "fiber length less than zero:\n"
                    "   Activation:      %f\n"
                    "   Actuator length: %f\n\n",
                    getName().c_str(),
                    penMdl.getOptimalFiberLength(),
                    abs(solnErr), tol,
                    iterations, maxIter,
                    clampedActivation,
                    fiberLength);

                    cerr << msgBuffer << endl;
            }break;

            default:
                printf("\n\nWARNING: invalid error flag returned from "
                       "static solution for %s."
                       "Setting tendon force to 0.0 and fiber length to the "
                       "optimal fiber length.",
                       getName().c_str());
                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());
        }

    } catch (const std::exception& e) {
        // If the initialization routine fails in some unexpected way, tell the
        // user and continue with some valid initial conditions.
        cerr << "\n\nWARNING: Millard2012EquilibriumMuscle static solution "
                "exception caught:" << endl;
        cerr << e.what() << endl;
        cerr << "Continuing with initial tendon force of 0 and a fiber length "
                "equal to the optimal fiber length.\n\n" << endl;
        setForce(s,0);
        setFiberLength(s,getOptimalFiberLength());
    }
}

//==============================================================================
// PROTECTED METHODS
//==============================================================================
void Millard2012EquilibriumMuscle::
postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    GeometryPath& path = upd_GeometryPath();
    path.postScale(s, aScaleSet);

    if (path.getPreScaleLength(s) > 0.0) {
        double scaleFactor = getLength(s) / path.getPreScaleLength(s);
        upd_optimal_fiber_length() *= scaleFactor;
        upd_tendon_slack_length() *= scaleFactor;
        path.setPreScaleLength(s, 0.0);
        ensureMuscleUpToDate();
    }
}

double Millard2012EquilibriumMuscle::clampActivation(double activation) const
{
    return clamp(getMinimumActivation(), activation, 1.0);
}

double Millard2012EquilibriumMuscle::
calcActivationDerivative(double activation, double excitation) const
{
    double da = 0.0;

    if(!get_ignore_activation_dynamics()) {
        // This model respects a lower bound on activation while preserving the
        // expected steady-state value.
        double clampedExcitation = clamp(getMinimumActivation(),excitation,1.0);
        double clampedActivation = clamp(getMinimumActivation(),activation,1.0);
        double tau = SimTK::NaN;

        if(clampedExcitation > clampedActivation) {
            tau = getActivationTimeConstant() * (0.5 + 1.5*clampedActivation);
        } else {
            tau = getDeactivationTimeConstant() / (0.5 + 1.5*clampedActivation);
        }
        da = (clampedExcitation - clampedActivation) / tau;
    }
    return da;
}

double Millard2012EquilibriumMuscle::
getStateVariableDeriv(const SimTK::State& s,
                      const std::string &aStateName) const
{
    return getCacheVariable<double>(s, aStateName + "_deriv");
}

void Millard2012EquilibriumMuscle::
setStateVariableDeriv(const SimTK::State& s,
                      const std::string &aStateName, double aValue) const
{
    double& cacheVariable = updCacheVariable<double>(s, aStateName + "_deriv");
    cacheVariable = aValue;
    markCacheVariableValid(s, aStateName + "_deriv");
}

//==============================================================================
// MUSCLE INFERFACE REQUIREMENTS -- MUSCLE LENGTH INFO
//==============================================================================
void Millard2012EquilibriumMuscle::calcMuscleLengthInfo(const SimTK::State& s,
    MuscleLengthInfo& mli) const
{
    // Get musculotendon actuator properties.
    double maxIsoForce    = getMaxIsometricForce();
    double optFiberLength = getOptimalFiberLength();
    double tendonSlackLen = getTendonSlackLength();

    try {
        // Get muscle-specific properties.
        const ActiveForceLengthCurve& falCurve = get_ActiveForceLengthCurve();
        const FiberForceLengthCurve&  fpeCurve = get_FiberForceLengthCurve();
        const TendonForceLengthCurve& fseCurve = get_TendonForceLengthCurve();

        if(get_ignore_tendon_compliance()) {                //rigid tendon
            mli.fiberLength = clampFiberLength(
                                penMdl.calcFiberLength(getLength(s),
                                tendonSlackLen));
        } else {                                            // elastic tendon
            mli.fiberLength = clampFiberLength(
                                getStateVariable(s, STATE_FIBER_LENGTH_NAME));
        }

        mli.normFiberLength   = mli.fiberLength / optFiberLength;
        mli.pennationAngle    = penMdl.calcPennationAngle(mli.fiberLength);
        mli.cosPennationAngle = cos(mli.pennationAngle);
        mli.sinPennationAngle = sin(mli.pennationAngle);
        mli.fiberLengthAlongTendon = mli.fiberLength * mli.cosPennationAngle;

        // Necessary even for the rigid tendon, as it might have gone slack.
        mli.tendonLength      = penMdl.calcTendonLength(mli.cosPennationAngle,
                                    mli.fiberLength, getLength(s));
        mli.normTendonLength  = mli.tendonLength / tendonSlackLen;
        mli.tendonStrain      = mli.normTendonLength - 1.0;

        mli.fiberPassiveForceLengthMultiplier =
            fpeCurve.calcValue(mli.normFiberLength);
        mli.fiberActiveForceLengthMultiplier =
            falCurve.calcValue(mli.normFiberLength);

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in Millard2012EquilibriumMuscle::"
                          "calcMuscleLengthInfo from " + getName() + "\n"
                          + x.what();
        throw OpenSim::Exception(msg);
    }
}


//==============================================================================
// MUSCLE INFERFACE REQUIREMENTS -- MUSCLE POTENTIAL ENERGY INFO
//==============================================================================
void Millard2012EquilibriumMuscle::
	calcMusclePotentialEnergyInfo(const SimTK::State& s,
		MusclePotentialEnergyInfo& mpei) const
{
    // Get musculotendon actuator properties.
    double maxIsoForce    = getMaxIsometricForce();
    double optFiberLength = getOptimalFiberLength();
    double tendonSlackLen = getTendonSlackLength();

    try {
		// Get the quantities that we've already computed.
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);

        // Get muscle-specific properties.
        const ActiveForceLengthCurve& falCurve = get_ActiveForceLengthCurve();
        const FiberForceLengthCurve&  fpeCurve = get_FiberForceLengthCurve();
        const TendonForceLengthCurve& fseCurve = get_TendonForceLengthCurve();


        // Note that the curves return normalized area.
        double fiberStrainAtFiso     = fpeCurve.getStrainAtOneNormForce()
                                       - fpeCurve.getStrainAtZeroForce();
        double fiberStretchAtFiso    = fiberStrainAtFiso * optFiberLength;
        double fiberPotentialScaling = (fiberStretchAtFiso*maxIsoForce)
                                       / fiberStrainAtFiso;

        mpei.fiberPotentialEnergy = fpeCurve.calcIntegral(mli.normFiberLength)
                                   * fiberPotentialScaling;

        mpei.tendonPotentialEnergy = 0;
        if(!get_ignore_tendon_compliance()) {               // elastic tendon
            double tendonStrainAtFiso  = fseCurve.getStrainAtOneNormForce();
            double tendonStretchAtFiso = tendonStrainAtFiso*tendonSlackLen;
            double tendonPotentialScaling = (tendonStretchAtFiso*maxIsoForce)
                                            / tendonStrainAtFiso;
            mpei.tendonPotentialEnergy =
                fseCurve.calcIntegral(mli.normTendonLength)
                * tendonPotentialScaling;
        }

        mpei.musclePotentialEnergy = mpei.fiberPotentialEnergy +
									 mpei.tendonPotentialEnergy;

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in Millard2012EquilibriumMuscle::"
                          "calcMusclePotentialEnergyInfo from " + getName() + "\n"
                          + x.what();
        throw OpenSim::Exception(msg);
    }
}


//==============================================================================
// MUSCLE INFERFACE REQUIREMENTS -- FIBER VELOCITY INFO
//==============================================================================
void Millard2012EquilibriumMuscle::
calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
    try {
        // Get the quantities that we've already computed.
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);

        // Get the static properties of this muscle.
        double dlenMcl   = getLengtheningSpeed(s);
        double optFibLen = getOptimalFiberLength();

        //======================================================================
        // Compute fv by inverting the force-velocity relationship in the
        // equilibrium equations.
        //======================================================================
        double dlce  = SimTK::NaN;
        double dlceN = SimTK::NaN;
        double fv    = SimTK::NaN;

        // Calculate fiber velocity.
        if(get_ignore_tendon_compliance()) {

            // Rigid tendon.

            if(mli.tendonLength < getTendonSlackLength()
                                  - SimTK::SignificantReal) {
                // The tendon is buckling, so fiber velocity is zero.
                dlce  = 0.0;
                dlceN = 0.0;
                fv    = 1.0;
            } else {
            dlce = penMdl.calcFiberVelocity(mli.cosPennationAngle,
                                                dlenMcl, 0.0);
            dlceN = dlce/(optFibLen*getMaxContractionVelocity());
            fv = get_ForceVelocityCurve().calcValue(dlceN);
            }

        } else if(!get_ignore_tendon_compliance() && !use_fiber_damping) {

            // Elastic tendon, no damping.

            double a = SimTK::NaN;
            if(!get_ignore_activation_dynamics()) {
                a = clampActivation(getStateVariable(s, STATE_ACTIVATION_NAME));
            } else {
                a = clampActivation(getControl(s));
            }

            const TendonForceLengthCurve& fseCurve =
                get_TendonForceLengthCurve();
            double fse = fseCurve.calcValue(mli.normTendonLength);

            SimTK_ERRCHK_ALWAYS(mli.cosPennationAngle > SimTK::SignificantReal,
                "calcFiberVelocityInfo",
                "%s: Pennation angle is 90 degrees, causing a singularity");
            SimTK_ERRCHK_ALWAYS(a > SimTK::SignificantReal,
                "calcFiberVelocityInfo",
                "%s: Activation is 0, causing a singularity");
            SimTK_ERRCHK_ALWAYS(mli.fiberActiveForceLengthMultiplier >
                                SimTK::SignificantReal,
                "calcFiberVelocityInfo",
                "%s: Active-force-length factor is 0, causing a singularity");

            fv = calcFv(a, mli.fiberActiveForceLengthMultiplier,
                        mli.fiberPassiveForceLengthMultiplier, fse,
                        mli.cosPennationAngle);

            // Evaluate the inverse force-velocity curve.
            dlceN = fvInvCurve.calcValue(fv);
            dlce  = dlceN*getMaxContractionVelocity()*optFibLen;

        } else {

            // Elastic tendon, with damping.

            double a = SimTK::NaN;
            if(!get_ignore_activation_dynamics()) {
                a = clampActivation(getStateVariable(s, STATE_ACTIVATION_NAME));
            } else {
                a = clampActivation(getControl(s));
            }

            const TendonForceLengthCurve& fseCurve =
                get_TendonForceLengthCurve();
            double fse = fseCurve.calcValue(mli.normTendonLength);

            // Newton solve for fiber velocity.
            fv = 1.0;
            dlce = -1;
            dlceN = -1;
            double beta = get_fiber_damping();

            SimTK_ERRCHK_ALWAYS(beta > SimTK::SignificantReal,
                "calcFiberVelocityInfo",
                "Fiber damping coefficient must be greater than 0.");

            SimTK::Vec3 fiberVelocityV = calcDampedNormFiberVelocity(
                getMaxIsometricForce(), a, mli.fiberActiveForceLengthMultiplier,
                mli.fiberPassiveForceLengthMultiplier, fse, beta,
                mli.cosPennationAngle);

            // If the Newton method converged, update the fiber velocity.
            if(fiberVelocityV[2] > 0.5) { //flag is set to 0.0 or 1.0
                dlceN = fiberVelocityV[0];
                dlce  = dlceN*getOptimalFiberLength()
                        *getMaxContractionVelocity();
                fv = get_ForceVelocityCurve().calcValue(dlceN);
            } else {
                // Throw an exception here because there is no point integrating
                // a muscle velocity that is invalid (it will end up producing
                // invalid fiber lengths and will ultimately cause numerical
                // problems). The idea is to produce an exception and catch this
                // early before it can cause more damage.
                throw (OpenSim::Exception(getName() +
                       " Fiber velocity Newton method did not converge"));
            }
        }

        // Compute the other velocity-related components.
        double dphidt = penMdl.calcPennationAngularVelocity(
            tan(mli.pennationAngle), mli.fiberLength, dlce);
        double dlceAT = penMdl.calcFiberVelocityAlongTendon(mli.fiberLength,
            dlce, mli.sinPennationAngle, mli.cosPennationAngle, dphidt);
        double dmcldt = getLengtheningSpeed(s);
        double dtl = 0;

        if(!get_ignore_tendon_compliance()) {
            dtl = penMdl.calcTendonVelocity(mli.cosPennationAngle,
                mli.sinPennationAngle, dphidt, mli.fiberLength, dlce, dmcldt);
        }

        // Check to see whether the fiber state is clamped.
        double fiberStateClamped = 0.0;
        if(isFiberStateClamped(mli.fiberLength,dlce)) {
            dlce = 0.0;
            dlceN = 0.0;
            dlceAT = 0.0;
            dphidt = 0.0;
            dtl = dmcldt;
            fv = 1.0; //to be consistent with a fiber velocity of 0
            fiberStateClamped = 1.0;
        }

        // Populate the struct.
        fvi.fiberVelocity                = dlce;
        fvi.normFiberVelocity            = dlceN;
        fvi.fiberVelocityAlongTendon     = dlceAT;
        fvi.pennationAngularVelocity     = dphidt;
        fvi.tendonVelocity               = dtl;
        fvi.normTendonVelocity           = dtl/getTendonSlackLength();
        fvi.fiberForceVelocityMultiplier = fv;

        fvi.userDefinedVelocityExtras.resize(1);
        fvi.userDefinedVelocityExtras[0] = fiberStateClamped;

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in Millard2012EquilibriumMuscle::"
                          "calcFiberVelocityInfo from " + getName() + "\n"
                           + x.what();
        throw OpenSim::Exception(msg);
    }
}

//==============================================================================
// MUSCLE INFERFACE REQUIREMENTS -- MUSCLE DYNAMICS INFO
//==============================================================================
void Millard2012EquilibriumMuscle::
calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
    try {
        // Get the quantities that we've already computed.
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
        const FiberVelocityInfo &mvi = getFiberVelocityInfo(s);
        double fiberStateClamped = mvi.userDefinedVelocityExtras[0];

        // Get the properties of this muscle.
        double tendonSlackLen = getTendonSlackLength();
        double optFiberLen    = getOptimalFiberLength();
        double fiso           = getMaxIsometricForce();
        double penHeight      = penMdl.getParallelogramHeight();
        const TendonForceLengthCurve& fseCurve = get_TendonForceLengthCurve();

        // Compute dynamic quantities.
        double a = SimTK::NaN;
        if(!get_ignore_activation_dynamics()) {
            a = clampActivation(getStateVariable(s, STATE_ACTIVATION_NAME));
        } else {
            a = clampActivation(getControl(s));
        }

        // Compute the stiffness of the muscle fiber.
        SimTK_ERRCHK_ALWAYS(mli.fiberLength > SimTK::SignificantReal,
            "calcMuscleDynamicsInfo",
            "The muscle fiber has a length of 0, causing a singularity");
        SimTK_ERRCHK_ALWAYS(mli.cosPennationAngle > SimTK::SignificantReal,
            "calcMuscleDynamicsInfo",
            "Pennation angle is 90 degrees, causing a singularity");

        double fm           = 0.0; //total fiber force
        double aFm          = 0.0; //active fiber force
        double p1Fm         = 0.0; //passive conservative fiber force
        double p2Fm         = 0.0; //passive non-conservative fiber force
        double pFm          = 0.0; //total passive fiber force
        double fmAT         = 0.0;
        double dFm_dlce     = 0.0;
        double dFmAT_dlceAT = 0.0;
        double dFt_dtl      = 0.0;
        double Ke           = 0.0;

        if(fiberStateClamped < 0.5) { //flag is set to 0.0 or 1.0
            SimTK::Vec4 fiberForceV;

            fiberForceV = calcFiberForce(fiso, a,
                                         mli.fiberActiveForceLengthMultiplier,
                                         mvi.fiberForceVelocityMultiplier,
                                         mli.fiberPassiveForceLengthMultiplier,
                                         mvi.normFiberVelocity);
            fm   = fiberForceV[0];
            aFm  = fiberForceV[1];
            p1Fm = fiberForceV[2];
            p2Fm = fiberForceV[3];
            pFm  = p1Fm + p2Fm;

            // Every configuration except the rigid tendon chooses a fiber
            // velocity that ensures that the fiber does not generate a
            // compressive force. Here, we must enforce that the fiber generates
            // only tensile forces by saturating the damping force generated by
            // the parallel element.
            if(get_ignore_tendon_compliance()) {
                if(fm < 0) {
                    fm   = 0.0;
                    p2Fm = -aFm - p1Fm;
                    pFm  = p1Fm + p2Fm;
                }
            }

            fmAT = fm * mli.cosPennationAngle;
            dFm_dlce = calcFiberStiffness(fiso, a,
                                          mvi.fiberForceVelocityMultiplier,
                                          mli.normFiberLength, optFiberLen);
            dFmAT_dlceAT = calc_DFiberForceAT_DFiberLengthAT(dFm_dlce,
                mli.sinPennationAngle, mli.cosPennationAngle, mli.fiberLength);

            // Compute the stiffness of the tendon.
            if(!get_ignore_tendon_compliance()) {
                dFt_dtl = fseCurve.calcDerivative(mli.normTendonLength,1)
                          *(fiso/tendonSlackLen);

                // Compute the stiffness of the whole musculotendon actuator.
                if (abs(dFmAT_dlceAT*dFt_dtl) > 0.0
                    && abs(dFmAT_dlceAT+dFt_dtl) > SimTK::SignificantReal) {
                    Ke = (dFmAT_dlceAT*dFt_dtl)/(dFmAT_dlceAT+dFt_dtl);
                }
            } else {
                dFt_dtl = SimTK::Infinity;
                Ke = dFmAT_dlceAT;
            }
        }

        double fse = 0.0;
        if(!get_ignore_tendon_compliance()) {
            fse = fseCurve.calcValue(mli.normTendonLength);
        } else {
            fse = fmAT/fiso;
        }

        mdi.activation                = a;
        mdi.fiberForce                = fm;
        mdi.fiberForceAlongTendon     = fmAT;
        mdi.normFiberForce            = fm/fiso;
        mdi.activeFiberForce          = aFm;
        mdi.passiveFiberForce         = pFm;
        mdi.tendonForce               = fse*fiso;
        mdi.normTendonForce           = fse;
        mdi.fiberStiffness            = dFm_dlce;
        mdi.fiberStiffnessAlongTendon = dFmAT_dlceAT;
        mdi.tendonStiffness           = dFt_dtl;
        mdi.muscleStiffness           = Ke;

        // Verify that the derivative of system energy minus work is zero within
        // a reasonable numerical tolerance.
        double dphidt       = mvi.pennationAngularVelocity;
        double dFibPEdt     = p1Fm*mvi.fiberVelocity; //only conservative part
                                                      //of passive fiber force
        double dTdnPEdt     = fse*fiso*mvi.tendonVelocity;
        double dFibWdt      = -(mdi.activeFiberForce+p2Fm)*mvi.fiberVelocity;
        double dmcldt       = getLengtheningSpeed(s);
        double dBoundaryWdt = mdi.tendonForce*dmcldt;

        double dSysEdt = (dFibPEdt + dTdnPEdt) - dFibWdt - dBoundaryWdt;
        double tol = sqrt(SimTK::Eps);

        // Populate the power entries.
        mdi.fiberActivePower  = dFibWdt;
        mdi.fiberPassivePower = -(dFibPEdt);
        mdi.tendonPower       = -dTdnPEdt;
        mdi.musclePower       = -dBoundaryWdt;

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in Millard2012EquilibriumMuscle::"
                          "calcMuscleDynamicsInfo from " + getName() + "\n"
                          + x.what();
		cerr << msg << endl;
        throw OpenSim::Exception(msg);
    }
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
void Millard2012EquilibriumMuscle::connectToModel(Model& model)
{
    Super::connectToModel(model);
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "Millard2012EquilibriumMuscle: Muscle properties are not up-to-date");

    double dummyValue = 0.0;
    if(!get_ignore_activation_dynamics()) {
        addStateVariable(STATE_ACTIVATION_NAME);
        addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", dummyValue,
                         SimTK::Stage::Dynamics);
    }
    if(!get_ignore_tendon_compliance()) {
        addStateVariable(STATE_FIBER_LENGTH_NAME);
        addCacheVariable(STATE_FIBER_LENGTH_NAME+"_deriv", dummyValue,
                         SimTK::Stage::Dynamics);
    }
}

void Millard2012EquilibriumMuscle::
initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

    if(!get_ignore_activation_dynamics()) {
        setActivation(s, getDefaultActivation());
    }
    if(!get_ignore_tendon_compliance()) {
        setFiberLength(s, getDefaultFiberLength());
    }
}

void Millard2012EquilibriumMuscle::
setPropertiesFromState(const SimTK::State& s)
{
    Super::setPropertiesFromState(s);

    if(!get_ignore_activation_dynamics()) {
        setDefaultActivation(getStateVariable(s,STATE_ACTIVATION_NAME));
    }
    if(!get_ignore_tendon_compliance()) {
        setDefaultFiberLength(getStateVariable(s,STATE_FIBER_LENGTH_NAME));
    }
    ensureMuscleUpToDate();
}

SimTK::Vector Millard2012EquilibriumMuscle::
computeStateVariableDerivatives(const SimTK::State& s) const
{
    SimTK::Vector derivs(getNumStateVariables(), 0.0);
    int idx = 0;

    if (!isDisabled(s)) {
        // Activation is the first state (if it is a state at all)
        if(!get_ignore_activation_dynamics() &&
           idx+1 <= getNumStateVariables()) {
               derivs[idx] = getActivationDerivative(s);
               idx++;
        }

        // Fiber length is the next state (if it is a state at all)
        if(!get_ignore_tendon_compliance() && idx+1 <= getNumStateVariables()) {
            derivs[idx] = getFiberVelocity(s);
        }
    }
    return derivs;
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================
SimTK::Vec3 Millard2012EquilibriumMuscle::
calcDampedNormFiberVelocity(double fiso,
                            double a,
                            double fal,
                            double fpe,
                            double fse,
                            double beta,
                            double cosPhi) const
{
    SimTK::Vec4 fiberForceV;
    SimTK::Vec3 result;

    int maxIter = 20; //this routine converges quickly; 20 is quite generous
    double tol = 1.0e-10*fiso;
    if(tol < SimTK::SignificantReal*100) {
        tol = SimTK::SignificantReal*100;
    }
    double perturbation   = 0.0;
    double fiberForce     = 0.0;
    double err            = 1.0e10;
    double derr_d_dlceNdt = 0.0;
    double delta          = 0.0;
    double iter           = 0.0;

    // Get a really excellent starting position to reduce the number of
    // iterations. This reduces the simulation time by about 1%.
    double fv = calcFv(max(a,0.01), max(fal,0.01), fpe, fse, max(cosPhi,0.01));
    double dlceN_dt = fvInvCurve.calcValue(fv);

    // The approximation is poor beyond the maximum velocities.
    if(dlceN_dt > 1.0) {
        dlceN_dt = 1.0;
    }
    if(dlceN_dt < -1.0) {
        dlceN_dt = -1.0;
    }

    double df_d_dlceNdt = 0.0;

    while(abs(err) > tol && iter < maxIter) {
        fv = get_ForceVelocityCurve().calcValue(dlceN_dt);
        fiberForceV = calcFiberForce(fiso,a,fal,fv,fpe,dlceN_dt);
        fiberForce = fiberForceV[0];

        err = fiberForce*cosPhi - fse*fiso;
        df_d_dlceNdt = calc_DFiberForce_DNormFiberVelocity(fiso,a,fal,
                                                           beta,dlceN_dt);
        derr_d_dlceNdt = df_d_dlceNdt*cosPhi;

        if(abs(err) > tol && abs(derr_d_dlceNdt) > SimTK::SignificantReal) {
            delta = -err/derr_d_dlceNdt;
            dlceN_dt = dlceN_dt + delta;

        } else if(abs(derr_d_dlceNdt) < SimTK::SignificantReal) {
            // Perturb the solution if we've lost rank. This should never happen
            // for this problem since dfv_d_dlceNdt > 0 and b > 0 (and so
            // derr_d_dlceNdt > 0).
            perturbation = 2.0*((double)rand())/((double)RAND_MAX)-1.0;
            dlceN_dt = dlceN_dt + perturbation*0.05;
        }
        iter++;
    }

    double converged = 1.0;

    // If we failed to converge, it's probably because the fiber is at its lower
    // bound. That decision is made further down the line, so if convergence
    // didn't happen, let the user know and return a NaN.
    if(abs(err) > tol) {
        dlceN_dt  = -1.0;
        converged = 0.0;
    }

    result[0] = dlceN_dt;
    result[1] = err;
    result[2] = converged;
    return result;
}

double Millard2012EquilibriumMuscle::calcFv(double a,
                                            double fal,
                                            double fp,
                                            double fse,
                                            double cosphi) const
{   return ( fse/cosphi - fp ) / (a*fal); }

SimTK::Vec4 Millard2012EquilibriumMuscle::
calcFiberForce(double fiso,
               double a,
               double fal,
               double fv,
               double fpe,
               double dlceN) const
{
    double beta = getFiberDamping();
    double fa   = fiso * (a*fal*fv);
    double fp1  = fiso * fpe;
    double fp2  = fiso * beta*dlceN;
    double fm   = fa + (fp1+fp2);

    SimTK::Vec4 fiberF;
    fiberF[0] = fm;
    fiberF[1] = fa;
    fiberF[2] = fp1; //conservative passive force
    fiberF[3] = fp2; //non-conservative passive force
    return fiberF;
}

double Millard2012EquilibriumMuscle::calcActivation(double fiso,
                                                    double ftendon,
                                                    double cosPhi,
                                                    double fal,
                                                    double fv,
                                                    double fpe,
                                                    double dlceN) const
{
    double beta = getFiberDamping();
    double activation = 0.0;

    // If the fiber cannot generate any force due to its pennation angle,
    // active-force-length or force-velocity values, leave activation as 0.
    if(cosPhi > SimTK::SignificantReal && fal*fv > SimTK::SignificantReal) {
        activation = ( (ftendon /(fiso*cosPhi)) - fpe - beta*dlceN ) / (fal*fv);
    }
    return activation;
}

double Millard2012EquilibriumMuscle::calcFiberStiffness(double fiso,
                                                        double a,
                                                        double fv,
                                                        double lceN,
                                                        double optFibLen) const
{
    const FiberForceLengthCurve& fpeCurve  = get_FiberForceLengthCurve();
    const ActiveForceLengthCurve& falCurve = get_ActiveForceLengthCurve();
    double DlceN_Dlce = 1.0/optFibLen;
    double Dfal_Dlce  = falCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfpe_Dlce  = fpeCurve.calcDerivative(lceN,1) * DlceN_Dlce;

    // DFm_Dlce
    return  fiso * (a*Dfal_Dlce*fv + Dfpe_Dlce);
}

double Millard2012EquilibriumMuscle::
calc_DFiberForce_DNormFiberVelocity(double fiso,
                                    double a,
                                    double fal,
                                    double beta,
                                    double dlceN_dt) const
{
    // dfm_d_dlceNdt
    return fiso * (a*fal*get_ForceVelocityCurve().calcDerivative(dlceN_dt,1)
                   + beta);
}

double Millard2012EquilibriumMuscle::
calc_DFiberForceAT_DFiberLength(double fiberForce,
                                double fiberStiffness,
                                double lce,
                                double sinPhi,
                                double cosPhi) const
{
    double Dphi_Dlce    = penMdl.calc_DPennationAngle_DfiberLength(lce);
    double Dcosphi_Dlce = -sinPhi*Dphi_Dlce;

    // The stiffness of the fiber along the direction of the tendon. For small
    // changes in length parallel to the fiber, this quantity is
    // D(FiberForceAlongTendon) / D(fiberLength)
    // dFmAT/dlce = d/dlce( fiso * (a *fal*fv + fpe + beta*dlceN)*cosPhi )
    return fiberStiffness*cosPhi + fiberForce*Dcosphi_Dlce;
}

double Millard2012EquilibriumMuscle::
calc_DFiberForceAT_DFiberLengthAT(double dFmAT_d_lce,
                                  double sinPhi,
                                  double cosPhi,
                                  double lce) const
{
    double dphi_d_lce = penMdl.calc_DPennationAngle_DfiberLength(lce);

    // The change in length of the fiber length along the tendon.
    // lceAT = lce*cos(phi)
    double DlceAT_Dlce = cosPhi - lce*sinPhi*dphi_d_lce;

    // dFmAT/dlceAT = (dFmAT/dlce)*(1/(dlceAT/dlce))
    //              = dFmAT/dlceAT
    return dFmAT_d_lce * (1.0/DlceAT_Dlce);
}

double Millard2012EquilibriumMuscle::
calc_DTendonForce_DFiberLength(double dFt_d_tl,
                               double lce,
                               double sinphi,
                               double cosphi) const
{
    double dphi_d_lce = penMdl.calc_DPennationAngle_DfiberLength(lce);
    double dtl_d_lce  = penMdl.calc_DTendonLength_DfiberLength(lce,sinphi,
                                                            cosphi,dphi_d_lce);
    // dFt_d_lce
    return dFt_d_tl*dtl_d_lce;
}

//==============================================================================
// PRIVATE UTILITY CLASS MEMBERS
//==============================================================================
bool Millard2012EquilibriumMuscle::
isFiberStateClamped(double lce, double dlceN) const
{
    bool clamped = false;

    // Get the minimum active fiber length (in meters).
    double minFiberLength = getMinimumFiberLength();

    // If the fiber is clamped and shortening, then the fiber is either shorter
    // than the pennation model allows or shorter than the active-force-length
    // curve allows.
    if( (lce <= minFiberLength && dlceN <= 0) || lce < minFiberLength) {
        clamped = true;
    }
    return clamped;
}

double Millard2012EquilibriumMuscle::clampFiberLength(double lce) const
{   return max(lce, getMinimumFiberLength()); }

SimTK::Vector Millard2012EquilibriumMuscle::
estimateMuscleFiberState(double aActivation,
                         double pathLength,
                         double pathLengtheningSpeed,
                         double aSolTolerance,
                         int aMaxIterations,
                         bool staticSolution) const
{
    // Results vector format:
    //   [0] flag: 0 = converged
    //             1 = diverged
    //             2 = no solution due to length singularity
    //             3 = no solution due to pennation angle singularity
    //   [1] solution error (N)
    //   [2] iterations
    //   [3] fiber length (m)
    //   [4] fiber velocity (N)
    //   [5] tendon force (N)
    SimTK::Vector results = SimTK::Vector(6);

    // Using short variable names to facilitate writing out long equations
    double ma        = aActivation;
    double ml        = pathLength;
    double dml       = pathLengtheningSpeed;
    double tsl       = getTendonSlackLength();
    double ofl       = getOptimalFiberLength();
    double ophi      = getPennationAngleAtOptimalFiberLength();
    double penHeight = penMdl.getParallelogramHeight();
    double fiso      = getMaxIsometricForce();
    double vmax      = getMaxContractionVelocity();

    const TendonForceLengthCurve& fseCurve = get_TendonForceLengthCurve();
    const FiberForceLengthCurve& fpeCurve  = get_FiberForceLengthCurve();
    const ActiveForceLengthCurve& falCurve = get_ActiveForceLengthCurve();
    double fse = 0.0;  // normalized tendon (series element) force
    double fal = 0.0;  // normalized active-force-length multiplier
    double fv  = 0.0;  // normalized force-velocity multiplier
    double fpe = 0.0;  // normalized parallel element force

    // Position level
    double lce = 0.0;
    double tl  = getTendonSlackLength()*1.01;  // begin with small tendon force

    lce = clampFiberLength(penMdl.calcFiberLength(ml,tl));

    double phi    = penMdl.calcPennationAngle(lce);
    double cosphi = cos(phi);
    double sinphi = sin(phi);
    double tlN    = tl/tsl;
    double lceN   = lce/ofl;

    // Velocity level
    double dtl    = 0.0;
    double dlce   = (staticSolution) ? 0.0 :
                        penMdl.calcFiberVelocity(cosphi,dml,dtl);
    double dlceN  = (staticSolution) ? 0.0 :
                        dlce/(vmax*ofl);
    double dphi   = (staticSolution) ? 0.0 :
                        penMdl.calcPennationAngularVelocity(tan(phi),lce,dlce);
    double dlceAT = (staticSolution) ? 0.0 :
                        penMdl.calcFiberVelocityAlongTendon(lce,dlce,sinphi,
                                                            cosphi,dphi);

    // Internal variables for the loop
    double dphi_d_lce   = 0.0;
    double dtl_d_lce    = 0.0;
    double Fm           = 0.0;   // fiber force
    double FmAT         = 0.0;   // fiber force along tendon
    double Ft           = 0.0;   // tendon force
    double ferr         = 10.0;  // solution error
    double dFm_dlce     = 0.0;   // partial of muscle force w.r.t. lce
    double dFmAT_dlce   = 0.0;   // partial of muscle force along tl w.r.t. lce
    double dFmAT_dlceAT = 0.0;   // partial of muscle force along tl w.r.t. lce
                                 //     along the tendon
    double dFt_d_lce    = 0.0;   // partial of tendon force w.r.t. lce
    double dFt_d_tl     = 0.0;   // partial of tendon force w.r.t. tl
    double dferr_d_lce  = 0.0;   // partial of solution error w.r.t lce
    double delta_lce    = 0.0;   // change in lce
    double Ke           = 0.0;   // linearized local stiffness of the muscle

    // Initialize the loop
    int iter = 0;
    int minFiberLengthCtr = 0;
    SimTK::Vec4 fiberForceV;

    while(abs(ferr) > aSolTolerance
            && iter < aMaxIterations
            && minFiberLengthCtr < 10) {

        // Update the multipliers and their partial derivatives
        fal = falCurve.calcValue(lceN);
        fpe = fpeCurve.calcValue(lceN);
        fse = fseCurve.calcValue(tlN);
        fv  = (staticSolution) ? 1.0
                               : get_ForceVelocityCurve().calcValue(dlceN);

        // Compute the force error
        fiberForceV = calcFiberForce(fiso,ma,fal,fv,fpe,dlceN);
        Fm   = fiberForceV[0];
        FmAT = Fm * cosphi;
        Ft   = fse*fiso;
        ferr = FmAT - Ft;

        // Compute the partial derivative of the force error w.r.t. lce
        dFm_dlce     = calcFiberStiffness(fiso,ma,fv,lceN,ofl);
        dFmAT_dlce   = calc_DFiberForceAT_DFiberLength(Fm,dFm_dlce,lce,
                                                       sinphi,cosphi);
        dFmAT_dlceAT = calc_DFiberForceAT_DFiberLengthAT(dFmAT_dlce,sinphi,
                                                         cosphi,lce);
        dFt_d_tl     = fseCurve.calcDerivative(tlN,1)*fiso/tsl;
        dFt_d_lce    = calc_DTendonForce_DFiberLength(dFt_d_tl,lce,
                                                      sinphi,cosphi);

        // Error derivative
        dferr_d_lce = dFmAT_dlce - dFt_d_lce;

        if(abs(ferr) > aSolTolerance) {
            if(abs(dferr_d_lce) > SimTK::SignificantReal) {
                // Take a full Newton Step if the derivative is nonzero
                delta_lce = -ferr/dferr_d_lce;
                lce       = lce + delta_lce;
            } else {
                // We've stagnated; perturb the current solution
                double perturbation =
                    2.0*((double)rand())/((double)RAND_MAX)-1.0;
                double lengthPerturbation =
                    0.5*perturbation*getOptimalFiberLength();
                lce += lengthPerturbation;
            }

            if(isFiberStateClamped(lce,dlceN)) {
                minFiberLengthCtr++;
                lce = getMinimumFiberLength();
            }

            // Update position level quantities only if they won't go singular
            phi    = penMdl.calcPennationAngle(lce);
            sinphi = sin(phi);
            cosphi = cos(phi);
            tl     = penMdl.calcTendonLength(cosphi,lce,ml);
            lceN   = lce/ofl;
            tlN    = tl/tsl;

            /* Update velocity-level quantities. Share the muscle velocity
            between the tendon and the fiber according to their relative
            stiffnesses:

            Fm-Ft = 0                     Equilibrium equation   [1]
            d/dt Fm - d/dt Ft = 0         Time derivative        [2]
            lp = lm + lt                  Path definition        [3]
            d/dt lp = d/dt lm + d/dt lt   Path derivative        [4]

            Computing a linearized model of [2]:
            Fm = Fm0 + Km*lceAT                                  [5]
            Ft = Ft0 Kt*xt                                       [6]

            Taking its time derivative:
            dFm_d_xm = Km*dlceAT + dKm_d_t*lceAT (assume dKm_d_t = 0)   [7]
            dFt_d_xt = Kt*dtl + dKt_d_t*dtl (assume dKt_d_t = 0)        [8]

            Subtituting 7 and 8 into 2:
            Km dlceAT - Kt dtl = 0

            Using Eqn 4, we have 2 equations in 2 unknowns. Can now solve for
            tendon velocity, or the velocity of the fiber along the tendon.

            This is a heuristic. The above assumptions are necessary since
            computing the partial derivatives of Km or Kt requires acceleration-
            level knowledge, which is not available in general.

            Stiffness of the muscle is the stiffness of the tendon and the fiber
            (along the tendon) in series.

            The "if" statement here is to handle the special case where the
            negative stiffness of the fiber (which happens in this model) is
            equal to the positive stiffness of the tendon. */
            if(!staticSolution) {
                if( abs(dFmAT_dlceAT + dFt_d_tl) > SimTK::SignificantReal
                    && tlN > 1.0) {
                        Ke  = (dFmAT_dlceAT*dFt_d_tl)/(dFmAT_dlceAT + dFt_d_tl);
                        dtl = (1/dFt_d_tl)*Ke*dml;
                } else {
                    dtl = dml;
                }

                dlce   = penMdl.calcFiberVelocity(cosphi,dml,dtl);
                dlceN  = dlce/(vmax*ofl);
                dphi   = penMdl.calcPennationAngularVelocity(tan(phi),lce,dlce);
                dlceAT = penMdl.calcFiberVelocityAlongTendon(lce,dlce,sinphi,
                                                             cosphi,dphi);
            }
        }
        iter++;
    }

    // Populate the results vector:
    //   [0] flag: 0 = converged
    //             1 = diverged
    //             2 = no solution due to length singularity
    //             3 = no solution due to pennation angle singularity
    //   [1] solution error (N)
    //   [2] iterations
    //   [3] fiber length (m)
    //   [4] fiber velocity (N)
    //   [5] tendon force (N)

    if(abs(ferr) < aSolTolerance) {
        // The solution converged
        results[0] = 0;
        results[1] = ferr;
        results[2] = (double)iter;
        results[3] = lce;
        results[4] = dlce;
        results[5] = fse*fiso;

    } else {
        // The fiber length hit its lower bound
        if(iter < aMaxIterations) {
            lce    = getMinimumFiberLength();
            phi    = penMdl.calcPennationAngle(lce);
            cosphi = cos(phi);
            tl     = penMdl.calcTendonLength(cosphi,lce,ml);
            lceN   = lce/ofl;
            tlN    = tl/tsl;
            fse    = fseCurve.calcValue(tlN);

            results[0] = 1.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = 0;
            results[5] = fse*fiso;

        } else {
            // The solution diverged
            results[0] = 2.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = SimTK::NaN;
            results[4] = SimTK::NaN;
            results[5] = SimTK::NaN;
        }
    }
    return results;
}

//==============================================================================
// XXXXXXXXXXXXXXXXXXXXXXXX  START OF TO BE DEPRECATED  XXXXXXXXXXXXXXXXXXXXXXXX
//==============================================================================
double Millard2012EquilibriumMuscle::
calcActiveFiberForceAlongTendon(double activation,
                                double fiberLength,
                                double fiberVelocity) const
{
    double activeFiberForce = SimTK::NaN;

    if(fiberLength <= getMinimumFiberLength()) {
        return 0.0;
    }

    try {
        //Clamp activation to a legal range
        double ca = clampActivation(activation);

        //Normalize fiber length and velocity
        double lceN  = fiberLength/getOptimalFiberLength();
        double dlceN = fiberVelocity /
                        (getOptimalFiberLength()*getMaxContractionVelocity());

        //Get the necessary curves
        const ActiveForceLengthCurve& falCurve =
            get_ActiveForceLengthCurve();
        const FiberForceLengthCurve& fpeCurve = get_FiberForceLengthCurve();

        //Evaluate the active-force-length and force-velocity multipliers
        double fal  = falCurve.calcValue(lceN);
        double fv   = get_ForceVelocityCurve().calcValue(dlceN);
        double fiso = getMaxIsometricForce();
        double fpe  = fpeCurve.calcValue(lceN);

        //Evaluate the pennation angle
        double phi = penMdl.calcPennationAngle(lceN);

        //Compute the active fiber force
        Vec4 fiberForceV = calcFiberForce(fiso,ca,fal,fv,fpe,dlceN);
        double fa = fiberForceV[1];
        activeFiberForce = fa * cos(phi);

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in: " + getName()
                            + ":calcActiveFiberForceAlongTendon\n" + x.what();
        throw OpenSim::Exception(msg);
    }
  
    return activeFiberForce;
}

SimTK::Vec4 Millard2012EquilibriumMuscle::
calcFiberStateGivenBoundaryCond(double lengthMT,
                                double velocityMT,
                                double tendonForce,
                                double dTendonForceDT) const
{
    SimTK::Vec4 output;

    output[0] = 0;
    output[1] = 0;
    output[2] = 0;
    output[3] = 0;

    double a, lt, vt, lm, vm, phi, dphidt = 0;
    double ltN, lmN, vmN = 0;

    try {
        //1. Compute tendon length given F
        if(!get_ignore_tendon_compliance() && tendonForce > 0) {
            //Use Newton's method to solve for the strain of the tendon
            const TendonForceLengthCurve& fseCurve =
                get_TendonForceLengthCurve();

            //Good initial guess at the tendon length
            lt = getTendonSlackLength()*
                 (1 + fseCurve.getStrainAtOneNormForce()*
                  tendonForce/getMaxIsometricForce());

            double tol = 1e-8*getMaxIsometricForce();
            if(tol < SimTK::SignificantReal*100) {
                tol = SimTK::SignificantReal*100;
            }

            int maxIter = 100;
            int iter = 0;
            double err = 1e10;
            double derr_dtlN = 0;

            ltN = lt/getTendonSlackLength();
            double delta_ltN = 0;

            while(abs(err) > tol && iter < maxIter) {
                //Compute error
                err = fseCurve.calcValue(ltN)*getMaxIsometricForce()
                      - tendonForce;
                derr_dtlN = fseCurve.calcDerivative(ltN,1) *
                            getMaxIsometricForce();

                //Check tolerance; take a Newton step if possible
                if(abs(err) > tol && abs(derr_dtlN) > SimTK::SignificantReal) {
                   delta_ltN = -err / derr_dtlN;
                   if(abs(delta_ltN) > 0.5*fseCurve.getStrainAtOneNormForce()) {
                        delta_ltN = 0.5*fseCurve.getStrainAtOneNormForce();
                   }
                   ltN = ltN + delta_ltN;
                }
                iter++;
            }

            if(abs(err) <= tol) {
                lt = ltN*getTendonSlackLength();
            } else {
                lt = SimTK::NaN;
            }

        } else {
            if(get_ignore_tendon_compliance()) {        // rigid tendon
                lt  = getTendonSlackLength();
                ltN = 1.0;
            } else if(tendonForce <= 0) {               // slack elastic tendon
                lt  = lengthMT - penMdl.getMinimumFiberLengthAlongTendon();
                ltN = lt/getTendonSlackLength();
            }
        }

        //If we have a valid tendon length, proceed
        if(!SimTK::isNaN(lt)) {

            //2. Compute tendon stretch velocity given dF/dt
            //   Due to the equilibrium assumption
            //     tendonForce = k*(lt - lt0)
            //     dtendonForce/dt = k*(dlt_dt)
            if(!get_ignore_tendon_compliance() && tendonForce > 0) {
                const TendonForceLengthCurve& fseCurve =
                    get_TendonForceLengthCurve();
                double ktN = fseCurve.calcDerivative(ltN,1);
                double kt = ktN*(getMaxIsometricForce()
                            /getTendonSlackLength());
                vt = dTendonForceDT / kt;
            } else {
                if(get_ignore_tendon_compliance()) {  // rigid tendon
                    vt = 0;
                }else if(tendonForce <= 0) {          //buckling elastic tendon
                    vt = velocityMT;
                }
            }

            //3. Compute fiber length, pennation angle
            lm  = penMdl.calcFiberLength(lengthMT,lt);
            lmN = lm/getOptimalFiberLength();
            phi = penMdl.calcPennationAngle(lm);

            //4. Compute fiber velocity, pennation angular velocity
            vm  = penMdl.calcFiberVelocity( cos(phi), velocityMT,vt);
            vmN = vm / (getOptimalFiberLength()*getMaxContractionVelocity());

            //5. Compute activation
            const ActiveForceLengthCurve& falCurve =
                get_ActiveForceLengthCurve();
            //fvCurve is a private member object
            const FiberForceLengthCurve& fpeCurve = get_FiberForceLengthCurve();

            double fal = falCurve.calcValue(lmN);
            double fpe = fpeCurve.calcValue(lmN);
            double fv  = get_ForceVelocityCurve().calcValue(vmN);
            a = calcActivation(getMaxIsometricForce(),tendonForce,
                               cos(phi),fal,fv,fpe,vmN);

            //6. Populate output vector
            output[0] = a;
            output[1] = lmN;
            output[2] = phi;
            output[3] = vmN;
        }

    } catch(const std::exception &x) {
        std::string msg = "Exception caught in: " + getName()
                          + ":calcFiberStateGivenBoundaryCond\n" + x.what();
        throw OpenSim::Exception(msg);
    }
    return output;
}

double Millard2012EquilibriumMuscle::
calcInextensibleTendonActiveFiberForce(SimTK::State& s,
                                       double aActivation) const
{
    double inextensibleTendonActiveFiberForce = 0.0;
    try {
        double lm    = getLength(s);
        double dlm   = getLengtheningSpeed(s);
        double ltslk = getTendonSlackLength();
        double dlt   = 0.0; //inextensible tendon
        double lce   = penMdl.calcFiberLength(lm,ltslk);
        double phi   = penMdl.calcPennationAngle(lce);
        double dlce  = penMdl.calcFiberVelocity(cos(phi),dlm,dlt);

        if(!SimTK::isNaN(dlce)) {
            inextensibleTendonActiveFiberForce =
                calcActiveFiberForceAlongTendon(aActivation,lce,dlce);
        }
    } catch(const std::exception &x) {
        std::string msg = "Exception caught in: " + getName()
                          + ":calcInextensibleTendonActiveFiberForce\n"
                          + x.what();
        throw OpenSim::Exception(msg);
    }
    return inextensibleTendonActiveFiberForce;
}

//==============================================================================
// XXXXXXXXXXXXXXXXXXXXXXXXX  END OF TO BE DEPRECATED  XXXXXXXXXXXXXXXXXXXXXXXXX
//==============================================================================
