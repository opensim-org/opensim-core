/* -------------------------------------------------------------------------- *
 *                OpenSim:  Millard2012AccelerationMuscle.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include "Millard2012AccelerationMuscle.h"
#include <OpenSim/Simulation/Model/Model.h>


//=============================================================================
// STATICS
//=============================================================================



using namespace std;
using namespace OpenSim;
using namespace SimTK;

///@cond  
const string Millard2012AccelerationMuscle::
    STATE_ACTIVATION_NAME = "activation";
const string Millard2012AccelerationMuscle::
    STATE_FIBER_LENGTH_NAME = "fiber_length";
const string Millard2012AccelerationMuscle::
    STATE_FIBER_VELOCITY_NAME = "fiber_velocity";
///@endcond  


const static int MLIfse     = 0;
const static int MLIfk      = 1;
const static int MLIfcphi   = 2;
const static int MLIfkPE    = 3;
const static int MLIfcphiPE = 4;

const static int MVIdlceAT = 0;

const static int MDIFiberAcceleration = 0;

//=============================================================================
// PROPERTY MANAGEMENT
//=============================================================================
void Millard2012AccelerationMuscle::setNull()
{

    setAuthors("Matthew Millard");
}


void Millard2012AccelerationMuscle::constructProperties()
{ 
    constructProperty_default_activation(0.0); //getMinActivation()

    constructProperty_default_fiber_length(getOptimalFiberLength());
    
    constructProperty_default_fiber_velocity(0.0);

        MuscleFirstOrderActivationDynamicModel defaultActMdl = 
            MuscleFirstOrderActivationDynamicModel();
        double tauAct = defaultActMdl.get_activation_time_constant();
        double tauDact= defaultActMdl.get_deactivation_time_constant();

    //Ensure the minimum allowed activation is 0.     
    constructProperty_MuscleFirstOrderActivationDynamicModel(
        MuscleFirstOrderActivationDynamicModel(tauAct,tauDact,0,getName()));

        ActiveForceLengthCurve defaultFal = ActiveForceLengthCurve();
        double lceMin  = defaultFal.getMinActiveFiberLength();
        double lceTrans= defaultFal.getTransitionFiberLength();
        double lceMax  = defaultFal.getMaxActiveFiberLength();
        double slope   = defaultFal.getShallowAscendingSlope();
        double minFal  = 0; //

    //Ensure the Active Force Length Curve can go to 0
    constructProperty_ActiveForceLengthCurve(
        ActiveForceLengthCurve(lceMin,lceTrans,lceMax,slope,minFal));

    ForceVelocityCurve defaultFv = ForceVelocityCurve();
    double concSlopeAtVmax = 0;
    double concSlopeNearVmax = defaultFv.getConcentricSlopeNearVmax();
    double isoSlope  = defaultFv.getIsometricSlope();
    double eccSlopeNearVmax = defaultFv.getEccentricSlopeNearVmax();
    double eccSlopeAtVmax  = 0;
    double maxFv     = defaultFv.getMaxEccentricVelocityForceMultiplier();
    double concCurviness= defaultFv.getConcentricCurviness();
    double eccCurviness = defaultFv.getEccentricCurviness();

    //Ensure the force velocity curve has asymptotes
    constructProperty_ForceVelocityCurve(
        ForceVelocityCurve( concSlopeAtVmax,
                            concSlopeNearVmax,
                            isoSlope,
                            eccSlopeAtVmax,
                            eccSlopeNearVmax,
                            maxFv,
                            concCurviness,
                            eccCurviness) );

    constructProperty_FiberForceLengthCurve(
        FiberForceLengthCurve());

    constructProperty_TendonForceLengthCurve(
        TendonForceLengthCurve());

    constructProperty_FiberCompressiveForceLengthCurve(
        FiberCompressiveForceLengthCurve());

    constructProperty_FiberCompressiveForceCosPennationCurve(
        FiberCompressiveForceCosPennationCurve());   

    //Nonlinear damping coefficients 
    constructProperty_tendon_force_length_damping(1e-1);
    constructProperty_fiber_compressive_force_length_damping(1.0);
    constructProperty_fiber_force_length_damping(1e-2);
    constructProperty_fiber_compressive_force_cos_pennation_damping(1.0);

    //Linear fiber damping as in Schutte's model
    constructProperty_fiber_damping(1e-2);

    //Mass property
    constructProperty_mass(0.1);

}

void Millard2012AccelerationMuscle::buildMuscle()
{
    double optFibLen = getOptimalFiberLength();
    double optPenAng = getPennationAngleAtOptimalFiberLength();
    std::string caller = getName();
    caller.append(".buildMuscle()");


    m_penMdl = MuscleFixedWidthPennationModel(  optFibLen,
                                                optPenAng, 
                                                SimTK::Pi/2.0);
   
    std::string aName = getName();

    std::string tmp = aName;
    tmp.append("_MuscleFirstOrderActivationDynamicModel");
    MuscleFirstOrderActivationDynamicModel& actMdl = upd_MuscleFirstOrderActivationDynamicModel();
    actMdl.setName(tmp);

    tmp = aName;
    tmp.append("_ActiveForceLengthCurve");
    ActiveForceLengthCurve& falCurve = upd_ActiveForceLengthCurve();
    falCurve.setName(tmp);

    tmp = aName;
    tmp.append("_ForceVelocityCurve");
    ForceVelocityCurve& fvCurve = upd_ForceVelocityCurve();
    fvCurve.setName(tmp);

    tmp = aName;
    tmp.append("_FiberForceLengthCurve");
    FiberForceLengthCurve& fpeCurve = upd_FiberForceLengthCurve();
    fpeCurve.setName(tmp);

    tmp = aName;
    tmp.append("_TendonForceLengthCurve");
    TendonForceLengthCurve& fseCurve = upd_TendonForceLengthCurve();
    fseCurve.setName(tmp);

    tmp = aName;
    tmp.append("_FiberCompressiveForceLengthCurve");
    FiberCompressiveForceLengthCurve& fkCurve 
        = upd_FiberCompressiveForceLengthCurve();
    fkCurve.setName(tmp);

    tmp = aName;
    tmp.append("_FiberCompressiveForceCosPennationCurve");
    FiberCompressiveForceCosPennationCurve& fcphi = 
        upd_FiberCompressiveForceCosPennationCurve();
    fcphi.setName(tmp);

     //Ensure all sub objects are up to date with properties;
    actMdl.finalizeFromProperties(); //TODO: Remove this once the activation
                                     //model has been made into a property.
    // TODO: Remove this once MuscleFixedWidthPennationModel has been made into
    //       a property.
    m_penMdl.finalizeFromProperties();

    falCurve.ensureCurveUpToDate();
    fvCurve.ensureCurveUpToDate();
    fpeCurve.ensureCurveUpToDate();
    fseCurve.ensureCurveUpToDate();
    fkCurve.ensureCurveUpToDate();
    fcphi.ensureCurveUpToDate();
    
   
    setObjectIsUpToDateWithProperties();
}

void Millard2012AccelerationMuscle::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    buildMuscle();
}


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

Millard2012AccelerationMuscle::Millard2012AccelerationMuscle()            
{    
    setNull();
    constructProperties();
}

Millard2012AccelerationMuscle::
Millard2012AccelerationMuscle(const std::string &aName,  double aMaxIsometricForce,
                  double aOptimalFiberLength,double aTendonSlackLength,
                  double aPennationAngle)
{
    setNull();
    constructProperties();

    setName(aName);    
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);
}

//=============================================================================
// Model Component Interface
//=============================================================================
 void Millard2012AccelerationMuscle::
     extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");

    addStateVariable(STATE_ACTIVATION_NAME);
    addStateVariable(STATE_FIBER_LENGTH_NAME);
    addStateVariable(STATE_FIBER_VELOCITY_NAME);
 }

void Millard2012AccelerationMuscle::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    setActivation(s, getDefaultActivation());
    setFiberLength(s, getDefaultFiberLength());
    setFiberVelocity(s,getDefaultFiberVelocity());
}
    
void Millard2012AccelerationMuscle::
    extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);

    setDefaultActivation(getStateVariableValue(s,STATE_ACTIVATION_NAME));
    setDefaultFiberLength(getStateVariableValue(s,STATE_FIBER_LENGTH_NAME));
    setDefaultFiberVelocity(getStateVariableValue(s,STATE_FIBER_VELOCITY_NAME));
}

void Millard2012AccelerationMuscle::
    computeStateVariableDerivatives(const SimTK::State& s) const 
{
    double adot=0, ldot=0, vdot=0;

    if(appliesForce(s)){
        adot = getActivationRate(s);
        ldot = getFiberVelocity(s);
        vdot = getFiberAcceleration(s);
    }

    setStateVariableDerivativeValue(s, STATE_ACTIVATION_NAME, adot);
    setStateVariableDerivativeValue(s, STATE_FIBER_LENGTH_NAME, ldot);
    setStateVariableDerivativeValue(s, STATE_FIBER_VELOCITY_NAME, vdot);
}

//=============================================================================
// STATE RELATED GET FUNCTIONS
//=============================================================================

double Millard2012AccelerationMuscle::getDefaultActivation() const
{
    return get_default_activation();
}

double Millard2012AccelerationMuscle::getDefaultFiberLength() const
{
    
    return get_default_fiber_length();
}

double Millard2012AccelerationMuscle::getDefaultFiberVelocity() const
{
   
    return get_default_fiber_velocity();
}

double Millard2012AccelerationMuscle::
    getActivationRate(const SimTK::State& s) const
{
 
    return calcActivationRate(s);
}

double Millard2012AccelerationMuscle::
    getFiberVelocity(const SimTK::State& s) const
{
    
    FiberVelocityInfo fvi = getFiberVelocityInfo(s);
    return fvi.fiberVelocity;
}

double Millard2012AccelerationMuscle::
    getFiberAcceleration(const SimTK::State& s) const
{
    
    MuscleDynamicsInfo fdi = getMuscleDynamicsInfo(s);
    return fdi.userDefinedDynamicsExtras[MDIFiberAcceleration];
}


//=============================================================================
// STATE RELATED SET FUNCTIONS
//=============================================================================

void Millard2012AccelerationMuscle::setDefaultActivation(double activation)
{
    set_default_activation(activation);
}

void Millard2012AccelerationMuscle::setDefaultFiberLength(double fiberLength)
{
    set_default_fiber_length(fiberLength);
}

void Millard2012AccelerationMuscle::
    setDefaultFiberVelocity(double fiberVelocity)
{
    set_default_fiber_velocity(fiberVelocity);
}

void Millard2012AccelerationMuscle::
    setActivation(SimTK::State& s, double activation) const
{
    setStateVariableValue(s, STATE_ACTIVATION_NAME, activation);
    markCacheVariableInvalid(s, _dynamicsInfoCV);
    
}

void Millard2012AccelerationMuscle::
    setFiberLength(SimTK::State& s, double fiberLength) const
{
    setStateVariableValue(s, STATE_FIBER_LENGTH_NAME, fiberLength);
    markCacheVariableInvalid(s, _lengthInfoCV);
    markCacheVariableInvalid(s, _velInfoCV);
    markCacheVariableInvalid(s, _dynamicsInfoCV);
    
}

void Millard2012AccelerationMuscle::
    setFiberVelocity(SimTK::State& s, double fiberVelocity) const
{
    setStateVariableValue(s, STATE_FIBER_VELOCITY_NAME, fiberVelocity);
    markCacheVariableInvalid(s, _velInfoCV);
    markCacheVariableInvalid(s, _dynamicsInfoCV);
    
}


//=============================================================================
// GET
//=============================================================================


double Millard2012AccelerationMuscle::
    getFiberCompressiveForceLengthMultiplier(SimTK::State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    return mli.userDefinedLengthExtras[MLIfk];
}

double Millard2012AccelerationMuscle::
    getFiberCompressiveForceCosPennationMultiplier(SimTK::State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    return mli.userDefinedLengthExtras[MLIfcphi];
}


double Millard2012AccelerationMuscle::
    getTendonForceMultiplier(SimTK::State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    return mli.userDefinedLengthExtras[MLIfse];
}

const MuscleFirstOrderActivationDynamicModel& Millard2012AccelerationMuscle::
    getActivationModel() const
{
    
    return get_MuscleFirstOrderActivationDynamicModel();
}

const MuscleFixedWidthPennationModel& Millard2012AccelerationMuscle::
    getPennationModel() const
{
    
    return m_penMdl;
}

const ActiveForceLengthCurve& Millard2012AccelerationMuscle::
    getActiveForceLengthCurve() const
{
    
    return get_ActiveForceLengthCurve();
}

const ForceVelocityCurve& Millard2012AccelerationMuscle::
    getForceVelocityCurve() const
{
    
    return get_ForceVelocityCurve();
}

const FiberForceLengthCurve& Millard2012AccelerationMuscle::
    getFiberForceLengthCurve() const
{
    
    return get_FiberForceLengthCurve();
}

const TendonForceLengthCurve& Millard2012AccelerationMuscle::
    getTendonForceLengthCurve() const
{
    
    return get_TendonForceLengthCurve();
}

const FiberCompressiveForceLengthCurve& Millard2012AccelerationMuscle::
    getFiberCompressiveForceLengthCurve() const
{
    
    return get_FiberCompressiveForceLengthCurve();
}

const FiberCompressiveForceCosPennationCurve& Millard2012AccelerationMuscle::
    getFiberCompressiveForceCosPennationCurve() const
{
    
    return get_FiberCompressiveForceCosPennationCurve();
}

double Millard2012AccelerationMuscle::
    getFiberStiffnessAlongTendon(const SimTK::State& s) const
{
    
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    return mdi.fiberStiffnessAlongTendon;
}

double Millard2012AccelerationMuscle::getMass() const
{
    
    return get_mass();
}


//=============================================================================
// SET
//=============================================================================


void Millard2012AccelerationMuscle::setActivationModel(
        MuscleFirstOrderActivationDynamicModel& aActivationMdl)
{
    set_MuscleFirstOrderActivationDynamicModel(aActivationMdl);
}
void Millard2012AccelerationMuscle::setActiveForceLengthCurve(
        ActiveForceLengthCurve& aActiveForceLengthCurve)
{
    set_ActiveForceLengthCurve(aActiveForceLengthCurve);
}

void Millard2012AccelerationMuscle::setForceVelocityCurve(
        ForceVelocityCurve& aForceVelocityCurve)
{   
    set_ForceVelocityCurve(aForceVelocityCurve);
}

void Millard2012AccelerationMuscle::setFiberForceLengthCurve(
        FiberForceLengthCurve& aFiberForceLengthCurve)
{
    set_FiberForceLengthCurve(aFiberForceLengthCurve);
}

void Millard2012AccelerationMuscle::setTendonForceLengthCurve(
        TendonForceLengthCurve& aTendonForceLengthCurve)
{
    set_TendonForceLengthCurve(aTendonForceLengthCurve);
}

void Millard2012AccelerationMuscle::setFiberCompressiveForceLengthCurve(
        FiberCompressiveForceLengthCurve& aFiberCompressiveForceLengthCurve)
{
    set_FiberCompressiveForceLengthCurve(
        aFiberCompressiveForceLengthCurve);
}

void Millard2012AccelerationMuscle::setFiberCompressiveForceCosPennationCurve(
        FiberCompressiveForceCosPennationCurve& 
        aFiberCompressiveForceCosPennationCurve)
{
    set_FiberCompressiveForceCosPennationCurve(
        aFiberCompressiveForceCosPennationCurve);
}

void Millard2012AccelerationMuscle::setMass(double mass) 
{
    SimTK_ERRCHK1_ALWAYS(mass >= 1e-3,
        "Millard2012AccelerationMuscle::setMass",
        "%s: The mass is set too small!",getName().c_str());

    set_mass(mass);
}



//==============================================================================
// SCALING
//==============================================================================
void Millard2012AccelerationMuscle::
extendPostScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "Millard2012AccelerationMuscle is not up-to-date with its properties.");

    Super::extendPostScale(s, scaleSet);

    GeometryPath& path = upd_GeometryPath();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_optimal_fiber_length() *= scaleFactor;
        upd_tendon_slack_length() *= scaleFactor;

        // Clear the pre-scale length that was stored in the GeometryPath.
        path.setPreScaleLength(s, 0.0);
    }
}


//==============================================================================
//                             START OF DEPRECATED
//==============================================================================
double Millard2012AccelerationMuscle::
calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                       double aActivation) const
{
        SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");

        string caller = getName();
        caller.append(  "Millard2012AccelerationMuscle::"
                        "calcInextensibleTendonActiveFiberForce");

        double inextensibleTendonActiveFiberForce = 0;

        double muscleLength = getLength(s);
        double muscleVelocity = getLengtheningSpeed(s);
        double tendonSlackLength = getTendonSlackLength();
        double tendonVelocity = 0.0; //Inextensible tendon;

        double fiberLength  = m_penMdl.calcFiberLength(muscleLength,
                                             tendonSlackLength);


        if(fiberLength > m_penMdl.getMinimumFiberLength()){
            double phi      = m_penMdl.calcPennationAngle(fiberLength);
        
            double fiberVelocity   = m_penMdl.calcFiberVelocity(cos(phi),
                                          muscleVelocity,tendonVelocity);

            inextensibleTendonActiveFiberForce = 
                calcActiveFiberForceAlongTendon(    aActivation,
                                                    fiberLength,
                                                    fiberVelocity);
        }


        return inextensibleTendonActiveFiberForce;
}


double Millard2012AccelerationMuscle::
            calcActiveFiberForceAlongTendon(double activation, 
                                            double fiberLength, 
                                            double fiberVelocity) const
{   

    string caller = getName();
    caller.append(  "::MillardAccelerationMuscle::"
                    "calcActiveFiberForceAlongTendon");

    double activeFiberForce = 0;    
    double clampedFiberLength = m_penMdl.clampFiberLength(fiberLength);

    //If the fiber is in a legal range, compute the force its generating
    if(fiberLength >  m_penMdl.getMinimumFiberLength()){

        //Clamp activation to a legal range
        MuscleFirstOrderActivationDynamicModel actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();
        double clampedActivation = actMdl.clampActivation(activation);

        //Normalize fiber length and velocity
        double normFiberLength    = clampedFiberLength/getOptimalFiberLength();
        double normFiberVelocity  = fiberVelocity / 
                        (getOptimalFiberLength() * getMaxContractionVelocity());

        //Get the necessary curves
        const ActiveForceLengthCurve& falCurve
            = getProperty_ActiveForceLengthCurve().getValue();
        const ForceVelocityCurve& fvCurve
            = getProperty_ForceVelocityCurve().getValue();

        //Evaluate the force active length and force velocity multipliers
        double fal  = falCurve.calcValue(normFiberLength);
        double fv   = fvCurve.calcValue(normFiberVelocity);
        double fiso = getMaxIsometricForce();

        //Evaluate the pennation angle
        double phi = m_penMdl.calcPennationAngle(fiberLength);

        //Compute the active fiber force 
        activeFiberForce = fiso * clampedActivation * fal * fv * cos(phi);
    }
    //Compute the active fiber force
    

    return activeFiberForce;
}

//==============================================================================
//                              END OF DEPRECATED
//==============================================================================



//==============================================================================
// Muscle.h Interface
//==============================================================================



double  Millard2012AccelerationMuscle::
    computeActuation(const SimTK::State& s) const
{    
    //ensureMuscleUpToDate();
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");

    
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setActuation(s, mdi.tendonForce);
    return( mdi.tendonForce );
}



void Millard2012AccelerationMuscle::
    computeInitialFiberEquilibrium(SimTK::State& s) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
    "Millard2012AccelerationMuscle: Muscle is not"
    " to date with properties");

    // Initialize the multibody system to the initial state vector.
    setFiberLength(s, getOptimalFiberLength());
    _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

    //Compute an initial muscle state that develops the desired force and
    //shares the muscle stretch between the muscle fiber and the tendon 
    //according to their relative stiffness.
    double activation = getActivation(s);

    //Tolerance, in Newtons, of the desired equilibrium
    double tol = 1e-8*getMaxIsometricForce();  //Should this be user settable?
    if(tol < SimTK::SignificantReal*10){
        tol = SimTK::SignificantReal*10;
    }
    int maxIter = 500;  //Should this be user settable?  
    double newtonStepFraction = 0.75;

    std::pair<StatusFromInitMuscleState, ValuesFromInitMuscleState> result =
        initMuscleState(s, activation, tol, maxIter, newtonStepFraction);

    switch(result.first) {

    case StatusFromInitMuscleState::Success_Converged:
        setActuation(s, result.second["tendon_force"]);
        setFiberLength(s, result.second["fiber_length"]);
        setFiberVelocity(s, result.second["fiber_velocity"]);
        break;

    case StatusFromInitMuscleState::Warning_FiberAtLowerBound:
        log_warn("Millard2012AccelerationMuscle initialization: '{}' is at its "
               "minimum fiber length of {}.",
               getName(), result.second["fiber_length"]);
        setActuation(s, result.second["tendon_force"]);
        setFiberLength(s, result.second["fiber_length"]);
        setFiberVelocity(s, result.second["fiber_velocity"]);
        break;

    case StatusFromInitMuscleState::Failure_MaxIterationsReached:
        // Report internal variables and throw exception.
        std::ostringstream ss;
        ss << "\n  Solution error " << abs(result.second["solution_error"])
           << " exceeds tolerance of " << tol << "\n"
           << "  Newton iterations reached limit of " << maxIter << "\n"
           << "  Activation is " << activation << "\n"
           << "  Fiber length is " << result.second["fiber_length"] << "\n";
        OPENSIM_THROW_FRMOBJ(MuscleCannotEquilibrate, ss.str());
        break;
    }
}


void Millard2012AccelerationMuscle::
    calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
{    
    //ensureMuscleUpToDate();
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");
    
    // double simTime = s.getTime(); //for debugging purposes

    try{
        //Get whole muscle properties
        // double maxIsoForce      = getMaxIsometricForce();
        double optFiberLength   = getOptimalFiberLength();
        double mclLength        = getLength(s);
        double tendonSlackLen   = getTendonSlackLength();
        std::string caller      = getName();
        caller.append(".calcMuscleLengthInfo");

        //Get muscle model specific properties
        const TendonForceLengthCurve& fseCurve = get_TendonForceLengthCurve(); 
        const FiberForceLengthCurve& fpeCurve  = get_FiberForceLengthCurve(); 
        const ActiveForceLengthCurve& falCurve = get_ActiveForceLengthCurve(); 

        const FiberCompressiveForceLengthCurve& fkCurve
            = get_FiberCompressiveForceLengthCurve(); 
        const FiberCompressiveForceCosPennationCurve& fcphiCurve
            = get_FiberCompressiveForceCosPennationCurve(); 

        //Populate the output struct
        mli.fiberLength       = getStateVariableValue(s, STATE_FIBER_LENGTH_NAME); 

        mli.normFiberLength   = mli.fiberLength/optFiberLength;
        mli.pennationAngle    = m_penMdl.calcPennationAngle(mli.fiberLength);
        mli.cosPennationAngle = cos(mli.pennationAngle);
        mli.sinPennationAngle = sin(mli.pennationAngle);

        mli.fiberLengthAlongTendon = mli.fiberLength*mli.cosPennationAngle;
    
        mli.tendonLength      = m_penMdl.calcTendonLength(mli.cosPennationAngle,
                                                       mli.fiberLength,mclLength);
        mli.normTendonLength  = mli.tendonLength / tendonSlackLen;
        mli.tendonStrain      = mli.normTendonLength -  1.0;

        mli.fiberPassiveForceLengthMultiplier= 
            fpeCurve.calcValue(mli.normFiberLength);
        mli.fiberActiveForceLengthMultiplier = 
            falCurve.calcValue(mli.normFiberLength); 

        double tendonForceLengthMultiplier=fseCurve.calcValue(mli.normTendonLength);

        //Put in the additional length related terms that are specific to this
        //particular muscle model.
        mli.userDefinedLengthExtras.resize(5);

        mli.userDefinedLengthExtras[MLIfse]     = tendonForceLengthMultiplier;
        mli.userDefinedLengthExtras[MLIfk]      = fkCurve.calcValue(
                                                    mli.normFiberLength);
        mli.userDefinedLengthExtras[MLIfcphi]   = fcphiCurve.calcValue(
                                                       mli.cosPennationAngle);

    }catch(const std::exception &x){
        std::string msg = "Exception caught in Millard2012AccelerationMuscle::" 
                        "calcMuscleLengthInfo\n"                 
                        "of " + getName()  + "\n"                            
                        + x.what();
        throw OpenSim::Exception(msg);
    }
}


void Millard2012AccelerationMuscle::calcMusclePotentialEnergyInfo(const SimTK::State& s,
        MusclePotentialEnergyInfo& mpei) const
{
    try {
        //Get whole muscle properties
        double maxIsoForce      = getMaxIsometricForce();
        double optFiberLength   = getOptimalFiberLength();
        // double mclLength        = getLength(s);
        double tendonSlackLen   = getTendonSlackLength();

        // Get the quantities that we've already computed.
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);

        //Get muscle model specific properties
        const TendonForceLengthCurve& fseCurve = get_TendonForceLengthCurve(); 
        const FiberForceLengthCurve& fpeCurve  = get_FiberForceLengthCurve(); 
        // const ActiveForceLengthCurve& falCurve = get_ActiveForceLengthCurve(); 

        const FiberCompressiveForceLengthCurve& fkCurve
            = get_FiberCompressiveForceLengthCurve(); 
        const FiberCompressiveForceCosPennationCurve& fcphiCurve
            = get_FiberCompressiveForceCosPennationCurve(); 

        //Note the curves return normalized area. Each area must be un-normalized   
        mpei.fiberPotentialEnergy =  fpeCurve.calcIntegral(mli.normFiberLength)
                                    *(optFiberLength*maxIsoForce);

        mpei.tendonPotentialEnergy=  fseCurve.calcIntegral(mli.normTendonLength)
                                    *(tendonSlackLen*maxIsoForce);

        double compForceLengthPE=   fkCurve.calcIntegral(mli.normFiberLength)
                                    *(optFiberLength*maxIsoForce);

        //Only the force dimension is normalized
        double compForceCosPennationPE = 
            fcphiCurve.calcIntegral(mli.cosPennationAngle)*(maxIsoForce);

        mpei.musclePotentialEnergy=  mpei.fiberPotentialEnergy 
                                  + mpei.tendonPotentialEnergy
                                  + compForceLengthPE
                                  + compForceCosPennationPE;
    }
    catch(const std::exception &x){
        std::string msg = "Exception caught in Thelen2003Muscle::" 
                          "calcMusclePotentialEnergyInfo\n"                 
                           "of " + getName()  + "\n"                            
                           + x.what();
        throw OpenSim::Exception(msg);
    }
}

void Millard2012AccelerationMuscle::
    calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
    //ensureMuscleUpToDate();
    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");

    // double simTime = s.getTime(); //for debugging purposes

    try{
        //Get the quantities that we've already computed
            const MuscleLengthInfo &mli = getMuscleLengthInfo(s);

        //Get the static properties of this muscle
            // double mclLength      = getLength(s);
            double mclVelocity    = getLengtheningSpeed(s);
            // double tendonSlackLen = getTendonSlackLength();
            double optFiberLen    = getOptimalFiberLength();

        //Prep strings that will be useful to make sensible exception messages
            std::string muscleName = getName();
            std::string fcnName     = ".calcFiberVelocityInfo";

            std::string caller      = muscleName;        
            caller.append(fcnName);

        //=========================================================================
        // Compute fv by inverting the force-velocity relationship in the 
        // equilibrium equations
        //=========================================================================

        //1. Get MuscleLengthInfo & available State information
        double dlce   = getStateVariableValue(s, STATE_FIBER_VELOCITY_NAME);
        double dlceN1  = dlce/(getMaxContractionVelocity()*optFiberLen);
        double lce    = mli.fiberLength;
        double phi    = mli.pennationAngle;
        double cosphi = mli.cosPennationAngle;
        double sinphi = mli.sinPennationAngle;

        // double fal  = mli.fiberActiveForceLengthMultiplier;
        // double fpe  = mli.fiberPassiveForceLengthMultiplier;
        // double fse  = mli.userDefinedLengthExtras[MLIfse];
        // double fk   = mli.userDefinedLengthExtras[MLIfk];
        // double fcphi= mli.userDefinedLengthExtras[MLIfcphi];

    
        //2. Compute fv - but check for singularities first     
        const ForceVelocityCurve& fvCurve = get_ForceVelocityCurve(); 
        double fv = fvCurve.calcValue(dlceN1);
    
        //7. Compute the other related velocity components
        //double dlceAT = m_penMdl.
        double tanPhi = tan(phi);
        double dphidt    = m_penMdl.calcPennationAngularVelocity(tanPhi,lce,dlce);   
        double dtl       = m_penMdl.calcTendonVelocity(cosphi,sinphi,dphidt,
                                                        lce,  dlce,mclVelocity);

        //Populate the struct;
        fvi.fiberVelocity               = dlce;
        fvi.fiberVelocityAlongTendon    = m_penMdl.calcFiberVelocityAlongTendon(lce,
                                                        dlce,sinphi,cosphi, dphidt);
        fvi.normFiberVelocity           = dlceN1;

        fvi.pennationAngularVelocity    = dphidt;

        fvi.tendonVelocity              = dtl;
        fvi.normTendonVelocity = dtl/getTendonSlackLength();

        fvi.fiberForceVelocityMultiplier = fv;

        fvi.userDefinedVelocityExtras.resize(1);
        fvi.userDefinedVelocityExtras[MVIdlceAT] = 
            m_penMdl.calcFiberVelocityAlongTendon(lce,
                                                dlce,
                                                mli.sinPennationAngle,
                                                mli.cosPennationAngle,
                                                dphidt);
    }catch(const std::exception &x){
        std::string msg = "Exception caught in Millard2012AccelerationMuscle::" 
                        "calcFiberVelocityInfo\n"                 
                        "of " + getName()  + "\n"                            
                        + x.what();
        throw OpenSim::Exception(msg);
    }
    
}


void Millard2012AccelerationMuscle::
    calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
        //ensureMuscleUpToDate();
        SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");
    
        // double simTime = s.getTime(); //for debugging purposes

    try{
    //Get the quantities that we've already computed
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
        const FiberVelocityInfo &mvi = getFiberVelocityInfo(s);        
        
    //Get the state of this muscle
        double a   = getStateVariableValue(s, STATE_ACTIVATION_NAME); 

    //Get the properties of this muscle
        // double mcl            = getLength(s);
        double dmcl_dt        = getLengtheningSpeed(s);
        // double tsl            = getTendonSlackLength();
        // double ofl            = getOptimalFiberLength();
        double fiso           = getMaxIsometricForce();
        // const TendonForceLengthCurve& fseCurve= get_TendonForceLengthCurve();

    //Prep strings that will be useful to make sensible exception messages
        std::string muscleName = getName();
        std::string fcnName     = ".calcMuscleDynamicsInfo";
        std::string caller      = muscleName;        
        caller.append(fcnName);

    //=========================================================================
    // Compute the fiber acceleration
    //=========================================================================

    //Get fiber/tendon kinematic information

        double lce          = mli.fiberLength;
        // double lceN         = lce/ofl;    
        double dlce_dt      = mvi.fiberVelocity;
        // double dlceN1_dt    = mvi.normFiberVelocity;
    
        double phi          = mli.pennationAngle;
        double cosPhi       = mli.cosPennationAngle;
        // double sinPhi       = mli.sinPennationAngle;
        double dphi_dt      = mvi.pennationAngularVelocity;
    
        double tl       = mli.tendonLength; 
        double dtl_dt   = mvi.tendonVelocity;
        // double tlN      = mli.normTendonLength;
   
        double fal  = mli.fiberActiveForceLengthMultiplier;
        double fpe  = mli.fiberPassiveForceLengthMultiplier;
        double fv   = mvi.fiberForceVelocityMultiplier;    
        double fse  = mli.userDefinedLengthExtras[MLIfse];
        double fk   = mli.userDefinedLengthExtras[MLIfk];
        double fcphi= mli.userDefinedLengthExtras[MLIfcphi];

    //========================================================================
    //Compute viscoelastic multipliers and their derivatives
    //========================================================================
        AccelerationMuscleInfo ami;
        calcAccelerationMuscleInfo( ami,
                                    lce  ,dlce_dt,
                                    phi  ,dphi_dt,
                                    tl   ,dtl_dt,
                                    fal  ,fv,fpe,fk,fcphi,fse);

    //========================================================================
    //Compute viscoelastic multipliers derivatives
    //========================================================================
        SimTK::Vec2 fiberForceIJ = calcFiberForceIJ(a,ami);
        double Fce  = calcFiberForce(fiberForceIJ,ami);    
        double FceAT= calcFiberForceAlongTendon(fiberForceIJ);

        double Fse = calcTendonForce(ami);
        double m = getMass();

        double ddlce_dtt = (1/m)*(Fse-FceAT)*cosPhi + lce*dphi_dt*dphi_dt;        

    //=========================================================================
    // Compute the stiffness properties
    //=========================================================================
    //Compute the stiffness of the fiber along the tendon
        SimTK::Vec2 fiberStiffnessIJ = calcFiberStiffnessIJ(a,ami);
        double dFce_dlce = calcFiberStiffness(fiberForceIJ,fiberStiffnessIJ, ami);

        double dFceAT_dlce = calc_DFiberForceAT_DFiberLength(fiberStiffnessIJ);
        double dFceAT_dlceAT=
            calc_DFiberForceAT_DFiberLengthAT(dFceAT_dlce, ami);   

    //Compute the stiffness of the tendon
        double dFt_dtl    = calcTendonStiffness(ami);

    //Compute the stiffness of the whole muscle/tendon complex
        double Ke = 0;
        if (abs(dFceAT_dlceAT*dFt_dtl)>0)
            Ke = (dFceAT_dlceAT*dFt_dtl)/(dFceAT_dlceAT+dFt_dtl);

    //Populate the output vector
   
        mdi.activation                   = a;
        mdi.fiberForce                   = Fce; 
        mdi.fiberForceAlongTendon        = FceAT;
        mdi.normFiberForce               = Fce/fiso;
        mdi.activeFiberForce             = a*ami.fal*ami.fv*fiso;
        mdi.passiveFiberForce            = (( ami.fpeVEM-ami.fkVEM)
                                             -ami.fcphiVEM*cosPhi)*fiso;
    //ami.fpeVEM*fiso; 
                                     
        mdi.tendonForce                  = Fse; 
        mdi.normTendonForce              = ami.fse;      
        //just the elastic component
                                     
        mdi.fiberStiffness               = dFce_dlce;
        mdi.fiberStiffnessAlongTendon    = dFceAT_dlceAT;
        mdi.tendonStiffness              = dFt_dtl;
        mdi.muscleStiffness              = Ke;
                                         
    //Check that the derivative of system energy less work is zero within
    //a reasonable numerical tolerance. 
        
        double dfpePEdt    =  (ami.fpe)  * ami.cosphi   * fiso * ami.dlceAT_dt;
        double dfkPEdt     = -(ami.fk)   * ami.cosphi   * fiso * ami.dlceAT_dt;    
        double dfcphiPEdt  = -(ami.fcphi)               * fiso * ami.dlceAT_dt;
        double dfsePEdt    =  (ami.fse)                 * fiso * ami.dtl_dt;

        double dfpeVdt    = -(ami.fpeV)   * ami.cosphi  * fiso * ami.dlceAT_dt;
        double dfkVdt     =  (ami.fkV)    * ami.cosphi  * fiso * ami.dlceAT_dt;    
        double dfcphiVdt  =  (ami.fcphiV)               * fiso * ami.dlceAT_dt;
        double dfseVdt    = -(ami.fseV)                 * fiso * ami.dtl_dt;
        double dfibVdt    = -(ami.fibV                  * fiso * ami.dlce_dt);

        // double dfpeVEMdt   =  ami.fpeVEM   * ami.cosphi  * fiso * ami.dlceAT_dt;
        // double dfkVEMdt    = -ami.fkVEM    * ami.cosphi  * fiso * ami.dlceAT_dt;
        // double dfcphiVEMdt = -ami.fcphiVEM *               fiso * ami.dlceAT_dt;
        // double dfseVEMdt   =  ami.fseVEM   *               fiso * ami.dtl_dt;

        double ddphi_dtt = m_penMdl.calcPennationAngularAcceleration(ami.lce,
                                                                ami.dlce_dt,
                                                                ddlce_dtt,
                                                                ami.sinphi,
                                                                ami.cosphi,
                                                                ami.dphi_dt);

        double ddlceAT_dtt = m_penMdl.calcFiberAccelerationAlongTendon(ami.lce,
                                                                    ami.dlce_dt,
                                                                    ddlce_dtt,
                                                                    ami.sinphi,
                                                                    ami.cosphi,
                                                                    ami.dphi_dt,
                                                                    ddphi_dtt);
        //(d/dt) KE = 1/2 * m * dlceAT_dt*dlceAT_dt
        double dKEdt = m*ami.dlceAT_dt*ddlceAT_dtt; 

        double dFibWdt      = -mdi.activeFiberForce*mvi.fiberVelocity;
        double dBoundaryWdt = mdi.tendonForce * dmcl_dt;
        /*double dSysEdt      = (dfpePEdt + dfkPEdt + dfcphiPEdt + dfsePEdt)
                             - dFibWdt 
                             - dBoundaryWdt 
                             + (dfpeVdt + dfkVdt + dfcphiVdt + dfseVdt);
                             */
        // double dSysEdt = dKEdt 
        //                 + (dfpePEdt + dfkPEdt + dfcphiPEdt + dfsePEdt)
        //                 - (dFibWdt + dBoundaryWdt)
        //                 - (dfpeVdt + dfkVdt + dfcphiVdt + dfseVdt)
        //                 - dfibVdt;

        // double tol = sqrt(SimTK::Eps);
    
        //For debugging purposes
        //if(abs(dSysEdt) >= tol){
        //    printf("KE+PE-W Tol Violation at time %f,by %f \n",
        //           simTime,dSysEdt);
        //    tol = sqrt(SimTK::Eps);
        //}
    
        /////////////////////////////
        //Populate the power entries
        /////////////////////////////
        mdi.fiberActivePower    = dFibWdt;

        //The kinetic energy term looks a little weird here, but for this 
        //interface this is, I think, the most logical place for it
        mdi.fiberPassivePower   = -(dKEdt + (dfpePEdt + dfkPEdt + dfcphiPEdt)                    
                                        - (dfpeVdt  + dfkVdt  + dfcphiVdt)
                                        - dfibVdt);
        mdi.tendonPower         = -(dfsePEdt-dfseVdt);       
        mdi.musclePower         = -dBoundaryWdt;


        //if(abs(tmp) > tol)
        //    printf("%s: d/dt(system energy-work) > tol, (%f > %f) at time %f",
         //           fcnName.c_str(), tmp, tol, (double)s.getTime());
    
    
        mdi.userDefinedDynamicsExtras.resize(1);
        mdi.userDefinedDynamicsExtras[MDIFiberAcceleration]=ddlce_dtt;

    }catch(const std::exception &x){
        std::string msg = "Exception caught in Millard2012AccelerationMuscle::" 
                        "calcMuscleDynamicsInfo\n"                 
                        "of " + getName()  + "\n"                            
                        + x.what();
        throw OpenSim::Exception(msg);
    }
}


//==============================================================================
// Numerical Guts: Initialization
//==============================================================================
std::pair<Millard2012AccelerationMuscle::StatusFromInitMuscleState,
          Millard2012AccelerationMuscle::ValuesFromInitMuscleState>
Millard2012AccelerationMuscle::initMuscleState(
                                    const SimTK::State& s,
                                    const double aActivation,
                                    const double aSolTolerance,
                                    const int aMaxIterations,
                                    const double aNewtonStepFraction) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");

    std::string caller = getName();
    caller.append(".initMuscleState");

    //I'm using smaller variable names here to make it possible to write out 
    //lengthy equations
    double a        = aActivation;
    double ml       = getLength(s);
    double dml_dt   = getLengtheningSpeed(s);

    //Shorter version of the constants
    double tsl = getTendonSlackLength();
    double ofl = getOptimalFiberLength();

    // double ophi= getPennationAngleAtOptimalFiberLength();
    // double penHeight = m_penMdl.getParallelogramHeight();
    double fiso= getMaxIsometricForce();
    double vmax = getMaxContractionVelocity();//getPropertyValue<double>(VmaxName);

    //Get muscle model specific properties       
    const ActiveForceLengthCurve&   falCurve = get_ActiveForceLengthCurve(); 
    const ForceVelocityCurve&       fvCurve  = get_ForceVelocityCurve(); 
    const FiberForceLengthCurve&    fpeCurve = get_FiberForceLengthCurve(); 
    const TendonForceLengthCurve&   fseCurve = get_TendonForceLengthCurve(); 

    const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve(); 
    

    //Shorter version of normalized muscle multipliers
    double fse = 0; //Normalized tendon (series element) force
    double fal = 0; //Normalized active force length multiplier
    double fv  = 0; //Normalized force-velocity multiplier
    double fpe = 0; //Normalized parallel element force
    double fk  = 0; //Normalized fiber compressive force
    double fcphi=0; //Normalized pennation compressive force


    //*******************************
    //Position level
    double lce = 0;
    double tl  = getTendonSlackLength()*1.01;

    lce = m_penMdl.calcFiberLength( ml, tl);
    
    double phi      = m_penMdl.calcPennationAngle(lce);
    double cosphi   = cos(phi);
    // double sinphi   = sin(phi);       
    double tlN      = tl/tsl;
    double lceN     = lce/ofl;
    
    //Velocity level
    double dtl_dt    = 0;
    
    double dlce_dt   = m_penMdl.calcFiberVelocity(cosphi,dml_dt, dtl_dt);
    double dlceN1_dt = dlce_dt/(vmax*ofl);
    
    double dphi_dt   = m_penMdl.calcPennationAngularVelocity(tan(phi),
                                                            lce,
                                                            dlce_dt);

    //*******************************
    //Internal Kinematics Related Variables for the loop
    // double dphi_dlce = 0;
    // double dtl_dlce = 0;
    //*******************************
    //Internal Force Related Variables for the loop
    // double Fce = 0;          // Fiber force
    double FceAT=0;          // Fiber force along tendon
    double Fse = 0;          // Tendon force
    double m = getMass();
    double ddlce_dtt = 1;        // Solution error
    
    // double dFce_dlce     = 0;  // Partial derivative of fiber force w.r.t. lce
    // double dFceAT_dlce   = 0;  // Partial derivative of fiber force along 
                               // tendon w.r.t. lce
    // double dFceAT_dlceAT = 0;  // Partial derivative of muscle force along
                               // tendon w.r.t. lce along the tendon.

    // double dFse_dlce   = 0;  // Partial derivative of tendon force w.r.t. lce
    // double dFse_dtl   = 0;   // Partial derivative of tendon force w.r.t. tl

    // double d_ddlcedtt_dlce = 0; // Partial derivative of the solution error w.r.t
                                // lce
    double delta_lce   = 0;     // Chance in lce

    double Ke          = 0;  // Linearized local stiffness of the muscle
    
    // double tmp1         = 0;
    // double tmp2         = 0;
    // double tmp3         = 0;
    //*******************************
    //Initialize the loop
    
    int iter = 0;
    int minFiberLengthCtr = 0;

    AccelerationMuscleInfo ami;

    while( abs(ddlce_dtt) > aSolTolerance 
        && iter < aMaxIterations
        && minFiberLengthCtr < 10){

        
        //Update the multipliers and their partial derivatives
        fal         = falCurve.calcValue(lceN);
        fpe         = fpeCurve.calcValue(lceN);
        fk          = fkCurve.calcValue(lceN);
        fcphi       = fcphiCurve.calcValue(cosphi);
        fse         = fseCurve.calcValue(tlN);            
        fv          = fvCurve.calcValue(dlceN1_dt);
            
        //========================================================================
        //Compute viscoelastic multipliers and their derivatives
        //========================================================================            
        calcAccelerationMuscleInfo( ami,
                                    lce  ,dlce_dt,
                                    phi  ,dphi_dt,
                                    tl   ,dtl_dt,
                                    fal  ,fv,fpe,fk,fcphi,fse);

        //==================================================================
        //Compute viscoelastic multipliers derivatives
        //==================================================================
        SimTK::Vec2 fiberForceIJ = calcFiberForceIJ(a,ami);
        /*Fce  = */calcFiberForce(fiberForceIJ,ami);    
        FceAT= calcFiberForceAlongTendon(fiberForceIJ);
        Fse = calcTendonForce(ami);            

        ddlce_dtt = (1/m)*(Fse-FceAT)*ami.cosphi 
                    + lce*ami.dphi_dt*ami.dphi_dt;     

        //==================================================================
        // Compute the stiffness properties
        //==================================================================
        //Compute the stiffness of the fiber along the tendon
        SimTK::Vec2 fiberStiffnessIJ = calcFiberStiffnessIJ(a,ami);

        /* double dFce_dlce = */
            calcFiberStiffness(fiberForceIJ,fiberStiffnessIJ, ami);

        double dFceAT_dlce = 
            calc_DFiberForceAT_DFiberLength(fiberStiffnessIJ);
            
        double dFceAT_dlceAT=
            calc_DFiberForceAT_DFiberLengthAT(dFceAT_dlce, ami);   

        //Compute the stiffness of the tendon
        double dFse_dtl    = calcTendonStiffness(ami);      
        double dFse_dlce   = calc_DTendonForce_DFiberLength(dFse_dtl,
                                                            ami,
                                                            caller);
        //==================================================================
        // Compute the error derivative
        //==================================================================

        //Acceleration derivative
        //(1/m)*(Fse-FceAT)*ami.cosphi 
                            //+ lce*ami.dphi_dt*ami.dphi_dt; 


        double d_ddlcedtt_dlce = 
                            (1/m)*(dFse_dlce-dFceAT_dlce)*ami.cosphi 
                            +(1/m)*(Fse-FceAT)*(-ami.sinphi*ami.dphi_dlce)
                            +  (1)*ami.dphi_dt*ami.dphi_dt
                            +  lce*(2*ami.dphi_dt*ami.d_dphidt_dlce); 

        if(abs(ddlce_dtt) > aSolTolerance){

            //If the derivative is good take a Newton step
            if(abs(d_ddlcedtt_dlce) > SimTK::SignificantReal){
                delta_lce   = - ddlce_dtt/d_ddlcedtt_dlce;
                lce         = lce + aNewtonStepFraction*delta_lce;
            }else{ //If we've stagnated, perturb the current solution
            
                double perturbation = 
                    2.0*((double)rand())/((double)RAND_MAX)-1.0;
                double lengthPerturbation = 
                    0.5*perturbation*getOptimalFiberLength();
                lce = lce + lengthPerturbation;

            }


            if(lce < m_penMdl.getMinimumFiberLength()){
                minFiberLengthCtr++;
                lce = m_penMdl.getMinimumFiberLength(); 
            }
           
            phi = m_penMdl.calcPennationAngle(lce);
            // sinphi = sin(phi);
            cosphi = cos(phi);
            tl  =m_penMdl.calcTendonLength(cosphi,lce,ml);
            lceN = lce/ofl;
            tlN  = tl/tsl;

        /*
        Update velocity level quantities: share the muscle velocity 
        between the tendon and the fiber according to their relative 
        stiffness:
                
        Linearizing the force balance equations
        FceAT = Ft at equilibrium (ignoring the small Coriolis term)
        FceAT = KceAT*(xceAT-xceAT0) + FceAT0
        Ft    = Kt*(xt-xt0) + Ft0
                
        Fm = Fce = Ft
        Ke*(xm-xm0)+Fe0 = KceAT*(xceAT-xceAT0) + FceAT0  
                        = Kt*(xt-xt0) + Ft0

        Taking a time derivative of the linearized equations yields
        dFm_dt = dFce_dt = dFt_dt

        Assuming that the time derivative of the stiffnesses (Ke,KceAT,
        and Kt) are zero (an approximation):

        Ke*dxm_dt = KceAT*dxceAT_dt  
                    = Kt   *dxt_dt
                
        Solving for dxceAT_dt and dxt_dt yields a way to split up the
        whole muscle velocity between the tendon and the fiber:

        dxt_dt      = Ke*dxm_dt/Kt
        dxceAT_dt   = Ke*dxm_dt/KceAT

        This is a heuristic. The above assumptions are necessary as 
        computing the partial derivatives of Kce or Kt w.r.t. time 
        requires acceleration level knowledge, which is not available in 
        general.
        */

        //Stiffness of the muscle is the stiffness of the tendon and the 
        //fiber (along the tendon) in series
        Ke = 1;
        if(abs(dFceAT_dlceAT + dFse_dtl) > SimTK::SignificantReal 
            && tl > getTendonSlackLength()){
            
            Ke     =(dFceAT_dlceAT*dFse_dtl)/(dFceAT_dlceAT + dFse_dtl);
            dtl_dt = (1/dFse_dtl)*Ke*dml_dt;

        }else{ //tendon is slack, so its doing all of the stretching.
            dtl_dt = dml_dt;
        }

        dlce_dt= m_penMdl.calcFiberVelocity(cosphi,dml_dt,dtl_dt);
                   
        dlceN1_dt = dlce_dt/(vmax*ofl);
        dphi_dt   = m_penMdl.calcPennationAngularVelocity(tan(phi),
                                                            lce,
                                                            dlce_dt);
       
        }
        
        iter++;
    }

    // Populate the result map.
    ValuesFromInitMuscleState resultValues;

    if (abs(ddlce_dtt) < aSolTolerance) {  // The solution converged.

        resultValues["solution_error"] = ddlce_dtt;
        resultValues["iterations"]     = (double)iter;
        resultValues["fiber_length"]   = lce;
        resultValues["fiber_velocity"] = dlce_dt;
        resultValues["passive_force"]  = ami.fpeVEM * fiso;
        resultValues["tendon_force"]   = ami.fseVEM * fiso;

        return std::pair<StatusFromInitMuscleState, ValuesFromInitMuscleState>
               (StatusFromInitMuscleState::Success_Converged, resultValues);
    }

    if (iter < aMaxIterations) {  // Fiber length is at its lower bound.

        lce = m_penMdl.getMinimumFiberLength();
        phi = m_penMdl.calcPennationAngle(lce);
        // sinphi = sin(phi);
        cosphi = cos(phi);
            
        tl  = m_penMdl.calcTendonLength(cosphi,lce,ml);
        dtl_dt = dml_dt;

        lceN = lce/ofl;
        tlN  = tl/tsl;    

        //this should be 0
        dlce_dt= m_penMdl.calcFiberVelocity(cosphi,dml_dt,dtl_dt);
                   
        dlceN1_dt = dlce_dt/(vmax*ofl);
        dphi_dt   = m_penMdl.calcPennationAngularVelocity(tan(phi),
                                                        lce,
                                                        dlce_dt);
        fal         = falCurve.calcValue(lceN);
        fpe         = fpeCurve.calcValue(lceN);
        fk          = fkCurve.calcValue(lceN);
        fcphi       = fcphiCurve.calcValue(cosphi);
        fse         = fseCurve.calcValue(tlN);            
        fv          = fvCurve.calcValue(dlceN1_dt);
            
        //============================================================
        //Compute viscoelastic multipliers and their derivatives 
        //when the fiber is at its minimum value.
        //============================================================            
        calcAccelerationMuscleInfo( ami,
                                    lce  ,dlce_dt,
                                    phi  ,dphi_dt,
                                    tl   ,dtl_dt,
                                    fal  ,fv,fpe,fk,fcphi,fse);
        Fse = calcTendonForce(ami);

        // TODO: Check for a pennation angle singularity

        resultValues["solution_error"] = ddlce_dtt;
        resultValues["iterations"]     = (double)iter;
        resultValues["fiber_length"]   = lce;
        resultValues["fiber_velocity"] = dlce_dt;
        resultValues["passive_force"]  = ami.fpeVEM * fiso;
        resultValues["tendon_force"]   = ami.fseVEM * fiso;

        return std::pair<StatusFromInitMuscleState, ValuesFromInitMuscleState>
           (StatusFromInitMuscleState::Warning_FiberAtLowerBound, resultValues);
    }

    resultValues["solution_error"] = ddlce_dtt;
    resultValues["iterations"]     = (double)iter;
    resultValues["fiber_length"]   = SimTK::NaN;
    resultValues["fiber_velocity"] = SimTK::NaN;
    resultValues["passive_force"]  = SimTK::NaN;
    resultValues["tendon_force"]   = SimTK::NaN;

    return std::pair<StatusFromInitMuscleState, ValuesFromInitMuscleState>
        (StatusFromInitMuscleState::Failure_MaxIterationsReached, resultValues);
}


//==============================================================================
// Protected Numerical Services
//==============================================================================

/** Get the rate change of activation */
double Millard2012AccelerationMuscle::
    calcActivationRate(const SimTK::State& s) const 
{    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012AccelerationMuscle: Muscle is not"
        " to date with properties");

    double excitation = getExcitation(s);
    double activation = getActivation(s);

    const MuscleFirstOrderActivationDynamicModel& actMdl 
        = getProperty_MuscleFirstOrderActivationDynamicModel().getValue(); 

    double dadt = actMdl.calcDerivative(activation,excitation);
    return dadt;
}  

//==============================================================================
// Private Numerical Services
//==============================================================================
double Millard2012AccelerationMuscle::
    calcTendonForce(const AccelerationMuscleInfo& ami) const
{

    double fiso = getMaxIsometricForce();
    double Fse  = fiso*ami.fseVEM;
    return Fse;
}

SimTK::Vec2 Millard2012AccelerationMuscle::
    calcFiberForceIJ( double a, 
                      const AccelerationMuscleInfo& ami) const
{    
    //Gross muscle properties
    double fiso     = getMaxIsometricForce();
    
    //Relabel the variable names to make them nicer to read
    double fal      = ami.fal;
    double fv       = ami.fv;
    double fpeVEM   = ami.fpeVEM;
    double fkVEM    = ami.fkVEM;
    double cosPhi   = ami.cosphi;
    double sinPhi   = ami.sinphi;
    double fcphiVEM = ami.fcphiVEM;
    double fibV     = ami.fibV;

    SimTK::Vec2 fceIJ;

    fceIJ[0] = fiso * ((a*fal*fv + fpeVEM - fkVEM + fibV)*cosPhi - fcphiVEM);
    fceIJ[1] = fiso * ((a*fal*fv + fpeVEM - fkVEM + fibV)*sinPhi);

    return fceIJ;
}

double Millard2012AccelerationMuscle::
    calcFiberForce(  SimTK::Vec2 fiberForceIJ, 
               const AccelerationMuscleInfo& ami) const
{
    double fce = fiberForceIJ[0]*ami.cosphi + fiberForceIJ[1]*ami.sinphi;
    return fce;
}


double Millard2012AccelerationMuscle::
    calcFiberForceAlongTendon(SimTK::Vec2 fiberForceIJ) const
{    
    return fiberForceIJ[0];
}

double Millard2012AccelerationMuscle::
    calcTendonStiffness(const AccelerationMuscleInfo& ami) const
{
    double fiso     = getMaxIsometricForce();
    double dFse_dtl = fiso*ami.dfseVEM_dtl;
    return dFse_dtl;
}

SimTK::Vec2 Millard2012AccelerationMuscle::
                  calcFiberStiffnessIJ( double a, 
                                        const AccelerationMuscleInfo& ami) const
{
    
    std::string caller = getName();
    caller.append(".calcFiberStiffnessIJ");

    //Gross muscle properties
    double fiso              = getMaxIsometricForce();
    
    //Relabel the variable names to make them nicer to read
    double fal      = ami.fal;
    double fv       = ami.fv;
    double fpeVEM   = ami.fpeVEM;
    double fkVEM    = ami.fkVEM;
    // double fcphiVEM = ami.fcphiVEM;
    double fibV     = ami.fibV;

    double dfal_dlce        = ami.dfal_dlce;    
    double dfpeVEM_dlce     = ami.dfpeVEM_dlce;
    double dfkVEM_dlce      = ami.dfkVEM_dlce;
    double dfcphiVEM_dlce   = ami.dfcphiVEM_dlce;
    double dfibV_dlce       = ami.dfibV_dlce;

    double sinPhi   = ami.sinphi;
    double cosPhi   = ami.cosphi;
    double dphi_dlce= ami.dphi_dlce;
    
    // I  d/dlce( fiso * ((a *  fal  *fv + fpeVEM - fkVEM)*cosPhi - fcphiVEM);    
    double DFm_DlceI = fiso*(
              (a*dfal_dlce*fv + dfpeVEM_dlce - dfkVEM_dlce + dfibV_dlce)*cosPhi 
            + (a*   fal   *fv + fpeVEM      -  fkVEM + fibV)*(-sinPhi*dphi_dlce)
              - (dfcphiVEM_dlce)
              );

    // J  d/dlce( fiso *  (a *  fal  *fv + fpeVEM - fkVEM)*sinPhi
    double DFm_DlceJ = fiso*(
         (a*dfal_dlce*fv + dfpeVEM_dlce - dfkVEM_dlce + dfibV_dlce)*sinPhi 
        + (a*   fal   *fv + fpeVEM      -   fkVEM     + fibV)*(cosPhi*dphi_dlce)             
              );

    SimTK::Vec2 DFm_DlceIJ;
    DFm_DlceIJ[0] = DFm_DlceI;
    DFm_DlceIJ[1] = DFm_DlceJ;

    return DFm_DlceIJ;
}

double Millard2012AccelerationMuscle::
    calcFiberStiffness( SimTK::Vec2 fiberForceIJ,
                        SimTK::Vec2 fiberStiffnessIJ, 
                        const AccelerationMuscleInfo& ami) const
{
    //F = Fi*cos(phi) + Fj*sin(phi)
    //dF/dlce = dFi_dlce*cos(phi) + Fi*(-sin(phi)*dphi_dlce)
    //          dFj_dlce*sin(phi) + Fj*(cos(phi)*dphi_dlce)
    double dFce_dlce =  fiberStiffnessIJ[0] * ami.cosphi 
                      + fiberForceIJ[0]     * (-ami.sinphi*ami.dphi_dlce)
                      + fiberStiffnessIJ[1] * ami.sinphi
                      + fiberForceIJ[1]     * (ami.cosphi*ami.dphi_dlce);

    return dFce_dlce;
}

double Millard2012AccelerationMuscle::
    calc_DFiberForceAT_DFiberLength(SimTK::Vec2 fiberStiffnessIJ) const
{
    return fiberStiffnessIJ[0];
}

double Millard2012AccelerationMuscle::
    calc_DFiberForceAT_DFiberLengthAT(  double dFmAT_dlce,                                       
                                        const AccelerationMuscleInfo& ami) const
{
    std::string caller = getName();
    caller.append("calcFiberStiffnessAlongTendon");

    double dlceAT_dlce = ami.dlceAT_dlce;
    SimTK_ERRCHK1_ALWAYS(dlceAT_dlce > 0,
        "Millard2012AccelerationMuscle:: calc_DFiberForceAT_DFiberLengthAT",
        "%s: Pennation angle is close to perpendicular", getName().c_str());

    //3. Compute 
    // dFmAT/dlceAT = (dFmAT/dlce)*(1/(dlceAT/dlce)) 
    //              = dFmAT/dlceAT
    double dFmAT_dlceAT = dFmAT_dlce * (1/dlceAT_dlce);

    return dFmAT_dlceAT;    
}

double Millard2012AccelerationMuscle::
    calc_DTendonForce_DFiberLength( double dFse_dtl,
                                    const AccelerationMuscleInfo& ami,
                                    std::string& caller) const
{    
    double dtl_dlce   = ami.dtl_dlce;
    double dFse_dlce = dFse_dtl*dtl_dlce;
    return dFse_dlce;
}

void Millard2012AccelerationMuscle::
    calcAccelerationMuscleInfo(AccelerationMuscleInfo& ami,
                                         double lce, 
                                         double dlce_dt,
                                         double phi,
                                         double dphi_dt,
                                         double tl,
                                         double dtl_dt,
                                         double fal,
                                         double fv,
                                         double fpe,
                                         double fk,
                                         double fcphi,
                                         double fse) const
{
    std::string caller = getName();
    caller.append("Millard2012AccelerationMuscle::"
                  "calcAccelerationMuscleInfo");

    const TendonForceLengthCurve& fseCurve
        = get_TendonForceLengthCurve();    
    const ActiveForceLengthCurve& falCurve
        = get_ActiveForceLengthCurve(); 
    // const ForceVelocityCurve& fvCurve 
    //     = get_ForceVelocityCurve(); 
    const FiberForceLengthCurve& fpeCurve 
        = get_FiberForceLengthCurve(); 
    const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve(); 
    
    // double m = get_mass();

    //Get a record of the kinematic independent variables
        ami.lce      = lce;
        ami.dlce_dt  = dlce_dt;
        ami.phi      = phi;
        ami.sinphi   = sin(phi);
        ami.cosphi   = cos(phi);
        ami.dphi_dt  = dphi_dt;
        ami.tl       = tl;
        ami.dtl_dt   = dtl_dt;
    
    //Calculated dependent kinematic quantities
        ami.dphi_dlce = m_penMdl.calc_DPennationAngle_DfiberLength(lce);

        ami.lceAT = m_penMdl.calcFiberLengthAlongTendon(ami.lce,ami.cosphi);

        ami.dlceAT_dlce = 
            m_penMdl.calc_DFiberLengthAlongTendon_DfiberLength( lce,
                                                                ami.sinphi,
                                                                ami.cosphi,
                                                                ami.dphi_dlce);

        ami.dlceAT_dt = m_penMdl.calcFiberVelocityAlongTendon(
                                                ami.lce,
                                                ami.dlce_dt,
                                                ami.sinphi,
                                                ami.cosphi,
                                                ami.dphi_dt);  

        ami.d_dphidt_dlce = 
            m_penMdl.calc_DPennationAngularVelocity_DfiberLength(lce,
                                                                 dlce_dt,
                                                                 ami.sinphi,
                                                                 ami.cosphi,
                                                                 ami.dphi_dt,
                                                                 ami.dphi_dlce);

        ami.d_dlceATdt_dlce = 
            m_penMdl.calc_DFiberVelocityAlongTendon_DfiberLength(lce,
                                                                dlce_dt,
                                                                ami.sinphi,
                                                                ami.cosphi,
                                                                dphi_dt,
                                                                ami.dphi_dlce,
                                                             ami.d_dphidt_dlce);

        ami.dtl_dlce = m_penMdl.calc_DTendonLength_DfiberLength(ami.lce,
                                                                ami.sinphi,
                                                                ami.cosphi,
                                                                ami.dphi_dlce);
    //Multipliers
        ami.fse      = fse;  
        ami.fal      = fal;  
        ami.fv       = fv;
        ami.fpe      = fpe;  
        ami.fk       = fk;   
        ami.fcphi    = fcphi;

    //Visco multiplier
    //Based on a Hunt Crossley Model where
    // f = x(1+ dx_dt^p*b) : if (1+ dx_dt^p*b) < 0, dx_dt^p*b = -1
    //   = x  + x*dx_dt*b, 
    //   = elastic (x) + visco (x*dx_dt*b)
        double bpe  = get_fiber_force_length_damping();
        double bk   = get_fiber_compressive_force_length_damping();
        double bcphi= get_fiber_compressive_force_cos_pennation_damping();
        double bse  = get_tendon_force_length_damping();
        double bfib = get_fiber_damping();

        double tendonSlackLength    = getTendonSlackLength();
        double optimalFiberLength   = getOptimalFiberLength();
        double optimalPennationAngle= getPennationAngleAtOptimalFiberLength();

        double dtlN_dtl = 1     / tendonSlackLength;
        double tlN      = tl    * dtlN_dtl;
        double dtlN_dt  = dtl_dt* dtlN_dtl;
        
        double dlceN_dlce   = 1      /optimalFiberLength;
        double lceN         = lce    *dlceN_dlce;
        double dlceN_dt     = dlce_dt*dlceN_dlce;
        
        double dlceNAT_dlceAT=1/(optimalFiberLength*cos(optimalPennationAngle)); 
        // double lceNAT        = ami.lceAT * dlceNAT_dlceAT;
        double dlceNAT_dt    = ami.dlceAT_dt * dlceNAT_dlceAT;

    
    //Multiplier partial derivatives 
        ami.dfse_dtl        = fseCurve.calcDerivative(tlN,1)  * dtlN_dtl;     
        ami.dfal_dlce       = falCurve.calcDerivative(lceN,1) * dlceN_dlce;
        ami.dfpe_dlce       = fpeCurve.calcDerivative(lceN,1) * dlceN_dlce;    
        ami.dfk_dlce        = fkCurve.calcDerivative(lceN,1)  * dlceN_dlce;     
        
        double dfcphi_dcosphi   = fcphiCurve.calcDerivative(ami.cosphi,1); 
        double dcosphi_dlce     = -ami.sinphi*ami.dphi_dlce;
        ami.dfcphi_dlce         = dfcphi_dcosphi* dcosphi_dlce;

    
    //Visco multiplier
        ami.fseV   =    ami.fse * dtlN_dt   * bse;
        ami.fpeV   =    ami.fpe * dlceN_dt  * bpe;
        ami.fkV    = -  ami.fk  * dlceN_dt  * bk;
        ami.fcphiV = - ami.fcphi* dlceNAT_dt* bcphi;
        ami.fibV   =              dlceN_dt  * bfib;

    //Viscoelastic multipliers
        ami.fseVEM      = ami.fse   + ami.fseV; 
        ami.fpeVEM      = ami.fpe   + ami.fpeV;
        ami.fkVEM       = ami.fk    + ami.fkV; 
        ami.fcphiVEM    = ami.fcphi + ami.fcphiV;


    //Visco multiplier partial derivatives
        //ami.dfse_dtl  * dtlN_dt   * bse
        //2* sqrt(ami.dfse_dtl/m) * m * dtlN_dt * bse;
        ami.dfseV_dtl    =     ami.dfse_dtl  * dtlN_dt   * bse;   
        ami.dfpeV_dlce   =     ami.dfpe_dlce * dlceN_dt  * bpe;
        ami.dfkV_dlce    =  -  ami.dfk_dlce  * dlceN_dt  * bk;
        

        //Since there is a kinematic dependence of dlceNAT_dt on the fiber
        //length, the partial derivative for fcphi is a little more involved
        

        double DdlceATdt_Dlce 
            = m_penMdl.calc_DFiberVelocityAlongTendon_DfiberLength(ami.lce,
                                                                ami.dlce_dt,
                                                                ami.sinphi,
                                                                ami.cosphi,
                                                                ami.dphi_dt,
                                                                ami.dphi_dlce,
                                                             ami.d_dphidt_dlce);

        double DdlceNATdt_Dlce = DdlceATdt_Dlce*dlceNAT_dlceAT;

        ami.dfcphiV_dlce =  - ami.dfcphi_dlce* dlceNAT_dt     * bcphi
                            - ami.fcphi      * DdlceNATdt_Dlce* bcphi;
        ami.dfibV_dlce   = 0; //Including for future upgrade ...

    //Viscoelastic multiplier partial derivatives      
        ami.dfseVEM_dtl     = ami.dfse_dtl    + ami.dfseV_dtl;  
        ami.dfpeVEM_dlce    = ami.dfpe_dlce   + ami.dfpeV_dlce;
        ami.dfkVEM_dlce     = ami.dfk_dlce    + ami.dfkV_dlce;
        ami.dfcphiVEM_dlce  = ami.dfcphi_dlce + ami.dfcphiV_dlce;

    //Viscoelastic saturation
    //    ensure that tension (compression) elements 
    //    generate only tensile forces (compressive)

        bool fseSAT = false;
        bool fpeSAT = false;
        bool fkSAT  = false;
        bool fcphiSAT = false;
 
        if(ami.fseVEM   < 0)    fseSAT   = true;
        if(ami.fpeVEM   < 0)    fpeSAT   = true;
        if(ami.fkVEM    < 0)    fkSAT    = true;
        if(ami.fcphiVEM < 0)    fcphiSAT = true;
  
        /*
        ami.fseV   =    ami.fse * dtlN_dt   * bse;
        ami.fpeV   =    ami.fpe * dlceN_dt  * bpe;
        */

        //Tensile elements
        if(fseSAT){     ami.fseV        = - ami.fse; 
                        ami.fseVEM      =   ami.fse + ami.fseV; //0
                        ami.dfseV_dtl   = - ami.dfse_dtl;
                        ami.dfseVEM_dtl =   ami.dfse_dtl + ami.dfseV_dtl; //0
        }
        if(fpeSAT){     ami.fpeV        = - ami.fpe;
                        ami.fpeVEM      =   ami.fpe + ami.fpeV; //0
                        ami.dfpeV_dlce  = - ami.dfpe_dlce;
                        ami.dfpeVEM_dlce=   ami.dfpe_dlce + ami.dfpeV_dlce;//0
        }

        /*
        ami.fkV    = -  ami.fk  * dlceN_dt  * bk;
        ami.fcphiV = - ami.fcphi* dlceNAT_dt* bcphi;
        */
        //Compressive elements: note the opposite signs
        if(fkSAT) {     ami.fkV         = -ami.fk;
                        ami.fkVEM       =  ami.fk + ami.fkV; //0
                        ami.dfkV_dlce   = -ami.dfk_dlce;
                        ami.dfkVEM_dlce =  ami.dfk_dlce + ami.dfkV_dlce; //0
        }

        if(fcphiSAT){   ami.fcphiV          = -ami.fcphi;
                        ami.fcphiVEM        =  ami.fcphi + ami.fcphiV;//0
                        ami.dfcphiV_dlce    = -ami.dfcphi_dlce;
                        ami.dfcphiVEM_dlce  = ami.dfcphi_dlce +ami.dfcphiV_dlce;
                        //0
        }

}

