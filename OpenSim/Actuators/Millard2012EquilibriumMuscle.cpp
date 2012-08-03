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
 * Author(s): Matthew Millard                                                 *
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
#include "Millard2012EquilibriumMuscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <iostream>

#include <SimTKcommon/internal/ExceptionMacros.h>

//=============================================================================
// STATICS
//=============================================================================



using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string Millard2012EquilibriumMuscle::
    STATE_ACTIVATION_NAME = "activation";
const string Millard2012EquilibriumMuscle::
    STATE_FIBER_LENGTH_NAME = "fiber_length"; 

const static int MLIfse     = 0;
const static int MLIfk      = 1;
const static int MLIfcphi   = 2;
const static int MLIfkPE    = 3;
const static int MLIfcphiPE = 4;

//=============================================================================
// PROPERTY MANAGEMENT
//=============================================================================
void Millard2012EquilibriumMuscle::setNull()
{
}


void Millard2012EquilibriumMuscle::constructProperties()
{
    constructProperty_default_activation(1.0);

    constructProperty_default_fiber_length(getOptimalFiberLength());
    
    constructProperty_MuscleFirstOrderActivationDynamicModel(
        MuscleFirstOrderActivationDynamicModel());

    constructProperty_ActiveForceLengthCurve(
        ActiveForceLengthCurve());

    constructProperty_ForceVelocityInverseCurve(
        ForceVelocityInverseCurve());

    constructProperty_FiberForceLengthCurve(
        FiberForceLengthCurve());

    constructProperty_TendonForceLengthCurve(
        TendonForceLengthCurve());

    constructProperty_FiberCompressiveForceLengthCurve(
        FiberCompressiveForceLengthCurve());

    constructProperty_FiberCompressiveForceCosPennationCurve(
        FiberCompressiveForceCosPennationCurve());       
}

void Millard2012EquilibriumMuscle::buildMuscle()
{
    double optFibLen = getOptimalFiberLength();
    double optPenAng = getPennationAngleAtOptimalFiberLength();
    std::string caller = getName();
    caller.append(".buildMuscle()");

    penMdl = MuscleFixedWidthPennationModel(optFibLen,optPenAng,caller);
    

    //Ensure all of the object names are uptodate
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
    tmp.append("_ForceVelocityInverseCurve");
    ForceVelocityInverseCurve& fvInvCurve = upd_ForceVelocityInverseCurve();
    fvInvCurve.setName(tmp);

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


    //For initialization only we need a force-velocity curve:
    double concentricSlope      = fvInvCurve.getConcentricMinSlope();
    double isometricSlope       = fvInvCurve.getIsometricMaxSlope();
    double eccentricSlope       = fvInvCurve.getEccentricMinSlope();
    double concentricCurviness  = fvInvCurve.getConcentricCurviness();
    double eccentricCurviness   = fvInvCurve.getEccentricCurviness();
    double eccentricForceMax    = 
        fvInvCurve.getMaxEccentricVelocityForceMultiplier();

     fvCurve = ForceVelocityCurve(  concentricSlope,
                                    isometricSlope,
                                    eccentricSlope,
                                    eccentricForceMax,
                                    concentricCurviness,
                                    eccentricCurviness,
                                    caller);

    setObjectIsUpToDateWithProperties();

}

void Millard2012EquilibriumMuscle::ensureMuscleUpToDate() const
{
    if(isObjectUpToDateWithProperties() == false)
    {
        Millard2012EquilibriumMuscle* mthis = 
            const_cast<Millard2012EquilibriumMuscle*>(this);
        mthis->buildMuscle(); 
    }
}


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================

Millard2012EquilibriumMuscle::Millard2012EquilibriumMuscle()            
{    
    setNull();
    constructProperties();
    ensureMuscleUpToDate();
}

Millard2012EquilibriumMuscle::
Millard2012EquilibriumMuscle(const std::string &aName,  double aMaxIsometricForce,
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
    
    ensureMuscleUpToDate();
}

//=============================================================================
// Model Component Interface
//=============================================================================
void Millard2012EquilibriumMuscle::connectToModel(Model& model)
{
    Super::connectToModel(model);
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    ensureMuscleUpToDate();

    addStateVariable(STATE_ACTIVATION_NAME);
    addStateVariable(STATE_FIBER_LENGTH_NAME);

    double value = 0.0;
	addCacheVariable(   STATE_ACTIVATION_NAME+"_deriv", 
                        value, 
                        SimTK::Stage::Dynamics);

    //Ask Ajay: why should this be a dynamics level quantity?
	addCacheVariable(   STATE_FIBER_LENGTH_NAME+"_deriv", 
                        value, 
                        SimTK::Stage::Dynamics);
}

void Millard2012EquilibriumMuscle::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

    ensureMuscleUpToDate();

    Millard2012EquilibriumMuscle* mthis = 
        const_cast<Millard2012EquilibriumMuscle*>(this);

    mthis->setActivation(s, getDefaultActivation());
    mthis->setFiberLength(s, getDefaultFiberLength());

}
    
void Millard2012EquilibriumMuscle::
    setPropertiesFromState(const SimTK::State& s)
{
    Super::setPropertiesFromState(s);
    
    ensureMuscleUpToDate();

    const MuscleFirstOrderActivationDynamicModel& actMdl 
        = get_MuscleFirstOrderActivationDynamicModel();

    setDefaultActivation(
        actMdl.clampActivation(
        getStateVariable(s,STATE_ACTIVATION_NAME)));
    setDefaultFiberLength(
        actMdl.clampActivation(
        getStateVariable(s,STATE_FIBER_LENGTH_NAME)));
}

SimTK::Vector Millard2012EquilibriumMuscle::
computeStateVariableDerivatives(const SimTK::State& s) const 
{
    SimTK::Vector derivs(getNumStateVariables(), 0.);
    if (!isDisabled(s)) {
        derivs[0] = getActivationRate(s);
        derivs[1] = getFiberVelocity(s);
    }
    return derivs;
}

//=============================================================================
// STATE RELATED GET FUNCTIONS
//=============================================================================

double Millard2012EquilibriumMuscle::getDefaultActivation() const
{
    ensureMuscleUpToDate();
    return get_default_activation();
}

double Millard2012EquilibriumMuscle::getDefaultFiberLength() const
{
    ensureMuscleUpToDate();
    return get_default_fiber_length();
}

double Millard2012EquilibriumMuscle::
    getActivationRate(const SimTK::State& s) const
{
    ensureMuscleUpToDate();     
    return calcActivationRate(s);
}

double Millard2012EquilibriumMuscle::
    getFiberVelocity(const SimTK::State& s) const
{
    ensureMuscleUpToDate();
    FiberVelocityInfo fvi = getFiberVelocityInfo(s);
    return fvi.fiberVelocity;
}

Array<std::string> Millard2012EquilibriumMuscle::getStateVariableNames() const
{
    ensureMuscleUpToDate();
    Array<std::string> stateVariableNames = 
        ModelComponent::getStateVariableNames();
	
	for(int i=0; i<stateVariableNames.getSize(); ++i){
		stateVariableNames[i] = getName()+"."+stateVariableNames[i];
	}
	return stateVariableNames;
}

SimTK::SystemYIndex Millard2012EquilibriumMuscle::
    getStateVariableSystemIndex(const std::string &stateVariableName) const
{
    ensureMuscleUpToDate();
    unsigned int start = stateVariableName.find(".");
	unsigned int end = stateVariableName.length();
	
	if(start == end)
		return ModelComponent::getStateVariableSystemIndex(stateVariableName);
	else{
		string localName = stateVariableName.substr(++start, end-start);
		return ModelComponent::getStateVariableSystemIndex(localName);
	}
}


double Millard2012EquilibriumMuscle::
    getStateVariableDeriv(const SimTK::State& s, 
                          const std::string &aStateName) const
{
	return getCacheVariable<double>(s, aStateName + "_deriv");
}

//=============================================================================
// STATE RELATED SET FUNCTIONS
//=============================================================================

void Millard2012EquilibriumMuscle::setDefaultActivation(double activation)
{
    set_default_activation(activation);
}

void Millard2012EquilibriumMuscle::setDefaultFiberLength(double fiberLength)
{
    set_default_fiber_length(fiberLength);
}

void Millard2012EquilibriumMuscle::
    setActivation(SimTK::State& s, double activation) const
{
    const MuscleFirstOrderActivationDynamicModel& actMdl 
        = get_MuscleFirstOrderActivationDynamicModel();
    double clampedActivation = actMdl.clampActivation(activation);

    setStateVariable(s, STATE_ACTIVATION_NAME, clampedActivation);    
    markCacheVariableInvalid(s,"velInfo");
    markCacheVariableInvalid(s,"dynamicsInfo");
    
}

void Millard2012EquilibriumMuscle::
    setFiberLength(SimTK::State& s, double fiberLength) const
{
    setStateVariable(s, STATE_FIBER_LENGTH_NAME, fiberLength);
    markCacheVariableInvalid(s,"lengthInfo");
    markCacheVariableInvalid(s,"velInfo");
    markCacheVariableInvalid(s,"dynamicsInfo");
    
}

void Millard2012EquilibriumMuscle::
    setStateVariableDeriv(  const SimTK::State& s, 
                            const std::string &aStateName, 
                            double aValue) const
{
	double& cacheVariable = updCacheVariable<double>(s, aStateName + "_deriv");
	cacheVariable = aValue;
	markCacheVariableValid(s, aStateName + "_deriv");
}

//=============================================================================
// GET
//=============================================================================


double Millard2012EquilibriumMuscle::
    getFiberCompressiveForceLengthMultiplier(SimTK::State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    return mli.userDefinedLengthExtras[MLIfk];
}

double Millard2012EquilibriumMuscle::
    getFiberCompressiveForceCosPennationMultiplier(SimTK::State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    return mli.userDefinedLengthExtras[MLIfcphi];
}


double Millard2012EquilibriumMuscle::
    getTendonForceMultiplier(SimTK::State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    return mli.userDefinedLengthExtras[MLIfse];
}

const MuscleFirstOrderActivationDynamicModel& Millard2012EquilibriumMuscle::
    getActivationModel() const
{
    ensureMuscleUpToDate();    
    return get_MuscleFirstOrderActivationDynamicModel();
}

const MuscleFixedWidthPennationModel& Millard2012EquilibriumMuscle::
    getPennationModel() const
{
    ensureMuscleUpToDate();
    return penMdl;
}

const ActiveForceLengthCurve& Millard2012EquilibriumMuscle::
    getActiveForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_ActiveForceLengthCurve();    
}

const ForceVelocityInverseCurve& Millard2012EquilibriumMuscle::
    getForceVelocityInverseCurve() const
{
    ensureMuscleUpToDate();
    return get_ForceVelocityInverseCurve();
}

const FiberForceLengthCurve& Millard2012EquilibriumMuscle::
    getFiberForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_FiberForceLengthCurve();
}

const TendonForceLengthCurve& Millard2012EquilibriumMuscle::
    getTendonForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_TendonForceLengthCurve();
}

const FiberCompressiveForceLengthCurve& Millard2012EquilibriumMuscle::
    getFiberCompressiveForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_FiberCompressiveForceLengthCurve();
}

const FiberCompressiveForceCosPennationCurve& Millard2012EquilibriumMuscle::
    getFiberCompressiveForceCosPennationCurve() const
{
    ensureMuscleUpToDate();
    return get_FiberCompressiveForceCosPennationCurve();
}

double Millard2012EquilibriumMuscle::
    getFiberStiffnessAlongTendon(const SimTK::State& s) const
{
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    return mdi.fiberStiffnessAlongTendon;
}

//=============================================================================
// SET
//=============================================================================


void Millard2012EquilibriumMuscle::setActivationModel(
        MuscleFirstOrderActivationDynamicModel& aActivationMdl)
{
    set_MuscleFirstOrderActivationDynamicModel(aActivationMdl);
}
void Millard2012EquilibriumMuscle::setActiveForceLengthCurve(
        ActiveForceLengthCurve& aActiveForceLengthCurve)
{
    set_ActiveForceLengthCurve(aActiveForceLengthCurve);
}

void Millard2012EquilibriumMuscle::setForceVelocityInverseCurve(
        ForceVelocityInverseCurve& aForceVelocityInverseCurve)
{   
    set_ForceVelocityInverseCurve(aForceVelocityInverseCurve);
}

void Millard2012EquilibriumMuscle::setFiberForceLengthCurve(
        FiberForceLengthCurve& aFiberForceLengthCurve)
{
    set_FiberForceLengthCurve(aFiberForceLengthCurve);
}

void Millard2012EquilibriumMuscle::setTendonForceLengthCurve(
        TendonForceLengthCurve& aTendonForceLengthCurve)
{
    set_TendonForceLengthCurve(aTendonForceLengthCurve);
}

void Millard2012EquilibriumMuscle::setFiberCompressiveForceLengthCurve(
        FiberCompressiveForceLengthCurve& aFiberCompressiveForceLengthCurve)
{
    set_FiberCompressiveForceLengthCurve(
        aFiberCompressiveForceLengthCurve);
}

void Millard2012EquilibriumMuscle::setFiberCompressiveForceCosPennationCurve(
        FiberCompressiveForceCosPennationCurve& 
        aFiberCompressiveForceCosPennationCurve)
{
    set_FiberCompressiveForceCosPennationCurve(
        aFiberCompressiveForceCosPennationCurve);
}

//==============================================================================
// Protected Useful Functions
//==============================================================================

void Millard2012EquilibriumMuscle::
postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	GeometryPath& path = upd_GeometryPath();

	path.postScale(s, aScaleSet);

	if (path.getPreScaleLength(s) > 0.0)
	{
		double scaleFactor = getLength(s) / path.getPreScaleLength(s);
		upd_optimal_fiber_length() *= scaleFactor;
		upd_tendon_slack_length() *= scaleFactor;
		path.setPreScaleLength(s, 0.0) ;
	}
}



//==============================================================================
// XXXXXXXXXXXXXXXXXXXX  START OF TO BE DEPRECATED   XXXXXXXXXXXXXXXXXXXXXXXXXXX
//==============================================================================
double Millard2012EquilibriumMuscle::
calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                       double aActivation) const
{
        string caller = getName();
        caller.append(  "Millard2012EquilibriumMuscle::"
                        "calcInextensibleTendonActiveFiberForce");
        double inextensibleTendonActiveFiberForce = 0;

        double muscleLength = getLength(s);
        double muscleVelocity = getLengtheningSpeed(s);
        double tendonSlackLength = getTendonSlackLength();
        double tendonVelocity = 0.0; //Inextensible tendon;

        double fiberLength  = penMdl.calcFiberLength(muscleLength,
                                             tendonSlackLength,caller);
        
        if(fiberLength > penMdl.getMinimumFiberLength()){
            double phi      = penMdl.calcPennationAngle(fiberLength,caller);
        
            double fiberVelocity   = penMdl.calcFiberVelocity(fiberLength,
                                            sin(phi),cos(phi),
                                            muscleLength,tendonSlackLength,
                                         muscleVelocity,tendonVelocity,caller);

            inextensibleTendonActiveFiberForce = 
                calcActiveFiberForceAlongTendon(    aActivation,
                                                    fiberLength,
                                                    fiberVelocity);
        }


        return inextensibleTendonActiveFiberForce;
}

double Millard2012EquilibriumMuscle::
            calcActiveFiberForceAlongTendon(double activation, 
                                            double fiberLength, 
                                            double fiberVelocity) const
{
    string caller = getName();
    caller.append("::MillardEquilibriumMuscle::calcActiveFiberForceAlongTendon");

    double activeFiberForce = 0;    
    double clampedFiberLength = penMdl.clampFiberLength(fiberLength);

    //If the fiber is in a legal range, compute the force its generating
    if(fiberLength > penMdl.getMinimumFiberLength()){

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
            = get_ActiveForceLengthCurve();

        //Evaluate the force active length and force velocity multipliers
        double fal  = falCurve.calcValue(normFiberLength);
        double fv   = fvCurve.calcValue(normFiberVelocity);
        double fiso = getMaxIsometricForce();

        //Evaluate the pennation angle
        double phi = penMdl.calcPennationAngle(fiberLength,caller);

        //Compute the active fiber force 
        activeFiberForce = fiso * clampedActivation * fal * fv * cos(phi);
    }
    //Compute the active fiber force
    

    return activeFiberForce;
}

//==============================================================================
// XXXXXXXXXXXXXXXXXXXX  END OF TO BE DEPRECATED   XXXXXXXXXXXXXXXXXXXXXXXXXXX
//==============================================================================

//==============================================================================
// Muscle.h Interface
//==============================================================================
double  Millard2012EquilibriumMuscle::
    computeActuation(const SimTK::State& s) const
{    
    ensureMuscleUpToDate();
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setForce(s,         mdi.tendonForce);
    return( mdi.tendonForce );
}



void Millard2012EquilibriumMuscle::
    computeInitialFiberEquilibrium(SimTK::State& s) const
{
    try{

        ensureMuscleUpToDate();
        //Initialize activation to the users desired setting
        const MuscleFirstOrderActivationDynamicModel& actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();
        setActivation(s, actMdl.clampActivation(getActivation(s)));

        //Initialize the multibody system to the initial state vector
        setFiberLength(s, getOptimalFiberLength());

        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

        //Compute an initial muscle state that develops the desired force and
        //shares the muscle stretch between the muscle fiber and the tendon 
        //according to their relative stiffness.    
        double activation = actMdl.clampActivation(getActivation(s));

        //tol is the desired tolerance in Newtons
        double tol = 1e-8*getMaxIsometricForce();  //Should this be user settable?
        if(tol < SimTK::SignificantReal*10){
            tol = SimTK::SignificantReal*10;
        }

        int maxIter = 200;                          //Should this be user settable?    
        SimTK::Vector soln = initMuscleState(s,activation, tol, maxIter);

        int flag_status    = (int)soln[0];
        double solnErr        = soln[1];
        double iterations     = (int)soln[2];
        double fiberLength    = soln[3];
        double passiveForce   = soln[4];
        double tendonForce    = soln[5];

        switch(flag_status){

            case 0: //converged, all is normal
            {
                setForce(s,tendonForce);
		        setFiberLength(s,fiberLength);
            }break;

            case 1: //lower fiber length bound hit
            {
                setForce(s,tendonForce);
                setFiberLength(s,fiberLength);
            
                std::string muscleName = getName();            
                printf( "\n\nMillard2012EquilibriumMuscle Initialization Message:"
                        " %s is at its minimum length of %f",
                        muscleName.c_str(), penMdl.getMinimumFiberLength());
            }break;

            case 2: //Maximum number of iterations exceeded.
            {
                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());

                std::string muscleName = getName();
                std::string fcnName = "\n\nWARNING: Millard2012EquilibriumMuscle::"
                                 "computeInitialFiberEquilibrium(SimTK::State& s)";
                    char msgBuffer[1000];
                    int n = sprintf(msgBuffer,
                        "WARNING: No suitable initial conditions found for\n"
                        "  %s: \n"
                        "  by %s \n"
                        "Continuing with an initial fiber force and "
                            "length of 0 and %f\n"
                        "    Here is a report from the routine:\n \n"
                        "        Solution Error      : %f > tol (%f) \n"
                        "        Newton Iterations   : %d of max. iterations (%d)\n"
                        "    Check that the initial activation is valid,"
                            " and that the whole \n"
                        "    length doesn't produce a pennation"
                            " angle of 90 degrees, nor a fiber\n"
                        "    length less than 0:\n"
                        "        Activation          : %f \n" 
                        "        Whole muscle length : %f \n\n", 
                        muscleName.c_str(),
                        fcnName.c_str(), 
                        penMdl.getOptimalFiberLength(),
                        abs(solnErr),
                        tol,
                        iterations,
                        maxIter,
                        activation, 
                        fiberLength);

                    cerr << msgBuffer << endl;
        
            }break;

            default:
                std::string muscleName = getName();            
                printf(
                    "\n\nWARNING: Millard2012EquilibriumMuscle Initialization:"
                    " %s invalid error flag, setting tendon force to 0.0 and "
                    "the fiber length to the optimal fiber length",
                        muscleName.c_str());

                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());
        }

    }catch (const std::exception& e) { 
        //If the initialization routine fails in some unexpected way, tell
        //the user and continue with some valid initial conditions
        cerr << "\n\nWARNING: Millard2012EquilibriumMuscle initialization"
                " exception caught:" << endl;
        cerr << e.what() << endl;
        
        cerr << "    Continuing with initial tendon force of 0 " << endl;
        cerr << "    and a fiber length equal to the optimal fiber length ..." 
             << endl;

        setForce(s,0);
		setFiberLength(s,getOptimalFiberLength());

    }                     
}

void Millard2012EquilibriumMuscle::calcMuscleLengthInfo(const SimTK::State& s, 
                                               MuscleLengthInfo& mli) const
{    
    ensureMuscleUpToDate();
    double simTime = s.getTime(); //for debugging purposes

    //Get whole muscle properties
    double maxIsoForce      = getMaxIsometricForce();
    double optFiberLength   = getOptimalFiberLength();
    double mclLength        = getLength(s);
    double tendonSlackLen   = getTendonSlackLength();
    std::string caller      = getName();
    caller.append(".calcMuscleLengthInfo");

    //Get muscle model specific properties
    const TendonForceLengthCurve& fseCurve 
        = get_TendonForceLengthCurve(); 
    const FiberForceLengthCurve& fpeCurve 
        = get_FiberForceLengthCurve(); 
    const ActiveForceLengthCurve& falCurve
        = get_ActiveForceLengthCurve(); 
    const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve(); 

    //Populate the output struct
    mli.fiberLength       = getStateVariable(s, STATE_FIBER_LENGTH_NAME); 

    mli.normFiberLength   = mli.fiberLength/optFiberLength;
    mli.pennationAngle    = penMdl.calcPennationAngle(mli.fiberLength,caller);
    mli.cosPennationAngle = cos(mli.pennationAngle);
    mli.sinPennationAngle = sin(mli.pennationAngle);

    mli.fiberLengthAlongTendon = mli.fiberLength*mli.cosPennationAngle;
    
    mli.tendonLength      = penMdl.calcTendonLength(mli.cosPennationAngle,
                                                   mli.fiberLength,mclLength);
    mli.normTendonLength  = mli.tendonLength / tendonSlackLen;
    mli.tendonStrain      = mli.normTendonLength -  1.0;
        
    //Note the curves return normalized area. Each area must be un-normalized   
    mli.fiberPotentialEnergy =  fpeCurve.calcIntegral(mli.normFiberLength)
                                *(optFiberLength*maxIsoForce);

    mli.tendonPotentialEnergy=  fseCurve.calcIntegral(mli.normTendonLength)
                                *(tendonSlackLen*maxIsoForce);

    double compForceLengthPE=   fkCurve.calcIntegral(mli.normFiberLength)
                                *(optFiberLength*maxIsoForce);

    //Only the force dimension is normalized
    double compForceCosPennationPE = 
        fcphiCurve.calcIntegral(mli.cosPennationAngle)*(maxIsoForce);

    mli.musclePotentialEnergy=  mli.fiberPotentialEnergy 
                              + mli.tendonPotentialEnergy
                              + compForceLengthPE
                              + compForceCosPennationPE;

    mli.fiberPassiveForceLengthMultiplier= 
        fpeCurve.calcValue(mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier = 
        falCurve.calcValue(mli.normFiberLength);
    
   

    double tendonForceLengthMultiplier=fseCurve.calcValue(mli.normTendonLength);

    //Put in the additional length related terms that are specific to this
    //particular muscle model.
    mli.userDefinedLengthExtras.resize(5);



    mli.userDefinedLengthExtras[MLIfse] =tendonForceLengthMultiplier;
    mli.userDefinedLengthExtras[MLIfk] =fkCurve.calcValue(mli.normFiberLength);
    mli.userDefinedLengthExtras[MLIfcphi] =fcphiCurve.calcValue(mli.cosPennationAngle);
    mli.userDefinedLengthExtras[MLIfkPE] =compForceLengthPE;
    mli.userDefinedLengthExtras[MLIfcphiPE] =compForceCosPennationPE;

}


void Millard2012EquilibriumMuscle::calcFiberVelocityInfo(const SimTK::State& s, 
                                               FiberVelocityInfo& fvi) const
{
    ensureMuscleUpToDate();
    double simTime = s.getTime(); //for debugging purposes


    //Get the quantities that we've already computed
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);

    //Get the static properties of this muscle
        double mclLength      = getLength(s);
        double tendonSlackLen = getTendonSlackLength();
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

    //1. Get MuscleLengthInfo information
    const MuscleFirstOrderActivationDynamicModel& actMdl 
        = get_MuscleFirstOrderActivationDynamicModel();

    double a    = actMdl.clampActivation(
        getStateVariable(s, STATE_ACTIVATION_NAME));

    double lce  = mli.fiberLength;
    double phi  = mli.pennationAngle;
    double cosphi=mli.cosPennationAngle;
    double sinphi = mli.sinPennationAngle;

    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;
    double fse  = mli.userDefinedLengthExtras[MLIfse];
    double fk   = mli.userDefinedLengthExtras[MLIfk];
    double fcphi= mli.userDefinedLengthExtras[MLIfcphi];

    
    //2. Compute fv - but check for singularities first
    SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::SignificantReal, fcnName.c_str(),
        "%s: Pennation angle is 90 degrees, and is causing a singularity", 
        muscleName.c_str());
    SimTK_ERRCHK1_ALWAYS(a > SimTK::SignificantReal, fcnName.c_str(),
        "%s: Activation is 0, and is causing a singularity", 
        muscleName.c_str());
    SimTK_ERRCHK1_ALWAYS(fal > SimTK::SignificantReal, fcnName.c_str(),
        "%s: The active force length curve value is 0,"
        " and is causing a singularity", muscleName.c_str());

    double fv = ( ((fse+fcphi)/cosphi) - (fpe-fk) ) / (a*fal);

    //3. Evaluate the inverse force velocity curve        
    const ForceVelocityInverseCurve& fvInvCurve 
        = get_ForceVelocityInverseCurve(); 

    double dlceN = fvInvCurve.calcValue(fv);
    double dlce  = dlceN*getMaxContractionVelocity()*optFiberLen;
    
    //7. Compute the other related velocity components
    //double dlceAT = penMdl.
    double tanPhi = tan(phi);
    double dphidt    = penMdl.calcPennationAngularVelocity(tanPhi,lce,
                                                            dlce   ,caller);
    double dmcldt = getLengtheningSpeed(s);
    double dtl       = penMdl.calcTendonVelocity(cosphi,sinphi,dphidt,
                                                    lce,  dlce,dmcldt);

    //Populate the struct;
    fvi.fiberVelocity               = dlce;
    fvi.fiberVelocityAlongTendon    = penMdl.calcFiberVelocityAlongTendon(lce,
                                                    dlce,sinphi,cosphi, dphidt);
    fvi.normFiberVelocity           = dlceN;

    fvi.pennationAngularVelocity    = dphidt;

    fvi.tendonVelocity              = dtl;
    fvi.normTendonVelocity = dtl/getTendonSlackLength();

    fvi.fiberForceVelocityMultiplier = fv;

   
    
}



void Millard2012EquilibriumMuscle::calcMuscleDynamicsInfo(const SimTK::State& s, 
                                               MuscleDynamicsInfo& mdi) const
{
        ensureMuscleUpToDate();
        double simTime = s.getTime(); //for debugging purposes

    //Get the quantities that we've already computed
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
        const FiberVelocityInfo &mvi = getFiberVelocityInfo(s);        
        

    //Get the properties of this muscle
        double mclLength      = getLength(s);
        double tendonSlackLen = getTendonSlackLength();
        double optFiberLen    = getOptimalFiberLength();
        double fiso           = getMaxIsometricForce();
        double penHeight      = penMdl.getParallelogramHeight();
        const TendonForceLengthCurve& fseCurve 
            = get_TendonForceLengthCurve();
    //Prep strings that will be useful to make sensible exception messages
        std::string muscleName = getName();
        std::string fcnName     = ".calcMuscleDynamicsInfo";
        std::string caller      = muscleName;        
        caller.append(fcnName);

    //=========================================================================
    // Compute required quantities
    //=========================================================================

    //1. Get fiber/tendon kinematic information
    const MuscleFirstOrderActivationDynamicModel& actMdl 
        = get_MuscleFirstOrderActivationDynamicModel();

    double a    = actMdl.clampActivation(
        getStateVariable(s, STATE_ACTIVATION_NAME));

    double lce      = mli.fiberLength;
    double lceN     = lce/optFiberLen;
    double dlce     = mvi.fiberVelocity;
    double phi      = mli.pennationAngle;
    double cosPhi   = mli.cosPennationAngle;
    double sinPhi   = mli.sinPennationAngle;

    double tl   = mli.tendonLength; 
    double dtl  = mvi.tendonVelocity;
    double tlN  = mli.normTendonLength;
   
    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;
    double fv   = mvi.fiberForceVelocityMultiplier;    
    double fse  = mli.userDefinedLengthExtras[MLIfse];
    double fk   = mli.userDefinedLengthExtras[MLIfk];
    double fcphi= mli.userDefinedLengthExtras[MLIfcphi];

    //Compute the stiffness of the muscle fiber
    SimTK_ERRCHK1_ALWAYS(lce > SimTK::SignificantReal, fcnName.c_str(),
        "%s: The muscle fiber has a length of 0, and is causing a singularity", 
        muscleName.c_str());
    SimTK_ERRCHK1_ALWAYS(cosPhi > SimTK::SignificantReal, fcnName.c_str(),
        "%s: Pennation angle is 90 degrees, and is causing a singularity", 
        muscleName.c_str());

    double Fm    = calcFiberForce(fiso,a,fal,fv,fpe,fk,fcphi,cosPhi);
    
    double FmAT  = calcFiberForceAlongTendon(fiso,a,fal,fv,fpe,fk,fcphi,cosPhi);

    double dFm_dlce = calcFiberStiffness(fiso,a,fal,fv,fpe,fk,fcphi,sinPhi,
                                         cosPhi,lce,lceN,optFiberLen);

    double dFmAT_dlceAT=calc_DFiberForceAT_DFiberLengthAT(  dFm_dlce,sinPhi,
                                                            cosPhi,lce);   

    //Compute the stiffness of the tendon
    double dFt_dtl    = fseCurve.calcDerivative(tlN,1)*(fiso/tendonSlackLen);

    //Compute the stiffness of the whole muscle/tendon complex

    double Ke = 0;
    if (abs(dFmAT_dlceAT*dFt_dtl)>0)
        Ke = (dFmAT_dlceAT*dFt_dtl)/(dFmAT_dlceAT+dFt_dtl);

    
    mdi.activation                   = a;
    mdi.fiberForce                   = Fm; 
    mdi.fiberForceAlongTendon        = FmAT;
    mdi.normFiberForce               = Fm/fiso;
    mdi.activeFiberForce             = a*fal*fv*fiso;
    mdi.passiveFiberForce            = ((fpe-fk)-fcphi*cosPhi)*fiso;
                                     
    mdi.tendonForce                  = fse*fiso;
    mdi.normTendonForce              = fse;
                                     
    mdi.fiberStiffness               = dFm_dlce;
    mdi.fiberStiffnessAlongTendon    = dFmAT_dlceAT;
    mdi.tendonStiffness              = dFt_dtl;
    mdi.muscleStiffness              = Ke;
                                     
    //Check that the derivative of system energy less work is zero within
    //a reasonable numerical tolerance. Throw an exception if this is not true
    
    double dphidt= mvi.pennationAngularVelocity;
    
    double dFibKPEdt    = -(fk)*fiso*dlce;
    double dFibPEdt     = (fpe)*fiso*dlce;
    double dFibCPhiPEdt = -fcphi*fiso*(dlce*cosPhi -lce*sinPhi*dphidt);
    double dTdnPEdt     = fse*fiso*dtl;

    double dFibWdt      = -mdi.activeFiberForce*mvi.fiberVelocity;
    double dmcldt       = getLengtheningSpeed(s);
    double dBoundaryWdt = mdi.tendonForce * dmcldt;


    double dSysEdt = (dFibKPEdt + dFibPEdt + dTdnPEdt + dFibCPhiPEdt)
                 - dFibWdt - dBoundaryWdt;
    double tol = sqrt(SimTK::Eps);
    
    /////////////////////////////
    //Populate the power entries
    /////////////////////////////
    mdi.fiberActivePower    =   dFibWdt;
    mdi.fiberPassivePower   =   -(dFibKPEdt + dFibPEdt + dFibCPhiPEdt); 
    mdi.tendonPower         =   -dTdnPEdt;       
    mdi.musclePower         =   -dBoundaryWdt;


    //For debugging purposes
    //if(abs(dSysEdt) >= tol){
    //    printf("KE+PE-W Tol Violation at time %f, by %f \n",simTime,dSysEdt);
    //    tol = sqrt(SimTK::Eps);
    //}
    
    //if(abs(tmp) > tol)
    //    printf("%s: d/dt(system energy-work) > tol, (%f > %f) at time %f",
     //           fcnName.c_str(), tmp, tol, (double)s.getTime());
    

}




//==============================================================================
// Numerical Guts: Initialization
//==============================================================================
SimTK::Vector Millard2012EquilibriumMuscle::
    initMuscleState(SimTK::State& s, double aActivation, 
                              double aSolTolerance, int aMaxIterations) const
{
    ensureMuscleUpToDate();   
    std::string caller = getName();
    caller.append(".initMuscleState");
    //results vector format
    //1: flag (0 = converged,
    //         1 = diverged,
    //         2= no solution due to singularity:length 0, 
    //         3= no solution due to pennation angle singularity    
    //2: solution error (N)
    //3: iterations
    //4: fiber length (m)
    //5: passive force (N)
    //6: tendon force (N)
    SimTK::Vector results = SimTK::Vector(6);

    //I'm using smaller variable names here to make it possible to write out 
    //lengthy equations
    double ma = aActivation;
    double ml = getLength(s);
    double dml= getLengtheningSpeed(s);

    //Shorter version of the constants
    double tsl = getTendonSlackLength();
    double ofl = getOptimalFiberLength();

    double ophi= getPennationAngleAtOptimalFiberLength();
    double penHeight = penMdl.getParallelogramHeight();
    double fiso= getMaxIsometricForce();
    double vmax = getMaxContractionVelocity();//getPropertyValue<double>(VmaxName);

    //Get muscle model specific properties
    const TendonForceLengthCurve& fseCurve 
        = get_TendonForceLengthCurve(); 
    const FiberForceLengthCurve& fpeCurve 
        = get_FiberForceLengthCurve(); 
    const ActiveForceLengthCurve& falCurve
        = get_ActiveForceLengthCurve(); 
    const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve(); 
    const ForceVelocityInverseCurve& fvInvCurve 
        = get_ForceVelocityInverseCurve(); 

   
    //Shorter version of normalized muscle multipliers

    double fse = 0; //Normalized tendon (series element) force
    double fal = 0; //Normalized active force length multiplier
    double fv  = 0; //Normalized force-velocity multiplier
    double fvInv=0; //Normalized inverse force-velocity factor
    double fpe = 0; //Normalized parallel element force
    double fk  = 0; //Normalized fiber compressive force
    double fcphi=0; //Normalized pennation compressive force


    //*******************************
    //Position level
    double lce = 0;
    double tl  = getTendonSlackLength()*1.01;

    lce = penMdl.calcFiberLength( ml, tl, caller);        

    double phi      = penMdl.calcPennationAngle(lce,caller);
    double cosphi   = cos(phi);
    double sinphi   = sin(phi);       
    double tlN  = tl/tsl;
    double lceN = lce/ofl;
    
    //Velocity level
    double dtl    = 0;
    

    double dlce   = penMdl.calcFiberVelocity(lce, sinphi, cosphi, 
                                              ml, tl, dml, dtl, caller);
    double dlceN  = dlce/(vmax*ofl);
    
    double dphi   = penMdl.calcPennationAngularVelocity(tan(phi),lce,dlce,
                                                                    caller);

    double dlceAT = penMdl.calcFiberVelocityAlongTendon(lce,dlce,sinphi,cosphi,
                                                        dphi);

    //*******************************
    //Internal Kinematics Related Variables for the loop
    double dphi_d_lce = 0;
    double dtl_d_lce = 0;
    //*******************************
    //Internal Force Related Variables for the loop
    double Fm = 0;          // Fiber force
    double FmAT=0;          // Fiber force along tendon
    double Ft = 0;          // Tendon force
    double ferr = 1;        // Solution error
    
    double dFm_dlce     = 0;  // Partial derivative of muscle force w.r.t. lce
    double dFmAT_dlce   = 0;  // Partial derivative of muscle force along 
                               // tendon w.r.t. lce
    double dFmAT_dlceAT = 0;  // Partial derivative of muscle force along
                               // tendon w.r.t. lce along the tendon.

    double dFt_d_lce   = 0;  // Partial derivative of tendon force w.r.t. lce
    double dFt_d_tl   = 0;   // Partial derivative of tendon force w.r.t. tl

    double dferr_d_lce = 0;  // Partial derivative of the solution error w.r.t
                             // lce
    double delta_lce   = 0;  // Chance in lce

    double Ke          = 0;  // Linearized local stiffness of the muscle
        

    double tmp1         = 0;
    double tmp2         = 0;
    double tmp3         = 0;
    //*******************************
    //Initialize the loop
    
    int iter = 0;    
    int minFiberLengthCtr = 0;

    while( abs(ferr) > aSolTolerance 
        && iter < aMaxIterations
        && minFiberLengthCtr < 10){

            //Update the multipliers and their partial derivativaes
            fal         = falCurve.calcValue(lceN);
            fpe         = fpeCurve.calcValue(lceN);
            fk          = fkCurve.calcValue(lceN);
            fcphi       = fcphiCurve.calcValue(cosphi);
            fse         = fseCurve.calcValue(tlN);            
            
            
        
            //SINGULARITY: When phi = pi/2, or activation = 0;
            fv          = fvCurve.calcValue(dlceN);
                //calcFv(ma, fal, fpe, fk, fcphi, fse, cosphi, caller);
            fvInv       = fvInvCurve.calcValue(fv); 

            //Compute the force error
            Fm   = calcFiberForce(fiso,ma,fal,fv,fpe,fk,fcphi,cosphi);
            FmAT = calcFiberForceAlongTendon(fiso,ma,fal,fv,fpe,fk,fcphi,cosphi);
            Ft = fse*fiso;
            ferr  = FmAT-Ft; 
        
            //Compute the partial derivative of the force error w.r.t. lce
            //Fiber:
            dFm_dlce   = calcFiberStiffness(fiso,ma,fal,fv,fpe,fk,fcphi,sinphi,
                                            cosphi,lce,lceN,ofl);
            
            dFmAT_dlce= calc_DFiberForceAT_DFiberLength(fiso,ma,fal,fv,fpe,fk,
                                            fcphi,sinphi,cosphi,lce,lceN,ofl);
           
            dFmAT_dlceAT = calc_DFiberForceAT_DFiberLengthAT(dFmAT_dlce,
                                           sinphi,cosphi,lce);


            //Tendon:
            dFt_d_tl    = fseCurve.calcDerivative(tlN,1)*fiso/tsl;
            
            dFt_d_lce   = calc_DTendonForce_DFiberLength(dFt_d_tl,lce,
                                                    sinphi,cosphi,caller);         

            //Error Derivative
            dferr_d_lce = dFmAT_dlce - dFt_d_lce;

            if(abs(ferr) > aSolTolerance){
                
                //Take a full Newton Step if the derivative is non-zero
                if(abs(dferr_d_lce) > SimTK::SignificantReal){
                    delta_lce   = - ferr/dferr_d_lce;
                    lce         = lce + delta_lce;
                }else{ //If we've stagnated, perturb the current solution            
                    double perturbation = 
                    2.0*((double)rand())/((double)RAND_MAX)-1.0;
                    double lengthPerturbation = 
                        0.5*perturbation*getOptimalFiberLength();
                    lce = lce + lengthPerturbation;
                }

                if(lce < penMdl.getMinimumFiberLength()){
                    minFiberLengthCtr++;
                    lce = penMdl.getMinimumFiberLength();
                }
                    
                //Update position level quantities, only if they won't go 
                //singular
                    
                phi = penMdl.calcPennationAngle(lce,caller);
                    

                sinphi = sin(phi);
                cosphi = cos(phi);
                tl  =penMdl.calcTendonLength(cosphi,lce,ml);
                lceN = lce/ofl;
                tlN  = tl/tsl;

                //Update velocity level quantities: share the muscle velocity 
                //between the tendon and the fiber according to their relative 
                //stiffness:
                //
                //Fm = Ft at equilbrium
                //Fm = Km*lceAT
                //Ft = Kt*xt
                //dFm_d_xm = Km*dlceAT + dKm_d_t*lceAT (assume dKm_d_t = 0)
                //dFt_d_xt = Kt*dtl + dKt_d_t*dtl (assume dKt_d_t = 0)
                //
                //This is a hueristic. The above assumptions are necessary as 
                //computing the partial derivatives of Km or Kt w.r.t. requires 
                //acceleration level knowledge, which is not available in 
                //general.

                //Stiffness of the muscle is the stiffness of the tendon and the 
                //fiber (along the tendon) in series

                //the if statement here is to handle the special case when the
                //negative stiffness of the fiber (which happens in this model)
                //is equal to the positive stiffness of the tendon.
                if( abs(dFmAT_dlceAT + dFt_d_tl) > SimTK::SignificantReal
                    && tl > getTendonSlackLength()){
                    Ke      = (dFmAT_dlceAT*dFt_d_tl)/(dFmAT_dlceAT + dFt_d_tl); 
                    dtl     = (1/dFt_d_tl)*Ke*dml;                    
                }else{
                    dtl     = dml;
                }

                dlce = penMdl.calcFiberVelocity(lce,sinphi,cosphi,
                                                        ml,tl,dml,dtl,caller); 
                dlceN    = dlce/(vmax*ofl);
                dphi = penMdl.calcPennationAngularVelocity(tan(phi),lce,
                                                            dlce,caller);
                dlceAT = penMdl.calcFiberVelocityAlongTendon(lce,dlce,
                                                    sinphi,cosphi,dphi);
                tmp1 = (dml - dtl)-dlceAT;

                

            }
        
        iter++;
    }

    //*******************************    
    //Populate the output vector
    //*******************************
    
    //1: flag (0 = converged
    //         1=diverged, 
    //         2= no solution due to singularity:length 0, 
    //         3= no solution due to pennation angle singularity
    //2: solution Error (N)
    //3: iterations
    //4: fiber length (m)
    //5: passive force (N)
    //6: tendon force (N)
        
    //If the solution converged
    if(abs(ferr) < aSolTolerance){    

        results[0] = 0;
        results[1] = ferr;
        results[2] = (double)iter;
        results[3] = lce;
        results[4] = fpe*fiso;
        results[5] = fse*fiso;
    }else{ 

        //If the fiber length hit its lower bound        
        if(iter < aMaxIterations){     

            lce = penMdl.getMinimumFiberLength();
            phi = penMdl.calcPennationAngle(lce,caller);
            cosphi = cos(phi);
            tl  = penMdl.calcTendonLength(cosphi,lce,ml);
            lceN = lce/ofl;
            tlN  = tl/tsl;                
            fse = fseCurve.calcValue(tlN);
            fpe = fpeCurve.calcValue(lceN);

            results[0] = 1.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = fpe*fiso;
            results[5] = fse*fiso; 
 
                      
        //If the solution diverged
        }else{
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
// Protected Numerical Services
//==============================================================================

/** Get the rate change of activation */
double Millard2012EquilibriumMuscle::
    calcActivationRate(const SimTK::State& s) const 
{    
    const MuscleFirstOrderActivationDynamicModel& actMdl 
        = get_MuscleFirstOrderActivationDynamicModel();

    double excitation = getExcitation(s);
    double activation = actMdl.clampActivation(getActivation(s));    

    //Both activation and excitation are clamped in calcDerivative
    double dadt = actMdl.calcDerivative(activation,excitation);
    return dadt;
}  

//==============================================================================
// Private Numerical Services
//==============================================================================
double Millard2012EquilibriumMuscle::calcFv(double a, 
                                            double fal, 
                                            double fpe, 
                                            double fk,                     
                                            double fcphi, 
                                            double fse, 
                                            double cosphi,
                                            std::string& caller) const
{
    SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::SignificantReal,
        "Millard2012EquilibriumMuscle::calcFv",
        "%s: The pennation angle is 90 degrees, and is causing a singularity", 
        caller.c_str());

    double fv = ( (fcphi+fse)/cosphi - (fpe-fk) ) / (a*fal);
    return fv;
}

double Millard2012EquilibriumMuscle::
    calcFiberForce( double fiso, 
                    double a, 
                    double fal,
                    double fv,                             
                    double fpe,
                    double fk, 
                    double fcphi,
                    double cosPhi) const
{
    //The force the fiber generates parallel to the fiber
    double fm = fiso * ((a*fal*fv + fpe - fk) - fcphi*cosPhi);
    return fm;
}

double Millard2012EquilibriumMuscle::
    calcFiberForceAlongTendon(  double fiso, 
                                double a, 
                                double fal,
                                double fv,                             
                                double fpe,
                                double fk, 
                                double fcphi,
                                double cosPhi) const
{
    //The force the fiber generates parallel to the fiber
    double fmAT = fiso * ((a*fal*fv + fpe - fk)*cosPhi - fcphi);
    return fmAT;
}

double Millard2012EquilibriumMuscle::
    calcFiberStiffness( double fiso, 
                        double a, 
                        double fal,
                        double fv,                             
                        double fpe,
                        double fk, 
                        double fcphi,
                        double sinPhi,
                        double cosPhi,
                        double lce,
                        double lceN,
                        double optFibLen) const
{
    
    std::string caller = getName();
    caller.append(".calcFiberStiffness");

    const FiberForceLengthCurve& fpeCurve 
        = get_FiberForceLengthCurve(); 
    const ActiveForceLengthCurve& falCurve
        = get_ActiveForceLengthCurve(); 
    const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve();

    
    double DlceN_Dlce = 1/optFibLen;

    double Dfal_Dlce     = falCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfpe_Dlce     = fpeCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfk_Dlce      = fkCurve.calcDerivative(lceN,1)  * DlceN_Dlce;   
    
    double Dphi_Dlce     = penMdl.calc_DPennationAngle_DfiberLength(lce,caller);
    double Dcosphi_Dlce  = -sinPhi*Dphi_Dlce;

    double Dfcphi_Dcosphi= fcphiCurve.calcDerivative(cosPhi,1);
    double Dfcphi_Dlce   = Dfcphi_Dcosphi * Dcosphi_Dlce;

    // The stiffness of the fiber parallel to the fiber
    //    d/dlce( fiso * ((a *  fal  *fv + fpe           - fk) + fcphi*cosPhi));
    double DFm_Dlce = fiso * ((a*Dfal_Dlce*fv + Dfpe_Dlce - Dfk_Dlce) 
                           - (Dfcphi_Dlce*cosPhi + fcphi*Dcosphi_Dlce));

    return DFm_Dlce;
}

double Millard2012EquilibriumMuscle::
    calc_DFiberForceAT_DFiberLength(double fiso, 
                                    double a, 
                                    double fal,
                                    double fv,                             
                                    double fpe,
                                    double fk, 
                                    double fcphi,
                                    double sinPhi,
                                    double cosPhi,
                                    double lce,
                                    double lceN,
                                    double optFibLen) const
{
    std::string caller = getName();
    caller.append("calcFiberStiffnessAlongTendon");

    const FiberForceLengthCurve& fpeCurve 
        = get_FiberForceLengthCurve(); 
    const ActiveForceLengthCurve& falCurve
        = get_ActiveForceLengthCurve(); 
    const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve();

    
    double DlceN_Dlce = 1/optFibLen;

    double Dfal_Dlce     = falCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfpe_Dlce     = fpeCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfk_Dlce      = fkCurve.calcDerivative(lceN,1)  * DlceN_Dlce;   
    
    double Dphi_Dlce     = penMdl.calc_DPennationAngle_DfiberLength(lce,caller);
    double Dcosphi_Dlce  = -sinPhi*Dphi_Dlce;
    
    double Dfcphi_Dcosphi= fcphiCurve.calcDerivative(cosPhi,1);
    double Dfcphi_Dlce   = Dfcphi_Dcosphi * Dcosphi_Dlce;
    //The stiffness of the fiber in the direction of the tendon
    //1. Compute the stiffness of the fiber along the direction of the 
    //   tendon, for small changes in length parallel to the fiber
    //   that is: D(FiberForceAlongTendon) D (fiberLength)
    // dFmAT/dlce = d/dlce( fiso * ((a *fal*fv + fpe  - fk)*cosPhi - fcphi))
    // 
    double DfmAT_Dlce = fiso * ((a*Dfal_Dlce*fv + Dfpe_Dlce - Dfk_Dlce)*cosPhi  
                            +(a*fal*fv + fpe - fk)*Dcosphi_Dlce - Dfcphi_Dlce);
   
    return DfmAT_Dlce;    
}

double Millard2012EquilibriumMuscle::
    calc_DFiberForceAT_DFiberLengthAT(  double dFmAT_d_lce,                                       
                                        double sinPhi,
                                        double cosPhi,
                                        double lce) const
{
    std::string caller = getName();
    caller.append("calcFiberStiffnessAlongTendon");

    double dphi_d_lce = penMdl.calc_DPennationAngle_DfiberLength(lce,caller);
    //2. Compute the change in length of the fiber length along the tendon
    //  lceAT = lce*cos(phi)
    // dlceAT/dlce = cos(phi) - lce*sin(phi)*dphi/dlce
    // 
    double DlceAT_Dlce = cosPhi - lce*sinPhi*dphi_d_lce;

    //3. Compute 
    // dFmAT/dlceAT = (dFmAT/dlce)*(1/(dlceAT/dlce)) 
    //              = dFmAT/dlceAT
    double DFmAT_DlceAT = dFmAT_d_lce * (1/DlceAT_Dlce);

    return DFmAT_DlceAT;    
}

double Millard2012EquilibriumMuscle::
    calc_DTendonForce_DFiberLength( double dFt_d_tl, 
                                    double lce,
                                    double sinphi,
                                    double cosphi,
                                    std::string& caller) const
{

    double dphi_d_lce  = penMdl.calc_DPennationAngle_DfiberLength(lce,caller);
    double dtl_d_lce   = penMdl.calc_DTendonLength_DfiberLength(lce,sinphi,
                                            cosphi,dphi_d_lce,caller);

    double dFt_d_lce = dFt_d_tl*dtl_d_lce;
    return dFt_d_lce;
}
