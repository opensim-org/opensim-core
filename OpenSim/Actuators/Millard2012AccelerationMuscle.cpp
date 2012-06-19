// Millard2012AccelerationMuscle.cpp
/* Author: Matthew Millard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */
//=============================================================================
// INCLUDES
//=============================================================================
#include "Millard2012AccelerationMuscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <Simbody.h>
#include <iostream>


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
}


void Millard2012AccelerationMuscle::constructProperties()
{ 
    constructProperty_default_activation(0.0); //getMinActivation()

    constructProperty_default_fiber_length(getOptimalFiberLength());
    
    constructProperty_default_fiber_velocity(0.0);

        MuscleFirstOrderActivationDynamicModel defaultActMdl = 
            MuscleFirstOrderActivationDynamicModel();
        double tauAct = defaultActMdl.getActivationTimeConstant();
        double tauDact= defaultActMdl.getDeactivationTimeConstant();

    //Ensure the minimum allowed activation is 0.     
    constructProperty_activation_model(
        MuscleFirstOrderActivationDynamicModel(tauAct,tauDact,0,getName()));

        ActiveForceLengthCurve defaultFal = ActiveForceLengthCurve();
        double lceMin  = defaultFal.getMinActiveFiberLength();
        double lceTrans= defaultFal.getTransitionFiberLength();
        double lceMax  = defaultFal.getMaxActiveFiberLength();
        double slope   = defaultFal.getShallowAscendingSlope();
        double minFal  = 0; //

    //Ensure the Active Force Length Curve can go to 0
    constructProperty_active_force_length_curve(
        ActiveForceLengthCurve(lceMin,lceTrans,lceMax,slope,minFal,getName()));

    ForceVelocityCurve defaultFv = ForceVelocityCurve();
    double concSlope = 0;
    double isoSlope  = defaultFv.getIsometricMaxSlope();
    double eccSlope  = 0;
    double maxFv     = defaultFv.getMaxEccentricVelocityForceMultiplier();
    double concCurviness= defaultFv.getConcentricCurviness();
    double eccCurviness = defaultFv.getEccentricCurviness();

    //Ensure the force velocity curve has asymptotes
    constructProperty_force_velocity_curve(
        ForceVelocityCurve( concSlope,
                            isoSlope,
                            eccSlope,
                            maxFv,
                            concCurviness,
                            eccCurviness,
                            getName()));

    constructProperty_fiber_force_length_curve(
        FiberForceLengthCurve());

    constructProperty_tendon_force_length_curve(
        TendonForceLengthCurve());

    constructProperty_fiber_compressive_force_length_curve(
        FiberCompressiveForceLengthCurve());

    constructProperty_fiber_compressive_force_cospennation_curve(
        FiberCompressiveForceCosPennationCurve());   

    //Nonlinear damping coefficients 
    constructProperty_tendon_force_length_damping(1e-1);
    constructProperty_fiber_compressive_force_length_damping(1e-3);
    constructProperty_fiber_force_length_damping(1e-3);
    constructProperty_fiber_compressive_force_cos_pennation_damping(1e-3);

    //Linear fiber damping as in Shutte's model
    constructProperty_fiber_damping(1e-2);

    //Mass property
    constructProperty_mass(0.01);
}

void Millard2012AccelerationMuscle::buildMuscle()
{
    double optFibLen = getOptimalFiberLength();
    double optPenAng = getPennationAngleAtOptimalFiberLength();
    std::string caller = getName();
    caller.append(".buildMuscle()");


    m_penMdl = MuscleFixedWidthPennationModel(optFibLen,optPenAng,caller);
    
    std::string aName = getName();

    std::string tmp = aName;
    tmp.append("_MuscleFirstOrderActivationDynamicModel");
    MuscleFirstOrderActivationDynamicModel& actMdl = upd_activation_model();
    actMdl.setName(tmp);

    tmp = aName;
    tmp.append("_ActiveForceLengthCurve");
    ActiveForceLengthCurve& falCurve = upd_active_force_length_curve();
    falCurve.setName(tmp);

    tmp = aName;
    tmp.append("_ForceVelocityCurve");
    ForceVelocityCurve& fvCurve = upd_force_velocity_curve();
    fvCurve.setName(tmp);

    tmp = aName;
    tmp.append("_FiberForceLengthCurve");
    FiberForceLengthCurve& fpeCurve = upd_fiber_force_length_curve();
    fpeCurve.setName(tmp);

    tmp = aName;
    tmp.append("_TendonForceLengthCurve");
    TendonForceLengthCurve& fseCurve = upd_tendon_force_length_curve();
    fseCurve.setName(tmp);

    tmp = aName;
    tmp.append("_FiberCompressiveForceLengthCurve");
    FiberCompressiveForceLengthCurve& fkCurve 
        = upd_fiber_compressive_force_length_curve();
    fkCurve.setName(tmp);

    tmp = aName;
    tmp.append("_FiberCompressiveForceCosPennationCurve");
    FiberCompressiveForceCosPennationCurve& fcphi = 
        upd_fiber_compressive_force_cospennation_curve();
    fcphi.setName(tmp);

    
    
    setObjectIsUpToDateWithProperties();

}

void Millard2012AccelerationMuscle::ensureMuscleUpToDate() const
{
    if(isObjectUpToDateWithProperties() == false)
    {
        Millard2012AccelerationMuscle* mthis = 
            const_cast<Millard2012AccelerationMuscle*>(this);
        mthis->buildMuscle(); 
    }
}


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

Millard2012AccelerationMuscle::Millard2012AccelerationMuscle()            
{    
    setNull();
    constructProperties();
    ensureMuscleUpToDate();
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

    ensureMuscleUpToDate();
}

//=============================================================================
// Model Component Interface
//=============================================================================
 void Millard2012AccelerationMuscle::connectToModel(Model& model)
 {
    Super::connectToModel(model);
    ensureMuscleUpToDate();
 }

 void Millard2012AccelerationMuscle::
     addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    ensureMuscleUpToDate();

    addStateVariable(STATE_ACTIVATION_NAME);
    addStateVariable(STATE_FIBER_LENGTH_NAME);
    addStateVariable(STATE_FIBER_VELOCITY_NAME);

    double value = 0.0;
	addCacheVariable(   STATE_ACTIVATION_NAME+"_deriv", 
                        value, 
                        SimTK::Stage::Dynamics);

    //These are dynamics level quantities so that when they are invalidated
    //they do not invalidate the stae of the entire multibody system
	addCacheVariable(   STATE_FIBER_LENGTH_NAME+"_deriv", 
                        value, 
                        SimTK::Stage::Dynamics);

    addCacheVariable(   STATE_FIBER_VELOCITY_NAME+"_deriv", 
                        value, 
                        SimTK::Stage::Dynamics);
}

void Millard2012AccelerationMuscle::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

    ensureMuscleUpToDate();

    Millard2012AccelerationMuscle* mthis = 
        const_cast<Millard2012AccelerationMuscle*>(this);

    mthis->setActivation(s, getDefaultActivation());
    mthis->setFiberLength(s, getDefaultFiberLength());
    mthis->setFiberVelocity(s,getDefaultFiberVelocity());
}
    
void Millard2012AccelerationMuscle::
    setPropertiesFromState(const SimTK::State& s)
{
    Super::setPropertiesFromState(s);
    
    ensureMuscleUpToDate();

    setDefaultActivation(getStateVariable(s,STATE_ACTIVATION_NAME));
    setDefaultFiberLength(getStateVariable(s,STATE_FIBER_LENGTH_NAME));
    setDefaultFiberVelocity(getStateVariable(s,STATE_FIBER_VELOCITY_NAME));
}

SimTK::Vector Millard2012AccelerationMuscle::
    computeStateVariableDerivatives(const SimTK::State& s) const 
{
    SimTK::Vector derivs(getNumStateVariables(),0.);

    if(!isDisabled(s)){
        derivs[0] = getActivationRate(s);
        derivs[1] = getFiberVelocity(s);
        derivs[2] = getFiberAcceleration(s);
    }
    return derivs;
}

//=============================================================================
// STATE RELATED GET FUNCTIONS
//=============================================================================

double Millard2012AccelerationMuscle::getDefaultActivation() const
{
    ensureMuscleUpToDate();
    return get_default_activation();
}

double Millard2012AccelerationMuscle::getDefaultFiberLength() const
{
    ensureMuscleUpToDate();
    return get_default_fiber_length();
}

double Millard2012AccelerationMuscle::getDefaultFiberVelocity() const
{
    ensureMuscleUpToDate();
    return get_default_fiber_velocity();
}

double Millard2012AccelerationMuscle::
    getActivationRate(const SimTK::State& s) const
{
    ensureMuscleUpToDate();
    return calcActivationRate(s);
}

double Millard2012AccelerationMuscle::
    getFiberVelocity(const SimTK::State& s) const
{
    ensureMuscleUpToDate();
    FiberVelocityInfo fvi = getFiberVelocityInfo(s);
    return fvi.fiberVelocity;
}

double Millard2012AccelerationMuscle::
    getFiberAcceleration(const SimTK::State& s) const
{
    ensureMuscleUpToDate();
    MuscleDynamicsInfo fdi = getMuscleDynamicsInfo(s);
    return fdi.userDefinedDynamicsExtras[MDIFiberAcceleration];
}



Array<std::string> Millard2012AccelerationMuscle::getStateVariableNames() const
{
    ensureMuscleUpToDate();
    Array<std::string> stateVariableNames = 
        ModelComponent::getStateVariableNames();
	
	for(int i=0; i<stateVariableNames.getSize(); ++i){
		stateVariableNames[i] = getName()+"."+stateVariableNames[i];
	}
	return stateVariableNames;
}

SimTK::SystemYIndex Millard2012AccelerationMuscle::
    getStateVariableSystemIndex(const std::string &stateVariableName) const
{
    unsigned int start = stateVariableName.find(".");
	unsigned int end = stateVariableName.length();
	
	if(start == end)
		return ModelComponent::getStateVariableSystemIndex(stateVariableName);
	else{
		string localName = stateVariableName.substr(++start, end-start);
		return ModelComponent::getStateVariableSystemIndex(localName);
	}    
}


double Millard2012AccelerationMuscle::
    getStateVariableDeriv(const SimTK::State& s, 
                          const std::string &aStateName) const
{
	return getCacheVariable<double>(s, aStateName + "_deriv");
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
    setStateVariable(s, STATE_ACTIVATION_NAME, activation);    
    markCacheVariableInvalid(s,"dynamicsInfo");
    
}

void Millard2012AccelerationMuscle::
    setFiberLength(SimTK::State& s, double fiberLength) const
{
    setStateVariable(s, STATE_FIBER_LENGTH_NAME, fiberLength);
    markCacheVariableInvalid(s,"lengthInfo");
    markCacheVariableInvalid(s,"velInfo");
    markCacheVariableInvalid(s,"dynamicsInfo");
    
}

void Millard2012AccelerationMuscle::
    setFiberVelocity(SimTK::State& s, double fiberVelocity) const
{
    setStateVariable(s, STATE_FIBER_VELOCITY_NAME, fiberVelocity);
    markCacheVariableInvalid(s,"velInfo");
    markCacheVariableInvalid(s,"dynamicsInfo");
    
}


void Millard2012AccelerationMuscle::
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
    ensureMuscleUpToDate();
    return get_activation_model();
}

const ActiveForceLengthCurve& Millard2012AccelerationMuscle::
    getActiveForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_active_force_length_curve();
}

const ForceVelocityCurve& Millard2012AccelerationMuscle::
    getForceVelocityCurve() const
{
    ensureMuscleUpToDate();
    return get_force_velocity_curve();
}

const FiberForceLengthCurve& Millard2012AccelerationMuscle::
    getFiberForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_fiber_force_length_curve();
}

const TendonForceLengthCurve& Millard2012AccelerationMuscle::
    getTendonForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_tendon_force_length_curve();
}

const FiberCompressiveForceLengthCurve& Millard2012AccelerationMuscle::
    getFiberCompressiveForceLengthCurve() const
{
    ensureMuscleUpToDate();
    return get_fiber_compressive_force_length_curve();
}

const FiberCompressiveForceCosPennationCurve& Millard2012AccelerationMuscle::
    getFiberCompressiveForceCosPennationCurve() const
{
    ensureMuscleUpToDate();
    return get_fiber_compressive_force_cospennation_curve();
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
    set_activation_model(aActivationMdl);
}
void Millard2012AccelerationMuscle::setActiveForceLengthCurve(
        ActiveForceLengthCurve& aActiveForceLengthCurve)
{
    set_active_force_length_curve(aActiveForceLengthCurve);
}

void Millard2012AccelerationMuscle::setForceVelocityCurve(
        ForceVelocityCurve& aForceVelocityCurve)
{   
    set_force_velocity_curve(aForceVelocityCurve);
}

void Millard2012AccelerationMuscle::setFiberForceLengthCurve(
        FiberForceLengthCurve& aFiberForceLengthCurve)
{
    set_fiber_force_length_curve(aFiberForceLengthCurve);
}

void Millard2012AccelerationMuscle::setTendonForceLengthCurve(
        TendonForceLengthCurve& aTendonForceLengthCurve)
{
    set_tendon_force_length_curve(aTendonForceLengthCurve);
}

void Millard2012AccelerationMuscle::setFiberCompressiveForceLengthCurve(
        FiberCompressiveForceLengthCurve& aFiberCompressiveForceLengthCurve)
{
    set_fiber_compressive_force_length_curve(
        aFiberCompressiveForceLengthCurve);
}

void Millard2012AccelerationMuscle::setFiberCompressiveForceCosPennationCurve(
        FiberCompressiveForceCosPennationCurve& 
        aFiberCompressiveForceCosPennationCurve)
{
    set_fiber_compressive_force_cospennation_curve(
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
// Protected Useful Functions
//==============================================================================

void Millard2012AccelerationMuscle::
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
// Muscle.h Interface
//==============================================================================

double  Millard2012AccelerationMuscle::
    computeActuation(const SimTK::State& s) const
{    
    ensureMuscleUpToDate();
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setForce(s,         mdi.tendonForce);
    return( mdi.tendonForce );
}



void Millard2012AccelerationMuscle::
    computeInitialFiberEquilibrium(SimTK::State& s) const
{
    ensureMuscleUpToDate();
    //Initialize activation to the users desired setting
    setActivation(s,getActivation(s));
    //Initialize the fiber length
    setFiberLength(s, getOptimalFiberLength());
    //Initialize the fiber velocity
    setFiberVelocity(s, getDefaultFiberVelocity());

    _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

    //Compute an initial muscle state that develops the desired force and
    //shares the muscle stretch between the muscle fiber and the tendon 
    //according to their relative stiffness.
    double activation = getActivation(s);

    //Tolerance, in Newtons, of the desired equilibrium
    double tol = 1e-8*getMaxIsometricForce();  //Should this be user settable?
    if(tol < SimTK::Eps*1000){
        tol = SimTK::Eps*1000;
    }
    int maxIter = 500;  //Should this be user settable?  
    double newtonStepFraction = 0.75;
    SimTK::Vector soln = initMuscleState(   s,
                                            activation, 
                                            tol, 
                                            maxIter, 
                                            newtonStepFraction);

    int flag_status    = (int)soln[0];
    double solnErr        = soln[1];
    double iterations     = (int)soln[2];
    double fiberLength    = soln[3];
    double fiberVelocity  = soln[4];
    double passiveForce   = soln[5];
    double tendonForce    = soln[6];

    std::string fcnName = "Millard2012AccelerationMuscle::"
                            "computeInitialFiberEquilibrium(SimTK::State& s)";
    std::string muscleName = getName();

    SimTK_ERRCHK3_ALWAYS(   flag_status == 0, 
                            fcnName.c_str(),
        "%s: \n"
     "    The initialization routine found no stable equilibrium fiber length\n"
     "    length. The initial activation (%f) or whole muscle length (%f)"
     "    might be unsuitable.", 
        muscleName.c_str(), 
        getActivation(s), 
        getLength(s));

    //1: flag (0 = converged
    //         1 = diverged, 
    //         2 = no solution due to singularity:length 0, 
    //         3 = no solution due to pennation angle singularity    
    //2: solution error (N)
    //3: iterations
    //4: fiber length (m)
    //5: fiber velocity (m/s)
    //6: passive force (N)
    //7: tendon force (N)                   
    setForce(s,tendonForce);
    setFiberLength(s,fiberLength);
    setFiberVelocity(s,fiberVelocity);
       
}

void Millard2012AccelerationMuscle::
    calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
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
    const TendonForceLengthCurve& fseCurve = get_tendon_force_length_curve(); 
    const FiberForceLengthCurve& fpeCurve  = get_fiber_force_length_curve(); 
    const ActiveForceLengthCurve& falCurve = get_active_force_length_curve(); 

    const FiberCompressiveForceLengthCurve& fkCurve
        = get_fiber_compressive_force_length_curve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_fiber_compressive_force_cospennation_curve(); 

    //Populate the output struct
    mli.fiberLength       = getStateVariable(s, STATE_FIBER_LENGTH_NAME); 

    mli.normFiberLength   = mli.fiberLength/optFiberLength;
    mli.pennationAngle    = m_penMdl.calcPennationAngle(mli.fiberLength,caller);
    mli.cosPennationAngle = cos(mli.pennationAngle);
    mli.sinPennationAngle = sin(mli.pennationAngle);

    mli.fiberLengthAlongTendon = mli.fiberLength*mli.cosPennationAngle;
    
    mli.tendonLength      = m_penMdl.calcTendonLength(mli.cosPennationAngle,
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



    mli.userDefinedLengthExtras[MLIfse]     = tendonForceLengthMultiplier;
    mli.userDefinedLengthExtras[MLIfk]      = fkCurve.calcValue(
                                                mli.normFiberLength);
    mli.userDefinedLengthExtras[MLIfcphi]   = fcphiCurve.calcValue(
                                                   mli.cosPennationAngle);
    mli.userDefinedLengthExtras[MLIfkPE]    = compForceLengthPE;
    mli.userDefinedLengthExtras[MLIfcphiPE] = compForceCosPennationPE;

}

void Millard2012AccelerationMuscle::
    calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
    ensureMuscleUpToDate();
    double simTime = s.getTime(); //for debugging purposes


    //Get the quantities that we've already computed
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);

    //Get the static properties of this muscle
        double mclLength      = getLength(s);
        double mclVelocity    = getLengtheningSpeed(s);
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

    //1. Get MuscleLengthInfo & available State information
    double dlce   = getStateVariable(s, STATE_FIBER_VELOCITY_NAME);
    double dlceN1  = dlce/(getMaxContractionVelocity()*optFiberLen);
    double lce    = mli.fiberLength;
    double phi    = mli.pennationAngle;
    double cosphi = mli.cosPennationAngle;
    double sinphi = mli.sinPennationAngle;

    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;
    double fse  = mli.userDefinedLengthExtras[MLIfse];
    double fk   = mli.userDefinedLengthExtras[MLIfk];
    double fcphi= mli.userDefinedLengthExtras[MLIfcphi];

    
    //2. Compute fv - but check for singularities first     
    const ForceVelocityCurve& fvCurve = get_force_velocity_curve(); 
    double fv = fvCurve.calcValue(dlceN1);
    
    //7. Compute the other related velocity components
    //double dlceAT = m_penMdl.
    double tanPhi = tan(phi);
    double dphidt    = m_penMdl.calcPennationAngularVelocity(tanPhi,lce,
                                                            dlce   ,caller);   
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
    
}


void Millard2012AccelerationMuscle::
    calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
        ensureMuscleUpToDate();
        double simTime = s.getTime(); //for debugging purposes

    //Get the quantities that we've already computed
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
        const FiberVelocityInfo &mvi = getFiberVelocityInfo(s);        
        
    //Get the state of this muscle
        double a   = getStateVariable(s, STATE_ACTIVATION_NAME); 

    //Get the properties of this muscle
        double mcl            = getLength(s);
        double dmcl_dt        = getLengtheningSpeed(s);
        double tsl            = getTendonSlackLength();
        double ofl            = getOptimalFiberLength();
        double fiso           = getMaxIsometricForce();
        const TendonForceLengthCurve& fseCurve= get_tendon_force_length_curve();

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
    double lceN         = lce/ofl;    
    double dlce_dt      = mvi.fiberVelocity;
    double dlceN1_dt    = mvi.normFiberVelocity;
    
    double phi          = mli.pennationAngle;
    double cosPhi       = mli.cosPennationAngle;
    double sinPhi       = mli.sinPennationAngle;
    double dphi_dt      = mvi.pennationAngularVelocity;
    
    double tl       = mli.tendonLength; 
    double dtl_dt   = mvi.tendonVelocity;
    double tlN      = mli.normTendonLength;
   
    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;
    double fv   = mvi.fiberForceVelocityMultiplier;    
    double fse  = mli.userDefinedLengthExtras[MLIfse];
    double fk   = mli.userDefinedLengthExtras[MLIfk];
    double fcphi= mli.userDefinedLengthExtras[MLIfcphi];

    //========================================================================
    //Compute visco elastic multipliers and their derivatives derivatives
    //========================================================================
    AccelerationMuscleInfo ami;
    calcAccelerationMuscleInfo( ami,
                                lce  ,dlce_dt,
                                phi  ,dphi_dt,
                                tl   ,dtl_dt,
                                fal  ,fv,fpe,fk,fcphi,fse);

    //========================================================================
    //Compute visco elastic multipliers derivatives
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
    mdi.passiveFiberForce            = ami.fpeVEM*fiso; 
                                     
    mdi.tendonForce                  = Fse; 
    mdi.normTendonForce              = ami.fse;      //just the elastic component
                                     
    mdi.fiberStiffness               = dFce_dlce;
    mdi.fiberStiffnessAlongTendon    = dFceAT_dlceAT;
    mdi.tendonStiffness              = dFt_dtl;
    mdi.muscleStiffness              = Ke;
                                     
    mdi.fiberPower                   = -mdi.activeFiberForce*mvi.fiberVelocity;

    mdi.fiberPowerAlongTendon        = -mdi.activeFiberForce*cosPhi 
                                         * mvi.fiberVelocityAlongTendon;

    mdi.tendonPower                  = -mdi.tendonForce * mvi.tendonVelocity;   
    
    mdi.musclePower                  = -mdi.tendonForce * dmcl_dt;

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

    double dfpeVEMdt   =  ami.fpeVEM   * ami.cosphi  * fiso * ami.dlceAT_dt;
    double dfkVEMdt    = -ami.fkVEM    * ami.cosphi  * fiso * ami.dlceAT_dt;
    double dfcphiVEMdt = -ami.fcphiVEM *               fiso * ami.dlceAT_dt;
    double dfseVEMdt   =  ami.fseVEM   *               fiso * ami.dtl_dt;

    double ddphi_dtt = m_penMdl.calcPennationAngularAcceleration(ami.lce,
                                                                ami.dlce_dt,
                                                                ddlce_dtt,
                                                                ami.sinphi,
                                                                ami.cosphi,
                                                                ami.dphi_dt,
                                                                caller);

    double ddlceAT_dtt = m_penMdl.calcFiberAccelerationAlongTendon(ami.lce,
                                                                    ami.dlce_dt,
                                                                    ddlce_dtt,
                                                                    ami.sinphi,
                                                                    ami.cosphi,
                                                                    ami.dphi_dt,
                                                                    ddphi_dtt);
    //(d/dt) KE = 1/2 * m * dlceAT_dt*dlceAT_dt
    double dKEdt = m*ami.dlceAT_dt*ddlceAT_dtt; 

    double dFibWdt      = mdi.fiberPower;
    double dBoundaryWdt = -mdi.musclePower;
    /*double dSysEdt      = (dfpePEdt + dfkPEdt + dfcphiPEdt + dfsePEdt)
                         - dFibWdt 
                         - dBoundaryWdt 
                         + (dfpeVdt + dfkVdt + dfcphiVdt + dfseVdt);
                         */
    double dSysEdt = dKEdt 
                    + (dfpePEdt + dfkPEdt + dfcphiPEdt + dfsePEdt)
                    - (dFibWdt + dBoundaryWdt)
                    - (dfpeVdt + dfkVdt + dfcphiVdt + dfseVdt)
                    - dfibVdt;

    double tol = sqrt(SimTK::Eps);
    
    //For debugging purposes
    if(abs(dSysEdt) >= tol){
        printf("KE+PE-W Tol Violation at time %f, by %f \n",simTime,dSysEdt);
        tol = sqrt(SimTK::Eps);
    }
    
    //if(abs(tmp) > tol)
    //    printf("%s: d/dt(system energy-work) > tol, (%f > %f) at time %f",
     //           fcnName.c_str(), tmp, tol, (double)s.getTime());
    
    
    mdi.userDefinedDynamicsExtras.resize(1);
    mdi.userDefinedDynamicsExtras[MDIFiberAcceleration]=ddlce_dtt;

}



//==============================================================================
// Numerical Guts: Initialization
//==============================================================================
SimTK::Vector Millard2012AccelerationMuscle::
    initMuscleState(SimTK::State& s, 
                    double aActivation, 
                    double aSolTolerance, 
                    int aMaxIterations,
                    double aNewtonStepFraction) const
{
    ensureMuscleUpToDate();   
    std::string caller = getName();
    caller.append(".initMuscleState");
    //results vector format
    //1: flag ( 0 = converged 
    //          1 = diverged 
    //          2 = no solution due to singularity:length 0, 
    //          3 = no solution due to pennation angle singularity    
    //2: solution error (N)
    //3: iterations
    //4: fiber length (m)
    //5: fiber velocity(m/s)
    //6: passive force (N)
    //7: tendon force (N)
    SimTK::Vector results = SimTK::Vector(7);

    //I'm using smaller variable names here to make it possible to write out 
    //lengthy equations
    double a        = aActivation;
    double ml       = getLength(s);
    double dml_dt   = getLengtheningSpeed(s);

    //Shorter version of the constants
    double tsl = getTendonSlackLength();
    double ofl = getOptimalFiberLength();

    double ophi= getPennationAngleAtOptimalFiberLength();
    double penHeight = m_penMdl.getParallelogramHeight();
    double fiso= getMaxIsometricForce();
    double vmax = getMaxContractionVelocity();//getPropertyValue<double>(VmaxName);

    //Get muscle model specific properties       
    const ActiveForceLengthCurve&   falCurve = get_active_force_length_curve(); 
    const ForceVelocityCurve&       fvCurve  = get_force_velocity_curve(); 
    const FiberForceLengthCurve&    fpeCurve = get_fiber_force_length_curve(); 
    const TendonForceLengthCurve&   fseCurve = get_tendon_force_length_curve(); 

    const FiberCompressiveForceLengthCurve& fkCurve
        = get_fiber_compressive_force_length_curve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_fiber_compressive_force_cospennation_curve(); 
    

    //Shorter version of normalized muscle multipliers
    double fse = 0; //Normalized tendon (series element) force
    double fal = 0; //Normalized active force length multiplier
    double fv  = 0; //Normalized force-velocity multiplier
    double fpe = 0; //Normalized parallel element force
    double fk  = 0; //Normalized fiber compressive force
    double fcphi=0; //Normalized pennation compressive force


    //*******************************
    //Position level
    double lce = 0.5*ofl;
    double tl  = tsl*1.01;

    if((ml - tl) > 0){
        lce      = m_penMdl.calcFiberLength( ml, tl, caller);
    }
    
    double phi      = m_penMdl.calcPennationAngle(lce,caller);
    double cosphi   = cos(phi);
    double sinphi   = sin(phi);       
    double tlN      = tl/tsl;
    double lceN     = lce/ofl;
    
    //Velocity level
    double dtl_dt    = 0;
    
    double dlce_dt   = m_penMdl.calcFiberVelocity(lce, sinphi,cosphi, 
                                              ml, tl, dml_dt, dtl_dt, caller);
    double dlceN1_dt = dlce_dt/(vmax*ofl);
    
    double dphi_dt   = m_penMdl.calcPennationAngularVelocity(tan(phi),
                                                            lce,
                                                            dlce_dt,
                                                            caller);

    //*******************************
    //Internal Kinematics Related Variables for the loop
    double dphi_dlce = 0;
    double dtl_dlce = 0;
    //*******************************
    //Internal Force Related Variables for the loop
    double Fce = 0;          // Fiber force
    double FceAT=0;          // Fiber force along tendon
    double Fse = 0;          // Tendon force
    double m = getMass();
    double ddlce_dtt = 1;        // Solution error
    
    double dFce_dlce     = 0;  // Partial derivative of fiber force w.r.t. lce
    double dFceAT_dlce   = 0;  // Partial derivative of fiber force along 
                               // tendon w.r.t. lce
    double dFceAT_dlceAT = 0;  // Partial derivative of muscle force along
                               // tendon w.r.t. lce along the tendon.

    double dFse_dlce   = 0;  // Partial derivative of tendon force w.r.t. lce
    double dFse_dtl   = 0;   // Partial derivative of tendon force w.r.t. tl

    double d_ddlcedtt_dlce = 0; // Partial derivative of the solution error w.r.t
                                // lce
    double delta_lce   = 0;     // Chance in lce

    double Ke          = 0;  // Linearized local stiffness of the muscle
    
    double tmp1         = 0;
    double tmp2         = 0;
    double tmp3         = 0;
    //*******************************
    //Initialize the loop
    
    int iter = 0;
    AccelerationMuscleInfo ami;

    while( abs(ddlce_dtt) > aSolTolerance && iter < aMaxIterations){

        if(lce > 0 + SimTK::Eps && phi < SimTK::Pi/2 - SimTK::Eps){
            //Update the multipliers and their partial derivativaes
            fal         = falCurve.calcValue(lceN);
            fpe         = fpeCurve.calcValue(lceN);
            fk          = fkCurve.calcValue(lceN);
            fcphi       = fcphiCurve.calcValue(cosphi);
            fse         = fseCurve.calcValue(tlN);            
            fv          = fvCurve.calcValue(dlceN1_dt);
            
            //========================================================================
            //Compute visco elastic multipliers and their derivatives derivatives
            //========================================================================            
            calcAccelerationMuscleInfo( ami,
                                        lce  ,dlce_dt,
                                        phi  ,dphi_dt,
                                        tl   ,dtl_dt,
                                        fal  ,fv,fpe,fk,fcphi,fse);

            //==================================================================
            //Compute visco elastic multipliers derivatives
            //==================================================================
            SimTK::Vec2 fiberForceIJ = calcFiberForceIJ(a,ami);
            Fce  = calcFiberForce(fiberForceIJ,ami);    
            FceAT= calcFiberForceAlongTendon(fiberForceIJ);
            Fse = calcTendonForce(ami);            

            ddlce_dtt = (1/m)*(Fse-FceAT)*ami.cosphi 
                      + lce*ami.dphi_dt*ami.dphi_dt;     

            //==================================================================
            // Compute the stiffness properties
            //==================================================================
            //Compute the stiffness of the fiber along the tendon
            SimTK::Vec2 fiberStiffnessIJ = calcFiberStiffnessIJ(a,ami);

            double dFce_dlce = 
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

            if(abs(ddlce_dtt) > aSolTolerance && abs(d_ddlcedtt_dlce) > 0){
                //Take a full Newton Step
                delta_lce   = - ddlce_dtt/d_ddlcedtt_dlce;
                lce         = lce + aNewtonStepFraction*delta_lce;
            
                if(lce > 0){
                    //Update position level quantities, only if they won't go 
                    //singular
                    try{ 
                        phi = m_penMdl.calcPennationAngle(lce,caller);
                    }catch(const std::exception&){
                        phi = SimTK::Pi-SimTK::Eps;
                    }

                    sinphi = sin(phi);
                    cosphi = cos(phi);
                    tl  =m_penMdl.calcTendonLength(cosphi,lce,ml);
                    lceN = lce/ofl;
                    tlN  = tl/tsl;

                /*
                Update velocity level quantities: share the muscle velocity 
                between the tendon and the fiber according to their relative 
                stiffness:
                
                Linearizing the force balance equations
                FceAT = Ft at equilbrium (ignoring the small Coriolis term)
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

                This is a hueristic. The above assumptions are necessary as 
                computing the partial derivatives of Kce or Kt w.r.t. time 
                requires acceleration level knowledge, which is not available in 
                general.
                */

                //Stiffness of the muscle is the stiffness of the tendon and the 
                //fiber (along the tendon) in series
                Ke = 1;
                if(abs(dFceAT_dlceAT*dFse_dtl) > 0 &&
                       abs(dFceAT_dlceAT + dFse_dtl) > 0){
                    Ke     =(dFceAT_dlceAT*dFse_dtl)/(dFceAT_dlceAT + dFse_dtl);
                    dtl_dt = (1/dFse_dtl)*Ke*dml_dt;

                    tmp1   = (1/dFceAT_dlceAT)*Ke*dml_dt;
                }else{
                    dtl_dt = 0;
                }

                dlce_dt= m_penMdl.calcFiberVelocity(lce,sinphi,cosphi,
                                ml,tl,dml_dt,dtl_dt,caller);
                   
                dlceN1_dt = dlce_dt/(vmax*ofl);
                dphi_dt   = m_penMdl.calcPennationAngularVelocity(tan(phi),
                                                                    lce,
                                                                    dlce_dt,
                                                                    caller);

                tmp2   = m_penMdl.calcFiberVelocityAlongTendon(lce,dlce_dt,
                                                    sinphi,cosphi,dphi_dt);
                    
                    
                }

            }
        }
        iter++;
    }

    //*******************************    
    //Populate the output vector
    //*******************************
    //If the solution converged
    if(abs(ddlce_dtt) < aSolTolerance){    
        //1: flag (0 = converged
        //         1 = diverged (not enough iterations) 
        //         2 = no solution due to singularity:length 0, 
        //         3 = no solution due to pennation angle singularity
        //2: solution Error (N)
        //3: iterations
        //4: fiber length (m)
        //5: fiber velocity (m/s)
        //6: passive force (N)
        //7: tendon force (N)
        
        results[0] = 0;
        results[1] = ddlce_dtt;
        results[2] = (double)iter;
        results[3] = lce;
        results[4] = dlce_dt;
        results[5] = ami.fpeVEM * fiso;
        results[6] = ami.fseVEM * fiso;
    }else{ //If the solution diverged
        
        //Check for the fiber length singularity
        if(lce < 0 + SimTK::Eps){
            results[0] = 2.0;
            results[1] = ddlce_dtt;
            results[2] = (double)iter;
            results[3] = 0.0;
            results[4] = 0.0;
            results[5] = 0.0;
            results[6] = 0.0;

            printf("Initialization failed: fiber length approaching 0, \n"
                   "                       for %s, a Millard2012AccelerationMuscle \n"
                   "                       with an error of %f", 
                   getName().c_str(), ddlce_dtt);
        //Check for a pennation angle singularity   
        }else if(phi > SimTK::Pi/2 - SimTK::Eps){
            results[0] = 3.0;
            results[1] = ddlce_dtt;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = 0.0;
            results[5] = 0.0;
            results[5] = 0.0;

            printf("Initialization failed: pennation angle approaching Pi/2, \n"
                   "                       for %s, a Millard2012AccelerationMuscle \n"
                   "                       with an error of %f", 
                   getName().c_str(), ddlce_dtt);

        //Not enough iterations
        }else{ 
            results[0] = 1.0;
            results[1] = ddlce_dtt;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = dlce_dt;
            results[5] = ami.fpeVEM * fiso;
            results[6] = ami.fseVEM * fiso;

            printf("Initialization failed: solution did not converge in %i, \n"
                   "                       for %s, a Millard2012AccelerationMuscle \n"
                   "                       with an error of %f", 
                   iter,getName().c_str(),ddlce_dtt);

        }
 
    }

    return results;
    
}


//==============================================================================
// Protected Numerical Services
//==============================================================================

/** Get the rate change of activation */
double Millard2012AccelerationMuscle::
    calcActivationRate(const SimTK::State& s) const 
{    
    double excitation = getExcitation(s);
    double activation = getActivation(s);

    const MuscleFirstOrderActivationDynamicModel& actMdl 
        = getProperty_activation_model().getValue(); 

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
    double fcphiVEM = ami.fcphiVEM;
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
    ensureMuscleUpToDate();
    std::string caller = getName();
    caller.append("Millard2012AccelerationMuscle::"
                  "calcAccelerationMuscleInfo");

    const TendonForceLengthCurve& fseCurve
        = get_tendon_force_length_curve();    
    const ActiveForceLengthCurve& falCurve
        = get_active_force_length_curve(); 
    const ForceVelocityCurve& fvCurve 
        = get_force_velocity_curve(); 
    const FiberForceLengthCurve& fpeCurve 
        = get_fiber_force_length_curve(); 
    const FiberCompressiveForceLengthCurve& fkCurve
        = get_fiber_compressive_force_length_curve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_fiber_compressive_force_cospennation_curve(); 
    
    double m = get_mass();

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
        ami.dphi_dlce = m_penMdl.calc_DPennationAngle_DfiberLength(lce,caller);

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
                                                                 ami.dphi_dlce,
                                                                 caller);

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
                                                                ami.dphi_dlce,
                                                                caller);
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
        double lceNAT        = ami.lceAT * dlceNAT_dlceAT;
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

    //Visco elastic multipliers
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

    //Visco elastic multiplier partial derivatives      
        ami.dfseVEM_dtl     = ami.dfse_dtl    + ami.dfseV_dtl;  
        ami.dfpeVEM_dlce    = ami.dfpe_dlce   + ami.dfpeV_dlce;
        ami.dfkVEM_dlce     = ami.dfk_dlce    + ami.dfkV_dlce;
        ami.dfcphiVEM_dlce  = ami.dfcphi_dlce + ami.dfcphiV_dlce;

    //Visco elastic saturation
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
        //Compressive elments: note the opposite signs
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

