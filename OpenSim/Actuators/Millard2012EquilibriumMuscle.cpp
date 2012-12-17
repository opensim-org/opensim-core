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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
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
    STATE_ACTIVATION_DERIVATIVE_NAME = "activation_velocity";
const string Millard2012EquilibriumMuscle::
    STATE_FIBER_LENGTH_NAME = "fiber_length"; 


const static int RigidTendon_NoActivation               = 0;
const static int RigidTendon_Activation                 = 1;
const static int RigidTendon_DampedFiber_NoActivation   = 2;
const static int RigidTendon_DampedFiber_Activation     = 3;

const static int ElasticTendon_NoActivation             = 4;
const static int ElasticTendon_Activation               = 5;
const static int ElasticTendon_DampedFiber_NoActivation = 6;
const static int ElasticTendon_DampedFiber_Activation   = 7;


//=============================================================================
// PROPERTY MANAGEMENT
//=============================================================================
void Millard2012EquilibriumMuscle::setNull()
{
    setAuthors("Matthew Millard");
}


void Millard2012EquilibriumMuscle::constructProperties()
{
      
    constructProperty_use_fiber_damping(false);
    constructProperty_fiber_damping(0.1);
    constructProperty_default_fiber_length(getOptimalFiberLength());
    
    MuscleFirstOrderActivationDynamicModel actMdl;
    actMdl.setMinimumActivation(0.01);

    constructProperty_MuscleFirstOrderActivationDynamicModel(actMdl);    
    constructProperty_default_activation(actMdl.getMinimumActivation());
    constructProperty_ActiveForceLengthCurve(ActiveForceLengthCurve());
    constructProperty_ForceVelocityInverseCurve(ForceVelocityInverseCurve());
    constructProperty_FiberForceLengthCurve(FiberForceLengthCurve());
    constructProperty_TendonForceLengthCurve(TendonForceLengthCurve());

    constructProperty_use_second_order_activation(false);

    MuscleSecondOrderActivationDynamicModel act2Mdl;
    actMdl.setMinimumActivation(0.01);
    constructProperty_MuscleSecondOrderActivationDynamicModel(act2Mdl);

}

void Millard2012EquilibriumMuscle::buildMuscle()
{
    
    std::string caller = getName();
    caller.append(".buildMuscle()");

    double optFibLen = getOptimalFiberLength();
    double optPenAng = getPennationAngleAtOptimalFiberLength();

    /* Allowing the pennation angle to go closer to Pi/2 causes large changes
       in fiber velocity close to Pi/2. Why? Because the fiber kinematic 
       equation for fiber velocity go singular as phi -> Pi/2

       lceAT = lce*cos(phi)
       dlceAT = dlcedt*cos(phi) - lce*sin(phi)*dphidt
       dlcedt = (dlceAT + lce*sin(phi)*dphidt)/cos(phi)

    */
    double maxPennationAngle = acos(0.1);         
    penMdl = MuscleFixedWidthPennationModel(optFibLen,
                                            optPenAng,
                                            maxPennationAngle,
                                            caller);
    
    //Ensure all of the object names are uptodate
    std::string aName = getName();

    std::string tmp = aName;
    tmp.append("_MuscleFirstOrderActivationDynamicModel");
    MuscleFirstOrderActivationDynamicModel& actMdl 
        = upd_MuscleFirstOrderActivationDynamicModel();    
    actMdl.setName(tmp);

    tmp = aName;
    tmp.append("_MuscleSecondOrderActivationDynamicModel");
    MuscleSecondOrderActivationDynamicModel& act2Mdl 
        = upd_MuscleSecondOrderActivationDynamicModel();    
    act2Mdl.setName(tmp);
    
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

    //For initialization only we need a force-velocity curve:
    double concentricSlopeAtVmax    = fvInvCurve.getConcentricSlopeAtVmax();
    double concentricSlopeNearVmax  = fvInvCurve.getConcentricSlopeNearVmax();
    double isometricSlope           = fvInvCurve.getIsometricSlope();
    double eccentricSlopeAtVmax     = fvInvCurve.getEccentricSlopeAtVmax();
    double eccentricSlopeNearVmax   = fvInvCurve.getEccentricSlopeNearVmax();
    double concentricCurviness  = fvInvCurve.getConcentricCurviness();
    double eccentricCurviness   = fvInvCurve.getEccentricCurviness();
    double eccentricForceMax    = 
        fvInvCurve.getMaxEccentricVelocityForceMultiplier();

     fvCurve = ForceVelocityCurve(  concentricSlopeAtVmax,
                                    concentricSlopeNearVmax,
                                    isometricSlope,
                                    eccentricSlopeAtVmax,
                                    eccentricSlopeNearVmax,
                                    eccentricForceMax,
                                    concentricCurviness,
                                    eccentricCurviness,
                                    caller);

    /*Handle Model Setting Specific Singularities:
        1. Activation Singularity
        2. Active Force Length Singularity
        3. Force velocity curve singularity
    */
    if(canStateGoSingular()){
                
            SimTK_ERRCHK1_ALWAYS(
                actMdl.getMinimumActivation() > SimTK::SignificantReal,
                caller.c_str(),
                "%s: Minimum activation must be greater than 0",
                getName().c_str());

            SimTK_ERRCHK1_ALWAYS(
                act2Mdl.getMinimumActivation() > SimTK::SignificantReal,
                caller.c_str(),
                "%s: Minimum activation must be greater than 0",
                getName().c_str());

            SimTK_ERRCHK1_ALWAYS(
                falCurve.getMinValue() > SimTK::SignificantReal,
                caller.c_str(),
                "%s: Minimum Active Force Length Value must be greater than 0"
                "for this muscle model.\n",
                getName().c_str());

            SimTK_ERRCHK1_ALWAYS(
                cos(penMdl.getMaximumPennationAngle()) > SimTK::SignificantReal,
                caller.c_str(),
                "%s: Maximum pennation angle must be less than Pi/2 radians.\n",
                getName().c_str());

            SimTK_ERRCHK1_ALWAYS(
                fvInvCurve.getConcentricSlopeAtVmax()> SimTK::SignificantReal
                && fvInvCurve.getEccentricSlopeAtVmax()>SimTK::SignificantReal,
                caller.c_str(),
                "%s: ForceVelocityInverseCurve: ConcentricMinSlope and "
                "EccentricMinSlope must be greater than 0.\n", 
                getName().c_str());               
    }else{
            actMdl.setMinimumActivation(0.0);
            act2Mdl.setMinimumActivation(0.0);
            falCurve.setMinValue(0.0);
            fvCurve.setCurveShape(0.0,fvInvCurve.getConcentricSlopeNearVmax(),
                fvInvCurve.getIsometricSlope(), 
                0.0, fvInvCurve.getEccentricSlopeNearVmax(),
                fvInvCurve.getMaxEccentricVelocityForceMultiplier());
    }

    //Ensure all sub objects are up to date;
    penMdl.ensureModelUpToDate();
    actMdl.ensureModelUpToDate();
    act2Mdl.ensureModelUpToDate();

    falCurve.ensureCurveUpToDate();
    fvCurve.ensureCurveUpToDate();
    fvInvCurve.ensureCurveUpToDate();
    fpeCurve.ensureCurveUpToDate();
    fseCurve.ensureCurveUpToDate();

    //Compute the minimum fiber length;

    //Minimum active fiber length in units of meters    
    double minActiveFiberLength 
        = falCurve.getMinActiveFiberLength()*getOptimalFiberLength();
 
    //Minimum pennated fiber length in units of meters.
    double minPennatedFiberLength = penMdl.getMinimumFiberLength();

    m_minimumFiberLength =  max(minActiveFiberLength,minPennatedFiberLength);
        
    //Minimum fiber length along the tendon
    double phi = penMdl.calcPennationAngle(m_minimumFiberLength);
    
    m_minimumFiberLengthAlongTendon 
        = penMdl.calcFiberLengthAlongTendon(m_minimumFiberLength,cos(phi));




    setObjectIsUpToDateWithProperties();    


}

void Millard2012EquilibriumMuscle::ensureMuscleUpToDate()
{
    if(isObjectUpToDateWithProperties() == false)
    {
        buildMuscle(); 
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
    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    double value = 0.0;
  
    if(isActivationAState()){
        addStateVariable(STATE_ACTIVATION_NAME);     
                addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", 
                value, 
                SimTK::Stage::Dynamics);
    
        if(get_use_second_order_activation()){
        addStateVariable(STATE_ACTIVATION_DERIVATIVE_NAME);     
                addCacheVariable(STATE_ACTIVATION_DERIVATIVE_NAME+"_deriv", 
                value, 
                SimTK::Stage::Dynamics);
        }
    }
   
    if(isFiberLengthAState()){
        addStateVariable(STATE_FIBER_LENGTH_NAME);  
	            addCacheVariable(STATE_FIBER_LENGTH_NAME+"_deriv", 
                value, 
                SimTK::Stage::Dynamics);
    }  

 
}

void Millard2012EquilibriumMuscle::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

    if(isActivationAState()){
        setActivation(s, getDefaultActivation());   

        //First time derivative of activation is set to zero ...
        if(get_use_second_order_activation()){
            setStateVariable(s,STATE_ACTIVATION_DERIVATIVE_NAME,0.0);    
            markCacheVariableInvalid(s,"velInfo");
            markCacheVariableInvalid(s,"dynamicsInfo");  
        }
    }

    if(isFiberLengthAState()){
        setFiberLength(s, getDefaultFiberLength());
    }

}
    

void Millard2012EquilibriumMuscle::
    setPropertiesFromState(const SimTK::State& s)
{
    Super::setPropertiesFromState(s);
       
    if(isActivationAState()){
        setDefaultActivation(getStateVariable(s,STATE_ACTIVATION_NAME));        
    }

    //First derivative of activation is left as zero for now.   

    if(isFiberLengthAState()){
        setDefaultFiberLength(getStateVariable(s,STATE_FIBER_LENGTH_NAME));
    }
    ensureMuscleUpToDate();
      
}

SimTK::Vector Millard2012EquilibriumMuscle::
computeStateVariableDerivatives(const SimTK::State& s) const 
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");
    SimTK::Vector derivs(getNumStateVariables(), 0.);
    
    int idx = 0;
    //Activation is the first state
    if (!isDisabled(s)) {
        if(isActivationAState() && idx+1 <= getNumStateVariables()){    
                
                derivs[idx] = getActivationDerivative(s,1);
                idx ++;

                if(get_use_second_order_activation() 
                    && idx+1 <= getNumStateVariables()){  

                    derivs[idx] = getActivationDerivative(s,2);
                    idx ++;            
                }
            
        }

        //Followed by fiber length, if it is a state
        if(isFiberLengthAState() && idx+1 <= getNumStateVariables()){       
                derivs[idx] = getFiberVelocity(s);        
        }
    }
    return derivs;   
}

//=============================================================================
// STATE RELATED GET FUNCTIONS
//=============================================================================

bool Millard2012EquilibriumMuscle::getUseSecondOrderActivationDynamics() const
{
    return get_use_second_order_activation();
}

double Millard2012EquilibriumMuscle::getDefaultActivation() const
{
    double defaultActivation = 0;


    if(get_use_second_order_activation()){
        const MuscleSecondOrderActivationDynamicModel& act2Mdl = 
                    get_MuscleSecondOrderActivationDynamicModel();
        defaultActivation =  
            act2Mdl.clampActivation(get_default_activation());
    }else{
        const MuscleFirstOrderActivationDynamicModel& actMdl = 
                    get_MuscleFirstOrderActivationDynamicModel();
        defaultActivation =  
            actMdl.clampActivation(get_default_activation());
    }
    return defaultActivation;
                        
}

double Millard2012EquilibriumMuscle::getDefaultFiberLength() const
{    
    return clampFiberLength(get_default_fiber_length());
}

double Millard2012EquilibriumMuscle::
    getActivationDerivative(const SimTK::State& s, int order) const
{
    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");
    double activationDerivative = 0;

    if(isActivationAState()){
        activationDerivative = calcActivationDerivative(s,order);
    }else{
        activationDerivative = 0;
    }

    return activationDerivative;
}

double Millard2012EquilibriumMuscle::
    getFiberVelocity(const SimTK::State& s) const
{
 
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    FiberVelocityInfo fvi = getFiberVelocityInfo(s);
    return fvi.fiberVelocity;
}

Array<std::string> Millard2012EquilibriumMuscle::getStateVariableNames() const
{
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
	SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    return getCacheVariable<double>(s, aStateName + "_deriv");
}



//=============================================================================
// STATE RELATED SET FUNCTIONS
//=============================================================================

void Millard2012EquilibriumMuscle::setDefaultActivation(double activation)
{
    //Even if activation isn't a state, all simulation methods have access
    //to the activation model
    double defaultAct = 0;

    if(get_use_second_order_activation()){
        const MuscleSecondOrderActivationDynamicModel &act2Mdl = 
            get_MuscleSecondOrderActivationDynamicModel();
            set_default_activation(act2Mdl.clampActivation(activation));      
    }else{
        const MuscleFirstOrderActivationDynamicModel &actMdl = 
            get_MuscleFirstOrderActivationDynamicModel();
        set_default_activation(actMdl.clampActivation(activation));  
    }
    ensureMuscleUpToDate();   
}

void Millard2012EquilibriumMuscle::setDefaultFiberLength(double fiberLength)
{
    set_default_fiber_length(clampFiberLength(fiberLength));
    ensureMuscleUpToDate();
}

void Millard2012EquilibriumMuscle::
    setActivation(SimTK::State& s, double activation) const
{   
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    if(isActivationAState())
    {        
        if(get_use_second_order_activation()){
                const MuscleSecondOrderActivationDynamicModel& act2Mdl 
                    = get_MuscleSecondOrderActivationDynamicModel();
                double clampedActivation=act2Mdl.clampActivation(activation);
                setStateVariable(s,STATE_ACTIVATION_NAME,clampedActivation);    
                markCacheVariableInvalid(s,"velInfo");
                markCacheVariableInvalid(s,"dynamicsInfo");    
        }else{
            const MuscleFirstOrderActivationDynamicModel& actMdl 
                = get_MuscleFirstOrderActivationDynamicModel();
            double clampedActivation=actMdl.clampActivation(activation);
            setStateVariable(s,STATE_ACTIVATION_NAME,clampedActivation);    
            markCacheVariableInvalid(s,"velInfo");
            markCacheVariableInvalid(s,"dynamicsInfo");        
            
        }
    }    
}



void Millard2012EquilibriumMuscle::
    setFiberLength(SimTK::State& s, double fiberLength) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    if(isFiberLengthAState()){
        setStateVariable(s, STATE_FIBER_LENGTH_NAME, 
                            clampFiberLength(fiberLength));
        markCacheVariableInvalid(s,"lengthInfo");
        markCacheVariableInvalid(s,"velInfo");
        markCacheVariableInvalid(s,"dynamicsInfo");
    }    
}

void Millard2012EquilibriumMuscle::
    setStateVariableDeriv(  const SimTK::State& s, 
                            const std::string &aStateName, 
                            double aValue) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    //Is there a way to check if aStateName is actually a state?
	double& cacheVariable = updCacheVariable<double>(s, aStateName + "_deriv");
	cacheVariable = aValue;
	markCacheVariableValid(s, aStateName + "_deriv");
}

void Millard2012EquilibriumMuscle::
    setUseSecondOrderActivation(bool use2ndOrderAct){
    set_use_second_order_activation(use2ndOrderAct);
    ensureMuscleUpToDate();
}

//=============================================================================
// GET
//=============================================================================

double Millard2012EquilibriumMuscle::
    getTendonForceMultiplier(SimTK::State& s) const
{
 
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    return mdi.normTendonForce;
}

const MuscleFirstOrderActivationDynamicModel& Millard2012EquilibriumMuscle::
    getFirstOrderActivationModel() const
{

    return get_MuscleFirstOrderActivationDynamicModel();                 
}

const MuscleSecondOrderActivationDynamicModel& Millard2012EquilibriumMuscle::
    getSecondOrderActivationModel() const
{

    return get_MuscleSecondOrderActivationDynamicModel();                 
}

const MuscleFixedWidthPennationModel& Millard2012EquilibriumMuscle::
    getPennationModel() const
{
    return penMdl;
}

const ActiveForceLengthCurve& Millard2012EquilibriumMuscle::
    getActiveForceLengthCurve() const
{
    return get_ActiveForceLengthCurve();    
}

const ForceVelocityInverseCurve& Millard2012EquilibriumMuscle::
    getForceVelocityInverseCurve() const
{
    return get_ForceVelocityInverseCurve();
}

const FiberForceLengthCurve& Millard2012EquilibriumMuscle::
    getFiberForceLengthCurve() const
{
    return get_FiberForceLengthCurve();
}

const TendonForceLengthCurve& Millard2012EquilibriumMuscle::
    getTendonForceLengthCurve() const
{

    return get_TendonForceLengthCurve();
}

double Millard2012EquilibriumMuscle::getMaximumPennationAngle() const
{

    return penMdl.getMaximumPennationAngle();
}


double Millard2012EquilibriumMuscle::
    getMinimumActivation() const
{
    double minActivation = 0 ;

    if(get_use_second_order_activation()){
        const MuscleSecondOrderActivationDynamicModel &act2Mdl 
            = get_MuscleSecondOrderActivationDynamicModel();   
        minActivation = act2Mdl.getMinimumActivation();
    }else{
        const MuscleFirstOrderActivationDynamicModel &actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();   
        minActivation = actMdl.getMinimumActivation();
    }

    return minActivation;
}


double Millard2012EquilibriumMuscle::
    getFiberStiffnessAlongTendon(const SimTK::State& s) const
{    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    return mdi.fiberStiffnessAlongTendon;
}

bool Millard2012EquilibriumMuscle::
    getUseFiberDamping() const
{
    return get_use_fiber_damping();
}

//=============================================================================
// SET
//=============================================================================


void Millard2012EquilibriumMuscle::setFirstOrderActivationModel(
        MuscleFirstOrderActivationDynamicModel& aActivationMdl)
{
    set_MuscleFirstOrderActivationDynamicModel(aActivationMdl);
    ensureMuscleUpToDate();    
}

void Millard2012EquilibriumMuscle::setSecondOrderActivationModel(
        MuscleSecondOrderActivationDynamicModel& aActivation2Mdl)
{
    set_MuscleSecondOrderActivationDynamicModel(aActivation2Mdl);
    ensureMuscleUpToDate();    
}

void Millard2012EquilibriumMuscle::setActiveForceLengthCurve(
        ActiveForceLengthCurve& aActiveForceLengthCurve)
{

   set_ActiveForceLengthCurve(aActiveForceLengthCurve);
   ensureMuscleUpToDate(); 
    
}

void Millard2012EquilibriumMuscle::setForceVelocityInverseCurve(
        ForceVelocityInverseCurve& aForceVelocityInverseCurve)
{   

    set_ForceVelocityInverseCurve(aForceVelocityInverseCurve);
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
    setMuscleConfiguration(bool ignoreTendonCompliance,
                           bool ignoreActivationDynamics,
                           bool useDamping,
                           bool useSecondOrderActivation)
{
    set_ignore_tendon_compliance(ignoreTendonCompliance);
    set_ignore_activation_dynamics(ignoreActivationDynamics);
    set_use_fiber_damping(useDamping);
    set_use_second_order_activation(useSecondOrderActivation);
    ensureMuscleUpToDate(); 
}

//==============================================================================
// Protected Useful Functions
//==============================================================================

void Millard2012EquilibriumMuscle::
postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

	GeometryPath& path = upd_GeometryPath();

	path.postScale(s, aScaleSet);

	if (path.getPreScaleLength(s) > 0.0)
	{
		double scaleFactor = getLength(s) / path.getPreScaleLength(s);
		upd_optimal_fiber_length() *= scaleFactor;
		upd_tendon_slack_length() *= scaleFactor;
		path.setPreScaleLength(s, 0.0) ;
        ensureMuscleUpToDate();
	}
}



//==============================================================================
// XXXXXXXXXXXXXXXXXXXX  START OF TO BE DEPRECATED   XXXXXXXXXXXXXXXXXXXXXXXXXXX
//==============================================================================
double Millard2012EquilibriumMuscle::
calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                       double aActivation) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

        string caller = getName();
        caller.append(  "Millard2012EquilibriumMuscle::"
                        "calcInextensibleTendonActiveFiberForce");
        double inextensibleTendonActiveFiberForce = 0;

        double lm    = getLength(s);
        double dlm   = getLengtheningSpeed(s);
        double ltslk = getTendonSlackLength();
        double dlt   = 0.0; //Inextensible tendon;

        double lce  = penMdl.calcFiberLength(lm,ltslk);
        double phi      = penMdl.calcPennationAngle(lce);
      
        double dlce = penMdl.calcFiberVelocity(lce,sin(phi),cos(phi),
                                                lm,ltslk,   dlm,dlt,
                                                caller);       

        if(SimTK::isNaN(dlce) == false){  
            inextensibleTendonActiveFiberForce = 
                calcActiveFiberForceAlongTendon(    aActivation,
                                                    lce,
                                                    dlce);            
        }

        return inextensibleTendonActiveFiberForce;
}

double Millard2012EquilibriumMuscle::
            calcActiveFiberForceAlongTendon(double activation, 
                                            double fiberLength, 
                                            double fiberVelocity) const
{
    string caller = getName();
    caller.append(  "::MillardEquilibriumMuscle"
                    "::calcActiveFiberForceAlongTendon");

    double activeFiberForce = 0;    
    //If the fiber is in a legal range, compute the force its generating
    if(isFiberStateClamped(fiberLength,fiberVelocity) == false){

        //Clamp activation to a legal range
        const MuscleFirstOrderActivationDynamicModel& actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();
        double ca = actMdl.clampActivation(activation);

        //Normalize fiber length and velocity
        double lceN    = fiberLength/getOptimalFiberLength();
        double dlceN   = fiberVelocity / 
                        (getOptimalFiberLength() * getMaxContractionVelocity());

        //Get the necessary curves
        const ActiveForceLengthCurve& falCurve
            = get_ActiveForceLengthCurve();
        const FiberForceLengthCurve& fpeCurve
            = get_FiberForceLengthCurve();

        //Evaluate the force active length and force velocity multipliers
        double fal  = falCurve.calcValue(lceN);
        double fv   = fvCurve.calcValue(dlceN);
        double fiso = getMaxIsometricForce();
        double fpe  = fpeCurve.calcValue(lceN);
        //Evaluate the pennation angle
        double phi = penMdl.calcPennationAngle(lceN);

        //Compute the active fiber force 
        Vec3 fiberForceV = calcFiberForce(fiso,ca,fal,fv,fpe,dlceN);
        double fa = fiberForceV[1];
        double faAT = calcFiberForceAlongTendon(fa,cos(phi));
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
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setForce(s,         mdi.tendonForce);
    return( mdi.tendonForce );
}

//==============================================================================
// Utility Functions
//==============================================================================

    SimTK::Vec4 Millard2012EquilibriumMuscle::
        calcFiberStateGivenBoundaryCond(double lengthMT, 
                                        double velocityMT,
                                        double tendonForce,
                                        double dTendonForceDT) const
{
    string caller = getName();
    caller.append(".calcFiberStateGivenBoundaryCond");

    SimTK::Vec4 output;

    output[0] = 0;
    output[1] = 0;
    output[2] = 0;
    output[3] = 0;


    double a, lt, vt, lm, vm, phi, dphidt = 0;
    double ltN, lmN, vmN = 0;

    //1. Compute tendon length given F
    if(isTendonElastic() && tendonForce > 0){
        //Use Newton's method to solve for the strain of the tendon
        const TendonForceLengthCurve& fseCurve = get_TendonForceLengthCurve();
        
        //Good initial guess at the tendon length
        lt = getTendonSlackLength()*(1 + 
         fseCurve.getStrainAtOneNormForce()*tendonForce/getMaxIsometricForce());
        

        double tol = 1e-8*getMaxIsometricForce();
        if(tol < SimTK::SignificantReal*100){
            tol = SimTK::SignificantReal*100;
        }

        int maxIter = 100;
        int iter = 0;
        double err = 1e10;
        double derr_dtlN = 0;

        ltN = lt/getTendonSlackLength();
        double delta_ltN = 0;
        
        while(abs(err) > tol && iter < maxIter){
            //Compute error
            err = fseCurve.calcValue(ltN)*getMaxIsometricForce() - tendonForce;            
            derr_dtlN = fseCurve.calcDerivative(ltN,1)*getMaxIsometricForce();

            //Check tolerance, if we can, take a Newton step
            if(abs(err) > tol && abs(derr_dtlN) > SimTK::SignificantReal){
               delta_ltN = -err / derr_dtlN;  
               if(abs(delta_ltN) > 0.5*fseCurve.getStrainAtOneNormForce()){
                    delta_ltN = 0.5*fseCurve.getStrainAtOneNormForce();
               }

               ltN = ltN + delta_ltN;
            } 
            iter++;
        }

        if(abs(err) <= tol){
            lt = ltN*getTendonSlackLength();
        }else{
            lt = SimTK::NaN;
        }


    }else{
        if(isTendonElastic() == false){  //Rigid Tendon Model
            lt = getTendonSlackLength(); 
            ltN = 1.0;
        }else if(tendonForce <= 0){      //Slack elastic tendon
            lt = lengthMT - penMdl.getMinimumFiberLengthAlongTendon();
            ltN = lt/getTendonSlackLength();
        }
    }

    //If we have a valid tendon length, proceed
    if(SimTK::isNaN(lt) == false){
          

        //2. Compute tendon stretch velocity given dF/dt    
            /*Due to the equilibrium assumption
                tendonForce = k*(lt - lt0)
                dtendonForce/dt = k*(dlt_dt)
            */
            if(isTendonElastic() && tendonForce > 0){
                const TendonForceLengthCurve& fseCurve 
                    = get_TendonForceLengthCurve();
                double ktN = fseCurve.calcDerivative(ltN,1);
                double kt = ktN*(getMaxIsometricForce()/getTendonSlackLength());
                vt  = dTendonForceDT / kt;      

            }else{
                if(isTendonElastic() == false){//Rigid tendon                   
                    vt = 0;
                }else if(tendonForce <= 0){//Buckling elastic tendon
                    vt = velocityMT;
                }
            }


        //3. Compute fiber length, pennation angle
            lm  = penMdl.calcFiberLength(lengthMT,lt);
            lmN = lm/getOptimalFiberLength();

            phi = penMdl.calcPennationAngle(lm);

        //4. Compute fiber velocity, pennation angular velocity

            vm = penMdl.calcFiberVelocity(  lm,
                                            sin(phi),   cos(phi),
                                            lengthMT,   lt,
                                            velocityMT, vt,
                                            caller);
            vmN= vm / (getOptimalFiberLength()*getMaxContractionVelocity());

        //5. Compute activation
            const ActiveForceLengthCurve& falCurve 
                = get_ActiveForceLengthCurve();
            //fvCurve is a private member object        
            const FiberForceLengthCurve& fpeCurve 
                = get_FiberForceLengthCurve();

            double fal = falCurve.calcValue(lmN);
            double fpe = fpeCurve.calcValue(lmN);
            double fv  = fvCurve.calcValue(vmN);
            
            a = calcActivation(getMaxIsometricForce(),tendonForce,
                               cos(phi),fal,fv,fpe,lmN);

        //6. Populate output vector
            output[0] = a;
            output[1] = lmN;
            output[2] = phi;
            output[3] = vmN;
    }

    return output;
}

//==============================================================================
// Initialization
//==============================================================================
void Millard2012EquilibriumMuscle::
    computeInitialFiberEquilibrium(SimTK::State& s) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    //Elastic tendon initialization routine
    if(isTendonElastic() == false){
        return;
    }

    try{        
        //Initialize activation to the users desired setting
        
        double clampedActivation = getDefaultActivation();

        if(get_use_second_order_activation()){
            const MuscleSecondOrderActivationDynamicModel& act2Mdl 
            = get_MuscleSecondOrderActivationDynamicModel();
            clampedActivation = act2Mdl.clampActivation(clampedActivation);
            setActivation(s, clampedActivation);
        }else{
            const MuscleFirstOrderActivationDynamicModel& actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();
            clampedActivation = actMdl.clampActivation(clampedActivation);
            setActivation(s,clampedActivation);
            
        }
        //Initialize the multibody system to the initial state vector
        setFiberLength(s, getOptimalFiberLength());

        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

        //Compute an initial muscle state that develops the desired force and
        //shares the muscle stretch between the muscle fiber and the tendon 
        //according to their relative stiffness.    
        double activation = clampedActivation;

        int flag_status       = -1;
        double solnErr        = SimTK::NaN;
        int iterations     = 0;
        double fiberLength    = SimTK::NaN;
        double fiberVelocity  = SimTK::NaN;
        double tendonForce    = SimTK::NaN;
                   
        //tol is the desired tolerance in Newtons
        double tol = 1e-8*getMaxIsometricForce();  
        if(tol < SimTK::SignificantReal*10){
            tol = SimTK::SignificantReal*10;
        }

        int maxIter = 200;                              
        double pathLength = getLength(s);
        double pathLengtheningSpeed = getLengtheningSpeed(s);
                
        SimTK::Vector soln;
        
        soln = estimateMuscleFiberState(
                activation, 
                pathLength, pathLengtheningSpeed, 
                tol       , maxIter);
                
        flag_status    = (int)soln[0];
        solnErr        = soln[1];
        iterations     = (int)soln[2];
        fiberLength    = soln[3];
        fiberVelocity  = soln[4];
        tendonForce    = soln[5];
              
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
                        " %s is at its minimum length of %f\n",
                        muscleName.c_str(), getMinimumFiberLength());
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
        
        cerr << "    Continuing with initial tendon force of 0 \n" << endl;
        cerr << "    and a fiber length equal to the optimal \n" << endl;
        cerr << "    fiber length ...\n\n" 
             << endl;

        setForce(s,0);
		setFiberLength(s,getOptimalFiberLength());

    }      
}


SimTK::Vector Millard2012EquilibriumMuscle::
    estimateMuscleFiberState(double aActivation, 
                    double pathLength, double pathLengtheningSpeed, 
                    double aSolTolerance, int aMaxIterations) const
{
      
    std::string caller = getName();
    caller.append(".estimateMuscleFiberState");
    //results vector format
    //1: flag (0 = converged,
    //         1 = diverged,
    //         2= no solution due to singularity:length 0, 
    //         3= no solution due to pennation angle singularity    
    //2: solution error (N)
    //3: iterations
    //4: fiber length (m)
    //5: fiber Velocity (N)
    //6: tendon force (N)
    SimTK::Vector results = SimTK::Vector(6);

    //I'm using smaller variable names here to make it possible to write out 
    //lengthy equations
    double ma = aActivation;
    double ml = pathLength;
    double dml= pathLengtheningSpeed;

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
   
    //Shorter version of normalized muscle multipliers

    double fse = 0; //Normalized tendon (series element) force
    double fal = 0; //Normalized active force length multiplier
    double fv  = 0; //Normalized force-velocity multiplier
    double fpe = 0; //Normalized parallel element force


    //*******************************
    //Position level
    double lce = 0;
    double tl  = getTendonSlackLength()*1.01;

    lce = clampFiberLength(penMdl.calcFiberLength( ml, tl));        

    double phi      = penMdl.calcPennationAngle(lce);
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
    SimTK::Vec3 fiberForceV;

    while( abs(ferr) > aSolTolerance 
        && iter < aMaxIterations
        && minFiberLengthCtr < 10){

            //Update the multipliers and their partial derivativaes
            fal         = falCurve.calcValue(lceN);
            fpe         = fpeCurve.calcValue(lceN);
            fse         = fseCurve.calcValue(tlN);                                          
            fv          = fvCurve.calcValue(dlceN);
           
            //Compute the force error edited to remove fk, fcphi terms
            fiberForceV   = calcFiberForce(fiso,ma,fal,fv,fpe,dlceN);
            Fm   = fiberForceV[0];
            FmAT = calcFiberForceAlongTendon(Fm,cosphi);
            Ft = fse*fiso;
            ferr  = FmAT-Ft; 
        
            //Compute the partial derivative of the force error w.r.t. lce
            //Fiber: edited to remove fk, fcphi terms
            dFm_dlce   = calcFiberStiffness(fiso,ma,fv,lceN,ofl);
            //edited to remove fk, fcphi terms
            dFmAT_dlce= calc_DFiberForceAT_DFiberLength(Fm,
                                                        dFm_dlce,
                                                        lce,
                                                        sinphi,
                                                        cosphi);
            //edited to remove fk, fcphi terms
            dFmAT_dlceAT = calc_DFiberForceAT_DFiberLengthAT(dFmAT_dlce,
                                                              sinphi,
                                                              cosphi,
                                                              lce);

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

                if(isFiberStateClamped(lce,dlceN)){
                    minFiberLengthCtr++;
                    lce = getMinimumFiberLength();
                }
                    
                //Update position level quantities, only if they won't go 
                //singular
                    
                phi = penMdl.calcPennationAngle(lce);
                    

                sinphi = sin(phi);
                cosphi = cos(phi);
                tl  =penMdl.calcTendonLength(cosphi,lce,ml);
                lceN = lce/ofl;
                tlN  = tl/tsl;

                /*Update velocity level quantities: share the muscle velocity 
                between the tendon and the fiber according to their relative 
                stiffness:
                
                Fm-Ft = 0               :Equilibrium equation    [1]
                d/dt Fm - d/dt Ft = 0   :1 Time derivative       [2]
                lp = lm + lt                : path definition    [3]
                d/dt lp = d/dt lm + d/dt lt :path derivative     [4]
                 
                Computing a linearized model of [2]
                Fm = Fm0 + Km*lceAT                              [5]
                Ft = Ft0 Kt*xt                                   [6]

                Taking its time derivative
                dFm_d_xm = Km*dlceAT + dKm_d_t*lceAT (assume dKm_d_t = 0)  [7]
                dFt_d_xt = Kt*dtl + dKt_d_t*dtl (assume dKt_d_t = 0)       [8]
                
                Subtituting 7 and 8 into 2
                Km dlceAT - Kt dtl = 0

                Using Eqn 4, we have 2 equations in 2 unknowns. Can now solve
                for tendon velocity, or the velocity of the fiber along the 
                tendon

                This is a hueristic. The above assumptions are necessary as 
                computing the partial derivatives of Km or Kt w.r.t. requires 
                acceleration level knowledge, which is not available in 
                general.

                Stiffness of the muscle is the stiffness of the tendon and the 
                fiber (along the tendon) in series

                the if statement here is to handle the special case when the
                negative stiffness of the fiber (which happens in this model)
                is equal to the positive stiffness of the tendon.         
                */                       
                if( abs(dFmAT_dlceAT + dFt_d_tl) > SimTK::SignificantReal
                    && tlN > 1.0){
                        Ke      = (dFmAT_dlceAT*dFt_d_tl)
                                   /(dFmAT_dlceAT + dFt_d_tl); 
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
        results[4] = dlce;
        results[5] = fse*fiso;
    }else{ 

        //If the fiber length hit its lower bound        
        if(iter < aMaxIterations){     

            lce = getMinimumFiberLength();
            phi = penMdl.calcPennationAngle(lce);
            cosphi = cos(phi);
            tl  = penMdl.calcTendonLength(cosphi,lce,ml);
            lceN = lce/ofl;
            tlN  = tl/tsl;                
            fse = fseCurve.calcValue(tlN);            

            results[0] = 1.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = 0;
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
//==============================================================================
//
// Muscle Length Info
//
//==============================================================================
//==============================================================================

void Millard2012EquilibriumMuscle::
    calcMuscleLengthInfo(const SimTK::State& s, 
                         MuscleLengthInfo& mli) const
{
    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

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
    

    int simMethod = getSimulationMethod();

    //=========================================================================
    // Rigid Tendon Fiber Length
    //=========================================================================
    if( isTendonElastic() == false){
        
       
        mli.fiberLength = clampFiberLength(
                                    penMdl.calcFiberLength(getLength(s),
                                    getTendonSlackLength()));

    //=========================================================================
    // Elastic Tendon Fiber Length
    //=========================================================================    
    }else if (isFiberLengthAState() && isTendonElastic()){
        mli.fiberLength       = clampFiberLength(
                                getStateVariable(s, STATE_FIBER_LENGTH_NAME));
    
    }else{
        printf("%s: Muscle Illegally Configured. This should not be "
               "possible to do. If you see this message contact the OpenSim"
               "deveopment team.\n",caller.c_str());
        mli.fiberLength = SimTK::NaN;
    }     

    mli.normFiberLength   = mli.fiberLength/optFiberLength;
    mli.pennationAngle    = penMdl.calcPennationAngle(mli.fiberLength);
    mli.cosPennationAngle = cos(mli.pennationAngle);
    mli.sinPennationAngle = sin(mli.pennationAngle);

    mli.fiberLengthAlongTendon = mli.fiberLength*mli.cosPennationAngle;
    
    //Necessary even for the rigid tendon, as it might have gone slack
    mli.tendonLength      = penMdl.calcTendonLength(mli.cosPennationAngle,
                                                   mli.fiberLength,mclLength);
    
    
    mli.normTendonLength  = mli.tendonLength / tendonSlackLen;
    mli.tendonStrain      = mli.normTendonLength -  1.0;
        
    //Note the curves return normalized area. Each area must be un-normalized   
    mli.fiberPotentialEnergy =  fpeCurve.calcIntegral(mli.normFiberLength)
                                *(optFiberLength*maxIsoForce);
    
     mli.tendonPotentialEnergy =  0;
    //Elastic tendon

    if(isTendonElastic()){     
        mli.tendonPotentialEnergy = fseCurve.calcIntegral(mli.normTendonLength)
                                    *(tendonSlackLen*maxIsoForce);
    }
    
    mli.musclePotentialEnergy=  mli.fiberPotentialEnergy 
                              + mli.tendonPotentialEnergy;

    mli.fiberPassiveForceLengthMultiplier= 
        fpeCurve.calcValue(mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier = 
        falCurve.calcValue(mli.normFiberLength);
    
}



//==============================================================================
//==============================================================================
//
// Fiber Velocity Info
//
//==============================================================================
//==============================================================================
void Millard2012EquilibriumMuscle::
     calcFiberVelocityInfo(const SimTK::State& s, 
                                            FiberVelocityInfo& fvi) const
{
  
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    double simTime = s.getTime(); //for debugging purposes

    //Get the quantities that we've already computed
    const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
        
    //Get the static properties of this muscle
    double lenMcl             = getLength(s);
    double dlenMcl            = getLengtheningSpeed(s);
    double tdnSlkLen          = getTendonSlackLength();
    double optFibLen          = getOptimalFiberLength();

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
    double lce  = mli.fiberLength;    
    double lceN = mli.normFiberLength;
    double tl   = mli.tendonLength;
    double tlN  = mli.normTendonLength;
    double phi  = mli.pennationAngle;
    double cosphi=mli.cosPennationAngle;
    double sinphi = mli.sinPennationAngle;

    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;

    double dlce = SimTK::NaN;
    double dlceN= SimTK::NaN;
    double fv   = SimTK::NaN;
    double fiso = getMaxIsometricForce();

    int simMethod = getSimulationMethod();

    const MuscleFirstOrderActivationDynamicModel& actMdl 
                = get_MuscleFirstOrderActivationDynamicModel();

    const MuscleSecondOrderActivationDynamicModel& act2Mdl 
                = get_MuscleSecondOrderActivationDynamicModel();

    //=========================================================================
    //Rigid Tendon Fiber Velocity Computation
    //=========================================================================
    if( isTendonElastic() == false){  

        double dtldt = 0;
        if(tl <= (tdnSlkLen-SimTK::SignificantReal)){
            dtldt = dlenMcl; //tendon is buckling, and thus has 100% of the 
                             //path velocity
        }
            dlce = penMdl.calcFiberVelocity(lce,
                                    sinphi,cosphi,
                                    lenMcl, tl, 
                                    dlenMcl,dtldt, 
                                    caller);
                
        dlceN = dlce/(optFibLen*getMaxContractionVelocity());
        fv = fvCurve.calcValue(dlceN);
    //=========================================================================
    //Approximate Elastic Tendon Fiber Velocity Computation
    //=========================================================================
    }else if( isTendonElastic() && get_use_fiber_damping() == false){
       
            double a = 0;
            if(isActivationAState()){
                a = getStateVariable(s, STATE_ACTIVATION_NAME);
            }else{
                a = getControl(s);
            }

            if(get_use_second_order_activation()){
                a = act2Mdl.clampActivation(a);
            }else{
                a = actMdl.clampActivation(a);
            }

            const TendonForceLengthCurve& fseCurve 
                = get_TendonForceLengthCurve();
            double fse  = fseCurve.calcValue(tlN);

            SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::SignificantReal, 
                fcnName.c_str(),
                "%s: Pennation angle is 90 degrees, "
                "and is causing a singularity", 
                muscleName.c_str());
            SimTK_ERRCHK1_ALWAYS(a > SimTK::SignificantReal, 
                fcnName.c_str(),
                "%s: Activation is 0, and is causing a singularity", 
                muscleName.c_str());
            SimTK_ERRCHK1_ALWAYS(fal > SimTK::SignificantReal, 
                fcnName.c_str(),
                "%s: The active force length curve value is 0,"
                " and is causing a singularity", muscleName.c_str());
                 
            fv = calcFv(a, fal,fpe,fse,cosphi,caller);

            //3. Evaluate the inverse force velocity curve        
            const ForceVelocityInverseCurve& fvInvCurve 
                = get_ForceVelocityInverseCurve(); 

            dlceN = fvInvCurve.calcValue(fv);
            dlce  = dlceN*getMaxContractionVelocity()*optFibLen;

    }else if( isTendonElastic() && get_use_fiber_damping() == true){
            
            double a = 0;
            if(isActivationAState()){
                a = getStateVariable(s, STATE_ACTIVATION_NAME);
            }else{
                a = getControl(s);
            }

            if(get_use_second_order_activation()){
                a = act2Mdl.clampActivation(a);
            }else{
                a = actMdl.clampActivation(a);
            }

            const TendonForceLengthCurve& fseCurve 
                = get_TendonForceLengthCurve();
            double fse  = fseCurve.calcValue(tlN);
                  
            //Newton solve for fiber velocity
            fv = 1.0;
            dlce = -1;
            dlceN = -1;
            double beta = get_fiber_damping();

            SimTK_ERRCHK1_ALWAYS(beta > SimTK::SignificantReal, 
                fcnName.c_str(),
                "%s: Damping must be greater than 0!", muscleName.c_str());

            
            SimTK::Vec3 fiberVelocityV 
                = calcDampedNormFiberVelocity(fiso,a,fal,fpe,fse,beta, cosphi);

            //If the Newton method converged update the fiber velocity
            if(fiberVelocityV[2] > 0.5){
                dlceN = fiberVelocityV[0];
                dlce  = dlceN*getOptimalFiberLength()
                                *getMaxContractionVelocity();
                fv = fvCurve.calcValue(dlceN);
            }else{
                cout << getName() <<
                    " Fiber Velocity Newton Method Did Not Converge" <<endl;
            }
           
        
    }else{
        printf("%s: Muscle Illegally Configured. This should not be "
               "possible to do. If you see this message contact the OpenSim"
               "deveopment team.\n",caller.c_str());
        dlce = SimTK::NaN;
        dlceN = SimTK::NaN;
        fv = SimTK::NaN;
    } 
   
    //7. Compute the other related velocity components
    //double dlceAT = penMdl.
    double tanPhi = tan(phi);
    double dphidt    = penMdl.calcPennationAngularVelocity(tanPhi,lce,
                                                            dlce   ,caller);    
    double dlceAT = penMdl.calcFiberVelocityAlongTendon(lce,
                                                    dlce,sinphi,cosphi, dphidt);
    double dmcldt = getLengtheningSpeed(s);
    double dtl = 0;

    if(isTendonElastic()){
        dtl       = penMdl.calcTendonVelocity(cosphi,sinphi,dphidt,
                                                    lce,  dlce,dmcldt);
    }
  
    //Check if the state of the fiber is clamped or not.
    double fiberStateClamped = 0.0;
    if(isFiberStateClamped(lce,dlce)){
        dlce = 0;
        dlceN = 0;
        dlceAT = 0;
        dphidt = 0;
        dtl = dmcldt;
        fv = 1.0; // to be consistent with a fiber velocity of 0
        fiberStateClamped = 1.0;
    }

    //Populate the struct;
    fvi.fiberVelocity               = dlce;
    fvi.normFiberVelocity           = dlceN;
    fvi.fiberVelocityAlongTendon    = dlceAT;

    fvi.pennationAngularVelocity    = dphidt;

    fvi.tendonVelocity              = dtl;
    fvi.normTendonVelocity = dtl/getTendonSlackLength();

    fvi.fiberForceVelocityMultiplier = fv;

    fvi.userDefinedVelocityExtras.resize(1);
    fvi.userDefinedVelocityExtras[0] = fiberStateClamped;
}

//==============================================================================
//==============================================================================
//
// Muscle Dynamics Info
//
//==============================================================================
//==============================================================================

void Millard2012EquilibriumMuscle::
    calcMuscleDynamicsInfo(const SimTK::State& s, 
                                       MuscleDynamicsInfo& mdi) const
{
       
        SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

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
    const MuscleSecondOrderActivationDynamicModel& act2Mdl 
        = get_MuscleSecondOrderActivationDynamicModel();

    double a = 0;   
    if(isActivationAState()){
        a = getStateVariable(s, STATE_ACTIVATION_NAME);
    }else{
        a = getControl(s);
    }

    if(get_use_second_order_activation()){
        a = act2Mdl.clampActivation(a);
    }else{
        a = actMdl.clampActivation(a);
    }


    double lce      = mli.fiberLength;
    double lceN     = lce/optFiberLen;
    double dlce     = mvi.fiberVelocity;
    double dlceN    = mvi.normFiberVelocity;
    double phi      = mli.pennationAngle;
    double cosPhi   = mli.cosPennationAngle;
    double sinPhi   = mli.sinPennationAngle;

    double tl   = mli.tendonLength; 
    double dtl  = mvi.tendonVelocity;
    double tlN  = mli.normTendonLength;
   
    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;
    double fv   = mvi.fiberForceVelocityMultiplier; 
  
    double fiberStateClamped = mvi.userDefinedVelocityExtras[0];

    //Compute the stiffness of the muscle fiber
    SimTK_ERRCHK1_ALWAYS(lce > SimTK::SignificantReal, fcnName.c_str(),
        "%s: The muscle fiber has a length of 0, and is causing a singularity", 
        muscleName.c_str());
    SimTK_ERRCHK1_ALWAYS(cosPhi > SimTK::SignificantReal, fcnName.c_str(),
        "%s: Pennation angle is 90 degrees, and is causing a singularity", 
        muscleName.c_str());

    double fm           = 0; //total fiber force
    double aFm          = 0; //active fiber force
    double pFm          = 0; //passive fiber force
    double fmAT         = 0;
    double dFm_dlce     = 0;
    double dFmAT_dlceAT = 0;
    double dFt_dtl      = 0;
    double Ke           = 0;

    

    if(fiberStateClamped < 0.5){
        SimTK::Vec3 fiberForceV;

        fiberForceV   = calcFiberForce(fiso,a,fal,fv,fpe,dlceN);
        fm = fiberForceV[0];
        aFm   = fiberForceV[1];
        pFm   = fiberForceV[2];

        //Every method except the rigid tendon chooses a fiber velocity
        //that ensures that the fiber does not generate a compressive force
        //Here we must enforce that the fiber generates only tensile forces
        //by saturating the damping force generated by the parallel element
        int simMethod = getSimulationMethod();
        if(isTendonElastic() == false){
            if(fm < 0){
                fm = 0;
                pFm= -aFm;
            }
        }

        fmAT  = calcFiberForceAlongTendon(fm,cosPhi);               
        dFm_dlce = calcFiberStiffness(fiso,a,fv,lceN,optFiberLen);
        dFmAT_dlceAT=calc_DFiberForceAT_DFiberLengthAT(  dFm_dlce,sinPhi,
                                                                cosPhi,lce);   
        //Compute the stiffness of the tendon
        if(isTendonElastic()){
            dFt_dtl    = fseCurve.calcDerivative(tlN,1)*(fiso/tendonSlackLen);

            //Compute the stiffness of the whole muscle/tendon complex
            if (    abs(dFmAT_dlceAT*dFt_dtl) > 0.0
                &&  abs(dFmAT_dlceAT+dFt_dtl) > SimTK::SignificantReal){
                Ke = (dFmAT_dlceAT*dFt_dtl)/(dFmAT_dlceAT+dFt_dtl);
            }
        }else{
            dFt_dtl = SimTK::Infinity;
            Ke = dFmAT_dlceAT;
        }
    }
    
    double fse = 0;
    if(isTendonElastic()){
        fse  = fseCurve.calcValue(tlN);
    }else{
        fse  = fmAT/fiso;
    }


    mdi.activation                   = a;
    mdi.fiberForce                   = fm; 
    mdi.fiberForceAlongTendon        = fmAT;
    mdi.normFiberForce               = fm/fiso;
    mdi.activeFiberForce             = aFm;
    mdi.passiveFiberForce            = pFm;                                        
                                     
    mdi.tendonForce                  = fse*fiso;
    mdi.normTendonForce              = fse;
                                     
    mdi.fiberStiffness               = dFm_dlce;
    mdi.fiberStiffnessAlongTendon    = dFmAT_dlceAT;
    mdi.tendonStiffness              = dFt_dtl;
    mdi.muscleStiffness              = Ke;
                                     
    //Check that the derivative of system energy less work is zero within
    //a reasonable numerical tolerance. Throw an exception if this is not true
    
    double dphidt       = mvi.pennationAngularVelocity;
    
    double dFibPEdt     = mdi.passiveFiberForce*mvi.fiberVelocity;
    double dTdnPEdt     = fse*fiso*dtl;

    double dFibWdt      = -mdi.activeFiberForce*mvi.fiberVelocity;
    double dmcldt       = getLengtheningSpeed(s);
    double dBoundaryWdt = mdi.tendonForce * dmcldt;

    double dSysEdt = (dFibPEdt + dTdnPEdt) - dFibWdt - dBoundaryWdt;
    double tol = sqrt(SimTK::Eps);
    
    /////////////////////////////
    //Populate the power entries
    /////////////////////////////
    mdi.fiberActivePower    =   dFibWdt; 
    mdi.fiberPassivePower   =   -(dFibPEdt); 
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
// Protected Numerical Services
//==============================================================================

/** Get the rate change of activation */
double Millard2012EquilibriumMuscle::
    calcActivationDerivative(const SimTK::State& s, int order) const 
{    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    double val = 0;

    if(isActivationAState()){        
        if(get_use_second_order_activation()){

            const MuscleSecondOrderActivationDynamicModel& act2Mdl 
            = get_MuscleSecondOrderActivationDynamicModel();

            double u    = getExcitation(s);
            double a    = act2Mdl.clampActivation(getActivation(s));   
            double dadt = getStateVariable(s,STATE_ACTIVATION_DERIVATIVE_NAME);

            switch(order){                
                case 0:{
                           val = a;
                       }break;
                case 1:{
                            val = dadt;
                       }break;
                case 2:{
                            val = act2Mdl.calcDerivative(dadt,a,u);
                       }break;
                default:
                    SimTK_ERRCHK_ALWAYS(false, 
                        "Millard2012EquilibriumMuscle::calcActivationDerivative",
                        "Invalid derivative requested");
            }

        }else{
            const MuscleFirstOrderActivationDynamicModel& actMdl         
                = get_MuscleFirstOrderActivationDynamicModel();

                double u    = getExcitation(s);
                double a    = actMdl.clampActivation(getActivation(s)); 

                switch(order){                
                    case 0:{
                               val = a;
                           }break;
                    case 1:{
                                val = actMdl.calcDerivative(a,u);
                           }break;
                    default:
                        SimTK_ERRCHK_ALWAYS(false, 
                            "Millard2012EquilibriumMuscle::calcActivationDerivative",
                            "Invalid derivative requested");
                }

            }
        
        
    }
    return val;
}  

//==============================================================================
// Private Numerical Services
//==============================================================================
int Millard2012EquilibriumMuscle::
    getSimulationMethod() const
{
    int simMethod = 0;

    bool ignoreTendonCompliance     = get_ignore_tendon_compliance();
    bool ignoreActivationDynamics   = get_ignore_activation_dynamics();
    bool useFiberDamping            = get_use_fiber_damping();

    return calcSimulationMethod(ignoreTendonCompliance,
                                ignoreActivationDynamics,
                                useFiberDamping);
   
}

int Millard2012EquilibriumMuscle::
    calcSimulationMethod(   bool ignoreTendonCompliance,
                            bool ignoreActivationDynamics,
                            bool useFiberDamping) const
{
    int simMethod = 0;
    if(ignoreTendonCompliance){
        if(ignoreActivationDynamics){
            if(useFiberDamping){
                simMethod = RigidTendon_DampedFiber_NoActivation;
            }else{
                simMethod = RigidTendon_NoActivation;
            }

        }else{
            if(useFiberDamping){
                simMethod = RigidTendon_DampedFiber_Activation;
            }else{
                simMethod = RigidTendon_Activation;
            }
        }
    }else{
        if(ignoreActivationDynamics){
            if(useFiberDamping){
                simMethod = ElasticTendon_DampedFiber_NoActivation;
            }else{
                simMethod = ElasticTendon_NoActivation;
            }
        }else{
            if(useFiberDamping){
                simMethod = ElasticTendon_DampedFiber_Activation;
            }else{
                simMethod = ElasticTendon_Activation;
            }
        }
    }
    return simMethod;
}

bool Millard2012EquilibriumMuscle::isFiberLengthAState() const
{
    bool fiberLengthState = false;

    switch(getSimulationMethod()){
        case RigidTendon_NoActivation:            
            fiberLengthState = false;
            break;
        case RigidTendon_Activation:              
            fiberLengthState = false;
            break;
        case RigidTendon_DampedFiber_NoActivation:
            fiberLengthState = false;
            break;
        case RigidTendon_DampedFiber_Activation:  
            fiberLengthState = false;
            break;
        case ElasticTendon_DampedFiber_NoActivation:
            fiberLengthState = true;
            break; 
        case ElasticTendon_DampedFiber_Activation:
            fiberLengthState = true;
            break;
        case ElasticTendon_NoActivation:
            fiberLengthState = true;
            break; 
        case ElasticTendon_Activation:
            fiberLengthState = true;
            break;        
        default:
            SimTK_ASSERT(false,
                "Invalid muscle configuration "
                "returned by getSimulationMethod()");
    }

    return fiberLengthState;
}

bool Millard2012EquilibriumMuscle::isTendonElastic() const
{
    bool elasticTendon = false;
    switch(getSimulationMethod()){ 
         case RigidTendon_NoActivation:            
            elasticTendon = false;
            break;
        case RigidTendon_Activation:              
            elasticTendon = false;
            break;
        case RigidTendon_DampedFiber_NoActivation:
            elasticTendon = false;
            break;
        case RigidTendon_DampedFiber_Activation:  
            elasticTendon = false;
            break;
        case ElasticTendon_DampedFiber_NoActivation:
            elasticTendon = true;
            break;
        case ElasticTendon_DampedFiber_Activation:
            elasticTendon = true;
            break;
        case ElasticTendon_NoActivation:
            elasticTendon = true;
            break;
        case ElasticTendon_Activation:
            elasticTendon = true;
            break;
        default:
             SimTK_ASSERT(false,
                "Invalid muscle configuration "
                "returned by isTendonElastic()");
    }

    return elasticTendon;
}

bool Millard2012EquilibriumMuscle::isActivationAState() const
{
    bool activationState = false;

    switch(getSimulationMethod()){
        case RigidTendon_NoActivation:            
        case RigidTendon_DampedFiber_NoActivation:                   
        case ElasticTendon_NoActivation:            
        case ElasticTendon_DampedFiber_NoActivation:
            activationState = false;
            break;
        case RigidTendon_Activation:    
        case RigidTendon_DampedFiber_Activation:            
        case ElasticTendon_Activation:         
        case ElasticTendon_DampedFiber_Activation:
            activationState = true;
            break;
        default:
            SimTK_ASSERT(false,
                "Invalid muscle configuration "
                "returned by getSimulationMethod()");
    }

    return activationState;

}


bool Millard2012EquilibriumMuscle::canStateGoSingular() const
{
    bool singularityPossible = false;
    switch(getSimulationMethod()){
        case RigidTendon_NoActivation:
            singularityPossible = false;
            break;
        case RigidTendon_Activation:
            singularityPossible = false;
            break;
        case RigidTendon_DampedFiber_NoActivation:
            singularityPossible = false;
            break;
        case RigidTendon_DampedFiber_Activation:
            singularityPossible = false;
            break;
        case ElasticTendon_NoActivation:
            singularityPossible = true;
            break;
        case ElasticTendon_Activation:
            singularityPossible = true;
            break;
        case ElasticTendon_DampedFiber_NoActivation:
            singularityPossible = false;
            break;
        case ElasticTendon_DampedFiber_Activation:
            singularityPossible = false;
            break;

        default:
            SimTK_ASSERT(false,
                "Invalid muscle configuration "
                "returned by getSimulationMethod()");
    }

    return singularityPossible;
}

bool Millard2012EquilibriumMuscle::
    isFiberStateClamped(double lce,double dlceN) const
{
    bool clamped = false;
    int simMethod = getSimulationMethod();

    //Get the minimum active fiber length in units of meters.    
    double minFiberLength = getMinimumFiberLength();

    //Is the fiber length  clamped and it is shortening, then the fiber length
    //is either shorter than the pennation model allows, or shorter than the
    //active fiber force length curve allows.
        if( (lce <= minFiberLength &&  dlceN <= 0) || lce < minFiberLength){
            clamped = true;
        }

    return clamped;
}


double Millard2012EquilibriumMuscle::getMinimumFiberLength() const
{
    return m_minimumFiberLength;
}


double Millard2012EquilibriumMuscle::getMinimumFiberLengthAlongTendon() const
{
    return m_minimumFiberLengthAlongTendon;
}

double Millard2012EquilibriumMuscle::clampFiberLength(double lce) const
{
    return max(lce, getMinimumFiberLength());
}

double Millard2012EquilibriumMuscle::calcFv(double a, 
                                            double fal, 
                                            double fp,                                                                 
                                            double fse, 
                                            double cosphi,
                                            std::string& caller) const
{
    SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::SignificantReal,
        "Millard2012EquilibriumMuscle::calcFv",
        "%s: The pennation angle is 90 degrees, and is causing a singularity", 
        caller.c_str());

    double fv = ( (fse)/cosphi - (fp) ) / (a*fal);
    return fv;
}

SimTK::Vec3 Millard2012EquilibriumMuscle::
    calcDampedNormFiberVelocity(  double fiso,
                            double a,
                            double fal,
                            double fpe,
                            double fse,
                            double beta,                            
                            double cosPhi) const
{
    SimTK::Vec3 fiberForceV;
    SimTK::Vec3 result;
    
    int maxIter = 20; //This converges fast - 20 iterations is quite generous
    double tol = 1e-8*fiso;
    //Necessary if anyone is simulating an insect ...
    if(tol < SimTK::SignificantReal*100){
        tol = SimTK::SignificantReal*100;
    }
    double perturbation = 0;
    double fiberForce = 0;

    double err = 1e10;
    double derr_d_dlceNdt = 0;
    double delta = 0;
    double iter = 0;

    double dlceN_dt = 0;    
    double fv = 0;
    double df_d_dlceNdt = 0;

    while(abs(err) > tol && iter < maxIter){

        fv     = fvCurve.calcValue(dlceN_dt);       
        fiberForceV = calcFiberForce(fiso,a,fal,fv,fpe,dlceN_dt);        
        fiberForce = fiberForceV[0];

        err = fiberForce*cosPhi - fse*fiso;        
        df_d_dlceNdt = calc_DFiberForce_DNormFiberVelocity(fiso,a,fal,
                                                             beta,dlceN_dt);
        derr_d_dlceNdt = df_d_dlceNdt*cosPhi;

        if(abs(err) > tol && abs(derr_d_dlceNdt) > SimTK::SignificantReal){
            delta = -err/derr_d_dlceNdt;
            /*if(abs(delta) > 0.25){
                delta = sign(delta)*0.25;
            }*/
            dlceN_dt = dlceN_dt + delta;
            
        }else if(abs(derr_d_dlceNdt) < SimTK::SignificantReal){
            //Perturb the solution if we've lost rank
            //This should never happen for this problem as dfv_d_dlceNdt > 0
            //and b > 0 and so derr_d_dlceNdt > 0
            perturbation = 2.0*((double)rand())/((double)RAND_MAX)-1.0;
            dlceN_dt = dlceN_dt + perturbation*0.05;
        }
       
        iter++;
    }

    
    double converged = 1.0;

    //If we failed to converge, it's because the fiber is probably at 
    //its lower bound. That decision is made further down the line, so if
    //convergence didn't happen the the user know and give them a NaN
    if(abs(err) > tol){
        dlceN_dt = -1;
        converged = 0.0;
    }

    result[0] = dlceN_dt;
    result[1] = err;
    result[2] = converged;

    return result;
}

double Millard2012EquilibriumMuscle::
    calc_DFiberForce_DNormFiberVelocity(double fiso, 
                                        double a, 
                                        double fal,                                                                   
                                        double beta,
                                        double dlceN_dt) const
{
    double dfv_d_dlceNdt = fvCurve.calcDerivative(dlceN_dt,1);
    // d/dlceN_dt(fm = fiso * (a*fal*fv +fpe + b*dlceN_dt));
    //  dfm_d_dlceNdt= fiso * (a*fal*dfv_d_dlceNdt + b)    
    double dfm_d_dlceNdt = fiso * (a*fal*dfv_d_dlceNdt + beta);
    return dfm_d_dlceNdt;
}


SimTK::Vec3 Millard2012EquilibriumMuscle::
    calcFiberForce( double fiso, 
                    double a, 
                    double fal,
                    double fv,                             
                    double fpe,
                    double dlceN) const
{
    //The force the fiber generates parallel to the fiber
    //double fm = fiso * (a*fal*fv + fpe + beta*dlceN);
    double beta = 0;

    if(get_use_fiber_damping() == true){
        beta = get_fiber_damping();
    }

    double fa = fiso * (a*fal*fv);
    double fp = fiso * (fpe + beta*dlceN);
    double fm = fa + fp;
    

    SimTK::Vec3 fiberF;
    fiberF[0] = fm;
    fiberF[1] = fa;
    fiberF[2] = fp;

    return fiberF;
}

double Millard2012EquilibriumMuscle::
    calcActivation( double fiso, 
                    double ftendon,
                    double cosPhi,
                    double fal,
                    double fv,                             
                    double fpe,
                    double dlceN) const
{
    //The force the fiber generates parallel to the fiber
    //double ft = fm cosPhi = fiso * (a*fal*fv + fpe + beta*dlceN) cosPhi;
    double beta = 0;

    if(get_use_fiber_damping() == true){
        beta = get_fiber_damping();
    }

    double activation = 0;

    //If the fiber cannot generate any force due to its pennation angle,
    //active-force--length or force-velocity values, leave activation as 0. 
    if(cosPhi > SimTK::SignificantReal && fal*fv > SimTK::SignificantReal){
        activation = ( (ftendon /(fiso*cosPhi)) - fpe - beta*dlceN ) / (fal*fv);
    }

    

    return activation;
}


double Millard2012EquilibriumMuscle::
    calcFiberForceAlongTendon(  double fiberForce,                                
                                double cosPhi) const
{
    //The force the fiber generates parallel to the fiber
    //double fmAT = fiso * ((a*fal*fv + fpe)*cosPhi);
    double fmAT = fiberForce*cosPhi;
    return fmAT;
}

double Millard2012EquilibriumMuscle::
    calcFiberStiffness( double fiso, 
                        double a,                         
                        double fv,                                                     
                        double lceN,
                        double optFibLen) const
{
    
    std::string caller = getName();
    caller.append(".calcFiberStiffness");

    const FiberForceLengthCurve& fpeCurve 
        = get_FiberForceLengthCurve(); 
    const ActiveForceLengthCurve& falCurve
        = get_ActiveForceLengthCurve(); 
       
    double DlceN_Dlce = 1/optFibLen;

    double Dfal_Dlce     = falCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfpe_Dlce     = fpeCurve.calcDerivative(lceN,1) * DlceN_Dlce; 
    

    // The stiffness of the fiber parallel to the fiber
    //    d/dlce( fiso * (a *  fal  *fv + fpe + beta*dlceN_dt);
    //double DFm_Dlce = fiso * (a*Dfal_Dlce*fv + Dfpe_Dlce);
    double DFm_Dlce = fiso * ((a*Dfal_Dlce*fv + Dfpe_Dlce));

    return DFm_Dlce;
}

double Millard2012EquilibriumMuscle::
    calc_DFiberForceAT_DFiberLength(double fiberForce,
                                    double fiberStiffness,
                                    double lce,
                                    double sinPhi,
                                    double cosPhi) const
{
    std::string caller = getName();
    caller.append("calcFiberStiffnessAlongTendon");
       
    double Dphi_Dlce     = penMdl.calc_DPennationAngle_DfiberLength(lce,caller);
    double Dcosphi_Dlce  = -sinPhi*Dphi_Dlce;
    
    //double Dfcphi_Dcosphi= fcphiCurve.calcDerivative(cosPhi,1);
    //double Dfcphi_Dlce   = Dfcphi_Dcosphi * Dcosphi_Dlce;
    //The stiffness of the fiber in the direction of the tendon
    //1. Compute the stiffness of the fiber along the direction of the 
    //   tendon, for small changes in length parallel to the fiber
    //   that is: D(FiberForceAlongTendon) D (fiberLength)
    //
    // dFmAT/dlce = d/dlce( fiso * (a *fal*fv + fpe + beta*dlceN)*cosPhi )
    double DfmAT_Dlce = fiberStiffness*cosPhi  
                       +fiberForce*Dcosphi_Dlce;
   
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
