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
    STATE_FIBER_LENGTH_NAME = "fiber_length"; 
const string Millard2012EquilibriumMuscle::
    MODELING_OPTION_USE_REDUCED_MODEL_NAME = "use_reduced_fiber_dynamics";

//const std::string Millard2012EquilibriumMuscle::
//    PREVIOUS_FIBER_LENGTH_NAME = "previous_fiber_length";
//const std::string Millard2012EquilibriumMuscle::
//    PREVIOUS_FIBER_VELOCITY_NAME = "previous_fiber_velocity";


//const static int MLIfse     = 0;
//const static int MLIfk      = 1;
//const static int MLIfcphi   = 2;
//const static int MLIfkPE    = 3;
//const static int MLIfcphiPE = 4;

const static int RigidTendonNoActivation            = 0;
const static int RigidTendonActivation              = 1;
const static int ApproxElasticTendonNoActivation    = 2;
const static int ApproxElasticTendonActivation      = 3;
const static int ElasticTendonNoActivation          = 4;
const static int ElasticTendonActivation            = 5;

//=============================================================================
// PROPERTY MANAGEMENT
//=============================================================================
void Millard2012EquilibriumMuscle::setNull()
{
    setAuthors("Matthew Millard");
}


void Millard2012EquilibriumMuscle::constructProperties()
{
   
    constructProperty_use_reduced_fiber_dynamics(false);

    constructProperty_default_fiber_length(getOptimalFiberLength());
    
    MuscleFirstOrderActivationDynamicModel actMdl;
    actMdl.setMinimumActivation(0.01);

    constructProperty_MuscleFirstOrderActivationDynamicModel(
        actMdl);
    
    constructProperty_default_activation(actMdl.getMinimumActivation());

    constructProperty_ActiveForceLengthCurve(
        ActiveForceLengthCurve());

    constructProperty_ForceVelocityInverseCurve(
        ForceVelocityInverseCurve());

    constructProperty_FiberForceLengthCurve(
        FiberForceLengthCurve());

    constructProperty_TendonForceLengthCurve(
        TendonForceLengthCurve());

    

    /*
    constructProperty_FiberCompressiveForceLengthCurve(
        FiberCompressiveForceLengthCurve());

    constructProperty_FiberCompressiveForceCosPennationCurve(
        FiberCompressiveForceCosPennationCurve());       
    */
}

void Millard2012EquilibriumMuscle::buildMuscle()
{
    double optFibLen = getOptimalFiberLength();
    double optPenAng = getPennationAngleAtOptimalFiberLength();
    std::string caller = getName();
    caller.append(".buildMuscle()");

    penMdl = MuscleFixedWidthPennationModel(optFibLen,
                                            optPenAng,
                                            acos(0.1),
                                            caller);
    
    //Ensure all of the object names are uptodate
    std::string aName = getName();

    std::string tmp = aName;
    tmp.append("_MuscleFirstOrderActivationDynamicModel");
    MuscleFirstOrderActivationDynamicModel& actMdl 
        = upd_MuscleFirstOrderActivationDynamicModel();
    
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

     /*Handle Model Setting Specific Singularities:
      1. Activation Singularity
      2. Active Force Length Singularity
      3. Pennation singularity
      4. Force velocity curve singularity

      *Note even when there isn't a pennation singularity in the model, 
       we can go closer to Pi/2, but we cannot reach Pi/2 as this causes 
       a singularity in the calculation of pennation angular velocity
      */
     switch(getInitialSimulationMethod()){
        case RigidTendonNoActivation:
            {//No singularities
                actMdl.setMinimumActivation(0.0);
                falCurve.setMinValue(0.0);                
                penMdl.setMaximumPennationAngle(acos(0.001));
                fvCurve.setConcentricMinSlope(0.0);
                fvCurve.setEccentricMinSlope(0.0);
            }break;
        case RigidTendonActivation:
            {//No singularities
                actMdl.setMinimumActivation(0.0);
                falCurve.setMinValue(0.0);
                penMdl.setMaximumPennationAngle(acos(0.001));
                fvCurve.setConcentricMinSlope(0.0);
                fvCurve.setEccentricMinSlope(0.0);
            }break;
        case ApproxElasticTendonNoActivation:
            {//No singularities, but fv should be invertible to guarantee
             //                  convergence of the Newton method
                actMdl.setMinimumActivation(0.0);
                //falCurve.setMinValue(0.0);
                penMdl.setMaximumPennationAngle(acos(0.001));

                SimTK_ERRCHK1_ALWAYS(
                    fvCurve.getConcentricMinSlope()> SimTK::SignificantReal
                    && fvCurve.getEccentricMinSlope() > SimTK::SignificantReal,
                    caller.c_str(),
                    "%s: ForceVelocityCurve: ConcentricMinSlope and "
                    "EccentricMinSlope must be greater than 0.\n", 
                    getName().c_str());


            }break;
        case ApproxElasticTendonActivation:
            {//No singularities, but fv should be invertible to guarantee
             //                  convergence of the Newton method
                actMdl.setMinimumActivation(0.0);
                //falCurve.setMinValue(0.0);
                penMdl.setMaximumPennationAngle(acos(0.001));

                SimTK_ERRCHK1_ALWAYS(
                    fvCurve.getConcentricMinSlope()> SimTK::SignificantReal
                    && fvCurve.getEccentricMinSlope() > SimTK::SignificantReal,
                    caller.c_str(),
                    "%s: ForceVelocityCurve: ConcentricMinSlope and "
                    "EccentricMinSlope must be greater than 0.\n", 
                    getName().c_str());

            }break;

        case ElasticTendonNoActivation:
            {
             SimTK_ERRCHK1_ALWAYS(
                actMdl.getMinimumActivation() > SimTK::SignificantReal,
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
                fvInvCurve.getConcentricMinSlope()> SimTK::SignificantReal
                && fvInvCurve.getEccentricMinSlope()>SimTK::SignificantReal,
                caller.c_str(),
                "%s: ForceVelocityInverseCurve: ConcentricMinSlope and "
                "EccentricMinSlope must be greater than 0.\n", 
                getName().c_str());

            }break;
        case ElasticTendonActivation:
            {
            SimTK_ERRCHK1_ALWAYS(
                actMdl.getMinimumActivation() > SimTK::SignificantReal,
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
                fvInvCurve.getConcentricMinSlope()> SimTK::SignificantReal
                && fvInvCurve.getEccentricMinSlope()>SimTK::SignificantReal,
                caller.c_str(),
                "%s: ForceVelocityInverseCurve: ConcentricMinSlope and "
                "EccentricMinSlope must be greater than 0.\n", 
                getName().c_str());

            }break;
        default: 
            {
            SimTK_ERRCHK1_ALWAYS(
                actMdl.getMinimumActivation() > SimTK::SignificantReal,
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
                fvInvCurve.getConcentricMinSlope()> SimTK::SignificantReal
                && fvInvCurve.getEccentricMinSlope()>SimTK::SignificantReal,
                caller.c_str(),
                "%s: ForceVelocityInverseCurve: ConcentricMinSlope and "
                "EccentricMinSlope must be greater than 0.\n", 
                getName().c_str());
            }
    }

    //Compute the minimum fiber length;

    //Minimum active fiber length in units of meters    
    double minActiveFiberLength 
        = falCurve.getMinActiveFiberLength()*getOptimalFiberLength();
 
    //Minimum pennated fiber lenght in units of meters.
    double minPennatedFiberLength = penMdl.getMinimumFiberLength();

    m_minimumFiberLength =  max(minActiveFiberLength,minPennatedFiberLength);
        
    //Minimum fiber length along the tendon
    double phi = penMdl.calcPennationAngle(m_minimumFiberLength);
    
    m_minimumFiberLengthAlongTendon 
        = penMdl.calcFiberLengthAlongTendon(m_minimumFiberLength,cos(phi));
   

    reducedFiberStateHint(0) = 0;           //n-1 fiber length
    reducedFiberStateHint(1) = 0;           //n-1 fiber velocity
    reducedFiberStateHint(2) = 0;           //n-1 time
    reducedFiberStateHint(3) = 0;           //n-2 fiber velocity
    reducedFiberStateHint(4) = 0;           //n-2 time;
    
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

    //addModelingOption(MODELING_OPTION_USE_REDUCED_MODEL_NAME, 1);

    ensureMuscleUpToDate();
    double value = 0.0;

  
    switch(getInitialSimulationMethod()){
        case RigidTendonNoActivation:{             
            }break;
        case RigidTendonActivation:{
                addStateVariable(STATE_ACTIVATION_NAME);     
                addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", 
                                    value, 
                                    SimTK::Stage::Dynamics);    
               } break;
        case ApproxElasticTendonNoActivation:{
              /*addDiscreteVariable(PREVIOUS_FIBER_LENGTH_NAME, 
                    SimTK::Stage::Topology); 
                addDiscreteVariable(PREVIOUS_FIBER_VELOCITY_NAME, 
                    SimTK::Stage::Topology);*/

               }break;
        case ApproxElasticTendonActivation:{
                addStateVariable(STATE_ACTIVATION_NAME);     
                addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", 
                                    value, 
                                    SimTK::Stage::Dynamics);   

              /* addDiscreteVariable(PREVIOUS_FIBER_LENGTH_NAME, 
                    SimTK::Stage::Topology); 
                addDiscreteVariable(PREVIOUS_FIBER_VELOCITY_NAME, 
                    SimTK::Stage::Topology);*/

                
               }break;
        case ElasticTendonNoActivation:{ 
                               
                    addStateVariable(STATE_FIBER_LENGTH_NAME);  
	                addCacheVariable(STATE_FIBER_LENGTH_NAME+"_deriv", 
                                    value, 
                                    SimTK::Stage::Dynamics);
               }break;
        case ElasticTendonActivation:{
                    addStateVariable(STATE_ACTIVATION_NAME);
                    addStateVariable(STATE_FIBER_LENGTH_NAME);  

                    addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", 
                                    value, 
                                    SimTK::Stage::Dynamics);

	                addCacheVariable(STATE_FIBER_LENGTH_NAME+"_deriv", 
                                    value, 
                                    SimTK::Stage::Dynamics);
               }break;
        default:{ 
                    addStateVariable(STATE_ACTIVATION_NAME);
                    addStateVariable(STATE_FIBER_LENGTH_NAME);
                    addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", 
                        value, 
                        SimTK::Stage::Dynamics);

	                addCacheVariable(STATE_FIBER_LENGTH_NAME+"_deriv", 
                                     value, 
                                     SimTK::Stage::Dynamics);

                }
    }
    

    

    
	
}

void Millard2012EquilibriumMuscle::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

    ensureMuscleUpToDate();

    switch(getInitialSimulationMethod()){
        case RigidTendonNoActivation:{
               }break;
        case RigidTendonActivation:{
                    setActivation(s, getDefaultActivation());    
               }break;
        case ApproxElasticTendonNoActivation:{
                    
               }break;
        case ApproxElasticTendonActivation:{
                    setActivation(s, getDefaultActivation());                
               }break;
        case ElasticTendonNoActivation:{
                    setFiberLength(s, getDefaultFiberLength());
               }break;
        case ElasticTendonActivation:{
                    setActivation(s, getDefaultActivation());
                    setFiberLength(s, getDefaultFiberLength());
               }break;
        default:{
                    setActivation(s, getDefaultActivation());
                    setFiberLength(s, getDefaultFiberLength());
                }    
    }

}
    

void Millard2012EquilibriumMuscle::
    setPropertiesFromState(const SimTK::State& s)
{
    Super::setPropertiesFromState(s);
    
    ensureMuscleUpToDate();

    switch(getInitialSimulationMethod()){
        case RigidTendonNoActivation:{ 
               }break;
        case RigidTendonActivation:{
                    setDefaultActivation(
                        getStateVariable(s,STATE_ACTIVATION_NAME));
               }break;
        case ApproxElasticTendonNoActivation:{ 
                         
               }break;
        case ApproxElasticTendonActivation:{ 
                    setDefaultActivation(
                        getStateVariable(s,STATE_ACTIVATION_NAME));
               }break;
        case ElasticTendonNoActivation:{ 
                    setDefaultFiberLength(
                        getStateVariable(s,STATE_FIBER_LENGTH_NAME));
               }break;
        case ElasticTendonActivation:{ 
                    setDefaultActivation(
                        getStateVariable(s,STATE_ACTIVATION_NAME));
                    setDefaultFiberLength(
                        getStateVariable(s,STATE_FIBER_LENGTH_NAME));
               }break;
        default:{
                    setDefaultActivation(
                        getStateVariable(s,STATE_ACTIVATION_NAME));
                    setDefaultFiberLength(
                        getStateVariable(s,STATE_FIBER_LENGTH_NAME));
                }
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

   
    switch (getInitialSimulationMethod()){
        case RigidTendonNoActivation:{ //Rigid tendon                              
               } break ;
        case RigidTendonActivation:{ //Rigid tendon + activation dynamics
                    if (!isDisabled(s)) {
                        derivs[0] = getActivationRate(s);
                    }  
               } break;
        case ApproxElasticTendonNoActivation:{ 
                if(!isDisabled(s)){
                    //if(abs(reducedFiberStateHint(2)-s.getTime())
                    //     > SimTK::SignificantReal){
                        reducedFiberStateHint(0) = getFiberLength(s);
                        reducedFiberStateHint(3) = reducedFiberStateHint(1);
                        reducedFiberStateHint(4) = reducedFiberStateHint(2);
                        reducedFiberStateHint(1) = getFiberVelocity(s);
                        reducedFiberStateHint(2) = s.getTime();
                    //}
                }
               } break;
        case ApproxElasticTendonActivation:{
                 if (!isDisabled(s)) {
                    //if(abs(reducedFiberStateHint(2)-s.getTime())
                    //    > SimTK::SignificantReal){
                        derivs[0] = getActivationRate(s);
                        reducedFiberStateHint(0) = getFiberLength(s);
                        reducedFiberStateHint(3) = reducedFiberStateHint(1);
                        reducedFiberStateHint(4) = reducedFiberStateHint(2);
                        reducedFiberStateHint(1) = getFiberVelocity(s);
                        reducedFiberStateHint(2) = s.getTime();
                    //}
                 }            
               } break;
        case ElasticTendonNoActivation:{ 
                if (!isDisabled(s)) {
                    derivs[0] = getFiberVelocity(s);
                }
            } break ;
        case ElasticTendonActivation:{ 
                    if (!isDisabled(s)) {
                        derivs[0] = getActivationRate(s);
                        derivs[1] = getFiberVelocity(s);
                    }
               } break ;
        default :{
            if (!isDisabled(s)) {
                derivs[0] = getActivationRate(s);
                derivs[1] = getFiberVelocity(s);
            }
        }
    }

    return derivs;   
}

//=============================================================================
// STATE RELATED GET FUNCTIONS
//=============================================================================

double Millard2012EquilibriumMuscle::getDefaultActivation() const
{
    ensureMuscleUpToDate();

    double defaultActivation = 0;

    const MuscleFirstOrderActivationDynamicModel& actMdl = 
                get_MuscleFirstOrderActivationDynamicModel();
    defaultActivation =  
        actMdl.clampActivation(get_default_activation());

    return defaultActivation;
                        
}

double Millard2012EquilibriumMuscle::getDefaultFiberLength() const
{
    //This is the same regardless of simulation method.
    ensureMuscleUpToDate();
    return clampFiberLength(get_default_fiber_length());
}

double Millard2012EquilibriumMuscle::
    getActivationRate(const SimTK::State& s) const
{
    //ensureMuscleUpToDate();    

    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");
    double activationRate = 0;

    if(isActivationAState()){
        activationRate = calcActivationRate(s);
    }else{
        activationRate = 0;
    }

    return activationRate;
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
    //ensureMuscleUpToDate();
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
    const MuscleFirstOrderActivationDynamicModel &actMdl = 
        get_MuscleFirstOrderActivationDynamicModel();
    set_default_activation(actMdl.clampActivation(activation));    
}

void Millard2012EquilibriumMuscle::setDefaultFiberLength(double fiberLength)
{
    set_default_fiber_length(clampFiberLength(fiberLength));
}

void Millard2012EquilibriumMuscle::
    setActivation(SimTK::State& s, double activation) const
{   
    if(isActivationAState())
    { 
        const MuscleFirstOrderActivationDynamicModel& actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();
        double clampedActivation=actMdl.clampActivation(activation);
        setStateVariable(s,STATE_ACTIVATION_NAME,clampedActivation);    
        markCacheVariableInvalid(s,"velInfo");
        markCacheVariableInvalid(s,"dynamicsInfo");            
    }    
}

void Millard2012EquilibriumMuscle::
    setFiberLength(SimTK::State& s, double fiberLength) const
{
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

	double& cacheVariable = updCacheVariable<double>(s, aStateName + "_deriv");
	cacheVariable = aValue;
	markCacheVariableValid(s, aStateName + "_deriv");
}

//=============================================================================
// GET
//=============================================================================

/*int Millard2012EquilibriumMuscle::
    getSimulationMethod(const SimTK::State& s) const
{
    int simMethod = 0;

    bool ignoreTendonCompliance     = getIgnoreTendonCompliance(s);
    bool ignoreActivationDynamics   = getIgnoreActivationDynamics(s);
    bool reducedFiberDynamics       = getUseReducedFiberDynamics(s);

    return calcSimulationMethod(ignoreTendonCompliance,
                                ignoreActivationDynamics,
                                reducedFiberDynamics);
}*/


int Millard2012EquilibriumMuscle::
    getInitialSimulationMethod() const
{
    int simMethod = 0;

    bool ignoreTendonCompliance     = get_ignore_tendon_compliance();
    bool ignoreActivationDynamics   = get_ignore_activation_dynamics();
    bool reducedFiberDynamics       = get_use_reduced_fiber_dynamics();

    return calcSimulationMethod(ignoreTendonCompliance,
                                ignoreActivationDynamics,
                                reducedFiberDynamics);
   
}

int Millard2012EquilibriumMuscle::
    calcSimulationMethod(   bool ignoreTendonCompliance,
                            bool ignoreActivationDynamics,
                            bool useReducedFiberDynamics) const
{
    int simMethod = 0;
    if(ignoreTendonCompliance){
        if(ignoreActivationDynamics){
            simMethod = RigidTendonNoActivation;        
        }else{
            simMethod = RigidTendonActivation;
        }
    }else{
        if(ignoreActivationDynamics){
            if(useReducedFiberDynamics){
                simMethod = ApproxElasticTendonNoActivation;
            }else{
                simMethod = ElasticTendonNoActivation;
            }
        }else{
            if(useReducedFiberDynamics){
                simMethod = ApproxElasticTendonActivation;
            }else{
                simMethod = ElasticTendonActivation;
            }
        }
    }
    return simMethod;
}

double Millard2012EquilibriumMuscle::
    getTendonForceMultiplier(SimTK::State& s) const
{
    ensureMuscleUpToDate();
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    return mdi.normTendonForce;
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

double Millard2012EquilibriumMuscle::getMaximumPennationAngle() const
{
    ensureMuscleUpToDate();
    return penMdl.getMaximumPennationAngle();
}


double Millard2012EquilibriumMuscle::
    getMinimumActivation() const
{
    ensureMuscleUpToDate();
    const MuscleFirstOrderActivationDynamicModel &actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();   
    double minActivation = actMdl.getMinimumActivation();

    return minActivation;
}


double Millard2012EquilibriumMuscle::
    getFiberStiffnessAlongTendon(const SimTK::State& s) const
{
    ensureMuscleUpToDate();
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    return mdi.fiberStiffnessAlongTendon;
}

bool Millard2012EquilibriumMuscle::
    getUseReducedFiberDynamics() const
{
    return get_use_reduced_fiber_dynamics();
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

bool Millard2012EquilibriumMuscle::
    setMaximumPennationAngle(double maxPennationAngle) 
{
    
    if(maxPennationAngle >= 0 && maxPennationAngle <= acos(0.001)){   
           penMdl.setMaximumPennationAngle(maxPennationAngle);
           return true;
    }else{
        return false;
    }
}

bool Millard2012EquilibriumMuscle::
    setMinimumActivation(double minActivation)
{
    double actLowerBound = 0;

    //Equilibrium equations - there is a singularity at activation = 0;
    if(getInitialSimulationMethod() == ElasticTendonNoActivation ||
       getInitialSimulationMethod() == ElasticTendonActivation){
        actLowerBound = 0.001;
    }

    if(minActivation > actLowerBound && minActivation < 1){
        MuscleFirstOrderActivationDynamicModel& actMdl 
            = upd_MuscleFirstOrderActivationDynamicModel();
        actMdl.setMinimumActivation(minActivation);        
        return true;
    }else{
        return false;
    }
}


bool Millard2012EquilibriumMuscle::
    setUseReducedFiberDynamics(bool useReducedModel)
{
    set_use_reduced_fiber_dynamics(useReducedModel);
    return true;
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

        double muscleLength = getLength(s);
        double muscleVelocity = getLengtheningSpeed(s);
        double tendonSlackLength = getTendonSlackLength();
        double tendonVelocity = 0.0; //Inextensible tendon;

        double fiberLength  = penMdl.calcFiberLength(muscleLength,
                                             tendonSlackLength);
        double phi      = penMdl.calcPennationAngle(fiberLength);
      
        double fiberVelocity   = penMdl.calcFiberVelocity(fiberLength,
                                            sin(phi),cos(phi),
                                            muscleLength,tendonSlackLength,
                                         muscleVelocity,tendonVelocity,caller);       

        if(SimTK::isNaN(fiberVelocity) == false){  
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
    caller.append(  "::MillardEquilibriumMuscle"
                    "::calcActiveFiberForceAlongTendon");

    double activeFiberForce = 0;    
    //If the fiber is in a legal range, compute the force its generating
    if(isFiberStateClamped(fiberLength,fiberVelocity) == false){

        //Clamp activation to a legal range
        const MuscleFirstOrderActivationDynamicModel& actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();
        double clampedActivation = actMdl.clampActivation(activation);

        //Normalize fiber length and velocity
        double normFiberLength    = fiberLength/getOptimalFiberLength();
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
        double phi = penMdl.calcPennationAngle(fiberLength);

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
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setForce(s,         mdi.tendonForce);
    return( mdi.tendonForce );
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
    if(isTendonElastic()){

    try{
        
        //ensureMuscleUpToDate();
        

        //Initialize activation to the users desired setting
        const MuscleFirstOrderActivationDynamicModel& actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();
        setActivation(s, actMdl.clampActivation(getDefaultActivation()));

        //Initialize the multibody system to the initial state vector
        setFiberLength(s, getOptimalFiberLength());

        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

        //Compute an initial muscle state that develops the desired force and
        //shares the muscle stretch between the muscle fiber and the tendon 
        //according to their relative stiffness.    
        double activation = actMdl.clampActivation(getDefaultActivation());
        double excitation0 = 0;
        double dactivation_dt = actMdl.calcDerivative(activation, excitation0);

        int flag_status       = -1;
        double solnErr        = SimTK::NaN;
        double iterations     = SimTK::NaN;
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
        
        SimTK::Vector soln = estimateElasticTendonFiberState2(
                                        reducedFiberStateHint,
                                        activation, dactivation_dt,
                                        pathLength, pathLengtheningSpeed, 
                                        tol, maxIter);

        /*SimTK::Vector soln = estimateElasticTendonFiberState(
                    activation, 
                    pathLength, pathLengtheningSpeed, 
                    tol       , maxIter);*/

        flag_status    = (int)soln[0];
        solnErr        = soln[1];
        iterations     = (int)soln[2];
        fiberLength    = soln[3];
        fiberVelocity  = soln[4];
        tendonForce    = soln[5];
        
        /*if(getInitialSimulationMethod() == ApproxElasticTendonActivation ||
            getInitialSimulationMethod() == ApproxElasticTendonNoActivation)
        {
            if(flag_status == 0){
                setDiscreteVariable(s,
                    PREVIOUS_FIBER_LENGTH_NAME,fiberLength);
                setDiscreteVariable(s,
                    PREVIOUS_FIBER_VELOCITY_NAME,fiberVelocity);
            }else{
                setDiscreteVariable(s,
                    PREVIOUS_FIBER_LENGTH_NAME,getOptimalFiberLength()*0.99);
                setDiscreteVariable(s,
                    PREVIOUS_FIBER_VELOCITY_NAME,0);
            }
        }*/
        



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
        
        cerr << "    Continuing with initial tendon force of 0 " << endl;
        cerr << "    and a fiber length equal to the optimal fiber length ..." 
             << endl;

        setForce(s,0);
		setFiberLength(s,getOptimalFiberLength());

    }  

    }
}


SimTK::Vector Millard2012EquilibriumMuscle::
    estimateElasticTendonFiberState(double aActivation, 
                    double pathLength, double pathLengtheningSpeed, 
                    double aSolTolerance, int aMaxIterations) const
{
    //ensureMuscleUpToDate();   
    std::string caller = getName();
    caller.append(".estimateElasticTendonFiberState");
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
    const ForceVelocityInverseCurve& fvInvCurve 
        = get_ForceVelocityInverseCurve(); 

   
    //Shorter version of normalized muscle multipliers

    double fse = 0; //Normalized tendon (series element) force
    double fal = 0; //Normalized active force length multiplier
    double fv  = 0; //Normalized force-velocity multiplier
    double fvInv=0; //Normalized inverse force-velocity factor
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

    while( abs(ferr) > aSolTolerance 
        && iter < aMaxIterations
        && minFiberLengthCtr < 10){

            //Update the multipliers and their partial derivativaes
            fal         = falCurve.calcValue(lceN);
            fpe         = fpeCurve.calcValue(lceN);
            fse         = fseCurve.calcValue(tlN);            
                               
            //SINGULARITY: When phi = pi/2, or activation = 0;
            fv          = fvCurve.calcValue(dlceN);
                //calcFv(ma, fal, fpe, fk, fcphi, fse, cosphi, caller);
            fvInv       = fvInvCurve.calcValue(fv); 

            //Compute the force error edited to remove fk, fcphi terms
            Fm   = calcFiberForce(fiso,ma,fal,fv,fpe);
            FmAT = calcFiberForceAlongTendon(fiso,ma,fal,fv,fpe,cosphi);
            Ft = fse*fiso;
            ferr  = FmAT-Ft; 
        
            //Compute the partial derivative of the force error w.r.t. lce
            //Fiber: edited to remove fk, fcphi terms
            dFm_dlce   = calcFiberStiffness(fiso,ma,fal,fv,fpe,sinphi,
                                            cosphi,lce,lceN,ofl);
            //edited to remove fk, fcphi terms
            dFmAT_dlce= calc_DFiberForceAT_DFiberLength(fiso,ma,fal,fv,fpe
                                            ,sinphi,cosphi,lce,lceN,ofl);
            //edited to remove fk, fcphi terms
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

                if(lce < getMinimumFiberLength()){
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

SimTK::Vector Millard2012EquilibriumMuscle::
    estimateElasticTendonFiberState2(
                    SimTK::Vec5 hint,
                    double aActivation, double dactivation_dt,
                    double pathLength, double pathLengtheningSpeed, 
                    double aSolTolerance, int aMaxIterations) const
{
    //ensureMuscleUpToDate();   
    std::string caller = getName();
    caller.append(".estimateElasticTendonFiberState2");
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
    double a = aActivation;
    double da_dt = dactivation_dt;
    double ml = pathLength;
    double dml_dt= pathLengtheningSpeed;

    //Shorter version of the constants
    double ltslk = getTendonSlackLength();
    double optL = getOptimalFiberLength();
    double optV = optL * getMaxContractionVelocity();

    double ophi= getPennationAngleAtOptimalFiberLength();
    double h = penMdl.getParallelogramHeight();
    double fiso= getMaxIsometricForce();
   

    //Get muscle model specific properties
    const TendonForceLengthCurve& fseCurve 
        = get_TendonForceLengthCurve(); 
    const FiberForceLengthCurve& fpeCurve 
        = get_FiberForceLengthCurve(); 
    const ActiveForceLengthCurve& falCurve
        = get_ActiveForceLengthCurve(); 
    const ForceVelocityInverseCurve& fvInvCurve 
        = get_ForceVelocityInverseCurve(); 


    //Shorter version of normalized muscle multipliers

    double fse = 0; //Normalized tendon (series element) force
    double fl = 0; //Normalized active force length multiplier
    double fv  = 0; //Normalized force-velocity multiplier
    double fpe = 0; //Normalized parallel element force

    //First order partial derivatives
    double dfse_d_tlN  = 0; 
    double dfl_d_lceN = 0; 
    double dfv_d_dlceN = 0;
    double dfpe_d_lceN = 0;

    //Second order partial derivatives
    double d2fse_d2_tlN  = 0; 
    double d2fl_d2_lceN = 0; 
    double d2fv_d2_dlceN = 0;
    double d2fpe_d2_lceN = 0;

    //*******************************
    double prevFiberLength    = hint(0);
    double prevFiberVelocity  = hint(1);
    double prevTime           = hint(2);
    double prev2FiberVelocity = hint(3);
    double prev2Time          = hint(4);

    if(prevFiberLength == 0){
        prevFiberLength = penMdl.calcFiberLength(pathLength,
                                    1.01*getTendonSlackLength());
        prevFiberLength = clampFiberLength(prevFiberLength);
    }

    //Position level
    double lce = clampFiberLength(prevFiberLength);
    double dlce_dt = prevFiberVelocity;
    double tl        = 0;
    double dtl_dt    = 0;
    //Compute fiber length and velocity assuming a 1% stretched rigid tendon
    /*if(lce <= getMinimumFiberLength() && dlce_dt == 0){        
        tl     = ltslk*1.01;
        dtl_dt = 0;
        
        lce = clampFiberLength(penMdl.calcFiberLength( ml, tl));
        dlce_dt   = penMdl.calcFiberVelocity(lce, sin(ophi), cos(ophi), 
                                              ml, tl, 
                                              dml_dt, dtl_dt, caller);           
    }*/

    double phi      = penMdl.calcPennationAngle(lce);    
    double cosphi   = cos(phi);
    double sinphi   = sin(phi);  
  
    tl = penMdl.calcTendonLength(cosphi,lce,ml);
   
    double tlN  = tl/ltslk;
    double lceN = lce/optL;
      
    //Velocity level     
    double dphi_dt  = penMdl.calcPennationAngularVelocity(tan(phi),
                                                       lce,dlce_dt,caller);
    dtl_dt = penMdl.calcTendonVelocity(cosphi, sinphi,dphi_dt,
                                                 lce,dlce_dt,dml_dt);

    double dlceN_dt  = dlce_dt/(optV);
    double dlceAT_dt = penMdl.calcFiberVelocityAlongTendon( lce,    dlce_dt,
                                                            sinphi, cosphi,
                                                                    dphi_dt);
    //Acceleration level
    //Assume the fiber acceleration is the same as the last time step
    double d2lce_d2t = 0; 

    //Estimating the fiber acceleration kills the convergence of this 
    //method: it either takes a lot longer, or doesn't converge. Not worth it.
    /*if(abs(prev2Time-prevTime) > SimTK::SignificantReal 
       && abs(prevFiberVelocity) > SimTK::SignificantReal
       && abs(prev2FiberVelocity) > SimTK::SignificantReal
       && prevFiberLength > SimTK::SignificantReal){
        //d2lce_d2t = (prevFiberVelocity-prev2FiberVelocity)/(prevTime-prev2Time);
          
    }*/

    //Error functions
    double F0 = 0; //Equlibrium equation
    double F1 = 0; //d/dt Equilibrium equation
    double DF0_D_lce = 0; //partial derivative of the eq. eqn w.r.t. lce
    double DF0_D_dlce= 0; //partial derivative of the eq. eqn w.r.t. dlce
    double DF1_D_lce = 0; //partial derivative of the eq. eqn w.r.t. lce
    double DF1_D_dlce= 0; //partial derivative of the eq. eqn w.r.t. dlce
    
    double ferr = 1e10;

    SimTK::Vector f(2),deltaL(2);
    SimTK::Matrix fJac(2,2);
    SimTK::FactorQTZ fJacQTZ;
    
    //Anti-stagnation routine variables
    //double  dFm_dlce,dFmAT_dlce,dFmAT_dlceAT,
    //        dFt_d_tl,dFt_d_lce,
    //        Ke;  
    
    //*******************************
    //Initialize the loop
    
    int iter = 0;    
    int minFiberLengthCtr = 0;

    int rank;
    double cond;
    double NewtonStepScale = 0;
    double NewtonStepSize = 0;

    //If it is obvious the tendon is going to be slack don't do the 
    //big expensive loop
    if(pathLength <= (getMinimumFiberLengthAlongTendon() + ltslk)){
        minFiberLengthCtr = 10;

    }
    //If the muscle is not obviously slack, solve for fiber length
    //and velocity assuming the fiber acceleration is 0
    while(  abs(ferr) > aSolTolerance
            && iter < aMaxIterations
            && minFiberLengthCtr < 10){

            //Update the multipliers and their partial derivativaes
            fl         = falCurve.calcValue(lceN);
            fpe         = fpeCurve.calcValue(lceN);
            fse         = fseCurve.calcValue(tlN);            
            fv          = fvCurve.calcValue(dlceN_dt);

            dfl_d_lceN = falCurve.calcDerivative(lceN,1); 
            dfv_d_dlceN = fvCurve.calcDerivative(dlceN_dt,1);
            dfpe_d_lceN = fpeCurve.calcDerivative(lceN,1);
            dfse_d_tlN  = fseCurve.calcDerivative(tlN,1); 
            
            d2fl_d2_lceN = falCurve.calcDerivative(lceN,2);  
            d2fv_d2_dlceN = fvCurve.calcDerivative(dlceN_dt,2);
            d2fpe_d2_lceN = fpeCurve.calcDerivative(lceN,2);
            d2fse_d2_tlN  = fseCurve.calcDerivative(tlN,2);  
            //
            //Compute F0: equilibrium equation F0
            //Compute F1: the derivative of the equilibrium equation
            //Compute the paritals of F0, and F1 w.r.t. lce, and dlce_dt
            //            DF0_D_lce, F0_D_dlce,
            //            DF1_D_lce, F1_D_dlce,
            //
            double t1 = a * fl;
            double t3 = t1 * fv + fpe;
            double t4 = lce * lce;
            double t5 = h * h;
            double t6 = t4 - t5;
            double t7 = sqrt(t6);
            double t8 = t3 * t7;
            double t9 = 0.1e1 / lce;

            double F0 = fiso * (t8 * t9 - fse);
            double t12 = da_dt * fl;
            double t14 = a * dfl_d_lceN;
            double t15 = 0.1e1 / optL;
            double t20 = 0.1e1 / optV;
            double t21 = dfv_d_dlceN * d2lce_d2t * t20;
            double t25 = t12 * fv + t14 * dlce_dt * t15 * fv + t1 * t21 
                            + dfpe_d_lceN * dlce_dt * t15;
            double t26 = t25 * t7;
            double t28 = t3 * t5;
            double t29 = 0.1e1 / t4;
            double t31 = 0.1e1 / t7;
            double t32 = t29 * dlce_dt * t31;
            double t34 = dlce_dt * t7;
            double t36 = t5 * dlce_dt;
            double t39 = dml_dt - t34 * t9 - t36 * t9 * t31;
            double t41 = 0.1e1 / ltslk;

            double F1 = fiso * (t26 * t9 + t28 * t32 - dfse_d_tlN * t39 * t41);
            double t44 = t15 * fv;
            double t45 = t14 * t44;
            double t46 = dfpe_d_lceN * t15;
            double t47 = t45 + t46;

            double DF0_D_lce = fiso * (t47 * t7 * t9 + t3 * t31 - t8 * t29 
                                + dfse_d_tlN * t31 * lce * t41);

            double t58 = dfv_d_dlceN * t20;
            double t59 = t7 * t9;
            double DF0_D_dlce = fiso * a * fl * t58 * t59;
            double t64 = optL * optL;
            double t65 = 0.1e1 / t64;
            double t88 = 0.1e1 / t7 / t6;
            double t92 = ltslk * ltslk;
            double t99 = t29 * t31;
            double t100 = t36 * t99;

            double DF1_D_lce = fiso * ((da_dt * dfl_d_lceN * t44 
                    + a * d2fl_d2_lceN * t65 * dlce_dt * fv 
                    + t14 * t15 * t21 + d2fpe_d2_lceN * t65 * dlce_dt) * t7 * t9 
                    + t25 * t31 - t26 * t29 + t47 * t5 * t32 
                    - 0.2e1 * t28 / t4 / lce * dlce_dt * t31 
                    - t28 * t9 * dlce_dt * t88 
                    + d2fse_d2_tlN * t31 * lce / t92 * t39 
                    - dfse_d_tlN * (-dlce_dt * t31 + t34 * t29 
                    + t100 + t36 * t88) * t41);

            double t111 = optV * optV;
            double DF1_D_dlce = fiso * ((t12 * t58 + t45 
                        + t14 * dlce_dt * t15 * dfv_d_dlceN * t20 
                        + t1 * d2fv_d2_dlceN / t111 * d2lce_d2t + t46) * t7 * t9
                        + t1 * t58 * t100 + t28 * t99 
                        - dfse_d_tlN * (-t59 - t5 * t9 * t31) * t41);

            f(0) = -F0;
            f(1) = -F1;

            fJac(0,0) = DF0_D_lce;
            fJac(0,1) = DF0_D_dlce;
            fJac(1,0) = DF1_D_lce;
            fJac(1,1) = DF1_D_dlce;

           
            ferr = sqrt(F0*F0 + F1*F1);

            fJacQTZ.factor(fJac);

            rank = fJacQTZ.getRank();
            cond = fJacQTZ.getRCondEstimate();

            //Full rank? Take a step
            if(rank == 2 && tlN > 1){                                
                fJacQTZ.solve(f,deltaL);
                NewtonStepScale = 1;
                NewtonStepSize = sqrt(deltaL(0)*deltaL(0) 
                                    + deltaL(1)*deltaL(1)); 
                //No big changes in fiber length!
                if(NewtonStepSize > 0.25*optL){
                    NewtonStepScale = abs(0.25*optL/NewtonStepSize);
                }

                lce = lce + NewtonStepScale*deltaL(0);
                dlce_dt = dlce_dt + NewtonStepScale*deltaL(1);                
            //If the tendon is slack set fiber length below the clamping value
            //else risk a dreaded local minimum
            }else if( tlN < 1){
                lce = 0;
                dlce_dt = -1;

                ferr = 1 + aSolTolerance;
            //Else we've lost rank and fiber length is not near its l
            //lower bound. Try spreading the velocity using the relative
            //stiffness of the fiber and the tendon
            }else{
                //Perturb the fiber length
                double perturbation1 = 
                    2.0*((double)rand())/((double)RAND_MAX)-1.0;
                double perturbation2 = 
                    2.0*((double)rand())/((double)RAND_MAX)-1.0;

                double lengthPerturbation = 
                    0.05*perturbation1*getOptimalFiberLength();
                double velocityPertubation = 
                    0.05*perturbation2*optV;

                lce = prevFiberLength + lengthPerturbation;   
                lce = clampFiberLength(lce);
                dlce_dt = prevFiberVelocity + velocityPertubation;

                lceN = lce/optL;
                    phi = penMdl.calcPennationAngle(lce);
                    cosphi = cos(phi);
                tl  =penMdl.calcTendonLength(cosphi,lce,ml);                
                tlN  = tl/ltslk;
                ferr = 1 + aSolTolerance;
                //Perturb the velocity by updating it according to the 
                //relative stiffnesses of the fiber and the tendon, using the
                //old fiber velocity
                /*
                fl         = falCurve.calcValue(lceN);
                fpe         = fpeCurve.calcValue(lceN);
                fse         = fseCurve.calcValue(tlN);            
                fv          = fvCurve.calcValue(dlceN_dt);
                
                dFm_dlce   = calcFiberStiffness(fiso,a,fl,fv,fpe,sinphi,
                                                cosphi,lce,lceN,optL);
                dFmAT_dlce= calc_DFiberForceAT_DFiberLength(fiso,a,fl,fv,fpe
                                                ,sinphi,cosphi,lce,lceN,optL);
                dFmAT_dlceAT = calc_DFiberForceAT_DFiberLengthAT(dFmAT_dlce,
                                               sinphi,cosphi,lce);

                //Tendon:
                dFt_d_tl    = fseCurve.calcDerivative(tlN,1)*fiso/ltslk;            
                dFt_d_lce   = calc_DTendonForce_DFiberLength(dFt_d_tl,lce,
                                                        sinphi,cosphi,caller); 

                if( abs(dFmAT_dlceAT + dFt_d_tl) > SimTK::SignificantReal
                    && tl > getTendonSlackLength()){
                    Ke      = (dFmAT_dlceAT*dFt_d_tl)/(dFmAT_dlceAT + dFt_d_tl); 
                    dtl_dt     = (1/dFt_d_tl)*Ke*dml_dt;                    
                }else{
                        dtl_dt     = dml_dt;
                }

                dlce_dt = penMdl.calcFiberVelocity(lce,sinphi,cosphi,
                                           ml,tl,dml_dt,dtl_dt,caller); 
                dlceN_dt    = dlce_dt/(optV);
                dphi_dt = penMdl.calcPennationAngularVelocity(tan(phi),lce,
                                                            dlce_dt,caller);
                dlceAT_dt = penMdl.calcFiberVelocityAlongTendon(lce,dlce_dt,
                                                    sinphi,cosphi,dphi_dt);

                
                //Ensure the error is not acceptable
                
                */
            }
                
            //Clamp fiber length
            if(isFiberStateClamped(lce,dlce_dt)){
                    minFiberLengthCtr++;
                    lce = clampFiberLength(lce);
                    dlce_dt = 0;                    
                    ferr = 1 + aSolTolerance;
            }
                 
    
            //Update position level quantities                 
            lceN = lce/optL;

            phi = penMdl.calcPennationAngle(lce);                
            sinphi = sin(phi);
            cosphi = cos(phi);

            tl  =penMdl.calcTendonLength(cosphi,lce,ml);                
            tlN  = tl/ltslk;

            dlceN_dt= dlce_dt/optV;
            dtl_dt = penMdl.calcTendonVelocity( cosphi,sinphi,phi,
                                                lce,dlce_dt,
                                                dml_dt);
            
        
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
        results[4] = dlce_dt;
        results[5] = fse*fiso;
    }else{ 
        //If the fiber length hit its lower bound        
        if(iter < aMaxIterations){     

            lce = getMinimumFiberLength();
            phi = penMdl.calcPennationAngle(lce);
            cosphi = cos(phi);
            tl  = penMdl.calcTendonLength(cosphi,lce,ml);
            lceN = lce/optL;
            tlN  = tl/ltslk;                
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
    //ensureMuscleUpToDate();
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
    

    int simMethod = getInitialSimulationMethod();

    //=========================================================================
    // Rigid Tendon Fiber Length
    //=========================================================================
    if(    simMethod==RigidTendonNoActivation 
        || simMethod==RigidTendonActivation)
    {
        mli.fiberLength = clampFiberLength(
                                    penMdl.calcFiberLength(getLength(s),
                                    getTendonSlackLength())); 

    //=========================================================================
    // Approximate method Fiber Length
    //=========================================================================
    }else if(   simMethod == ApproxElasticTendonNoActivation 
             || simMethod == ApproxElasticTendonActivation)
    {
        if(simTime > 0.5){
            double tmp = simTime;
        }
        
        const MuscleFirstOrderActivationDynamicModel 
            &actMdl = getActivationModel();
        double activation = 0;
        double dactivation_dt = 0;
        if(isActivationAState()){
            activation = actMdl.clampActivation(
                getStateVariable(s, STATE_ACTIVATION_NAME));
            double excitation = getControl(s);
            dactivation_dt = actMdl.calcDerivative(activation,excitation);
        }else{
            activation = actMdl.clampActivation(getControl(s));
        }

        //tol is the desired tolerance in Newtons
        double tol = 1e-8*getMaxIsometricForce();  
        if(tol < SimTK::SignificantReal*10){
            tol = SimTK::SignificantReal*10;
        }

        int maxIter = 200;                         
        double pathLength = getLength(s);
        double pathLengtheningSpeed = getLengtheningSpeed(s);

        //For some reason the hint above is totally trashing my simulation
        //results ... I have no idea why.
        
        //printf("Hint Before: %f,%f\n",  reducedFiberStateHint(0),
        //                              reducedFiberStateHint(1));

        SimTK::Vector soln = estimateElasticTendonFiberState2(
                                reducedFiberStateHint,
                                activation, dactivation_dt,
                                pathLength, pathLengtheningSpeed, 
                                tol, maxIter);

        /*SimTK::Vector soln = estimateElasticTendonFiberState(
                    activation, 
                    pathLength, pathLengtheningSpeed, 
                    tol       , maxIter);*/

        double flag_status    = soln[0];
        double solnErr        = soln[1];
        double iterations     = soln[2];
        double fiberLength    = soln[3];
        double fiberVelocity  = soln[4];
        double tendonForce    = soln[5];

        //Debugging
        /*if( tendonForce == 0 
            && abs(reducedFiberStateHint(0)-fiberLength)>0.002){
            cout << "Big nonlinearity at " << simTime << endl;
            soln = estimateElasticTendonFiberState2(
                                reducedFiberStateHint,
                                activation, dactivation_dt,
                                pathLength, pathLengtheningSpeed, 
                                tol, maxIter);

        } */

        //lower fiber length bound has been hit
        if(flag_status == 1.0){
            fiberLength = clampFiberLength(0);
            fiberVelocity = -1.0; // this way it gets clamped later
        }

        if(flag_status == 2.0){
            //solution has diverged!
            fiberLength = SimTK::NaN;
            fiberVelocity = SimTK::NaN;

            /*SimTK::Vector soln = estimateElasticTendonFiberState2(
                                reducedFiberStateHint,
                                activation, dactivation_dt,
                                pathLength, pathLengtheningSpeed, 
                                tol, maxIter);*/
        }

        /*
            flag_status:
            0: All is well
            1: Lower fiber bound hit, valid state still computed
            2: Solution diverged 

            I'd give a messsage when case 2 happens, 
            but the user will be flooded with messages 
            as OpenSim currently has no messaging interface.
        */
        mli.fiberLength       = fiberLength; 
        mli.userDefinedLengthExtras.resize(1);
        mli.userDefinedLengthExtras[0] = fiberVelocity;

    //=========================================================================
    // Integrated Fiber Length
    //=========================================================================    
    }else if ( simMethod == ElasticTendonNoActivation 
            || simMethod == ElasticTendonActivation)
    {
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
  //ensureMuscleUpToDate();
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    double simTime = s.getTime(); //for debugging purposes

    int simMethod = getInitialSimulationMethod();

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

    
    //=========================================================================
    //Rigid Tendon Fiber Velocity Computation
    //=========================================================================
    if(simMethod==RigidTendonNoActivation || simMethod==RigidTendonActivation){    
        double dtldt = 0;
        if(tl <= (tdnSlkLen-SimTK::SignificantReal)){
            dtldt = dlenMcl; //tendon is buckling
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
    }else if( simMethod == ApproxElasticTendonNoActivation
           || simMethod == ApproxElasticTendonActivation){
    
        dlce = mli.userDefinedLengthExtras[0];       

        dlceN= dlce/(optFibLen*getMaxContractionVelocity());
        fv = fvCurve.calcValue(dlceN);
    //=========================================================================
    //Integrated Elastic Tendon Fiber Velocity Computation
    //=========================================================================
    }else if( simMethod == ElasticTendonNoActivation 
        || simMethod == ElasticTendonActivation){
    
        const MuscleFirstOrderActivationDynamicModel& actMdl 
                = get_MuscleFirstOrderActivationDynamicModel();
        double a = 0;

        if(isActivationAState()){
             a = actMdl.clampActivation(
                 getStateVariable(s, STATE_ACTIVATION_NAME));        
        }else{
            a = actMdl.clampActivation(getControl(s));
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
            
        fv = ( ((fse)/cosphi) - (fpe) ) / (a*fal);

        //3. Evaluate the inverse force velocity curve        
        const ForceVelocityInverseCurve& fvInvCurve 
            = get_ForceVelocityInverseCurve(); 

        dlceN = fvInvCurve.calcValue(fv);
        dlce  = dlceN*getMaxContractionVelocity()*optFibLen;

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
        //ensureMuscleUpToDate();
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

    double a = 0;

    if(isActivationAState()){
        a    = actMdl.clampActivation(
                    getStateVariable(s, STATE_ACTIVATION_NAME));
    }else{
        a = actMdl.clampActivation(getControl(s));
    }

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
  
    double fiberStateClamped = mvi.userDefinedVelocityExtras[0];

    //Compute the stiffness of the muscle fiber
    SimTK_ERRCHK1_ALWAYS(lce > SimTK::SignificantReal, fcnName.c_str(),
        "%s: The muscle fiber has a length of 0, and is causing a singularity", 
        muscleName.c_str());
    SimTK_ERRCHK1_ALWAYS(cosPhi > SimTK::SignificantReal, fcnName.c_str(),
        "%s: Pennation angle is 90 degrees, and is causing a singularity", 
        muscleName.c_str());

    double Fm           = 0;
    double aFm          = 0;
    double FmAT         = 0;
    double dFm_dlce     = 0;
    double dFmAT_dlceAT = 0;
    double dFt_dtl      = 0;
    double Ke           = 0;

    if(fiberStateClamped < 0.5){
        //edited to remove fk, fcphi
        Fm    = calcFiberForce(fiso,a,fal,fv,fpe);
    
        aFm   = calcActiveFiberForce(fiso,a,fal,fv);
        //edited to remove fk, fcphi
        FmAT  = calcFiberForceAlongTendon(fiso,a,fal,fv,fpe,cosPhi);       

        //edited to remove fk, fcphi
        dFm_dlce = calcFiberStiffness(fiso,a,fal,fv,fpe,sinPhi,
                                             cosPhi,lce,lceN,optFiberLen);

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
        fse  = FmAT/fiso;
    }


    mdi.activation                   = a;
    mdi.fiberForce                   = Fm; 
    mdi.fiberForceAlongTendon        = FmAT;
    mdi.normFiberForce               = Fm/fiso;
    mdi.activeFiberForce             = aFm;
    mdi.passiveFiberForce            = fpe*fiso;                                        
                                     
    mdi.tendonForce                  = fse*fiso;
    mdi.normTendonForce              = fse;
                                     
    mdi.fiberStiffness               = dFm_dlce;
    mdi.fiberStiffnessAlongTendon    = dFmAT_dlceAT;
    mdi.tendonStiffness              = dFt_dtl;
    mdi.muscleStiffness              = Ke;
                                     
    //Check that the derivative of system energy less work is zero within
    //a reasonable numerical tolerance. Throw an exception if this is not true
    
    double dphidt= mvi.pennationAngularVelocity;
    
    double dFibPEdt     = (fpe)*fiso*dlce;
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
    calcActivationRate(const SimTK::State& s) const 
{    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "Millard2012EquilibriumMuscle: Muscle is not"
        " to date with properties");

    double dadt = 0;

    if(isActivationAState()){
        const MuscleFirstOrderActivationDynamicModel& actMdl 
            = get_MuscleFirstOrderActivationDynamicModel();

        double excitation = getExcitation(s);
        double activation = actMdl.clampActivation(getActivation(s));    

        //Both activation and excitation are clamped in calcDerivative
        dadt = actMdl.calcDerivative(activation,excitation);
    }
    return dadt;
}  

//==============================================================================
// Private Numerical Services
//==============================================================================
bool Millard2012EquilibriumMuscle::isFiberLengthAState() const
{
    bool fiberLengthState = false;

    switch(getInitialSimulationMethod()){
        case RigidTendonNoActivation:
            fiberLengthState = false;
            break;
        case RigidTendonActivation:
            fiberLengthState = false;
            break;
        case ApproxElasticTendonNoActivation:
            fiberLengthState = false;
            break;
        case ApproxElasticTendonActivation:
            fiberLengthState = false;
            break;
        case ElasticTendonNoActivation:
            fiberLengthState = true;
            break; 
        case ElasticTendonActivation:
            fiberLengthState = true;
            break;
        default:
            fiberLengthState = true;
    }

    return fiberLengthState;
}

bool Millard2012EquilibriumMuscle::isTendonElastic() const
{
    bool elasticTendon = false;
    switch(getInitialSimulationMethod()){
        case RigidTendonNoActivation:
            elasticTendon = false;
            break;
        case RigidTendonActivation:
            elasticTendon = false;
            break;
        case ApproxElasticTendonNoActivation:
            elasticTendon = true;
            break;
        case ApproxElasticTendonActivation:
            elasticTendon = true;
            break;
        case ElasticTendonNoActivation:
            elasticTendon = true;
            break;
        case ElasticTendonActivation:
            elasticTendon = true;
            break;
        default:
            elasticTendon = true;
    }

    return elasticTendon;
}

bool Millard2012EquilibriumMuscle::isActivationAState() const
{
    bool activationState = false;
    switch(getInitialSimulationMethod()){
        case RigidTendonNoActivation:
            activationState = false;
            break;
        case RigidTendonActivation:
            activationState = true;
            break;
        case ApproxElasticTendonNoActivation:
            activationState = false;
            break;
        case ApproxElasticTendonActivation:
            activationState = true;
            break;
        case ElasticTendonNoActivation:
            activationState = false;
            break;
        case ElasticTendonActivation:
            activationState = true;
            break;
        default:
            activationState = true;
    }

    return activationState;

}

bool Millard2012EquilibriumMuscle::
    isFiberStateClamped(double lce,double dlceN) const
{
    bool clamped = false;

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
    ensureMuscleUpToDate();
    return m_minimumFiberLength;
}


double Millard2012EquilibriumMuscle::getMinimumFiberLengthAlongTendon() const
{
    ensureMuscleUpToDate();
    return m_minimumFiberLengthAlongTendon;
}

double Millard2012EquilibriumMuscle::clampFiberLength(double lce) const
{
    return max(lce, getMinimumFiberLength());
}

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
                    double fpe) const
{
    //The force the fiber generates parallel to the fiber
    //double fm = fiso * ((a*fal*fv + fpe - fk) - fcphi*cosPhi);
    double fm = fiso * (a*fal*fv + fpe);
    return fm;
}

double Millard2012EquilibriumMuscle::
    calcActiveFiberForce( double fiso, 
                    double a, 
                    double fal,
                    double fv) const
{
    //The force the fiber generates parallel to the fiber
    //double fm = fiso * ((a*fal*fv + fpe - fk) - fcphi*cosPhi);
    double fm = fiso * (a*fal*fv);
    return fm;
}

double Millard2012EquilibriumMuscle::
    calcFiberForceAlongTendon(  double fiso, 
                                double a, 
                                double fal,
                                double fv,                             
                                double fpe,                                
                                double cosPhi) const
{
    //The force the fiber generates parallel to the fiber
    //double fmAT = fiso * ((a*fal*fv + fpe - fk)*cosPhi - fcphi);
    double fmAT = fiso * ((a*fal*fv + fpe)*cosPhi);
    return fmAT;
}

double Millard2012EquilibriumMuscle::
    calcFiberStiffness( double fiso, 
                        double a, 
                        double fal,
                        double fv,                             
                        double fpe,                        
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
    /*const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve();*/


    
    double DlceN_Dlce = 1/optFibLen;

    double Dfal_Dlce     = falCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfpe_Dlce     = fpeCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    //double Dfk_Dlce      = fkCurve.calcDerivative(lceN,1)  * DlceN_Dlce;   
    
    double Dphi_Dlce     = penMdl.calc_DPennationAngle_DfiberLength(lce,caller);
    double Dcosphi_Dlce  = -sinPhi*Dphi_Dlce;

    //double Dfcphi_Dcosphi= fcphiCurve.calcDerivative(cosPhi,1);
    //double Dfcphi_Dlce   = Dfcphi_Dcosphi * Dcosphi_Dlce;

    // The stiffness of the fiber parallel to the fiber
    //    d/dlce( fiso * ((a *  fal  *fv + fpe           - fk) + fcphi*cosPhi));
    //    which is now, having removed fk, and fcphi
    //    d/dlce( fiso * (a *  fal  *fv + fpe);
    //double DFm_Dlce = fiso * ((a*Dfal_Dlce*fv + Dfpe_Dlce - Dfk_Dlce) 
    //                       - (Dfcphi_Dlce*cosPhi + fcphi*Dcosphi_Dlce));

    double DFm_Dlce = fiso * ((a*Dfal_Dlce*fv + Dfpe_Dlce));

    return DFm_Dlce;
}

double Millard2012EquilibriumMuscle::
    calc_DFiberForceAT_DFiberLength(double fiso, 
                                    double a, 
                                    double fal,
                                    double fv,                             
                                    double fpe,
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
    /*const FiberCompressiveForceLengthCurve& fkCurve
        = get_FiberCompressiveForceLengthCurve(); 
    const FiberCompressiveForceCosPennationCurve& fcphiCurve
        = get_FiberCompressiveForceCosPennationCurve();*/

    
    double DlceN_Dlce = 1/optFibLen;

    double Dfal_Dlce     = falCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    double Dfpe_Dlce     = fpeCurve.calcDerivative(lceN,1) * DlceN_Dlce;
    //double Dfk_Dlce      = fkCurve.calcDerivative(lceN,1)  * DlceN_Dlce;   
    
    double Dphi_Dlce     = penMdl.calc_DPennationAngle_DfiberLength(lce,caller);
    double Dcosphi_Dlce  = -sinPhi*Dphi_Dlce;
    
    //double Dfcphi_Dcosphi= fcphiCurve.calcDerivative(cosPhi,1);
    //double Dfcphi_Dlce   = Dfcphi_Dcosphi * Dcosphi_Dlce;
    //The stiffness of the fiber in the direction of the tendon
    //1. Compute the stiffness of the fiber along the direction of the 
    //   tendon, for small changes in length parallel to the fiber
    //   that is: D(FiberForceAlongTendon) D (fiberLength)
    // dFmAT/dlce = d/dlce( fiso * ((a *fal*fv + fpe  - fk)*cosPhi - fcphi))
    // 
    // now without fk, and fcphi
    //
    // dFmAT/dlce = d/dlce( fiso * (a *fal*fv + fpe)*cosPhi )
    double DfmAT_Dlce = fiso * ((a*Dfal_Dlce*fv + Dfpe_Dlce)*cosPhi  
                            +(a*fal*fv + fpe)*Dcosphi_Dlce);
   
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
