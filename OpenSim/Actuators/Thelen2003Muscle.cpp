/* -------------------------------------------------------------------------- *
 *                       OpenSim:  Thelen2003Muscle.cpp                       *
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
#include <OpenSim/Simulation/Model/Model.h>
#include "Thelen2003Muscle.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTORS
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.


//_____________________________________________________________________________
/**
 * Default constructor.
 */
Thelen2003Muscle::Thelen2003Muscle()          
{    
    setNull();
    constructInfrastructure();
    ensureMuscleUpToDate();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Thelen2003Muscle::
Thelen2003Muscle(const std::string& aName,  double aMaxIsometricForce,
                  double aOptimalFiberLength,double aTendonSlackLength,
                  double aPennationAngle)
{
    setNull();
    constructInfrastructure();
    setName(aName);
    
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);

    ensureMuscleUpToDate();

}
//====================================================================
// Model Component Interface
//====================================================================
void Thelen2003Muscle::addToSystem(SimTK::MultibodySystem& system) const 
{
    Super::addToSystem(system);
    string errMsg =  getConcreteClassName()+" "+ getName() +
                  " is not up to date with its properties";
    SimTK_ASSERT(isObjectUpToDateWithProperties(), errMsg.c_str());
}

void Thelen2003Muscle::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);
    ensureMuscleUpToDate();
}

void Thelen2003Muscle::ensureMuscleUpToDate()
{
    if(isObjectUpToDateWithProperties()==false){
        buildMuscle();
    }
}

void Thelen2003Muscle::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

}
    

void Thelen2003Muscle::
    setPropertiesFromState(const SimTK::State& s)
{
    Super::setPropertiesFromState(s);      
    ensureMuscleUpToDate();
      
}


void Thelen2003Muscle::buildMuscle(){
    std::string caller(getName());
    caller.append("_Thelen2003Muscle::buildMuscle");

    //Appropriately set the properties of the activation model
    double activationTimeConstant  = getActivationTimeConstant();
    double deactivationTimeConstant= getDeactivationTimeConstant();
    double activationMinValue      = getMinimumActivation();


    MuscleFirstOrderActivationDynamicModel tmp1(activationTimeConstant, 
                                                deactivationTimeConstant, 
                                                activationMinValue, 
                                                caller);
    actMdl = tmp1; 

    //Appropriately set the properties of the pennation model    
    double optimalFiberLength = getOptimalFiberLength();
    double pennationAngle     = getPennationAngleAtOptimalFiberLength();

    MuscleFixedWidthPennationModel tmp2( optimalFiberLength,
                                        pennationAngle, 
                                        acos(0.1));

    penMdl = tmp2;   

    //Ensure all sub objects are up to date with properties.
    actMdl.ensureModelUpToDate();
    penMdl.ensureModelUpToDate();

    setObjectIsUpToDateWithProperties();
}
//_____________________________________________________________________________
// Set the data members of this muscle to their null values.
void Thelen2003Muscle::setNull()
{
    // no data members
    setAuthors("Matthew Millard");
}



//_____________________________________________________________________________
/**
 * Populate this objects properties
 */
void Thelen2003Muscle::constructProperties()
{
    constructProperty_activation_time_constant(0.015);
    constructProperty_deactivation_time_constant(0.050);
    constructProperty_FmaxTendonStrain(0.04); // was 0.033
    constructProperty_FmaxMuscleStrain(0.6);
    constructProperty_KshapeActive(0.45);   
    constructProperty_KshapePassive(5.0);   
    constructProperty_Af(0.25); 
    constructProperty_Flen(1.4);    //was 1.8, 
    constructProperty_fv_linear_extrap_threshold(0.95);
    //acos(0.05) = 84.26 degrees    
}

//=============================================================================
// GET
//=============================================================================

double Thelen2003Muscle::getActivationTimeConstant() const 
{   return get_activation_time_constant(); }

double Thelen2003Muscle::getDeactivationTimeConstant() const 
{   return get_deactivation_time_constant(); }

double Thelen2003Muscle::getMinimumActivation() const 
{   return actMdl.getMinimumActivation(); }


double Thelen2003Muscle::getFmaxTendonStrain() const 
{   return get_FmaxTendonStrain(); }

double Thelen2003Muscle::getFmaxMuscleStrain()  const 
{   return get_FmaxMuscleStrain(); }

double Thelen2003Muscle::getKshapeActive()  const 
{   return get_KshapeActive(); }

double Thelen2003Muscle::getKshapePassive() const 
{   return get_KshapePassive(); }

double Thelen2003Muscle::getAf() const 
{   return get_Af(); }

double Thelen2003Muscle::getFlen() const 
{   return get_Flen(); }

double Thelen2003Muscle::getForceVelocityExtrapolationThreshold() const 
{   return get_fv_linear_extrap_threshold(); }

const MuscleFirstOrderActivationDynamicModel& Thelen2003Muscle::
    getActivationModel() const
{    
    return actMdl;
}

const MuscleFixedWidthPennationModel& Thelen2003Muscle::
    getPennationModel() const
{    
    return penMdl;
}

double Thelen2003Muscle::getMaximumPennationAngle() const
{
    return penMdl.getMaximumPennationAngle();
}

//=============================================================================
// SET
//=============================================================================

bool Thelen2003Muscle::setActivationTimeConstant(double aActTimeConstant)
{
    if(aActTimeConstant > 0){
        set_activation_time_constant(aActTimeConstant);    
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setDeactivationTimeConstant(double aDeActTimeConstant)
{
    if(aDeActTimeConstant > 0){
        set_deactivation_time_constant(aDeActTimeConstant);       
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setFmaxTendonStrain(double aFmaxTendonStrain)
{
    if(aFmaxTendonStrain > 0){
        set_FmaxTendonStrain(aFmaxTendonStrain);
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setFmaxFiberStrain(double aFmaxMuscleStrain)
{
    if(aFmaxMuscleStrain > 0){
        set_FmaxMuscleStrain(aFmaxMuscleStrain);
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setKshapeActive(double aKShapeActive)
{
    if(aKShapeActive > 0){
        set_KshapeActive(aKShapeActive);
        ensureMuscleUpToDate();
        return true; 
    }
    return false;
}

bool Thelen2003Muscle::setKshapePassive(double aKshapePassive)
{
    if(aKshapePassive > 0){
        set_KshapePassive(aKshapePassive);
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}


bool Thelen2003Muscle::setAf(double aAf)
{
    if(aAf > 0){
        set_Af(aAf);
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setFlen(double aFlen)
{
    if(aFlen > 1.0){
        set_Flen(aFlen);
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}
  


bool Thelen2003Muscle::
                 setForceVelocityExtrapolationThreshold(double aFvThresh)
{
    if(aFvThresh > 1.0/getFlen()){
        set_fv_linear_extrap_threshold(aFvThresh);
        ensureMuscleUpToDate();
        return true;
    }
    return false;
}

/*bool Thelen2003Muscle::
    setMaximumPennationAngle(double maxPennationAngle) 
{
    if(maxPennationAngle > 0.0 &&
       maxPennationAngle < acos(0.001)){    
           penMdl.setMaximumPennationAngle(maxPennationAngle);
           ensureMuscleUpToDate();
           return true;
    }

    return false;
}*/

bool Thelen2003Muscle::setMinimumActivation(double aActivationMinValue)
{
    if(aActivationMinValue > 0.001){        
        actMdl.setMinimumActivation(aActivationMinValue);     
        ensureMuscleUpToDate();
        return true;
    }else{
        return false;
    }

}

//==============================================================================
// XXXXXXXXXXXXXXXXXXXX  START OF TO BE DEPRECATED   XXXXXXXXXXXXXXXXXXXXXXXXXXX
//==============================================================================
double Thelen2003Muscle::
calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                       double aActivation) const
{      
        SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

        string caller = getName();
        caller.append(  "Thelen2003Muscle::"
                        "calcInextensibleTendonActiveFiberForce");
       
        double inextensibleTendonActiveFiberForce = 0;

        double muscleLength = getLength(s);
        double muscleVelocity = getLengtheningSpeed(s);
        double tendonSlackLength = getTendonSlackLength();
        double tendonVelocity = 0.0; //Inextensible tendon;

        double fiberLength  = penMdl.calcFiberLength(muscleLength,
                                             tendonSlackLength);
        
        if(fiberLength > penMdl.getMinimumFiberLength()){
            double phi      = penMdl.calcPennationAngle(fiberLength);
        
            double fiberVelocity   = penMdl.calcFiberVelocity(cos(phi),
                                          muscleVelocity,tendonVelocity);

            inextensibleTendonActiveFiberForce = 
                calcActiveFiberForceAlongTendon(    aActivation,
                                                    fiberLength,
                                                    fiberVelocity);
        }


        return inextensibleTendonActiveFiberForce;
}

double Thelen2003Muscle::
            calcActiveFiberForceAlongTendon(double activation, 
                                            double fiberLength, 
                                            double fiberVelocity) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

    string caller = getName();
    caller.append("::Thelen2003Muscle::calcActiveFiberForceAlongTendon");

    double activeFiberForce = 0;    
    double clampedFiberLength = penMdl.clampFiberLength(fiberLength);

    //If the fiber is in a legal range, compute the force its generating
    if(fiberLength > penMdl.getMinimumFiberLength()){

        //Clamp activation to a legal range        
        double clampedActivation = actMdl.clampActivation(activation);

        //Normalize fiber length and velocity
        double normFiberLength    = clampedFiberLength/getOptimalFiberLength();
        double normFiberVelocity  = fiberVelocity / 
                        (getOptimalFiberLength() * getMaxContractionVelocity());


        //Evaluate the force active length and force velocity multipliers
        double fal  = calcfal(normFiberLength);

        double fv = SimTK::NaN;
        double fvTol = 1e-6;
        int fvMaxIter= 100;        
        SimTK::Vector fvSoln   = calcfvInv( clampedActivation,
                                            fal,
                                            normFiberVelocity,
                                            fvTol,
                                            fvMaxIter);
        //if the Newton method converged for fv
        if(fvSoln[0] > 0.5){
            fv = fvSoln[1];
        }

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

double  Thelen2003Muscle::computeActuation(const SimTK::State& s) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    const FiberVelocityInfo& mvi = getFiberVelocityInfo(s);
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setForce(s,         mdi.tendonForce);
    return( mdi.tendonForce );
}




void Thelen2003Muscle::computeInitialFiberEquilibrium(SimTK::State& s) const
{
    try{

        SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

        //Initialize activation to the users desired setting, while enforcing
        //that it lie in within the allowable bounds.
        double clampedActivation = actMdl.clampActivation(getActivation(s));
        setActivation(s,clampedActivation);

        //Initialize the multibody system to the initial state vector
        setFiberLength(s, getOptimalFiberLength());
        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);        

        //Tolerance, in Newtons, of the desired equilibrium
        double tol = 1e-8*getMaxIsometricForce();  //Should this be user settable?
        if(tol < SimTK::SignificantReal*10){
            tol = SimTK::SignificantReal*10;
        }
        int maxIter = 200;  //Should this be user settable?

    
        SimTK::Vector soln = initMuscleState(s,clampedActivation, tol, maxIter);
    
        int flag_status    = (int)soln[0];
        double solnErr        = soln[1];
        int iterations     = (int)soln[2];
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
                printf( "\n\nThelen2003Muscle Initialization Message:"
                        " %s is at its minimum length of %f",
                        muscleName.c_str(), penMdl.getMinimumFiberLength());
            }break;

            case 2: //Maximum number of iterations exceeded.
            {
                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());

                std::string muscleName = getName();
                std::string fcnName = "\n\nWARNING: Thelen2003Muscle::"
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
                        clampedActivation, 
                        fiberLength);

                    cerr << msgBuffer << endl;
        
            }break;

            default:
                std::string muscleName = getName();            
                printf( "\n\nWARNING: Thelen2003Muscle Initialization:"
                        " %s invalid error flag. Continuing with an initial "
                        "tendon force of 0, and a fiber length equal to the "
                        "optimal fiber length",
                        muscleName.c_str());

                setForce(s,0.0);
                setFiberLength(s,penMdl.getOptimalFiberLength());
        }
 


    }catch (const std::exception& e) { 
        //If the initialization routine fails in some unexpected way, tell
        //the user and continue with some valid initial conditions
        cerr    << "\n\nWARNING: Thelen2003Muscle initialization exception caught:" 
                << endl;
        cerr << e.what() << endl;
        
        cerr << "    Continuing with initial tendon force of 0 " << endl;
        cerr << "    and a fiber length equal to the optimal fiber length ..." 
             << endl;

        setForce(s,0.0);
        setFiberLength(s,penMdl.getOptimalFiberLength());

    }
}

void Thelen2003Muscle::calcMuscleLengthInfo(const SimTK::State& s,
                                            MuscleLengthInfo& mli) const
{    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

    double simTime = s.getTime(); //for debugging purposes

    try{
        double optFiberLength   = getOptimalFiberLength();
        double mclLength        = getLength(s);
        double tendonSlackLen   = getTendonSlackLength();

        std::string caller      = getName();
        caller.append("_Thelen2003Muscle::calcMuscleLengthInfo");

        //Clamp the minimum fiber length to its minimum physical value.
        mli.fiberLength  = penMdl.clampFiberLength(
                                getStateVariable(s, STATE_FIBER_LENGTH_NAME));

        mli.normFiberLength   = mli.fiberLength/optFiberLength;       
        mli.pennationAngle = penMdl.calcPennationAngle(mli.fiberLength);    

        mli.cosPennationAngle = cos(mli.pennationAngle);
        mli.sinPennationAngle = sin(mli.pennationAngle);

        mli.fiberLengthAlongTendon = mli.fiberLength*mli.cosPennationAngle;
    
        mli.tendonLength      = penMdl.calcTendonLength(mli.cosPennationAngle,
                                                        mli.fiberLength,mclLength);
        mli.normTendonLength  = mli.tendonLength / tendonSlackLen;
        mli.tendonStrain      = mli.normTendonLength -  1.0;
        

    
        mli.fiberPassiveForceLengthMultiplier= calcfpe(mli.normFiberLength);
        mli.fiberActiveForceLengthMultiplier = calcfal(mli.normFiberLength);
    }catch(const std::exception &x){
        std::string msg = "Exception caught in Thelen2003Muscle::" 
                          "calcMuscleLengthInfo\n"                 
                           "of " + getName()  + "\n"                            
                           + x.what();
        throw OpenSim::Exception(msg);
    }
}

void Thelen2003Muscle::calcMusclePotentialEnergyInfo(const SimTK::State& s,
        MusclePotentialEnergyInfo& mpei) const
{
    try {
        // Get the quantities that we've already computed.
        const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
        mpei.fiberPotentialEnergy = calcfpefisoPE(mli.fiberLength);
        mpei.tendonPotentialEnergy= calcfsefisoPE(mli.tendonStrain);
        mpei.musclePotentialEnergy=  mpei.fiberPotentialEnergy 
                                    + mpei.tendonPotentialEnergy;
    }
    catch(const std::exception &x){
        std::string msg = "Exception caught in Thelen2003Muscle::" 
                          "calcMusclePotentialEnergyInfo\n"                 
                           "of " + getName()  + "\n"                            
                           + x.what();
        throw OpenSim::Exception(msg);
    }
}

//=======================================
// computeMuscleLengthInfo helper functions
//=======================================

void Thelen2003Muscle::calcFiberVelocityInfo(const SimTK::State& s, 
                                               FiberVelocityInfo& fvi) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

    double simTime = s.getTime(); //for debugging purposes

    try{
        //Get the quantities that we've already computed
            const MuscleLengthInfo &mli = getMuscleLengthInfo(s);

        //Get the static properties of this muscle
            double mclLength      = getLength(s);
            double tendonSlackLen = getTendonSlackLength();
            double optFiberLen    = getOptimalFiberLength();

        //Prep strings that will be useful to make sensible exception messages
            std::string muscleName = getName();
            std::string fcnName     = "Thelen2003Muscle::calcFiberVelocityInfo";

            std::string caller      = muscleName;
            caller.append("_");
            caller.append(fcnName);

        //=========================================================================
        // Compute fv by inverting the force-velocity relationship in the 
        // equilibrium equations
        //=========================================================================

        //1. Get fiber/tendon kinematic information

        //clamp activation to a legal range
        double a = actMdl.clampActivation(
                                getStateVariable(s, STATE_ACTIVATION_NAME));
   

        double lce  = mli.fiberLength;   
        double phi  = mli.pennationAngle;
        double cosphi=mli.cosPennationAngle;
        double sinphi = mli.sinPennationAngle;

        //2. compute the tendon length ... because we can with this kind of model
        double tl = mli.tendonLength;

        
        //3. Compute force multipliers and the fiber velocity
    
            //This exception was causing headaches, so we're handling the special
            //case of a fiber that is at its minimum length instead
                //SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::Eps, fcnName.c_str(),
                //    "%s: Pennation angle is 90 degrees, and is causing a singularity", 
                //    muscleName.c_str());
    

            //6. Invert the force velocity curve to get dlce. Check for singularities 
            //   first

            //These exceptions were causing headaches, so we're handling the special
            //case of a fiber that is at its minimum length instead
                //SimTK_ERRCHK1_ALWAYS(a > SimTK::Eps, fcnName.c_str(),
                //    "%s: Activation is 0, and is causing a singularity", 
                //    muscleName.c_str());
                //SimTK_ERRCHK1_ALWAYS(fal > SimTK::Eps, fcnName.c_str(),
                //    "%s: The active force length curve value is 0,"
                //    " and is causing a singularity", 
                //    muscleName.c_str());
    
            //Check for singularity conditions, and clamp output appropriately

        double dmcldt = getLengtheningSpeed(s);

        //default values that are appropriate when fiber length has been clamped
        //to its mimimum allowable value.
    

        double fse  = calcfse(tl/tendonSlackLen);    
        double fal  = mli.fiberActiveForceLengthMultiplier;
        double fpe  = mli.fiberPassiveForceLengthMultiplier;

        double afalfv   = ((fse/cosphi)-fpe); //we can do this without fear of
                                              //a singularity because fiber length
                                              //is clamped
        double fv       = afalfv/(a*fal);

        double dlceN    = calcdlceN(a,fal,afalfv);
    
        double dlce     = dlceN*getMaxContractionVelocity()*optFiberLen;
        double tanPhi = tan(phi);
        double dphidt    = penMdl.calcPennationAngularVelocity(tanPhi,lce,dlce);
        double dlceAT   = penMdl.calcFiberVelocityAlongTendon(lce,
                                                        dlce,sinphi,cosphi, dphidt);

        double dtl       = penMdl.calcTendonVelocity(cosphi,sinphi,dphidt,
                                                            lce,  dlce,dmcldt);
    
    
        //Switching condition: if the fiber is clamped and the tendon and the 
        //                   : fiber are out of equilibrium
        double fiberStateClamped = 0.0;
        if(isFiberStateClamped(s,dlceN)){
             dlce = 0;
             dlceAT = 0;
             dlceN = 0;
             dphidt = 0;
             dtl = dmcldt;
             fv = 1.0;
             fiberStateClamped = 1.0;
        }
        
    

        //Populate the struct;
        fvi.fiberVelocity               = dlce;
        fvi.fiberVelocityAlongTendon    = dlceAT;
        fvi.normFiberVelocity           = dlceN;

        fvi.pennationAngularVelocity    = dphidt;

        fvi.tendonVelocity              = dtl;
        fvi.normTendonVelocity          = dtl/getTendonSlackLength();

        fvi.fiberForceVelocityMultiplier = fv;

        fvi.userDefinedVelocityExtras.resize(2);
        fvi.userDefinedVelocityExtras[0]=fse;
        fvi.userDefinedVelocityExtras[1]=fiberStateClamped;
    }catch(const std::exception &x){
        std::string msg = "Exception caught in Thelen2003Muscle::" 
                            "calcFiberVelocityInfo\n"                 
                            "of " + getName()  + "\n"                            
                            + x.what();
        throw OpenSim::Exception(msg);
    }
}





//=======================================
// computeFiberVelocityInfo helper functions
//=======================================

void Thelen2003Muscle::calcMuscleDynamicsInfo(const SimTK::State& s, 
                                               MuscleDynamicsInfo& mdi) const
{
        SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

        double simTime = s.getTime(); //for debugging purposes

    try{
        //Get the quantities that we've already computed
            const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
            const FiberVelocityInfo &mvi = getFiberVelocityInfo(s);
        //Get the static properties of this muscle
            double mclLength      = getLength(s);
            double tendonSlackLen = getTendonSlackLength();
            double optFiberLen    = getOptimalFiberLength();
            double fiso           = getMaxIsometricForce();
            double penHeight      = penMdl.getParallelogramHeight();

        //Prep strings that will be useful to make sensible exception messages
            std::string muscleName = getName();
            std::string fcnName     = "Thelen2003Muscle::calcMuscleDynamicsInfo";

            std::string caller      = muscleName;
            caller.append("_");
            caller.append(fcnName);

        //=========================================================================
        // Compute required quantities
        //=========================================================================

        //1. Get fiber/tendon kinematic information
        double a    = actMdl.clampActivation(
                                getStateVariable(s, STATE_ACTIVATION_NAME));

        double lce      = mli.fiberLength;
        double fiberStateClamped = mvi.userDefinedVelocityExtras[1];
        double dlce     = mvi.fiberVelocity;
        double phi      = mli.pennationAngle;
        double cosphi   = mli.cosPennationAngle;
        double sinphi   = mli.sinPennationAngle;

        double tl   = mli.tendonLength; 
        double dtl  = mvi.tendonVelocity;
        double tlN  = mli.normTendonLength;
   

    

        //These exceptions were causing problems, so we're going to clamp the state
        //of the muscle when its fiber becomes so short that the pennation model
        //goes singular

            //Compute the stiffness of the muscle fiber
            //SimTK_ERRCHK1_ALWAYS(lce > SimTK::Eps, fcnName.c_str(),
            //    "%s: The muscle fiber has a length of 0, and is"
            //    " causing a singularity", 
            //    muscleName.c_str());
            //SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::Eps, fcnName.c_str(),
            //    "%s: Pennation angle is 90 degrees, and is causing a singularity", 
            //    muscleName.c_str());

        //Default values appropriate when the fiber is clamped to its minimum length
        //and is generating no force

        //These quantities should already be set to legal values from
        //calcFiberVelocityInfo
        double fal  = mli.fiberActiveForceLengthMultiplier;
        double fpe  = mli.fiberPassiveForceLengthMultiplier;
        double fv   = mvi.fiberForceVelocityMultiplier;
        double fse  = mvi.userDefinedVelocityExtras[0];
    
        double aFm          = 0; //active fiber force
        double Fm           = 0;
        double dFm_dlce     = 0;
        double dFmAT_dlce   = 0;
        double dFmAT_dlceAT = 0;    
        double dFt_dtl      = 0; 
        double Ke           = 0;

        if(fiberStateClamped < 0.5){        
            aFm          = calcActiveFm(a,fal,fv,fiso);
            Fm           = calcFm(a,fal,fv,fpe,fiso);
            dFm_dlce     = calcDFmDlce(lce,a,fv,fiso,optFiberLen);
            dFmAT_dlce   = calcDFmATDlce(lce,phi,cosphi,Fm,dFm_dlce,penHeight);

            //The expression below is correct only because we are using a pennation
            //model that has a parallelogram of constant height.
            dFmAT_dlceAT= dFmAT_dlce*cosphi;

            dFt_dtl = calcDFseDtl(tl, fiso, tendonSlackLen);

            //Compute the stiffness of the whole muscle/tendon complex
            Ke = (dFmAT_dlceAT*dFt_dtl)/(dFmAT_dlceAT+dFt_dtl);
        }
    
        mdi.activation                   = a;
        mdi.fiberForce                   = Fm; 
        mdi.fiberForceAlongTendon        = Fm*cosphi;
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
        double dFibPEdt     = fpe*fiso*dlce;
        double dTdnPEdt     = fse*fiso*dtl;
        double dFibWdt      = -mdi.activeFiberForce*mvi.fiberVelocity;
        double dmcldt       = getLengtheningSpeed(s);
        double dBoundaryWdt = mdi.tendonForce * dmcldt;
        double ddt_KEPEmW   = dFibPEdt+dTdnPEdt-dFibWdt-dBoundaryWdt;
        SimTK::Vector userVec(1);
        userVec(0) = ddt_KEPEmW;  
        mdi.userDefinedDynamicsExtras = userVec;

        /////////////////////////////
        //Populate the power entries
        /////////////////////////////
        mdi.fiberActivePower             = dFibWdt;
        mdi.fiberPassivePower            = -dFibPEdt;
        mdi.tendonPower                  = -dTdnPEdt;       
        mdi.musclePower                  = -dBoundaryWdt;

        //Assertions for testing purposes. Commented out here as they are not
        //appropriate in all run-time conditions.

        //double tol = sqrt(SimTK::Eps);    
        //if(abs(dFibPEdt) > tol || abs(tmp) >= tol){
        //    tol = sqrt(SimTK::Eps);
        //}
    
        /*if(abs(tmp) > tol)
            printf("\n%s: d/dt(system energy-work) > tol, (%f > %f) at time %f\n",
                    fcnName.c_str(), tmp, tol, (double)s.getTime());*/
 
        /*SimTK_ERRCHK1( ((abs(tmp) < tol) && initializedModel)||!initializedModel, 
                fcnName.c_str(),
                "%\ns: Energy is not being conserved! d/dt(KE+PE-W) > tol >> 0 \n"
                "    Try tightening the integrator tolerances and re-simulating\n", 
                muscleName.c_str());
        */
    }catch(const std::exception &x){
    std::string msg = "Exception caught in Thelen2003Muscle::" 
                        "calcMuscleDynamicsInfo\n"                 
                        "of " + getName()  + "\n"                            
                        + x.what();
    throw OpenSim::Exception(msg);
    }
   
}

double Thelen2003Muscle:: getMinimumFiberLength() const
{
    return penMdl.getMinimumFiberLength();
}


bool Thelen2003Muscle::
    isFiberStateClamped(const SimTK::State& s, double dlceN) const
{
    bool clamped = false;

    //Is the fiber length  clamped and it is shortening, then the fiber length
    //not valid
    if( (getStateVariable(s, STATE_FIBER_LENGTH_NAME) 
            <= getMinimumFiberLength())
        && dlceN <= 0){
        clamped = true;
    }

    return clamped;     
}

//==============================================================================
// ActivationFiberLength.h Interface
//==============================================================================


/** Get the rate change of activation */
double Thelen2003Muscle::calcActivationRate(const SimTK::State& s) const 
{    
    double excitation = getExcitation(s);
    double activation = getActivation(s);
    double dadt = actMdl.calcDerivative(activation,excitation);
    return dadt;
}  





//==============================================================================
// Numerical Guts: Initialization
//==============================================================================
SimTK::Vector Thelen2003Muscle::
    initMuscleState(    SimTK::State& s, 
                        double aActivation, 
                        double aSolTolerance, 
                        int aMaxIterations) const
{
    //results vector format
    //1: flag (0 = converged 
    //         1=diverged, 
    //         2= no solution due to singularity:length 0, 
    //         3= no solution due to pennation angle singularity    
    //2: solution error (N)
    //3: iterations
    //4: fiber length (m)
    //5: passive force (N)
    //6: tendon force (N)
    SimTK::Vector results = SimTK::Vector(6);
    std::string caller = getName();
    caller.append(".initMuscleState");

    //I'm using smaller variable names here to make it possible to write out 
    //lengthy equations
    double ma = aActivation;
    double ml = getLength(s);
    double dml= getLengtheningSpeed(s);

    //Shorter version of the constants
    double tsl = getTendonSlackLength();
    double ofl = getOptimalFiberLength();
    double ophi= getPennationAngleAtOptimalFiberLength();
    double vol = ofl * sin(ophi);
    double fiso= getMaxIsometricForce();
    double vmax = getMaxContractionVelocity();//getPropertyValue<double>(VmaxName);

    //Shorter version of normalized muscle multipliers
    double fse = 0; //Normalized tendon (series element) force
    double fal = 0; //Normalized active force length multiplier
    double fpe = 0; //Normalized parallel element force
    double fv  = 0; //Normalized force-velocity multiplier

    double dfse_d_tlN = 0; //Partial derivative of fse w.r.t. norm. tendon len
    double dfal_d_lceN = 0; //Partial derivative of fal w.r.t. norm. fiber len
    double dfpe_d_lceN = 0; //Partial derivative of fpe w.r.t. norm. fiber len


    //*******************************
    //Position level
    double lce = 0;
    double tl  = getTendonSlackLength()*1.01;

   
    lce = penMdl.calcFiberLength( ml, tl);    
    
    double phi      = penMdl.calcPennationAngle(lce);
    double cosphi   = cos(phi);
    double sinphi   = sin(phi);  

    //Normalized quantities
    double tlN  = tl/tsl;
    double lceN = lce/ofl;
    //Velocity level
    double dtl      = 0;
    double dlce     = (dml - dtl) * cos(phi);
    double dlceN    = dlce/(vmax*ofl);
    double dphi     = -(dlce/lce)*tan(phi);
    double dlceAT   = dlce*cosphi -vol*dphi;

    //*******************************
    //Internal variables for the loop
    double Fm = 0;          // Muscle force
    double FmAT=0;          // Muscle force along tendon
    double Ft = 0;          // Tendon force
    double ferr = 1;        // Solution error
    
    double dphi_d_lce  = 0;  // Partial derivative of phi w.r.t. lce
    double dtl_d_lce   = 0;  // Partial derivative of tendon length w.r.t lce
    double dcosphi_d_lce=0;  // Partial derivative of cos(phi) w.r.t. lce
    
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
    
    SimTK::Vector fvInv(2);
    double tmp1         = 0;
    double tmp2         = 0;
    double tmp3         = 0;
    //*******************************
    //Initialize the loop
    
    int iter = 0;
    int minFiberLengthCtr = 0;

    while( (abs(ferr) > aSolTolerance) 
            && (iter < aMaxIterations)             
            && minFiberLengthCtr < 10){

       
            //Update the multipliers and their partial derivativaes
            fse         = calcfse(tlN);
            fal         = calcfal(lceN);
            fpe         = calcfpe(lceN);
        
            //SINGULARITY: When phi = pi/2, or activation = 0;
            fvInv          = calcfvInv(ma,fal,dlceN,aSolTolerance, 100); 
            fv = fvInv[1];

            //Compute the force error
            Fm   = (ma*fal*fv + fpe)*fiso;
            FmAT = Fm*cosphi;
            Ft = fse*fiso;
            ferr  = FmAT-Ft; 
        
            //Compute the partial derivative of the force error w.r.t. lce
            dFm_dlce   = calcDFmDlce(lce, ma, fv, fiso, ofl);
            dFmAT_dlce = calcDFmATDlce(lce,phi,cosphi,Fm, dFm_dlce, vol);
            dFmAT_dlceAT= dFmAT_dlce*cosphi;

            dFt_d_tl    = calcDFseDtl(tl, fiso, tsl);
            dFt_d_lce   = calcDFseDlce(tl, lce, phi, cosphi,fiso, tsl, vol);
            

            dferr_d_lce = dFmAT_dlce - dFt_d_lce;

            if(abs(ferr) > aSolTolerance){
                //Take a full Newton Step                            
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

                phi = penMdl.calcPennationAngle(lce);
                cosphi = cos(phi);
                tl  = ml - lce*cosphi;
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
                //negative stiffness of the fiber (which could happen in this
                // model) is equal to the positive stiffness of the tendon.
                if( abs(dFmAT_dlceAT + dFt_d_tl) > SimTK::SignificantReal
                    && tl > getTendonSlackLength()){
                    Ke      = (dFmAT_dlceAT*dFt_d_tl)/(dFmAT_dlceAT + dFt_d_tl); 
                    dtl     = (1/dFt_d_tl)*Ke*dml;
                }else{
                    dtl     = dml;
                }

                dlce = penMdl.calcFiberVelocity(cosphi,dml,dtl); 
                dlceN    = dlce/(vmax*ofl);
                dphi = penMdl.calcPennationAngularVelocity(tan(phi),lce,dlce);
                dlceAT = penMdl.calcFiberVelocityAlongTendon(lce,dlce,
                                                    sinphi,cosphi,dphi);               
            }
        
        iter++;
    }

    //*******************************    
    //Populate the output vector
    //*******************************
    //If the solution converged
    if(abs(ferr) < aSolTolerance){    
        //1: flag (0 = converged
        //         1 = diverged (not enough iterations) 
        //         2= no solution due to singularity:length 0, 
        //         3= no solution due to pennation angle singularity
        //2: solution Error (N)
        //3: iterations
        //4: fiber length (m)
        //5: passive force (N)
        //6: tendon force (N)
        
        results[0] = 0;
        results[1] = ferr;
        results[2] = (double)iter;
        results[3] = lce;
        results[4] = fpe*fiso;
        results[5] = fse*fiso;

    }else{ 
        
        if(iter < aMaxIterations){ //if the fiber length hit its lower bound

            lce = penMdl.getMinimumFiberLength();
            phi = penMdl.calcPennationAngle(lce);
            cosphi = cos(phi);
            tl  = penMdl.calcTendonLength(cosphi,lce,ml);
            lceN = lce/ofl;
            tlN  = tl/tsl;                
            fse = calcfse(tlN);
            fpe = calcfpe(lceN);

            results[0] = 1.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = fpe*fiso;
            results[5] = fse*fiso;  
        
        }else{//If the solution diverged
                 
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
//
// STIFFNESS RELATED FUNCTIONS
//
//==============================================================================
double Thelen2003Muscle::calcFm(double ma, double fal, double fv, double fpe,
                                 double fiso) const
{
    double Fm = (ma*fal*fv + fpe)*fiso;
    return Fm;
}

double Thelen2003Muscle::calcActiveFm(double ma, double fal, double fv,
                                 double fiso) const
{
    double aFm = (ma*fal*fv)*fiso;
    return aFm;
}

double Thelen2003Muscle::calcDFmDlce(double lce, double ma, double fv, 
                                      double fiso, double ofl) const
{

            double lceN = lce/ofl;
            double dfal_d_lceN = calcDfalDlceN(lceN);
            double dfpe_d_lceN = calcDfpeDlceN(lceN);

            double dFm_d_lce = ((ma*fv)*dfal_d_lceN + dfpe_d_lceN)*fiso
                              *(1/ofl);                   
            return dFm_d_lce;
}

double Thelen2003Muscle::calcDFmATDlce(double lce, double phi, double cosphi, 
    double Fm, double d_Fm_d_lce, double penHeight) const
{
            std::string caller = getName();
            caller.append("_Thelen2003Muscle::calcDFmATDlce");        

            //SINGULARITY: when vol*vol/(lce*lce) = 1,same as phi=pi/2
            double tmp1 = penHeight*penHeight;
            double tmp2 = lce*lce;
            double tmp3 = tmp2*lce;
            double dcosphi_d_lce = (tmp1 /(tmp3*pow((1-(tmp1/tmp2)),0.5 ) ));


            double d_FmAT_d_lce = d_Fm_d_lce*cosphi + Fm*dcosphi_d_lce;
            return d_FmAT_d_lce;
}

double Thelen2003Muscle::calcDFseDlce(double tl, double lce, double phi, double cosphi,
                                      double fiso, double tsl, double vol) const
{
            double tlN = tl/tsl;
                //SINGULARITY: When lce = 0
            double tmp1 = vol/lce;        

            //SINGULARITY: when vol/lce = 1 - equivalent to when phi = pi/2
            double dphi_d_lce = -vol / ( lce*lce * pow( (1-(tmp1)*(tmp1)),0.5)); 
            double dtl_d_lce  = -cos(phi) + lce*sin(phi)*dphi_d_lce;

            double dfse_d_tlN  = calcDfseDtlN(tlN); 
                   tmp1 = (fiso/tsl);
            double dFt_d_lce = dfse_d_tlN*dtl_d_lce*tmp1;  
            return dFt_d_lce;
}

double Thelen2003Muscle::calcDFseDtl(double tl, double fiso, double tsl) const
{
    double dfse_d_tlN  = calcDfseDtlN(tl/tsl);
    double tmp1 = (fiso/tsl);
    double dFt_d_tl= dfse_d_tlN*tmp1;
    return dFt_d_tl;
}


//==============================================================================
//
// Convenience Method
//
//==============================================================================

void Thelen2003Muscle::printCurveToCSVFile(const CurveType ctype, 
                                           const std::string&path)
{
    ensureMuscleUpToDate();
    /*
        //Only compute up to the 2nd derivative
    SimTK::Matrix results = calcSampledMuscleCurve(2);
    SimTK::Array_<std::string> colNames(results.ncol());
    colNames[0] = "x";
    colNames[1] = "y";
    colNames[2] = "dy/dx";
    colNames[3] = "d2y/dx2";
    
    if(results.ncol() == 5){
        colNames[4] = "int_y(x)";
    }

            std::string fname = _name;
            SimTK_ERRCHK_ALWAYS(fname.length() > 0,
                "SmoothSegmentedFunction::printMuscleCurveToCSVFile",
                "Muscle Curve name is empty!");
            fname.append(".csv");

            printMatrixToFile(results,colNames,path,fname);*/

    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
                    "Thelen2003Muscle: Muscle is not"
                    " to date with properties");

    std::string fname = getName();

    switch(ctype){
        case FiberActiveForceLength:{
            
            fname.append("_FiberActiveForceLength.csv");
            
            SimTK::Matrix data(100,3);
            SimTK::Array_<std::string> colNames(data.ncol());
            colNames[0] = "lceN";
            colNames[1] = "fal";
            colNames[2] = "DfalDlceN";

            double delta = (2.0-0.0)/(data.nrow()-1.0);
            double lceN = 0;
            double lceN0 = 0;
            for(int i=0; i<data.nrow(); i++){
                lceN = lceN0 + i*delta;
                data(i,0) = lceN;
                data(i,1) = calcfal(lceN);
                data(i,2) = calcDfalDlceN(lceN);
            }

            printMatrixToFile(data,colNames,path, fname);

        }break;
        case FiberPassiveForceLength:{
            fname.append("_FiberPassiveForceLength.csv");
            
            SimTK::Matrix data(100,3);
            SimTK::Array_<std::string> colNames(data.ncol());
            colNames[0] = "lceN";
            colNames[1] = "fpe";
            colNames[2] = "DfpeDlceN";

            double lceNMax = get_FmaxMuscleStrain()+1.0;
            double lceNMin = 1.0;
            double lceNExtra = 0.1*(lceNMax-lceNMin);

            double delta = ((lceNMax + lceNExtra)-(lceNMin-lceNExtra))
                           /(data.nrow()-1.0);
            double lceN = 0;
            double lceN0 = lceNMin-lceNExtra;

            for(int i=0; i<data.nrow(); i++){
                lceN = lceN0 + i*delta;
                data(i,0) = lceN;
                data(i,1) = calcfpe(lceN);
                data(i,2) = calcDfpeDlceN(lceN);
            }

            printMatrixToFile(data,colNames,path, fname);

        }break;
        case FiberForceVelocity:{

            fname.append("_FiberForceVelocity.csv");
            
            SimTK::Matrix data(1000,5);
            SimTK::Array_<std::string> colNames(data.ncol());
            colNames[0] = "a";
            colNames[1] = "fal";
            colNames[2] = "dlceN";
            colNames[3] = "fv";
            colNames[4] = "DfvDdlceN";

            //Thelen's fv varies with activation
            double a = 0;
            double a0 = 0.1;
            double da = 0.1;

            //Thelen's fv varies with fiber velocity
            double dlceNMax = 1.0;
            double dlceNMin = -1.0;
            double dlceNExtra = 0.1*(dlceNMax-dlceNMin);

            double delta = ((dlceNMax + dlceNExtra)-(dlceNMin-dlceNExtra))
                           /(100.0-1.0);
            double dlceN = 0;
            double dlceN0 = dlceNMin-dlceNExtra;

            //Thelen's fv also varies with the value of the active force 
            //length curve. Most experiments that establish fv do so at 
            //a normalized fiber length of 1.0, where fal is 1. We will
            //also make that assumption here

            SimTK::Vector fvInv;
            double DdlceDaFalFv = 0;
            double DfvDdlceN = 0;

            const double fal = 1.0;

            int idx = 0;
            for(int i=0; i<=9; i++){
                a = a0 + da*i;

                for(int j=0; j<100; j++){
                    dlceN = dlceN0 + j*delta;
                    fvInv = calcfvInv(a, fal, dlceN, 1e-6,100);
                    DdlceDaFalFv = calcDdlceDaFalFv(a,fal,a*fal*fvInv[1]);
                    DfvDdlceN = (1/(DdlceDaFalFv*a*fal));

                    data(idx,0) = a;
                    data(idx,1) = fal;
                    data(idx,2) = dlceN;
                    data(idx,3) = fvInv[1];
                    data(idx,4) = DfvDdlceN;
                    
                    idx++;
                }
            }
            printMatrixToFile(data,colNames,path, fname);

        }break;
        case TendonForceLength:{
            fname.append("_TendonForceLength.csv");
            
            SimTK::Matrix data(100,3);
            SimTK::Array_<std::string> colNames(data.ncol());
            colNames[0] = "ltN";
            colNames[1] = "fse";
            colNames[2] = "DfseDtlN";

            double ltNMax = get_FmaxTendonStrain()+1.0;
            double ltNMin = 1.0;
            double ltNExtra = 0.1*(ltNMax-ltNMin);

            double delta = ((ltNMax + ltNExtra)-(ltNMin-ltNExtra))
                           /(data.nrow()-1.0);
            double ltN = 0;
            double ltN0 = ltNMin-ltNExtra;

            for(int i=0; i<data.nrow(); i++){
                ltN = ltN0 + i*delta;
                data(i,0) = ltN;
                data(i,1) = calcfse(ltN);
                data(i,2) = calcDfseDtlN(ltN);
            }

            printMatrixToFile(data,colNames,path, fname);

        }break;
        default:{
            std::string msg = "Thelen2003Muscle::printCurveToCSVFile ";
            msg.append(getName());
            msg.append(" invalid curve type");
            SimTK_ASSERT(false,msg.c_str());
        }
    }

}

/*
This function will print cvs file of the column vector col0 and the matrix data

@params data: A matrix of data
@params filename: The name of the file to print
*/
void Thelen2003Muscle::
    printMatrixToFile(SimTK::Matrix& data, SimTK::Array_<std::string>& colNames,
    const std::string& path, const std::string& filename) const
{
    
    ofstream datafile;
    std::string fullpath = path;
    
    if(fullpath.length() > 0)
        fullpath.append("/");
    
    fullpath.append(filename);

    datafile.open(fullpath.c_str(),std::ios::out);

    if(!datafile){
        datafile.close();
        string name = getName();
        SimTK_ERRCHK2_ALWAYS( false, 
                "Thelen2003Muscle::printMatrixToFile",
                "%s: Failed to open the file path: %s", 
                name.c_str(),
                fullpath.c_str());
    }


    for(int i = 0; i < (signed)colNames.size(); i++){
        if(i < (signed)colNames.size()-1)
            datafile << colNames[i] << ",";
        else
            datafile << colNames[i] << "\n";
    }

    for(int i = 0; i < data.nrow(); i++){       
        for(int j = 0; j < data.ncol(); j++){
            if(j<data.ncol()-1)
                datafile << data(i,j) << ",";
            else
                datafile << data(i,j) << "\n";
        }   
    }
    datafile.close();
} 

//==============================================================================
//
// TENDON RELATED FUNCTIONS
//
//==============================================================================

double Thelen2003Muscle::calcfse(const double tlN) const 
{
    double x = tlN-1;
    double e0 = getFmaxTendonStrain();
    
    /*The paper reports etoe = 0.609e0, however, this is a severely rounded off
        The exact answer, to SimTK::Eps is   
        etoe =  99*e0*e^3 / ( 166*e^3 - 67)
        klin =  67 /( 100*(e0 - (99*e0*e^3)/(166*e^3-67)) )
        See thelenINIT_20120127.mw for details
    */    
    double kToe = 3.0;
    double Ftoe = 33.0/100.0;
    double t1   = exp(0.3e1);
    double eToe = (0.99e2*e0*t1) / (0.166e3*t1 - 0.67e2);
    t1 = exp(0.3e1);
    double klin = (0.67e2/0.100e3) 
                * 1.0/(e0 - (0.99e2*e0*t1) / (0.166e3*t1 - 0.67e2));

    //Compute tendon force
    double fse = 0;
    if (x > eToe){
        fse = klin*(x-eToe)+Ftoe;
    }else if (x>0.0){ 
        fse =(Ftoe/(exp(kToe)-1.0))*(exp(kToe*x/eToe)-1.0);
    }else{
        fse=0.;}

    return fse;
}


double Thelen2003Muscle::calcDfseDtlN(const double tlN) const {
    double x = tlN-1;
    double e0 = getFmaxTendonStrain();
    
    /*The paper reports etoe = 0.609e0, however, this is a severely rounded off
    result of the exact answer:    
        etoe =  99*e0*e^3 / ( 166*e^3 - 67)
        See thelenINIT_20120127.mw for details
    */
    double kToe = 3.0;
    double Ftoe = 33.0/100.0;

    double t1   = exp(0.3e1);
    double eToe = (0.99e2*e0*t1) / (0.166e3*t1 - 0.67e2);

    /*The paper reports etoe = 0.609e0, however, this is a severely rounded off
    result of the exact answer:    
        klin =  67 /( 100*(e0 - (99*e0*e^3)/(166*e^3-67)) )
        See thelenINIT_20120127.mw for details
    */
    t1 = exp(0.3e1);
    double klin = (0.67e2/0.100e3) 
                * 1.0/(e0 - (0.99e2*e0*t1) / (0.166e3*t1 - 0.67e2));

    //Compute tendon force
    double dfse_d_dtlN = 0;
    if (x > eToe){
        dfse_d_dtlN = klin;
    }else if (x>0.0){ 
        dfse_d_dtlN =(Ftoe/(exp(kToe)-1.0)) * (kToe/eToe) * (exp(kToe*x/eToe));
    }else{
        dfse_d_dtlN=0.;}

    return dfse_d_dtlN;
}

double Thelen2003Muscle::calcfsefisoPE(double tendonStrain) const
{

    double tendon_strain =  tendonStrain;
    double fmaxTendonStrain = getFmaxTendonStrain();       

    //Future optimization opportunity: precompute kToe, fToe, eToe and klin
    //when the muscle is initialized. Store these values rather than 
    //computing them every time.

    double kToe = 3.0;
    double Ftoe = 33.0/100.0;

    double t1   = exp(0.3e1);
    double eToe = (0.99e2*fmaxTendonStrain*t1) / (0.166e3*t1 - 0.67e2);

    t1 = exp(0.3e1);
    double klin = (0.67e2/0.100e3) 
                * 1.0/(fmaxTendonStrain - (0.99e2*fmaxTendonStrain*t1) 
                / (0.166e3*t1 - 0.67e2));

    //Compute the energy stored in the tendon. 
    //Integrals computed symbolically in muscle_kepew_20111021.mw just to check
    double tendonPE = 0.0;
    double lenR        = getTendonSlackLength();
    double lenTdn    = (tendon_strain+1)*lenR;
    double lenToe    = (eToe+1.0)*lenR;    
    double fiso        = getMaxIsometricForce();

    if (tendon_strain>eToe){
       //compute the energy stored in the toe portion of the tendon strain curve
        double len = lenToe;
        double toePE_len = (fiso*Ftoe/(exp(kToe)-1.0))
                            *((lenR*eToe/kToe)
                            *exp(kToe*(len-lenR)/(lenR*eToe)) - len);
        len =  lenR;
        double toePE_0    =  (fiso*Ftoe/(exp(kToe)-1.0))
                            *((lenR*eToe/kToe)
                            *exp(kToe*(len-lenR)/(lenR*eToe)) - len);
        double toePEtest = toePE_len-toePE_0;

        //compute the energy stored in the linear section of the 
        //tendon strain curve from ..... 0 to len
        len = lenTdn;
        double linPE_len = (1.0/2.0)*(fiso*klin*(len*len)/lenR) 
                           + fiso*len*(klin*(-1.0-eToe)+Ftoe);
        //ditto from 0 .... eToe
        len = lenToe;
        double linPE_eToe= (1.0/2.0)*(fiso*klin*(len*len)/lenR) 
                            + fiso*len*(klin*(-1.0-eToe)+Ftoe);       
        
        //compute the total potential energy stored in the tendon
         tendonPE =(toePE_len-toePE_0) + (linPE_len-linPE_eToe);
    }else if (tendon_strain>0.0){ 
        //PE from 0 .... len
        double len = lenTdn;
        double toePE_len = (fiso*Ftoe/(exp(kToe)-1.0)) * ((lenR*eToe/kToe) 
            * exp(kToe*(len-lenR)/(lenR*eToe)) - len);
        //PE from 0 .... eToe
        len = lenR;
        double toePE_0    =  (fiso*Ftoe/(exp(kToe)-1.0)) * ((lenR*eToe/kToe) 
            * exp(kToe*(len-lenR)/(lenR*eToe)) - len);

        //Compute the total PE stored in the tendon
        tendonPE = toePE_len-toePE_0;
    }else{
        tendonPE = 0.0;
    }
    
    
    return tendonPE;
}

//==============================================================================
//
// ACTIVE FORCE LENGTH FUNCTIONS
//
//==============================================================================
double Thelen2003Muscle::calcfal(const double lceN) const{       
    double kShapeActive = getKshapeActive();   
    double x=(lceN-1.)*(lceN-1.);
    double fal = exp(-x/kShapeActive);
    return fal;
}
double Thelen2003Muscle::calcDfalDlceN(const double lceN) const {
    double kShapeActive = getKshapeActive();   
    double t1 = lceN - 0.10e1;
    double t2 = 0.1e1 / kShapeActive;
    double t4 = t1 * t1;
    double t6 = exp(-t4 * t2);
    double dfal_d_lceN = -0.2e1 * t1 * t2 * t6;
    return dfal_d_lceN;
}

//=============================================================================
//
// FIBER PARALLEL ELEMENT HELPER FUNCTIONS
//
//=============================================================================
double Thelen2003Muscle::calcfpe(const double lceN) const {
    double fpe = 0;
    double e0 = getFmaxMuscleStrain();
    double kpe = getKshapePassive();

    //Compute the passive force developed by the muscle
    if(lceN > 1.0){
        double t5 = exp(kpe * (lceN - 0.10e1) / e0);
        double t7 = exp(kpe);
        fpe = (t5 - 0.10e1) / (t7 - 0.10e1);
    }
    return fpe;
}

double Thelen2003Muscle::calcDfpeDlceN(const double lceN) const {
    double dfpe_d_lceN = 0;
    double e0 = getFmaxMuscleStrain();
    double kpe = getKshapePassive();

    if(lceN > 1.0){
        double t1 = 0.1e1 / e0;
        double t6 = exp(kpe * (lceN - 0.10e1) * t1);
        double t7 = exp(kpe);
        dfpe_d_lceN = kpe * t1 * t6 / (t7 - 0.10e1);
    }
    return dfpe_d_lceN;
}

double Thelen2003Muscle::calcfpefisoPE(double lceN) const
{
    double fmaxMuscleStrain = getFmaxMuscleStrain();
    double kShapePassive = getKshapePassive();

    double musclePE = 0.0;
    //Compute the potential energy stored in the muscle
    if(lceN > 1.0){
        //Shorter variable names to make the equations readable.
        double lenR = getOptimalFiberLength();
        double fiso = getMaxIsometricForce();
        double len = lceN*lenR;        
        double kpe = kShapePassive;
        double e0 = fmaxMuscleStrain;


        //PE storage at current stretch
        double fpePE_len = (fiso/(exp(kpe)-1))
            *( (lenR*e0/kpe)*exp( (kpe/e0)*( (len/lenR)-1)) - len); 
        
        //PE stored between 0 and 1 for the exponental function that is 
        //used to represent fpe for normalized fiber lengths > 1.
        len = lenR;
        double fpePE_0 = (fiso/(exp(kpe)-1))
            *( (lenR*e0/kpe)*exp( (kpe/e0)*( (len/lenR)-1)) - len); 

        musclePE = fpePE_len - fpePE_0;

    }else{
        musclePE = 0.0;
    }
    return musclePE;
}


//=============================================================================
//
// FIBER FORCE - VELOCITY CURVE FUNCTIONS
//
//=============================================================================

double Thelen2003Muscle::calcdlceN(double act,double fal,double actFalFv) const
{
    //The variable names have all been switched to closely match 
    //with the notation in Thelen 2003.
    double dlceN = 0.0;      //contractile element velocity    
    double af   = getAf();

    double a    = act;
    double afl  = a*fal; //afl = a*fl
    double Fm   = actFalFv;     //Fm = a*fl*fv    
    double flen = getFlen();
    double Fmlen_afl = flen*afl;

    double dlcedFm = 0.0; //partial deriviative of contactile element
                          // velocity w.r.t. Fm

    double b = 0;
    double db= 0;

    double Fm_asyC = 0;           //Concentric contraction asymptote
    double Fm_asyE = afl*flen;    
                                //Eccentric contraction asymptote
    double asyE_thresh = getForceVelocityExtrapolationThreshold();

    //If fv is in the appropriate region, use 
    //Thelen 2003 Eqns 6 & 7 to compute dlceN
    if (Fm > Fm_asyC && Fm < Fm_asyE*asyE_thresh){

        if( Fm <= afl ){        //Muscle is concentrically contracting
            b = afl + Fm/af;
            db= 1/af;
        }else{                    //Muscle is eccentrically contracting
            b = ((2+2/af)*(afl*flen-Fm))/(flen-1); 
            db= ((2+2/af)*(-1))/(flen-1); 
        }

        dlceN = (0.25 + 0.75*a)*(Fm-afl)/b; 
        //Scaling by VMAX is left out, and is post multiplied outside 
        //of the function


    }else{  //Linear extrapolation
            double Fm0 = 0.0; //Last Fm value from the Thelen curve

            //Compute d and db/dFm from Eqn 7. of Thelen2003
            //for the last
            if(Fm <= Fm_asyC){ //Concentrically contracting
                Fm0 = Fm_asyC;
                b = afl + Fm0/af;
                db= 1/af;               
            }else{             //Eccentrically contracting
                Fm0 = asyE_thresh*Fm_asyE;
                b = ((2+2/af)*(afl*flen-Fm0))/(flen-1); 
                db= ((2+2/af)*(-1))/(flen-1); 
            }

            //Compute the last dlceN value that falls in the region where
            //Thelen 2003 Eqn. 6 is valid
            double dlce0 = (0.25 + 0.75*a)*(Fm0-afl)/b;

            //Compute the dlceN/dfm of Eqn. 6 of Thelen 2003 at the last
            //valid point
            dlcedFm = (0.25 + 0.75*a)*(1)/b 
                    - ((0.25 + 0.75*a)*(Fm0-afl)/(b*b))*db;

            //Linearily extrapolate Eqn. 6 from Thelen 2003 to compute
            //the new value for dlceN/dFm
            dlceN = dlce0 + dlcedFm*(Fm-Fm0);            
        }
            
        return dlceN;
}

double Thelen2003Muscle::
    calcfv(double aFse,     double aFpe, double aFal, 
           double aCosPhi,  double aAct) const
{
    //This only works for an equilibrium model, but its a lot less 
    //computationally expensive (and error prone) than trying to invert the 
    //weird function that defines the fv curve in the Thelen model
    double fv = ((aFse/aCosPhi) - aFpe)/(aAct*aFal);
    return fv;
}

double Thelen2003Muscle::calcDdlceDaFalFv(double aAct, 
                                          double aFal, double aFalFv) const
{
    //The variable names have all been switched to closely match with 
    //the notation in Thelen 2003.
    double dlceN = 0.0;      //contractile element velocity    
    double af   = getAf();

    double a    = aAct;
    double afl  = aAct*aFal;  //afl = a*fl
    double Fm   = aFalFv;    //Fm = a*fl*fv    
    double flen = getFlen();
    double Fmlen_afl = flen*aAct*aFal;

    double dlcedFm = 0.0; //partial derivative of contractile element 
                          //velocity w.r.t. Fm

    double b = 0;
    double db= 0;

    double Fm_asyC = 0;           //Concentric contraction asymptote
    double Fm_asyE = aAct*aFal*flen;    
                                //Eccentric contraction asymptote
    double asyE_thresh = getForceVelocityExtrapolationThreshold();

    //If fv is in the appropriate region, use 
    //Thelen 2003 Eqns 6 & 7 to compute dlceN
    if (Fm > Fm_asyC && Fm < Fm_asyE*asyE_thresh){

        if( Fm <= afl ){        //Muscle is concentrically contracting
            b = afl + Fm/af;
            db= 1/af;
        }else{                    //Muscle is eccentrically contracting
            b = ((2+2/af)*(afl*flen-Fm))/(flen-1); 
            db= ((2+2/af)*(-1))/(flen-1); 
        }

        //This variable may have future use outside this function
        dlcedFm = (0.25 + 0.75*a)*(1)/b - ((0.25 + 0.75*a)*(Fm-afl)/(b*b))*db;            

    }else{  //Linear extrapolation
            double Fm0 = 0.0; //Last Fm value from the Thelen curve

            //Compute d and db/dFm from Eqn 7. of Thelen2003
            //for the last
            if(Fm <= Fm_asyC){ //Concentrically contracting
                Fm0 = Fm_asyC;
                b = afl + Fm0/af;
                db= 1/af;               
            }else{             //Eccentrically contracting
                Fm0 = asyE_thresh*Fm_asyE;
                b = ((2+2/af)*(afl*flen-Fm0))/(flen-1); 
                db= ((2+2/af)*(-1))/(flen-1); 
            }

            
            //Compute the dlceN/dfm of Eqn. 6 of Thelen 2003 at the last
            //valid point
            dlcedFm = (0.25 + 0.75*a)*(1)/b 
                - ((0.25 + 0.75*a)*(Fm0-afl)/(b*b))*db;
          
        }
            
        return dlcedFm;
}

//This is here because it is non-trivial to correctly invert the piece wise
//continuous force velocity curve specified by the modified Thelen2003Muscle
//force velocity curve. This converges quickly and is well tested.
SimTK::Vector Thelen2003Muscle::
        calcfvInv(double aAct,double aFal,double dlceN,
                  double tolerance, int maxIterations) const
{
    SimTK::Vector result(2);
    result(0) = 0.0; //value of flag: 0 diverged, 1 converged
    result(1) = 0.0; //value of fv
    double ferr=1;
    double iter= 0;

    double dlceN1 = 0;
    double dlceN1_d_Fm = 0;
    double fv = 1;
    double aFalFv = fv*aAct*aFal;
    double delta_aFalFv = 0;

    while(abs(ferr) >= tolerance && iter < maxIterations)
    {
        dlceN1 = calcdlceN(aAct,aFal, aFalFv);
        ferr   = dlceN1-dlceN;
        dlceN1_d_Fm = calcDdlceDaFalFv(aAct,aFal,aFalFv);


        if(abs(dlceN1_d_Fm) > SimTK::SignificantReal){
           delta_aFalFv = -ferr/(dlceN1_d_Fm);
           aFalFv = aFalFv + delta_aFalFv;
        }
        iter = iter+1;
    }

    if(abs(ferr) < tolerance){
        result[0] = 1.0;
        result[1] = aFalFv/(aAct*aFal);
    }

    return result;
}
