// Thelen2003Muscle.cpp
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
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <SimTKcommon/internal/ExceptionMacros.h>

#include "Thelen2003Muscle.h"
#include <iostream>

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
    constructProperties();
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
    constructProperties();
    setName(aName);
    
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);
}

void Thelen2003Muscle::addToSystem(SimTK::MultibodySystem& system) const 
{
    Super::addToSystem(system);

    //const cast *this so you can initialize the member variables actMdl and
    //penMdl
    Thelen2003Muscle* mthis =  const_cast<Thelen2003Muscle*>(this);

    //Appropriately set the properties of the activation model
        double activationTimeConstant  = getActivationTimeConstant();
        double deactivationTimeConstant= getDeactivationTimeConstant();
        double activationMinValue      = getActivationMinimumValue();


        MuscleFirstOrderActivationDynamicModel tmp1(activationTimeConstant, 
                                                    deactivationTimeConstant, 
                                                    activationMinValue, 
                                                    getName());
    mthis->actMdl = tmp1; 

    //Appropriately set the properties of the pennation model
        std::string caller(getName());
        caller.append("_Thelen2003Muscle::addToSystem");
        double optimalFiberLength = getOptimalFiberLength();
        double pennationAngle     = getPennationAngleAtOptimalFiberLength();

        MuscleFixedWidthPennationModel tmp2( optimalFiberLength,
                                            pennationAngle, 
                                            caller);

    mthis->penMdl = tmp2;    
 }

//_____________________________________________________________________________
// Set the data members of this muscle to their null values.
void Thelen2003Muscle::setNull()
{
    // no data members
}

//_____________________________________________________________________________
/**
 * Populate this objects properties
 */
void Thelen2003Muscle::constructProperties()
{
    constructProperty_activation_time_constant(0.015);
    constructProperty_deactivation_time_constant(0.050);
    constructProperty_FmaxTendonStrain(0.033);
    constructProperty_FmaxMuscleStrain(0.6);
    constructProperty_KshapeActive(0.45);   
    constructProperty_KshapePassive(5.0);   
    constructProperty_Af(0.25); 
    constructProperty_Flen(1.8);
    constructProperty_activation_minimum_value(0.01);  
    constructProperty_fv_linear_extrap_threshold(0.95);
}

//=============================================================================
// GET
//=============================================================================

double Thelen2003Muscle::getActivationTimeConstant() const 
{   return get_activation_time_constant(); }

double Thelen2003Muscle::getDeactivationTimeConstant() const 
{   return get_deactivation_time_constant(); }

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

double Thelen2003Muscle::getActivationMinimumValue() const 
{   return get_activation_minimum_value(); }

double Thelen2003Muscle::getForceVelocityExtrapolationThreshold() const 
{   return get_fv_linear_extrap_threshold(); }

//=============================================================================
// SET
//=============================================================================

bool Thelen2003Muscle::setActivationTimeConstant(double aActTimeConstant)
{
    if(aActTimeConstant > 0){
        set_activation_time_constant(aActTimeConstant);           
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setActivationMinimumValue(double aActMinValue)
{
    if(aActMinValue > 0 && aActMinValue < 1.0){
        set_activation_minimum_value(aActMinValue);
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setDeactivationTimeConstant(double aDeActTimeConstant)
{
    if(aDeActTimeConstant > 0){
        set_deactivation_time_constant(aDeActTimeConstant);       
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setFmaxTendonStrain(double aFmaxTendonStrain)
{
    if(aFmaxTendonStrain > 0){
        set_FmaxTendonStrain(aFmaxTendonStrain);
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setFmaxFiberStrain(double aFmaxMuscleStrain)
{
    if(aFmaxMuscleStrain > 0){
        set_FmaxMuscleStrain(aFmaxMuscleStrain);
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setKshapeActive(double aKShapeActive)
{
    if(aKShapeActive > 0){
        set_KshapeActive(aKShapeActive);
        return true; 
    }
    return false;
}

bool Thelen2003Muscle::setKshapePassive(double aKshapePassive)
{
    if(aKshapePassive > 0){
        set_KshapePassive(aKshapePassive);
        return true;
    }
    return false;
}


bool Thelen2003Muscle::setAf(double aAf)
{
    if(aAf > 0){
        set_Af(aAf);
        return true;
    }
    return false;
}

bool Thelen2003Muscle::setFlen(double aFlen)
{
    if(aFlen > 1.0){
        set_Flen(aFlen);
        return true;
    }
    return false;
}
  


bool Thelen2003Muscle::
                 setForceVelocityExtrapolationThreshold(double aFvThresh)
{
    if(aFvThresh > 1.0/getFlen()){
        set_fv_linear_extrap_threshold(aFvThresh);
        return true;
    }
    return false;
}


//==============================================================================
// Muscle.h Interface
//==============================================================================

double  Thelen2003Muscle::computeActuation(const SimTK::State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    const FiberVelocityInfo& mvi = getFiberVelocityInfo(s);
    const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
    setForce(s,         mdi.tendonForce);
    return( mdi.tendonForce );
}

/*To be deprecated: this is just for backwards compatibility */
double Thelen2003Muscle::computeIsometricForce(SimTK::State& s, 
                                                double activation) const
{
    //Initialize activation to the users desired setting
    setActivation(s,activation);
    computeInitialFiberEquilibrium(s);
    return getTendonForce(s);
}


void Thelen2003Muscle::computeInitialFiberEquilibrium(SimTK::State& s) const
{
    //Initialize activation to the users desired setting
    setActivation(s,getActivation(s));

    //Initialize the multibody system to the initial state vector
    setFiberLength(s, getOptimalFiberLength());
    _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

    //Compute an initial muscle state that develops the desired force and
    //shares the muscle stretch between the muscle fiber and the tendon 
    //according to their relative stiffness.
    double activation = getActivation(s);
    double tol = 1e-8;  //Should this be user settable?
    int maxIter = 200;  //Should this be user settable?

    SimTK::Vector soln = initMuscleState(s,activation, tol, maxIter);

    int flag_status    = (int)soln[0];
    double solnErr        = soln[1];
    double iterations     = (int)soln[2];
    double fiberLength    = soln[3];
    double passiveForce   = soln[4];
    double tendonForce    = soln[5];

    std::string fcnName = "Thelen2003Muscle::"
                            "computeInitialFiberEquilibrium(SimTK::State& s)";
    std::string muscleName = getName();

    SimTK_ERRCHK7_ALWAYS(flag_status == 0, fcnName.c_str(),
        "%s: \n"
     "    The initialization routine found no stable equilibrium fiber length\n"
     "    length. Here is a report from the routine:\n \n"
     "        Solution Error      : %f > tol (%f) \n"
     "        Newton Iterations   : %i of max. iterations (%i) \n"
     "    Check that the initial activation is valid, and that the whole \n"
     "    length doesn't produce a pennation angle of 90 degrees, nor a fiber\n"
     "    length less than 0:\n"
     "        Activation          : %f \n" 
     "        Whole muscle length : %f \n", 
        muscleName.c_str(), 
        abs(solnErr),
        tol,
        iterations,
        maxIter,
        getActivation(s), 
        getLength(s));

    //1: flag (0 = converged, 
    //         1=diverged (not enough iterations), 
    //         2= no solution due to singularity:length 0, 
    //         3= no solution due to pennation angle singularity    
    //2: solution error (N)
    //3: iterations
    //4: fiber length (m)
    //5: passive force (N)
    //6: tendon force (N)  
    
    //Thelen2003Muscle* mthis =  const_cast<Thelen2003Muscle*>(this);
    //mthis->initializedModel = true;
    setForce(s,tendonForce);
    setFiberLength(s,fiberLength);
 
}

void Thelen2003Muscle::calcMuscleLengthInfo(const SimTK::State& s, 
                                               MuscleLengthInfo& mli) const
{    
    double simTime = s.getTime(); //for debugging purposes

    double optFiberLength   = getOptimalFiberLength();
    double mclLength        = getLength(s);
    double tendonSlackLen   = getTendonSlackLength();

    std::string caller      = getName();
    caller.append("_Thelen2003Muscle::calcMuscleLengthInfo");

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
        
    mli.fiberPotentialEnergy = calcfpefisoPE(mli.fiberLength);
    mli.tendonPotentialEnergy= calcfsefisoPE(mli.tendonStrain);
    mli.musclePotentialEnergy=  mli.fiberPotentialEnergy 
                              + mli.tendonPotentialEnergy;

    mli.fiberPassiveForceLengthMultiplier= calcfpe(mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier = calcfal(mli.normFiberLength);
}

//=======================================
// computeMuscleLengthInfo helper functions
//=======================================



void Thelen2003Muscle::calcFiberVelocityInfo(const SimTK::State& s, 
                                               FiberVelocityInfo& fvi) const
{
    double simTime = s.getTime(); //for debugging purposes


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
    double a    = getStateVariable(s, STATE_ACTIVATION_NAME);
    double lce  = mli.fiberLength;
    double phi  = mli.pennationAngle;
    double cosphi=mli.cosPennationAngle;
    double sinphi = mli.sinPennationAngle;

    //2. compute the tendon length ... because we can with this kind of model
    double tl = mli.tendonLength;

    //3. Get the normalized tendon force
    double fse = calcfse(tl/tendonSlackLen);

    //4. Get the active force length, and passive length multiplier
    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;
    
    //5. Compute fv - but check for singularities first
    SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::Eps, fcnName.c_str(),
        "%s: Pennation angle is 90 degrees, and is causing a singularity", 
        muscleName.c_str());
    double afalfv   = ((fse/cosphi)-fpe);

    //6. Invert the force velocity curve to get dlce. Check for singularities 
    //   first
    SimTK_ERRCHK1_ALWAYS(a > SimTK::Eps, fcnName.c_str(),
        "%s: Activation is 0, and is causing a singularity", 
        muscleName.c_str());
    SimTK_ERRCHK1_ALWAYS(fal > SimTK::Eps, fcnName.c_str(),
        "%s: The active force length curve value is 0,"
        " and is causing a singularity", 
        muscleName.c_str());
    double dlceN = calcdlceN(a,fal,afalfv);
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

    fvi.fiberForceVelocityMultiplier = afalfv/(a*fal);

    fvi.userDefinedVelocityExtras.resize(1);
    fvi.userDefinedVelocityExtras(0)=fse;
}





//=======================================
// computeFiberVelocityInfo helper functions
//=======================================

void Thelen2003Muscle::calcMuscleDynamicsInfo(const SimTK::State& s, 
                                               MuscleDynamicsInfo& mdi) const
{
        double simTime = s.getTime(); //for debugging purposes

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
    double a    = getStateVariable(s, STATE_ACTIVATION_NAME);

    double lce      = mli.fiberLength;
    double dlce     = mvi.fiberVelocity;
    double phi      = mli.pennationAngle;
    double cosphi   = mli.cosPennationAngle;
    double sinphi   = mli.sinPennationAngle;

    double tl   = mli.tendonLength; 
    double dtl  = mvi.tendonVelocity;
    double tlN  = mli.normTendonLength;
   

    double fal  = mli.fiberActiveForceLengthMultiplier;
    double fpe  = mli.fiberPassiveForceLengthMultiplier;
    double fv   = mvi.fiberForceVelocityMultiplier;
    double fse  = mvi.userDefinedVelocityExtras(0);

    //Compute the stiffness of the muscle fiber
    SimTK_ERRCHK1_ALWAYS(lce > SimTK::Eps, fcnName.c_str(),
        "%s: The muscle fiber has a length of 0, and is causing a singularity", 
        muscleName.c_str());
    SimTK_ERRCHK1_ALWAYS(cosphi > SimTK::Eps, fcnName.c_str(),
        "%s: Pennation angle is 90 degrees, and is causing a singularity", 
        muscleName.c_str());

    double Fm           = calcFm(a,fal,fv,fpe,fiso);
    double dFm_dlce     = calcDFmDlce(lce,a,fv,fiso,optFiberLen);
    double dFmAT_dlce   =calcDFmATDlce(lce,phi,cosphi,Fm,dFm_dlce,penHeight);

    //The expression below is correct only because we are using a pennation
    //model that has a parallelogram of constant height.
    double dFmAT_dlceAT= dFmAT_dlce*cosphi;

    //Compute the stiffness of the tendon
    double dFt_dtl    = calcDFseDtl(tl, optFiberLen, tendonSlackLen);

    //Compute the stiffness of the whole muscle/tendon complex
    double Ke = (dFmAT_dlceAT*dFt_dtl)/(dFmAT_dlceAT+dFt_dtl);

    
    mdi.activation                   = a;
    mdi.fiberForce                   = Fm; 
    mdi.fiberForceAlongTendon        = Fm*cosphi;
    mdi.normFiberForce               = Fm/fiso;
    mdi.activeFiberForce             = a*fal*fv*fiso;
    mdi.passiveFiberForce            = fpe*fiso;
                                     
    mdi.tendonForce                  = fse*fiso;
    mdi.normTendonForce              = fse;
                                     
    mdi.fiberStiffness               = dFm_dlce;
    mdi.fiberStiffnessAlongTendon    = dFmAT_dlceAT;
    mdi.tendonStiffness              = dFt_dtl;
    mdi.muscleStiffness              = Ke;
                                     
    mdi.fiberPower                   = -mdi.activeFiberForce*mvi.fiberVelocity;

    //This is not necessary, and will be removed soon.
    mdi.fiberPowerAlongTendon        = -mdi.activeFiberForce*cosphi 
                                         * mvi.fiberVelocityAlongTendon;

    mdi.tendonPower                  = -mdi.tendonForce * mvi.tendonVelocity;   

    double dmcldt = getLengtheningSpeed(s);
    mdi.musclePower                  = -mdi.tendonForce * dmcldt;

    //Check that the derivative of system energy less work is zero within
    //a reasonable numerical tolerance. Throw an exception if this is not true
    
    

    double dFibPEdt = fpe*fiso*dlce;
    double dTdnPEdt = fse*fiso*dtl;
    double dFibWdt  = mdi.fiberPower;
    double dBoundaryWdt = -mdi.musclePower;
    double ddt_KEPEmW = dFibPEdt+dTdnPEdt-dFibWdt-dBoundaryWdt;
    SimTK::Vector userVec(1);
    userVec(0) = ddt_KEPEmW;  
    mdi.userDefinedDynamicsExtras = userVec;

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
    initMuscleState(SimTK::State& s, double aActivation, 
                              double aSolTolerance, int aMaxIterations) const
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
    double l0cosphi = ml - tsl*1.01;
    double phi      = atan(vol/l0cosphi);
    double cosphi   = cos(phi);
    double lce = 0;
    if(phi > sqrt(SimTK::Eps)){
        lce      = vol/sin(phi);
    }else{
        lce      = 0.5*ofl;
    }

    double tl       = tsl*1.01;
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
    

    while( abs(ferr) > aSolTolerance && iter < aMaxIterations){

        if(lce > 0 + SimTK::Eps && phi < SimTK::Pi/2 - SimTK::Eps){
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

            if(abs(ferr) > aSolTolerance && abs(dferr_d_lce) > 0){
                //Take a full Newton Step
                delta_lce   = - ferr/dferr_d_lce;
                lce         = lce + delta_lce;
            
                if(lce > 0){
                    //Update position level quantities, only if they won't go 
                    //singular
                    if(vol/lce < 1){
                        phi = asin(vol/lce);
                    }else{
                        phi = SimTK::Pi-SimTK::Eps;
                    }
                    cosphi = cos(phi);
                    tl  = ml - lce*cos(phi);
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
                    Ke      = (dFmAT_dlceAT*dFt_d_tl)/(dFmAT_dlceAT + dFt_d_tl); 
                    dtl     = (1/dFt_d_tl)*Ke*dml;
                    dlceAT  = (1/dFmAT_dlceAT)*Ke*dml;

                    //dlceAT = dlce*cos(phi) - lce*sin(phi)*dphi
                    //After substituting in dphi = -(dlce/lce)*tan(phi), and solving
                    //for dlce, we're left with (see eq_sym_work.mw)
                    dlce    = cosphi*dlceAT;
                    dlceN    = dlce/(vmax*ofl);
                    dphi    = -(dlce/lce)*tan(phi);
                }

            }
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
    }else{ //If the solution diverged
        
    	std::string muscleName = getName();

        //Check for the fiber length singularity
        if(lce < 0 + SimTK::Eps){
            results[0] = 2.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = 0.0;
            results[4] = 0.0;
            results[5] = 0.0;

            printf("\nInitialization failed: fiber length approaching 0, \n"
                   "                       for %s, a Thelen2003Muscle \n"
                   "                       with an error of %f\n", 
                   muscleName.c_str(), ferr);
        //Check for a pennation angle singularity   
        }else if(phi > SimTK::Pi/2 - SimTK::Eps){
            results[0] = 3.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = 0.0;
            results[5] = 0.0;

            printf("\nInitialization failed: pennation angle approaching Pi/2, \n"
                   "                       for %s, a Thelen2003Muscle \n"
                   "                       with an error of %f\n", 
                   muscleName.c_str(), ferr);

        //Not enough iterations
        }else{ 
            results[0] = 1.0;
            results[1] = ferr;
            results[2] = (double)iter;
            results[3] = lce;
            results[4] = fpe*fiso;
            results[5] = fse*fiso;

            printf("\nInitialization failed: solution did not converge in %i, \n"
                   "                       for %s, a Thelen2003Muscle \n"
                   "                       with an error of %f\n", 
                   iter, muscleName.c_str() ,ferr);

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
    calcfv(const double aFse, const double aFpe, const double aFal, 
                           const double aCosPhi, const double aAct) const
{
    //This only works for an equilibrium model, but its a lot less 
    //computationally expensive (and error prone) than trying to invert the 
    //weird function that defines the fv curve in the Thelen model
    double fv = ((aFse/aCosPhi) - aFpe)/(aAct*aFal);
    return fv;
}

double Thelen2003Muscle::calcDdlceDaFalFv(const double aAct, 
                                   const double aFal, const double aFalFv) const
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


SimTK::Vector Thelen2003Muscle::
        calcfvInv(const double aAct,const double aFal, const double dlceN,
                               const double tolerance, int maxIterations) const
{
    SimTK::Vector result(2);
    result(0) = 0.0; //value of flag: 0 converged, 1 diverged
    result(1) = 0.0; //value of fv
    double ferr=1;
    double iter= 0;

    double dlceN1 = 0;
    double dlceN1_d_Fm = 0;
    double fv = 1;
    double aFalFv = fv*aAct*aFal;
    double delta_aFalFv = 0;

    while(abs(ferr) > tolerance && iter < maxIterations)
    {
        dlceN1 = calcdlceN(aAct,aFal, aFalFv);
        ferr   = dlceN1-dlceN;
        dlceN1_d_Fm = calcDdlceDaFalFv(aAct,aFal,aFalFv);


        if(abs(dlceN1_d_Fm) > SimTK::Eps){
           delta_aFalFv = -ferr/(dlceN1_d_Fm);
           aFalFv = aFalFv + delta_aFalFv;
        }
    }

    if(abs(ferr) < tolerance){
        result[0] = 1.0;
        result[1] = aFalFv/(aAct*aFal);
    }

    return result;
}
