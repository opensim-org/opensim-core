// Thelen2003MuscleV1.cpp
/* Author: Matthew Millard
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
#include <OpenSim/Actuators/Thelen2003MuscleV1.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/Storage.h>
#include <iostream>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Thelen2003MuscleV1::Thelen2003MuscleV1() :
   ActivationFiberLengthMuscle_Deprecated(),
    _activationTimeConstant(_activationTimeConstantProp.getValueDbl()),
    _deactivationTimeConstant(_deactivationTimeConstantProp.getValueDbl()),
    _vmax(_vmaxProp.getValueDbl()),
    _vmax0(_vmax0Prop.getValueDbl()),
    _fmaxTendonStrain(_fmaxTendonStrainProp.getValueDbl()),
    _fmaxMuscleStrain(_fmaxMuscleStrainProp.getValueDbl()),
    _kShapeActive(_kShapeActiveProp.getValueDbl()),
    _kShapePassive(_kShapePassiveProp.getValueDbl()),
    _damping(_dampingProp.getValueDbl()),
    _af(_afProp.getValueDbl()),
    _flen(_flenProp.getValueDbl()),
    _act(_actProp.getValueDbl()),
    _dact(_dactProp.getValueDbl()),
    _fal(_falProp.getValueDbl()),
    _fpe(_fpeProp.getValueDbl()),
    _fse(_fseProp.getValueDbl()),
    _fv(_fvProp.getValueDbl()),
    _tl(_tlProp.getValueDbl()),
    _lce(_lceProp.getValueDbl()),
    _dlce(_dlceProp.getValueDbl()),
    _fvVmax(_fvVmaxProp.getValueDbl()),
    _u(_uProp.getValueDbl()),
    _ca(_caProp.getValueDbl()),
    _tendonPE(_tendonPEProp.getValueDbl()),
    _musclePE(_musclePEProp.getValueDbl()),
    _musclePWR(_musclePWRProp.getValueDbl()),
    _muscleV(_muscleVProp.getValueDbl()),
    _muscleF(_muscleFProp.getValueDbl()),
    _splineFal(_splineFalProp.getValueBool()),
    _splineFv(_splineFvProp.getValueBool())			
{
    
    setNull();
    setupProperties();
    if(_splineFal == true){
        loadForceActiveLengthCurve();
    }
    if(_splineFv == true){
        loadForceVelocityCurve();
    }
    
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Thelen2003MuscleV1::Thelen2003MuscleV1(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle) :
   ActivationFiberLengthMuscle_Deprecated(),
    _activationTimeConstant(_activationTimeConstantProp.getValueDbl()),
    _deactivationTimeConstant(_deactivationTimeConstantProp.getValueDbl()),
    _vmax(_vmaxProp.getValueDbl()),
    _vmax0(_vmax0Prop.getValueDbl()),
    _fmaxTendonStrain(_fmaxTendonStrainProp.getValueDbl()),
    _fmaxMuscleStrain(_fmaxMuscleStrainProp.getValueDbl()),
    _kShapeActive(_kShapeActiveProp.getValueDbl()),
    _kShapePassive(_kShapePassiveProp.getValueDbl()),
    _damping(_dampingProp.getValueDbl()),
    _af(_afProp.getValueDbl()),
    _flen(_flenProp.getValueDbl()),
    _act(_actProp.getValueDbl()),
    _dact(_dactProp.getValueDbl()),
    _fal(_falProp.getValueDbl()),
    _fpe(_fpeProp.getValueDbl()),
    _fse(_fseProp.getValueDbl()),
    _fv(_fvProp.getValueDbl()),
    _tl(_tlProp.getValueDbl()),
    _lce(_lceProp.getValueDbl()),
    _dlce(_dlceProp.getValueDbl()),
    _fvVmax(_fvVmaxProp.getValueDbl()),
    _u(_uProp.getValueDbl()),
    _ca(_caProp.getValueDbl()),
    _tendonPE(_tendonPEProp.getValueDbl()),
    _musclePE(_musclePEProp.getValueDbl()),
    _musclePWR(_musclePWRProp.getValueDbl()),
    _muscleV(_muscleVProp.getValueDbl()),
    _muscleF(_muscleFProp.getValueDbl()),
    _splineFal(_splineFalProp.getValueBool()),
    _splineFv(_splineFvProp.getValueBool())
{
    setNull();
    setupProperties();
    if(_splineFal == true){
        loadForceActiveLengthCurve();
    }
    if(_splineFv == true){
        loadForceVelocityCurve();
    }

    setName(aName);
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Thelen2003MuscleV1::~Thelen2003MuscleV1()
{

}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle Thelen2003MuscleV1 to be copied.
 */
Thelen2003MuscleV1::Thelen2003MuscleV1(const Thelen2003MuscleV1 &aMuscle) :
   ActivationFiberLengthMuscle_Deprecated(aMuscle),
    _activationTimeConstant(_activationTimeConstantProp.getValueDbl()),
    _deactivationTimeConstant(_deactivationTimeConstantProp.getValueDbl()),
    _vmax(_vmaxProp.getValueDbl()),
    _vmax0(_vmax0Prop.getValueDbl()),
    _fmaxTendonStrain(_fmaxTendonStrainProp.getValueDbl()),
    _fmaxMuscleStrain(_fmaxMuscleStrainProp.getValueDbl()),
    _kShapeActive(_kShapeActiveProp.getValueDbl()),
    _kShapePassive(_kShapePassiveProp.getValueDbl()),
    _damping(_dampingProp.getValueDbl()),
    _af(_afProp.getValueDbl()),
    _flen(_flenProp.getValueDbl()),
    _act(_actProp.getValueDbl()),
    _dact(_dactProp.getValueDbl()),
    _fal(_falProp.getValueDbl()),
    _fpe(_fpeProp.getValueDbl()),
    _fse(_fseProp.getValueDbl()),
    _fv(_fvProp.getValueDbl()),
    _tl(_tlProp.getValueDbl()),
    _lce(_lceProp.getValueDbl()),
    _dlce(_dlceProp.getValueDbl()),
    _fvVmax(_fvVmaxProp.getValueDbl()),
    _u(_uProp.getValueDbl()),
    _ca(_caProp.getValueDbl()),
    _tendonPE(_tendonPEProp.getValueDbl()),
    _musclePE(_musclePEProp.getValueDbl()),
    _musclePWR(_musclePWRProp.getValueDbl()),
    _muscleV(_muscleVProp.getValueDbl()),
    _muscleF(_muscleFProp.getValueDbl()),
    _splineFal(_splineFalProp.getValueBool()),
    _splineFv(_splineFvProp.getValueBool())
{
    setNull();
    setupProperties();
    if(_splineFal == true){
        loadForceActiveLengthCurve();
    }    
    if(_splineFv == true){
        loadForceVelocityCurve();
    }

    copyData(aMuscle);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Thelen2003MuscleV1 to another.
 *
 * @param aMuscle Thelen2003MuscleV1 to be copied.
 */
void Thelen2003MuscleV1::copyData(const Thelen2003MuscleV1 &aMuscle)
{
    _activationTimeConstant = aMuscle._activationTimeConstant;
    _deactivationTimeConstant = aMuscle._deactivationTimeConstant;
    _vmax = aMuscle._vmax;
    _vmax0 = aMuscle._vmax0;
    _fmaxTendonStrain = aMuscle._fmaxTendonStrain;
    _fmaxMuscleStrain = aMuscle._fmaxMuscleStrain;
    _kShapeActive = aMuscle._kShapeActive;
    _kShapePassive = aMuscle._kShapePassive;
    _damping = aMuscle._damping;
    _af = aMuscle._af;
    _flen = aMuscle._flen;
    //MM: Hmm, this will not copy over the spline curve fit data if there is any, as it is private.
}

//_____________________________________________________________________________
/**
 * Set the data members of this Thelen2003MuscleV1 to their null values.
 */
void Thelen2003MuscleV1::setNull()
{
}

//_____________________________________________________________________________
/**
 * Populate spline arrays for muscle curves. 
 */
void Thelen2003MuscleV1::loadForceActiveLengthCurve(){
    if(_splineFal == true){
        string fname = "muscle_afl.sto";
        ifstream ifile(fname.c_str());
        //SimTK::Spline_<Real> sTK = SimTK::SplineFitter<Real>::fitForSmoothingParameter(3,xK,yKVal,0.0).getSpline();
        if(ifile){
            _ncsfal        = get1DSpline(fname);
        }else{
            _splineFal = false;
            cout << "Thelen2003MuscleV1 couldn't find " << fname << " reverting to default\n";
        }
    }
}

void Thelen2003MuscleV1::loadForceVelocityCurve(){
    if(_splineFv == true){
        string fname = "muscle_fv.sto";
        ifstream ifile(fname.c_str());
        //SimTK::Spline_<Real> sTK = SimTK::SplineFitter<Real>::fitForSmoothingParameter(3,xK,yKVal,0.0).getSpline();
        if(ifile){
            _ncsfv        = get1DSpline(fname);
        }else{
            _splineFv = false;
            cout << "Thelen2003MuscleV1 couldn't find " << fname << " reverting to default\n";
        }
    }
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Thelen2003MuscleV1::setupProperties()
{
    _activationTimeConstantProp.setName("activation_time_constant");
    _activationTimeConstantProp.setValue(0.01);
    _activationTimeConstantProp.setComment("time constant for ramping up of muscle activation");
    _propertySet.append(&_activationTimeConstantProp, "Parameters");

    _deactivationTimeConstantProp.setName("deactivation_time_constant");
    _deactivationTimeConstantProp.setValue(0.04);
    _deactivationTimeConstantProp.setComment("time constant for ramping down of muscle activation");
    _propertySet.append(&_deactivationTimeConstantProp, "Parameters");

    _vmaxProp.setName("Vmax");
    _vmaxProp.setValue(10.0);
    _vmaxProp.setComment("maximum contraction velocity at full activation in fiber lengths per second");
    _propertySet.append(&_vmaxProp, "Parameters");

    _vmax0Prop.setName("Vmax0");
    _vmax0Prop.setValue(5.0);
    _vmax0Prop.setComment("maximum contraction velocity at low activation in fiber lengths per second");
    _propertySet.append(&_vmax0Prop, "Parameters");

    _fmaxTendonStrainProp.setName("FmaxTendonStrain");
    _fmaxTendonStrainProp.setValue(0.033);
    _fmaxTendonStrainProp.setComment("tendon strain due to maximum isometric muscle force");
    _propertySet.append(&_fmaxTendonStrainProp, "Parameters");

    _fmaxMuscleStrainProp.setName("FmaxMuscleStrain");
    _fmaxMuscleStrainProp.setValue(0.6);
    _fmaxMuscleStrainProp.setComment("passive muscle strain due to maximum isometric muscle force");
    _propertySet.append(&_fmaxMuscleStrainProp, "Parameters");

    _kShapeActiveProp.setName("KshapeActive");
    _kShapeActiveProp.setValue(0.5);
    _kShapeActiveProp.setComment("shape factor for Gaussian active muscle force-length relationship");
    _propertySet.append(&_kShapeActiveProp, "Parameters");

    _kShapePassiveProp.setName("KshapePassive");
    _kShapePassiveProp.setValue(4.0);
    _kShapePassiveProp.setComment("exponential shape factor for passive force-length relationship");
    _propertySet.append(&_kShapePassiveProp, "Parameters");

    _dampingProp.setName("damping");
    _dampingProp.setValue(0.05);
    _dampingProp.setComment("passive damping in the force-velocity relationship");
    _propertySet.append(&_dampingProp, "Parameters");

    _afProp.setName("Af");
    _afProp.setValue(0.3);
    _afProp.setComment("force-velocity shape factor");
    _propertySet.append(&_afProp, "Parameters");

    _flenProp.setName("Flen");
    _flenProp.setValue(1.8);
    _flenProp.setComment("maximum normalized lengthening force");
    _propertySet.append(&_flenProp, "Parameters");

    //M.Millard.
    //These variables are NOT used for computation of the model, just for computing
    //and storing their corresponding muscle properties to produce the dimensionless
    //muscle charts
    _dactProp.setName("dact");
    _dactProp.setValue(0);    
    _dactProp.setComment("Muscle d/dt Activation");
    _propertySet.append(&_dactProp,"Parameters");
    
    _actProp.setName("act");
    _actProp.setValue(0);    
    _actProp.setComment("Muscle Activation");
    _propertySet.append(&_actProp,"Parameters");
    
    _falProp.setName("fal");
    _falProp.setValue(0);    
    _falProp.setComment("Normalized Active Length Multiplication Factor");
    _propertySet.append(&_falProp,"Parameters");


    _fseProp.setName("fse");
    _fseProp.setValue(0);    
    _fseProp.setComment("Normalized Tendon Force");
    _propertySet.append(&_fseProp,"Parameters");

    _fpeProp.setName("fpe");
    _fpeProp.setValue(0);    
    _fpeProp.setComment("Normalized Passive Muscle Force");
    _propertySet.append(&_fpeProp,"Parameters");

    _fvProp.setName("fv");
    _fvProp.setValue(0);    
    _fvProp.setComment("Normalized Velocity Multiplication Factor");
    _propertySet.append(&_fvProp,"Parameters");

    _tlProp.setName("tl");
    _tlProp.setValue(0);    
    _tlProp.setComment("Normalized Length");
    _propertySet.append(&_tlProp,"Parameters");

    _lceProp.setName("lce");
    _lceProp.setValue(0);    
    _lceProp.setComment("Normalized Contractile Element Length");
    _propertySet.append(&_lceProp,"Parameters");

    _dlceProp.setName("dlce");
    _dlceProp.setValue(0);    
    _dlceProp.setComment("Normalized Contractile Element Velocity");
    _propertySet.append(&_dlceProp,"Parameters");

    _fvVmaxProp.setName("Vmax");
    _fvVmaxProp.setValue(10.0);    
    _fvVmaxProp.setComment("fv curve Vmax");
    _propertySet.append(&_fvVmaxProp,"Parameters");

    _uProp.setName("u");
    _uProp.setValue(0);
    _uProp.setComment("Muscle excitation");
    _propertySet.append(&_uProp,"Parameters");

    _caProp.setName("ca");
    _caProp.setValue(1.0);
    _caProp.setComment("Cosine of pennation");
    _propertySet.append(&_caProp,"Parameters");

    _tendonPEProp.setName("tendonPE");
    _tendonPEProp.setValue(0.0);
    _tendonPEProp.setComment("Tendon potential energy");
    _propertySet.append(&_tendonPEProp,"Parameters");

    _musclePEProp.setName("musclePE");
    _musclePEProp.setValue(0.0);
    _musclePEProp.setComment("Muscle potential energy");
    _propertySet.append(&_musclePEProp,"Parameters");
    
    _musclePWRProp.setName("musclePWR");
    _musclePWRProp.setValue(0.0);
    _musclePWRProp.setComment("Muscle Power");
    _propertySet.append(&_musclePWRProp,"Parameters");

    _muscleVProp.setName("muscleV");
    _muscleVProp.setValue(0.0);
    _muscleVProp.setComment("Muscle Velocity");
    _propertySet.append(&_muscleVProp,"Parameters");

    //Active force length spline curve backdoor
    _splineFalProp.setName("splineFalBackdoor");
    _splineFalProp.setValue(false); // Set to true if you'd like to use
                                    // spline curves to define the active force length
                                    // curve instead of the Gaussian. 
                                    // This file should be a *.sto file in the setup
                                    // directory, and the file should be named 
                                    // muscle_afl.sto
    _splineFalProp.setComment("Active Force Length Spline Backdoor Flag");
    _propertySet.append(&_splineFalProp,"Parameters");
    
    //Set to true if you'd like the 1D version of the Thelen force velocity
    //surface to be used
    _splineFvProp.setName("Fv1DBackdoor");
    _splineFvProp.setValue(false);
    _splineFvProp.setComment("Force Velocity Backdoor Flag");
    _propertySet.append(&_splineFvProp,"Parameters");

}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Thelen2003MuscleV1& Thelen2003MuscleV1::operator=(const Thelen2003MuscleV1 &aMuscle)
{
    // BASE CLASS
    ActivationFiberLengthMuscle_Deprecated::operator=(aMuscle);
    copyData(aMuscle);

    return(*this);
}


//=============================================================================
// GET
//=============================================================================

//-----------------------------------------------------------------------------
// ACTIVATION TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant for ramping up of muscle force.
 *
 * @param aActivationTimeConstant The time constant for ramping up of muscle force.
 * @return Whether the time constant was successfully changed.
 */
bool Thelen2003MuscleV1::setActivationTimeConstant(double aActivationTimeConstant)
{
    _activationTimeConstant = aActivationTimeConstant;
    return true;
}

//-----------------------------------------------------------------------------
// DEACTIVATION TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant for ramping down of muscle force.
 *
 * @param aDeactivationTimeConstant The time constant for ramping down of muscle force.
 * @return Whether the time constant was successfully changed.
 */
bool Thelen2003MuscleV1::setDeactivationTimeConstant(double aDeactivationTimeConstant)
{
    _deactivationTimeConstant = aDeactivationTimeConstant;
    return true;
}


//-----------------------------------------------------------------------------
// MAX CONTRACTION VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum contraction velocity of the fibers, in optimal fiber lengths per second.
 *
 * @param aVmax The maximum contraction velocity of the fibers, in optimal fiber lengths per second.
 * @return Whether the maximum contraction velocity was successfully changed.
 */
bool Thelen2003MuscleV1::setVmax(double aVmax)
{
    _vmax = aVmax;
    return true;
}

//-----------------------------------------------------------------------------
// MAX CONTRACTION VELOCITY AT LOW ACTIVATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum contraction velocity at low activation of the fibers, in optimal fiber lengths per second.
 *
 * @param aVmax The maximum contraction velocity at low activation of the fibers, in optimal fiber lengths per second.
 * @return Whether the maximum contraction velocity was successfully changed.
 */
bool Thelen2003MuscleV1::setVmax0(double aVmax0)
{
    _vmax0 = aVmax0;
    return true;
}

//-----------------------------------------------------------------------------
// TENDON STRAIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the tendon strain due to maximum isometric muscle force.
 *
 * @param aFmaxTendonStrain The tendon strain due to maximum isometric muscle force.
 * @return Whether the tendon strain was successfully changed.
 */
bool Thelen2003MuscleV1::setFmaxTendonStrain(double aFmaxTendonStrain)
{
    _fmaxTendonStrain = aFmaxTendonStrain;
    return true;
}

//-----------------------------------------------------------------------------
// MUSCLE STRAIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the passive muscle strain due to maximum isometric muscle force.
 *
 * @param aFmaxMuscleStrain The passive muscle strain due to maximum isometric muscle force.
 * @return Whether the passive muscle strain was successfully changed.
 */
bool Thelen2003MuscleV1::setFmaxMuscleStrain(double aFmaxMuscleStrain)
{
    _fmaxMuscleStrain = aFmaxMuscleStrain;
    return true;
}

//-----------------------------------------------------------------------------
// SHAPE FACTOR ACTIVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the shape factor for Gaussian active muscle force-length relationship.
 *
 * @param aKShapeActive The shape factor for Gaussian active muscle force-length relationship.
 * @return Whether the shape factor was successfully changed.
 */
bool Thelen2003MuscleV1::setKshapeActive(double aKShapeActive)
{
    _kShapeActive = aKShapeActive;
    return true;
}

//-----------------------------------------------------------------------------
// SHAPE FACTOR PASSIVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the shape factor for Gaussian passive muscle force-length relationship.
 *
 * @param aKshapePassive The shape factor for Gaussian passive muscle force-length relationship.
 * @return Whether the shape factor was successfully changed.
 */
bool Thelen2003MuscleV1::setKshapePassive(double aKshapePassive)
{
    _kShapePassive = aKshapePassive;
    return true;
}

//-----------------------------------------------------------------------------
// DAMPING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the damping factor related to maximum contraction velocity.
 *
 * @param aDamping The damping factor related to maximum contraction velocity.
 * @return Whether the damping factor was successfully changed.
 */
bool Thelen2003MuscleV1::setDamping(double aDamping)
{
    _damping = aDamping;
    return true;
}

//-----------------------------------------------------------------------------
// FORCE-VELOCITY SHAPE FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the force-velocity shape factor.
 *
 * @param aAf The force-velocity shape factor.
 * @return Whether the shape factor was successfully changed.
 */
bool Thelen2003MuscleV1::setAf(double aAf)
{
    _af = aAf;
    return true;
}

//-----------------------------------------------------------------------------
// FORCE-VELOCITY SHAPE FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum normalized lengthening force.
 *
 * @param aFlen The maximum normalized lengthening force.
 * @return Whether the maximum normalized lengthening force was successfully changed.
 */
bool Thelen2003MuscleV1::setFlen(double aFlen)
{
    _flen = aFlen;
    return true;
}

/** Get the rate change of activation */
double Thelen2003MuscleV1::getdactdt(){return _dact;}    

/** Get activation */
double Thelen2003MuscleV1::getact(){return _act;}    //activation

/** Get normalized active force length */
double Thelen2003MuscleV1::getfal(){return _fal;}    //normalized active force length

/** Get normalized tendon force */
double Thelen2003MuscleV1::getfse(){return _fse;}    //normalized tendon force

/** Get normalized passive muscle force */
double Thelen2003MuscleV1::getfpe(){return _fpe;}    //normalized passive muscle force

/** Get the fv multiplicative factor */
double Thelen2003MuscleV1::getfv(){return _fv;}        //normalized fv multiplicative factor

/** Get Vmax, a variable taht is constant in Thelen 2003 Eqn. 6, but varies here */
double Thelen2003MuscleV1::getfvVmax(){return _fvVmax;}    //internal variable to fv & dlce - Vmax

/** Get normalized tendon length*/
double Thelen2003MuscleV1::gettl(){return _tl;}        //normalized tendon length

/** Get normalized contractile element length*/
double Thelen2003MuscleV1::getlce(){return _lce;}    //normalized contractile element length

/** Get normalized muscle velocity */
double Thelen2003MuscleV1::getdlce(){return _dlce;}    //normalized contractile element velocity

/** Get normalized muscle excitation */
double Thelen2003MuscleV1::getu(){return _u;}    //normalized contractile element velocity

double Thelen2003MuscleV1::getca(){return _ca;} //cosine(pennation_angle)

double Thelen2003MuscleV1::getTendonPE(){    return _tendonPE;}    //get the potential energy stored in the tendon
double Thelen2003MuscleV1::getMusclePE(){    return _musclePE;}    //get the potential energy stored in the muscle
double Thelen2003MuscleV1::getMusclePWR(){    return _musclePWR;}        //get the work done by the muscle

double Thelen2003MuscleV1::getMuscleF(){    return _muscleF;}        //get the velocity of the muscle along the tendon
double Thelen2003MuscleV1::getMuscleV(){    return _muscleV;}        //get the force of the muscle along the tendon

void Thelen2003MuscleV1::useSplineFal(){    
    _splineFal=true;
    loadForceActiveLengthCurve();    
}

void Thelen2003MuscleV1::useSplineFv(){
    _splineFv = true;
    loadForceVelocityCurve();   
}

void Thelen2003MuscleV1::useDefaultThelen(){
    _splineFal=false;
    _splineFv = false;
}


//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * MMillard 2011/10
 * 
 * This function has almost entirely been rewritten, as the old version contained
 * implementation errors. The new version works as follows
 *
 * 1. Limit variables that cause singularities in the model to valid regions
 *
 * a)    0 <= u <= 1
 * b)    0 <= a <= 1
 *            {    0 < a < 1 : da(u,a)
 *        da={     a <= 0 && da(u,a) > 0 : da(u,a), else 0 
 *            {    a >= 0 && da(u,a) < 0 : da(u,a), else 0
 * c) 0 <= cos(pennation) <= cos(89 degrees)
 * d) fal is limited to values greater than zero by the computeActiveForce
 *    function
 * 
 * These steps remove the singularities present when one tries to use the 
 * equilibrium equation to solve for fv*fal*a.
 * 
 * 2. Compute the tendon force, and the muscle's parallel element force
 * 3. Solve for fv*fal*a using the equilibrium equation
 * 4. Invert the force velocity curve supplied by Thelen2003 in an invertible
 *    region. Use linear extrapolations of this function outside of the 
 *      (easily numerically) invertible regions.
 * 5. Set the fiber velocity
 * 6. Return the tendon force
 *
 *  @return The force in the tendon.
 */
double  Thelen2003MuscleV1::computeActuation(const SimTK::State& s) const
{
    double tendonForce;
    double passiveForce;
    double activationDeriv;
    double fiberLengthDeriv;

    double norm_tendon_length, ca;
    double norm_muscle_tendon_length, pennation_angle;
    double excitation = getExcitation(s);

	const double &maxIsometricForce = _maxIsometricForce; //getPropertyValue<double>("max_isometric_force");
    const double &optimalFiberLength = _optimalFiberLength; //getPropertyValue<double>("optimal_fiber_length");
	const double &tendonSlackLength = _tendonSlackLength; //getPropertyValue<double>("tendon_slack_length");
	const double &pennationAngleAtOptimal = _pennationAngleAtOptimal; //getPropertyValue<double>("pennation_angle_at_optimal");

    /* Normalize the muscle states */
    double activation = getActivation(s);
    double normFiberLength = getFiberLength(s) / optimalFiberLength;

    _lce = normFiberLength; //MM

    // Maximum contraction velocity is an activation scaled value
    double Vmax = _vmax;
    
    //MM: Not documented in Thelen2003
    //if (activation < 1.0) {
    //    Vmax = _vmax0 + activation*(Vmax-_vmax0);
    //}
    
    _fvVmax = Vmax; //MM
    

    //MM excitation bounds check
    if (excitation < 0.0){
        excitation = 0.0;
    }
    if(excitation > 1.0){
        excitation = 1.0;
    }

    double activation_min = 0.01; 
    if (activation >= 1.0){    
        activation = 1.0;    //This will not affect the state, unfortunately, but it will prevent
    }                        //the model from using an activation greater than 1
    
    if (activation <= activation_min){
        activation = activation_min;      
    }                                    //This will prevent a lot of singularites in later code
                                            

    /* Compute normalized muscle state derivatives */
    double tau_a = 0.0; //MM updated according to paper
    if (excitation >= activation) {
        tau_a = _activationTimeConstant * (0.5 + 1.5*activation);    //MM updated according to paper
        activationDeriv = (excitation - activation) / tau_a;        //MM updated according to paper
    } else {
        tau_a = _deactivationTimeConstant/(0.5+1.5*activation);        //MM updated according to paper
        activationDeriv = (excitation - activation) / tau_a;        //MM updated according to paper
    }

    //MM: bounds check on activation
    //MM FIX: hard coded to help Edith quickly
    //MM: I'd also set activation directly, but I cannot as it is a state variable
    if (activation >= 1.0 && activationDeriv > 0.0){    
        activationDeriv = 0.0;
    }
    if (activation <= activation_min && activationDeriv < 0.0){
        activationDeriv = 0.0;
        activation = activation_min;     //This will not affect the state, but it will
                        //prevent a lot of singularites in later code
    }    
    

    _act = activation; //MM
    _u = excitation; //MM
    _dact = activationDeriv; //MM

    //MM update calcPennation: it returns 0 when normFiberLength = 0, should return Pi/2
    if(normFiberLength > 0.0){
        pennation_angle = calcPennation( normFiberLength, 1.0, pennationAngleAtOptimal);
    }else{
        pennation_angle = SimTK_PI/2;
    }
    ca = cos(pennation_angle);    

    //MM bounding ca away from singularity
    //MM FIX: hard coded constant of cos(89 degrees) quick and dirty for Edith
    double ca_min = 0.0175; //equivalent to 89 degrees.
    if(ca <= ca_min){
        ca = ca_min;
        pennation_angle = acos(ca);
    }
    _ca = ca; //MM

    norm_muscle_tendon_length = getLength(s) / optimalFiberLength;
    norm_tendon_length = norm_muscle_tendon_length - normFiberLength * ca;

    double tmp_norm_resting_length = tendonSlackLength / optimalFiberLength; //MM
    double tmp_tendon_strain =  (norm_tendon_length - tmp_norm_resting_length) / tmp_norm_resting_length; //MM

    _tl = tmp_tendon_strain; //MM

    tendonForce = calcTendonForce(s,norm_tendon_length);
    passiveForce = calcPassiveForce(s,normFiberLength);
    double compressiveForce = calcPassiveCompressiveForce(s,normFiberLength);
    double activeForce = calcActiveForce(s,normFiberLength);

    _fse = tendonForce;//MM
    _fpe = passiveForce;//MM
    _fal = activeForce; //MM

    //MM for generating the dimensionless plots
    double velocity_dependent_force = tendonForce / ca - passiveForce; //MM
    _fv = velocity_dependent_force / (activation * activeForce);
    

    fiberLengthDeriv = calcFiberVelocity(s,activation,activeForce,velocity_dependent_force);
    _dlce = Vmax*fiberLengthDeriv; //MM

    /* Un-normalize the muscle state derivatives and forces. */
    setActivationDeriv(s, activationDeriv );
    setFiberLengthDeriv(s, Vmax*optimalFiberLength*fiberLengthDeriv ); 

    tendonForce = tendonForce *  maxIsometricForce;
    setForce(s, tendonForce);
    setTendonForce(s, tendonForce);
    setPassiveForce( s, passiveForce * maxIsometricForce);

    //Compute muscle work: tendonForce*dlce/dt
    //            

    double dlcedt = (Vmax*optimalFiberLength*fiberLengthDeriv);
    double lce = (normFiberLength*optimalFiberLength);

    _muscleF = -(velocity_dependent_force*maxIsometricForce*ca);
    _muscleV = dlcedt*ca + lce*(-sin(pennation_angle))*((-dlcedt/lce)*tan(pennation_angle));
    _musclePWR = _muscleF*_muscleV;

    return( tendonForce );
}

//_____________________________________________________________________________
/**
 * Edited by M.Millard 2011/10.
 * Original version did not match the equations in the paper because it had a
 * very shallow ramp added to it. Now this function matches the Thelen2003 paper
 *
 * From cmg_dt.c - calc_tendon_force_dt
 *
 * CALC_TENDON_FORCE_DT: this routine calculates the force in tendon by finding
 * tendon strain and using it in an exponential function (JBME 2003 - Thelen)
 * FmaxTendonStrain - Function is parameterized by the tendon strain due to maximum isometric muscle force
 * This should be specified as a dynamic parameter in the muscle file
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double Thelen2003MuscleV1::calcTendonForce(const SimTK::State& s, double aNormTendonLength) const
{
	const double &maxIsometricForce = _maxIsometricForce; //getPropertyValue<double>("max_isometric_force");
    const double &optimalFiberLength = _optimalFiberLength; //getPropertyValue<double>("optimal_fiber_length");
	const double &tendonSlackLength = _tendonSlackLength; //getPropertyValue<double>("tendon_slack_length");

    //MM - This business of normalizing the tendon length with respect to the fiber length is weird
    //It works, but is a candidate for revision.
    double norm_resting_length = tendonSlackLength / optimalFiberLength;
    double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

    
    double kToe = 3.0;
    double eToe = 0.609*_fmaxTendonStrain;
    double Ftoe = 0.333333;
    double klin = 1.712/_fmaxTendonStrain;

    //Compute tendon force
    double tendon_force;
    if (tendon_strain>eToe)
        tendon_force = klin*(tendon_strain-eToe)+Ftoe;
    else if (tendon_strain>0.0) 
        tendon_force = (Ftoe/(exp(kToe)-1.0)) * (exp(kToe*tendon_strain/eToe)-1.0);
    else
        tendon_force=0.;

    //Compute the energy stored in the tendon. 
    //Integrals computed symbolically in muscle_kepew_20111021.mw just to check
    _tendonPE = 0.0;
    double lenTdn    = (tendon_strain+1)*tendonSlackLength;
    double lenToe    = (eToe+1.0)*tendonSlackLength;    
    double lenR        = tendonSlackLength;
    double fiso        =maxIsometricForce;

    if(s.getTime() >= 0.1){
        double debuggingTime = 1.0; //MM temporary, remove later.
    }

    if (tendon_strain>eToe){
        //compute the energy stored in the toe portion of the tendon strain curve
        double len = lenToe;
        double toePE_len = (fiso*Ftoe/(exp(kToe)-1.0)) * ((lenR*eToe/kToe) * exp(kToe*(len-lenR)/(lenR*eToe)) - len);
        len =  lenR;
        double toePE_0    =  (fiso*Ftoe/(exp(kToe)-1.0)) * ((lenR*eToe/kToe) * exp(kToe*(len-lenR)/(lenR*eToe)) - len);
        double toePEtest = toePE_len-toePE_0;


        //compute the energy stored in the linear section of the tendon strain curve from ..... 0 to len
        len = lenTdn;
        double linPE_len = (1.0/2.0)*(fiso*klin*(len*len)/lenR) + fiso*len*(klin*(-1.0-eToe)+Ftoe);
        //ditto from 0 .... eToe
        len = lenToe;
        double linPE_eToe= (1.0/2.0)*(fiso*klin*(len*len)/lenR) + fiso*len*(klin*(-1.0-eToe)+Ftoe);
        double linPEtest = linPE_len-linPE_eToe;
        
        //compute the total potential energy stored in the tendon
        _tendonPE =(toePE_len-toePE_0) + (linPE_len-linPE_eToe);
    }else if (tendon_strain>0.0){ 
        //PE from 0 .... len
        double len = lenTdn;
        double toePE_len = (maxIsometricForce*Ftoe/(exp(kToe)-1.0)) * ((lenR*eToe/kToe) * exp(kToe*(len-lenR)/(lenR*eToe)) - len);
        //PE from 0 .... eToe
        len = lenR;
        double toePE_0    =  (maxIsometricForce*Ftoe/(exp(kToe)-1.0)) * ((lenR*eToe/kToe) * exp(kToe*(len-lenR)/(lenR*eToe)) - len);

        //Compute the total PE stored in the tendon
        _tendonPE = toePE_len-toePE_0;
    }else{
        _tendonPE = 0.0;
    }
    
    
    return tendon_force;
}

//_____________________________________________________________________________
/**
 * M.Millard 2011/10.
 * The original version (in Thelen2003Muscle) did not match the equations in the 
 * paper because it had a slightly different equations. Now this function matches 
 * that returned non-zero values even when the normalized muscle length was < 1.
 * The implementation below matches Eqn. 3 of Thelen 2003, and does not develop
 * force until the normalized muscle length is greater than 1.0.
 * 
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The passive force in the muscle fibers.
 */
double Thelen2003MuscleV1::calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const
{
    double passive_force;

	const double &maxIsometricForce = _maxIsometricForce; //getPropertyValue<double>("max_isometric_force");
    const double &optimalFiberLength = _optimalFiberLength; //getPropertyValue<double>("optimal_fiber_length");

    //Compute the passive force developed by the muscle
    if(aNormFiberLength > 1.0){
        passive_force = (exp(_kShapePassive*(aNormFiberLength-1.0)/_fmaxMuscleStrain)-1.0) / (exp(_kShapePassive)-1.0);
    }else{
        passive_force = 0.0;
    }

    _musclePE = 0.0;
    //Compute the potential energy stored in the muscle
    if(aNormFiberLength > 1.0){
        //Shorter variable names to make the equations readable.
        double len = aNormFiberLength*optimalFiberLength;
        double lenR = optimalFiberLength;
        double kpe = _kShapePassive;
        double e0 = _fmaxMuscleStrain;
        double fiso = maxIsometricForce;

        //PE storage at current stretch
        double fpePE_len = (fiso/(exp(kpe)-1))*( (lenR*e0/kpe)*exp( (kpe/e0)*( (len/lenR)-1)) - len); 
        
        //PE when muscle goes slack.
        len = optimalFiberLength;
        double fpePE_0 = (fiso/(exp(kpe)-1))*( (lenR*e0/kpe)*exp( (kpe/e0)*( (len/lenR)-1)) - len); 

        _musclePE = fpePE_len - fpePE_0;

    }else{
        _musclePE = 0.0;
    }


    return passive_force;
}

//_____________________________________________________________________________
/**
 * Edited by M.Millard 2011/10.
 * Now includes the option to use spline fit interpolations of active force length
 * 
 * From gmc.dt.c - calc_active_force_dt
 *
 * CALC_ACTIVE_FORCE_DT: this routine calculates the active component of force
 * in the muscle fibers. It uses the current fiber length to interpolate the
 * active force-length curve - described by Gaussian curve as in Thelen, JBME 2003
 * *
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The active force in the muscle fibers.
 */
double Thelen2003MuscleV1::calcActiveForce(const SimTK::State& s, double aNormFiberLength) const
{
    

    double fal = 0.0;
    double fal_min = 0.01;
    
    if(_splineFal == true){
        //MM The spline class needs revision if this much code has to be written
        //     just to get a value from it. 
        SimTK::Vector fiblen(1,aNormFiberLength);
        fal = (double)_ncsfal.calcValue(fiblen);
    }else{
        double x=(aNormFiberLength-1.)*(aNormFiberLength-1.);
        fal = exp(-x/_kShapeActive);
    }

    //Lower bound on fal to prevent a singularity
    if(fal < fal_min){
        fal = fal_min;
    }

    return fal;
}

//_____________________________________________________________________________
/**
 * M.Millard 2011/10.
 *
 * Presently not included in the muscle model, but would be a useful addition.
 *
 * This function was added to prevent this muscle model from computing contractile
 * element lengths that are shorter than what is physiologically possible 
 * (0.5*optimal_force_length). Without this addition the Thelen model will converge
 * to contractile element lengths that have no lower bound if geometry allows. This
 * function will generate a compressive force using a function that mirrors the
 * muscle's parallel element (but in compression) when the fiber length is less than
 * half of its optimal value.
 * 
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The active force in the muscle fibers.
 */
double Thelen2003MuscleV1::calcPassiveCompressiveForce(const SimTK::State& s, double aNormFiberLength) const
{
    double minFibLen = 0.5;
    double passive_force = 0.0;
    
    if(aNormFiberLength < minFibLen){
        double x = minFibLen-aNormFiberLength;
        passive_force = (exp(_kShapePassive*(x-1)/_fmaxMuscleStrain)-1) / (exp(_kShapePassive)-1);
    }

    return passive_force;
}

//_____________________________________________________________________________
/**
 * From gmc_dt.c - calc_norm_fiber_velocity_dt
 *
 * CALC_NORM_FIBER_VELOCITY_DT: written by Matthew Millard on 2011/10/13
 * The original function written by (?) had some implementation errors in it.
 * This function has been completely re-written to reflect the force-velocity
 * curve that is actually documented in the Thelen 2003 paper. The only difference
 * is that this model includes a linear extrapolation that the Thelen model does
 * not. The model switches from the Thelen model to a continuous and smooth linear
 * extrapolation at the following locations on the force-velocity curve
 *
 *        -infinity  < fv < 0                    : Linear extrapolation
 *                 0 < fv < 0.9*(a*fal*fmax)    : Thelen force velocity curve 
 * 0.9*(a*fal*fmax)< fv                        : Linear extrapolation that is a continuous
 *                                              and smooth extrapolation of the
 *                                              Thelen curves
 *
 * Why make the transition at 0.9? The slopeis not too shallow in this region
 * so transitioning at 0.9 will not cause the integrator undue distress. Why 
 * is the integrator our top priority in this region? Your simulation should not
 * be spending much time beyond this region, as the equilibrium model will never
 * be able to give good results in this region without resulting in 
 * tremendously stiff equations.
 *
 * Dynamic Parameters:
 *   _af - velocity shape factor from Hill's equation
 *   _flen    - Maximum normalized force when muscle is lengthening
 *
 * @param aActivation Activation of the muscle.
 * @param aActiveForce Active force in the muscle fibers.
 * @param aVelocityDependentForce Force value that depends on fiber velocity.
 * @return The velocity of the muscle fibers.
 */
double Thelen2003MuscleV1::calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const
{
    //The variable names have all been switched to closely match with the notation in
    //Thelen 2003.
    double dlce = 0.0;      //contractile element velocity

    if(_splineFv == true){
        double fv = aVelocityDependentForce/(aActivation*aActiveForce);
        SimTK::Vector fvV(1,fv);
        dlce = (double)_ncsfv.calcValue(fvV);
    }else{
            double a    = aActivation;
            double afl  = aActivation*aActiveForce;  //afl = a*fl
            double Fm = aVelocityDependentForce;    //Fm = a*fl*fv    
            double Fmlen = _flen;
            double Fmlen_afl = _flen*aActivation*aActiveForce;

            double dlcedFm = 0.0; //partial deriviative of contactile element velocity w.r.t. Fm

            double b = 0;
            double db= 0;

            double Fm_asyC = 0;                                    //Concentric contraction asymptote
            double Fm_asyE = aActivation*aActiveForce*_flen;    //Eccentric contraction asymptote
            double asyE_thresh = 0.9;    //Should this be user settable?
                                        //What fraction of Fm_asyE Fv is allowed to have before a linear
                                        //model is used

                //If fv is in the appropriate region, use Thelen 2003 Eqns 6 & 7 to compute dlce
                if (Fm > Fm_asyC && Fm < Fm_asyE*asyE_thresh){

                    if( Fm <= afl ){        //Muscle is concentrically contracting
                       b = afl + Fm/_af;
                       db= 1/_af;
                    }else{                    //Muscle is eccentrically contracting
                       b = ((2+2/_af)*(afl*Fmlen-Fm))/(Fmlen-1); 
                       db= ((2+2/_af)*(-1))/(Fmlen-1); 
                    }

                    dlce = (0.25 + 0.75*a)*(Fm-afl)/b; //Scaling by VMAX is left out, and is post multiplied outside of the function

                    //This variable may have future use outside this function
                    dlcedFm = (0.25 + 0.75*a)*(1)/b - ((0.25 + 0.75*a)*(Fm-afl)/(b*b))*db;            

                }else{  //Linear extrapolation
                        double Fm0 = 0.0; //Last Fm value from the Thelen curve

                        //Compute d and db/dFm from Eqn 7. of Thelen2003
                        //for the last
                        if(Fm <= Fm_asyC){ //Concentrically contracting
                            Fm0 = Fm_asyC;
                            b = afl + Fm0/_af;
                            db= 1/_af;               
                        }else{             //Eccentrically contracting
                            Fm0 = asyE_thresh*Fm_asyE;
                            b = ((2+2/_af)*(afl*Fmlen-Fm0))/(Fmlen-1); 
                            db= ((2+2/_af)*(-1))/(Fmlen-1); 
                        }

                        //Compute the last dlce value that falls in the region where
                        //Thelen 2003 Eqn. 6 is valid
                        double dlce0 = (0.25 + 0.75*a)*(Fm0-afl)/b;

                        //Compute the dlce/dfm of Eqn. 6 of Thelen 2003 at the last
                        //valid point
                        dlcedFm = (0.25 + 0.75*a)*(1)/b - ((0.25 + 0.75*a)*(Fm0-afl)/(b*b))*db;

                        //Linearily extrapolate Eqn. 6 from Thelen 2003 to compute
                        //the new value for dlce/dFm
                        dlce = dlce0 + dlcedFm*(Fm-Fm0);            
                    }
            }
        return dlce;
}

//_____________________________________________________________________________
/**
 * MM 2011 10 18
 *
 * This method is called when the constructor is executed. It is used to fetch data from
 * a user specified *.sto file, spline interpolate it using the SimmSpline
 * class and return the result. Note that the columns of the *.sto file need to be
 * 'time' and then 'col0'. Why? Because the class that reads in the data requires it.
 * This should be fixed.
 *
 * @param aFileName: a reference to a string of the filename to read. 
 * @returns SimmSpline* to the interpolated curve in the file.
 */
SimTK::Spline_<SimTK::Real> Thelen2003MuscleV1::get1DSpline(const std::string &aFileName){
    
    Storage curve(aFileName);
    OpenSim::Array<double> curveX, curveY;
    curveX.setSize(curve.getSize());
    curveY.setSize(0);    //MM:This is necessary as getDataColumn appends to this reference!
                            //     the documentation would have you think that the array needs to be
                            //   its full size, and then it is populated. Very confusing!
    curve.getTimeColumn(curveX); //MM: Storage assumes the first column is time ... this does not generalize well
    string colname = "col0";
    curve.getDataColumn(colname,curveY,curveX[0]); //MM: I'm using this because I cannot grab the column
                                                             //    data by index alone - it barfs when I ask for column 0
                                                             //    (not sure why) when I use the function that would 
                                                             //    otherwise do this.    

    //new SimmSpline(curvefalX.getSize(),&curvefalX[0],&curvefalY[0],"Active-Force Length Curve Approximation");
    SimTK::Vector x(curveX.getSize());
    SimTK::Vector y(curveY.getSize());

    double tmpx=0; //just so I can look at the intermediate values in Watch
    double tmpy=0; 

    for(int i=0; i<x.size(); i++){
        tmpx = curveX[i];
        tmpy = curveY[i];

        x(i) = (SimTK::Real)tmpx;
        y(i) = (SimTK::Real)tmpy;
    }

    SimTK::Spline_<SimTK::Real> tmp = SimTK::SplineFitter<SimTK::Real>::fitForSmoothingParameter(3,x,y,0.0).getSpline();

    return tmp;
}



//_____________________________________________________________________________
/**
 * M.Millard 2011/10 Note: I haven't touched this yet. It appears to work, but 
 * a standard numerical method that is well documented should be used. This is
 * a difficult to read, has unknown convergence properties, and may have bugs.
 *__________________________________________________________________________
 * Find the force produced by an actuator (the musculotendon unit), assuming
 * static equilibrium. Using the total muscle-tendon length, it finds the
 * fiber and tendon lengths so that the forces in each match. This routine
 * takes pennation angle into account, so its definition of static equilibrium
 * is when tendon_force = fiber_force * cos(pennation_angle). This funcion
 * will modify the object's values for length, fiberLength, activeForce, 
 * and passiveForce.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double Thelen2003MuscleV1::
computeIsometricForce(SimTK::State& s, double aActivation) const
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

    int i;
    double length,tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
    double cos_factor, fiber_stiffness;
    double old_fiber_length, length_change, tendon_stiffness, percent;
    double error_force = 0.0, old_error_force, tendon_force, norm_tendon_length;
    double passiveForce;
    double fiberLength;

	const double &maxIsometricForce = _maxIsometricForce; //getPropertyValue<double>("max_isometric_force");
    const double &optimalFiberLength = _optimalFiberLength; //getPropertyValue<double>("optimal_fiber_length");
	const double &tendonSlackLength = _tendonSlackLength; //getPropertyValue<double>("tendon_slack_length");
	const double &pennationAngleAtOptimal = _pennationAngleAtOptimal; //getPropertyValue<double>("pennation_angle_at_optimal");

    if (optimalFiberLength < ROUNDOFF_ERROR) {
       setStateVariable(s, STATE_FIBER_LENGTH_NAME, 0.0);
       setPassiveForce(s, 0.0);
       setForce(s, 0.0);
       setTendonForce(s, 0.0);
       return 0.0;
    }

    length = getLength(s);

   // Make first guess of fiber and tendon lengths. Make fiber length equal to
   // optimal_fiber_length so that you start in the middle of the active+passive
   // force-length curve. Muscle_width is the width, or thickness, of the
   // muscle-tendon unit. It is the shortest allowable fiber length because if
   // the muscle-tendon length is very short, the pennation angle will be 90
   // degrees and the fibers will be vertical (assuming the tendon is horizontal).
   // When this happens, the fibers are as long as the muscle is wide.
   // If the resting tendon length is zero, then set the fiber length equal to
   // the muscle tendon length / cosine_factor, and find its force directly.

   double muscle_width = optimalFiberLength * sin(pennationAngleAtOptimal);

    if (tendonSlackLength < ROUNDOFF_ERROR) {
        tendon_length = 0.0;
        cos_factor = cos(atan(muscle_width / length));
        fiberLength = length / cos_factor;

        double activeForce = calcActiveForce(s, fiberLength / optimalFiberLength)  * aActivation;
        if (activeForce < 0.0) activeForce = 0.0;

        passiveForce = calcPassiveForce(s, fiberLength / optimalFiberLength);
        if (passiveForce < 0.0) passiveForce = 0.0;

        setPassiveForce(s, passiveForce );
        setStateVariable(s, STATE_FIBER_LENGTH_NAME, fiberLength);
        tendon_force = (activeForce + passiveForce) * maxIsometricForce * cos_factor;
        setForce(s, tendon_force);
        setTendonForce(s, tendon_force);
        return tendon_force;
   } else if (length < tendonSlackLength) {
        setPassiveForce(s, 0.0);
      setStateVariable(s, STATE_FIBER_LENGTH_NAME, muscle_width);
      _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
      setForce(s, 0.0);
      setTendonForce(s, 0.0);
      return 0.0;
   } else {
      fiberLength = optimalFiberLength;
      cos_factor = cos(calcPennation( fiberLength, optimalFiberLength,  pennationAngleAtOptimal ));  
      tendon_length = length - fiberLength * cos_factor;

      /* Check to make sure tendon is not shorter than its slack length. If it
       * is, set the length to its slack length and re-compute fiber length.
       */

      if (tendon_length < tendonSlackLength) {
         tendon_length = tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor;
         if (fiberLength < muscle_width)
           fiberLength = muscle_width;
      }

   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
        double activeForce = calcActiveForce(s, fiberLength/ optimalFiberLength) * aActivation;
      if( activeForce <  0.0) activeForce = 0.0;

      passiveForce =  calcPassiveForce(s, fiberLength / optimalFiberLength);
      if (passiveForce < 0.0) passiveForce = 0.0;

      fiber_force = (activeForce + passiveForce) * maxIsometricForce * cos_factor;

      norm_tendon_length = tendon_length / optimalFiberLength;
      tendon_force = calcTendonForce(s, norm_tendon_length) * maxIsometricForce;
        setForce(s, tendon_force);
        setTendonForce(s, tendon_force);

      old_error_force = error_force;
 
      error_force = tendon_force - fiber_force;

      if (DABS(error_force) <= ERROR_LIMIT) // muscle-tendon force found!
         break;

      if (i == 0)
         old_error_force = error_force;

      if (DSIGN(error_force) != DSIGN(old_error_force)) {
         percent = DABS(error_force) / (DABS(error_force) + DABS(old_error_force));
         tmp_fiber_length = old_fiber_length;
         old_fiber_length = fiberLength;
         fiberLength = fiberLength + percent * (tmp_fiber_length - fiberLength);
      } else {
         // Estimate the stiffnesses of the tendon and the fibers. If tendon
         // stiffness is too low, then the next length guess will overshoot
         // the equilibrium point. So we artificially raise it using the
         // normalized muscle force. (_activeForce+_passiveForce) is the
         // normalized force for the current fiber length, and we assume that
         // the equilibrium length is close to this current length. So we want
         // to get force = (_activeForce+_passiveForce) from the tendon as well.
         // We hope this will happen by setting the tendon stiffness to
         // (_activeForce+_passiveForce) times its maximum stiffness.
            double tendon_elastic_modulus = 1200.0;
            double tendon_max_stress = 32.0;

         tendon_stiffness = calcTendonForce(s, norm_tendon_length) *
                maxIsometricForce / tendonSlackLength;

         min_tendon_stiffness = (activeForce + passiveForce) *
             tendon_elastic_modulus * maxIsometricForce /
             (tendon_max_stress * tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = maxIsometricForce / optimalFiberLength *
            (calcActiveForce(s, fiberLength / optimalFiberLength)  +
            calcPassiveForce(s, fiberLength / optimalFiberLength));

         // determine how much the fiber and tendon lengths have to
         // change to make the error_force zero. But don't let the
          // length change exceed half the optimal fiber length because
          // that's too big a change to make all at once.
         length_change = fabs(error_force/(fiber_stiffness / cos_factor + tendon_stiffness));

         if (fabs(length_change / optimalFiberLength) > 0.5)
            length_change = 0.5 * optimalFiberLength;

         // now change the fiber length depending on the sign of the error
         // and the sign of the fiber stiffness (which equals the sign of
         // the slope of the muscle's force-length curve).
         old_fiber_length = fiberLength;

         if (error_force > 0.0)
             fiberLength += length_change;
         else
             fiberLength -= length_change;


      }
      cos_factor = cos(calcPennation(fiberLength, optimalFiberLength, pennationAngleAtOptimal ));
      tendon_length = length - fiberLength * cos_factor;

      // Check to make sure tendon is not shorter than its slack length. If it is,
      // set the length to its slack length and re-compute fiber length.
      if (tendon_length < tendonSlackLength) {
         tendon_length = tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor ;
      }
    }

    setPassiveForce(s, passiveForce );
    _model->getMultibodySystem().realize(s, SimTK::Stage::Position);
    setStateVariable(s, STATE_FIBER_LENGTH_NAME,  fiberLength);

//cout << "ThelenMuscle computeIsometricForce " << getName() << "  t=" << s.getTime() << " force = " << tendon_force << endl;

   return tendon_force;
}
