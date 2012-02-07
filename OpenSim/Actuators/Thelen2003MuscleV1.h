#ifndef __Thelen2003MuscleV1_h__
#define __Thelen2003MuscleV1_h__

// Thelen2003MuscleV1.h
/*	Author: Matthew Millard
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


// INCLUDE
#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Common/NaturalCubicSpline.h>


#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMACTUATORS_API Thelen2003MuscleV1 : public ActivationFiberLengthMuscle  
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Activation time constant */  
	PropertyDbl _activationTimeConstantProp;
	double &_activationTimeConstant;

	/** Deactivation time constant */
	PropertyDbl _deactivationTimeConstantProp;
	double &_deactivationTimeConstant;

	/** Max contraction velocity full activation in fiber lengths per second */
	PropertyDbl _vmaxProp;
	double &_vmax;

	/** Max contraction velocity at low activation */
	PropertyDbl _vmax0Prop;
	double &_vmax0;

	/** Tendon strain due to maximum isometric muscle force */
	PropertyDbl _fmaxTendonStrainProp;
	double &_fmaxTendonStrain;

	/** Passive muscle strain due to maximum isometric muscle force */
	PropertyDbl _fmaxMuscleStrainProp;
	double &_fmaxMuscleStrain;

	/** Shape factor for Gaussian active muscle force-length relationship */
	PropertyDbl _kShapeActiveProp;
	double &_kShapeActive;

	/** Exponential shape factor for passive force-length relationship */
	PropertyDbl _kShapePassiveProp;
	double &_kShapePassive;

	/** Passive damping included in the force-velocity relationship */
	PropertyDbl _dampingProp;
	double &_damping;

	/** Force-velocity shape factor */
	PropertyDbl _afProp;
	double &_af;

	/** Force length shape factor*/
	PropertyDbl _flenProp;
	double &_flen;


	//M.Millard. All of these extra variables are soley to characterize the muscle models
	//           and they are set in calcActuation
	/** d Activation dt*/
	PropertyDbl _dactProp;
	double &_dact;

	/**Activation */
	PropertyDbl _actProp;
	double &_act;

	/** Dimensionless contractile element active length scaling factor: fal */
	PropertyDbl _falProp;
	double &_fal;

	/** Dimensionless tendon force: fse */
	PropertyDbl _fseProp;
	double &_fse;

	/** Dimensionless muscle property: fpe */
	PropertyDbl _fpeProp;
	double &_fpe;

	/** Dimensionless muscle property: fv */
	PropertyDbl _fvProp;	
	double &_fv;

	/** Dimensionless muscle property fv Internal Variable */
	PropertyDbl _fvVmaxProp;
	double &_fvVmax;

	/** Dimensionless (normalized) tendon element length */
	PropertyDbl _tlProp;
	double &_tl;

	/** Dimensionless (normalized) muscle contractile element length */
	PropertyDbl _lceProp;
	double &_lce;

	/** Dimensionless (normalized) muscle property normalized contractile element velocity */
	PropertyDbl _dlceProp;
	double &_dlce;

	/** Excitation of the Muscle*/
	PropertyDbl _uProp;
	double &_u;

	/** Cos of the pennation angle*/
	PropertyDbl _caProp;
	double &_ca;

	//MM splined versions of the active force length curve
	//NaturalCubicSpline *_ncsfal;

    SimTK::Spline_<SimTK::Real> _ncsfal;
    SimTK::Spline_<SimTK::Real> _ncsfv;
	//MM flags to select splined or computed curve versions.

	//M.Millard system energy variables
	PropertyDbl _tendonPEProp;
	double &_tendonPE;

	PropertyDbl _musclePEProp;
	double &_musclePE;

	PropertyDbl _musclePWRProp;
	double &_musclePWR;

	PropertyDbl _muscleVProp;
	double &_muscleV;

	PropertyDbl _muscleFProp;
	double &_muscleF;

	PropertyBool _splineFalProp; //This is a back door flag. When set to true, it
	bool &_splineFal;			 //will replace the active force length curve with
								 //a spline interpolated version that is stored in
								 //muscle_afl.sto, which it expects in the directory
								 //that holds the setup files.
    PropertyBool _splineFvProp;      //This is a back door flag. When set to true, it
	bool &_splineFv;			     //will use only the fv part of the surface that
                                 //Thelen's equations use when a=fal=1

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Thelen2003MuscleV1();
	Thelen2003MuscleV1(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	Thelen2003MuscleV1(const Thelen2003MuscleV1 &aMuscle);
	virtual ~Thelen2003MuscleV1();
	virtual Object* copy() const;

	//MM accessor functions for all of the internal variables necessary to create
	//standard dimensionless muscle fiber plots
	double getdactdt();	//Rate change of activation
	double getact();	//activation
	double getfal();	//normalized active force length
	double getfse();	//normalized tendon force
	double getfpe();	//normalized passive muscle force
	double getfv();		//normalized fv multiplicative factor
	double getfvVmax();	//internal variable to fv & dlce - Vmax
	double gettl();		//normalized tendon length
	double getlce();	//normalized contractile element length
	double getdlce();	//normalized contractile element velocity
	double getu();		//excitation of the muscle
	double getca();		//get cosine of the pennation angle

	double getTendonPE();	//get the potential energy stored in the tendon
	double getMusclePE();	//get the potential energy stored in the muscle
	double getMusclePWR();	//get the work done by the muscle
	double getMuscleF();	//get the work done by the muscle
	double getMuscleV();	//get the work done by the muscle


    void useSplineFal();
    void useSplineFv();
    void useDefaultThelen();


#ifndef SWIG
	Thelen2003MuscleV1& operator=(const Thelen2003MuscleV1 &aMuscle);
#endif
	void copyData(const Thelen2003MuscleV1 &aMuscle);

#ifndef SWIG

#endif

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getActivationTimeConstant() const { return _activationTimeConstant; }
	virtual double getDeactivationTimeConstant() const { return _deactivationTimeConstant; }
	virtual double getVmax() const { return _vmax; }
	virtual double getVmax0() const { return _vmax0; }
	virtual double getFmaxTendonStrain() const { return _fmaxTendonStrain; }
	virtual double getFmaxMuscleStrain() const { return _fmaxMuscleStrain; }
	virtual double getKshapeActive() const { return _kShapeActive; }
	virtual double getKshapePassive() const { return _kShapePassive; }
	virtual double getDamping() const { return _damping; }
	virtual double getAf() const { return _af; }
	virtual double getFlen() const { return _flen; }
	virtual bool setActivationTimeConstant(double aActivationTimeConstant);
	virtual bool setDeactivationTimeConstant(double aDeactivationTimeConstant);
	virtual bool setVmax(double aVmax);
	virtual bool setVmax0(double aVmax0);
	virtual bool setFmaxTendonStrain(double aFmaxTendonStrain);
	virtual bool setFmaxMuscleStrain(double aFmaxMuscleStrain);
	virtual bool setKshapeActive(double aKShapeActive);
	virtual bool setKshapePassive(double aKshapePassive);
	virtual bool setDamping(double aDamping);
	virtual bool setAf(double aAf);
	virtual bool setFlen(double aFlen);



	// Computed quantities
	//--------------------------------------------------------------------------
	// FORCE-LENGTH-VELOCITY PROPERTIES
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation(const SimTK::State& s) const;
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;
	double calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcActiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcPassiveCompressiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;

	//MM Internal functions to compute the potential energy stored in the muscle-tendon unit and the work done by the fiber.
	/*double calcTendonPE(const SimTK::State& s, double aNormTendonLength, double aTendonRestLength ,double aMaxIsometricForce) const;
	double calcMusclePE(const SimTK::State& s, double aNormFiberLength, double aFiberRestLength ,double aMaxIsometricForce) const;
	double calcMuscleW(const SimTK::State& s,  double aNormFiberLength, double aFiberRestLength, double aNormFiberVelocity, double aMaxFiberVelocity, double aMaxIsometricForce) const;
	*/
	OPENSIM_DECLARE_DERIVED(Thelen2003MuscleV1, ActivationFiberLengthMuscle);

private:
	void setNull();
	void setupProperties();
	void loadForceActiveLengthCurve(); //MM
    void loadForceVelocityCurve(); //MM
	SimTK::Spline_<SimTK::Real> get1DSpline(const std::string &aFileName); //MM
	//double get1DSplineValue(const NaturalCubicSpline *aSpline, double xval); //MM 
//=============================================================================
};	// END of class Thelen2003MuscleV1
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Thelen2003MuscleV1_h__
