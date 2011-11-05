#ifndef __Thelen2003MuscleV1_h__
#define __Thelen2003MuscleV1_h__

// Thelen2003MuscleV1.h
/*	Author: Matthew Millard
 *	Copyright (c)  2011, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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
	NaturalCubicSpline *_ncsfal;
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
#ifndef SWIG

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

#endif
	OPENSIM_DECLARE_DERIVED(Thelen2003MuscleV1, ActivationFiberLengthMuscle);

private:
	void setNull();
	void setupProperties();
	void setStandardMuscleCurves(); //MM
	NaturalCubicSpline* get1DSpline(const std::string &aFileName); //MM
	double get1DSplineValue(const NaturalCubicSpline *aSpline, double xval); //MM 
//=============================================================================
};	// END of class Thelen2003MuscleV1
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Thelen2003MuscleV1_h__
