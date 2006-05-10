#ifndef _MuscleZajac_h_
#define _MuscleZajac_h_
// MuscleZajac.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <RD/Tools/rdTools.h>
#include <RD/Tools/Math.h>
#include <RD/Tools/PropertyDbl.h> //added
#include <RD/Tools/Storage.h>
#include <RD/Simulation/Model/Model.h>
#include <RD/Simulation/Model/Force.h>
// #include <Muscle.h>  // Not used?
#include "Actuators.h"


//=============================================================================
//=============================================================================
/**
 * A class that represents a lumped parameter muscle in series with a linear
 * tendon.  The muscle is comprised of three elements: the contractile
 * element models the force-length-velocity properties of muscle; the series
 * elastic element is in series with the contractile element an models the
 * short-range elastic response of muscle; the parallel elastic element is
 * in parallel with both the series elastic element and the contractile
 * element and models the passive elastic properties of muscle.
 *
 * For details concerning this model consult
 * Zajac, F.E. (1989).  ??.  Reviews in Sports Science ??.
 *
 * Controls (1):
 *	0) excitation
 *
 * States (2):
 * 0) activation level
 * 1) musculotendon force
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDACTUATORS_API MuscleZajac : public Force 
{
//=============================================================================
// DATA
//=============================================================================
public:
	// MUSCLE CONSTANTS
	static const double C1;
	static const double C2;
	static const double PE1;
	static const double SIG0;
	static const double AF;
	static const double AS;
	static const double MS;
	static const double MF;
	static const double CONPE1 ;


	static const double LMNORMMAX;
	static const double flcoef[][6];
	static const double fvcoef[][6];
	static const double vfcoef[][6];

protected:
	/** Excitation (control 0). */
	double _x;
	/** Activation level (state 0). */
	double _a;
	/** Actuator force (state 1).  Note that this state is a member variable
	of class Force. */ 
	/** Rise time of muscle activation in seconds. (serialized) */
	//double *_tRise;
	/** Fall time of muscle activation in seconds.  (serialized) */
	//double *_tFall;
	/** Optimal muscle force in Newtons.  (serialized) */
	//double *_fmOpt;
	/** Optimal muscle fiber length in meters. (serialized) */
	//double *_lmOpt;
	/** Optimal muscle pennation angle in degrees.  (serialized) */
	//double *_alphaOpt;
	/** Maximum muscle fiber shortening velocity in optimal fiber lengths per
	second.  (serialized) */
	//double *_vmMax;
	/** Tendon slack length in meters.  (serialized) */
	//double *_ltSlack;
	// INITIALIZED VARIABLES
	/** Reciprocal of muscle length. */
	double _lmOptRecip;
	/** Muscle width in meters. */
	double _width;
	/** Muscle width squared. */
	double _widthSquared;
	/** Reciprocal of normalized tendon stiffness. */
	double _ktRecip;
	/** Reciprocal of normalized tendon stiffness times optimal fiber length. */
	double _ktLmOptRecip;

	// WORK VARIABLES
	/** Actuator length. */
	double _lmt;
	
	// ILSE ADDED CLAY would define them locally
	/** length of contractile part */
	double _lm;
	//PROPERTIES - previous serialized protected data
   PropertyDbl _proptRise;
   PropertyDbl _proptFall;
   PropertyDbl _propfmOpt;
	PropertyDbl _proplmOpt;
	PropertyDbl _proplmRecipOpt;
	PropertyDbl _propalphaOpt;
	PropertyDbl _propvmMax;
	PropertyDbl _propltSlack;

// REFERENCES
	double &_tRise;
	double &_tFall;
	double &_fmOpt;
	double &_lmOpt;
	double &_lmRecipOpt;
	double &_alphaOpt;
   double &_vmMax;
   double &_ltSlack;
	//=============================================================================
// METHODS
//=============================================================================
public:
	MuscleZajac(int aQID=-1,int aNX=1,int aNY=2,int aNYP=0);
	MuscleZajac(DOMElement *aElement,
		int aNX=1,int aNY=2,int aNYP=0);
	MuscleZajac(const MuscleZajac &aActuator);
	virtual ~MuscleZajac();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
private:
	void setNull();
	void setupSerializedMembers();
	void copyData(const MuscleZajac &aActuator);
   	void setupProperties();
public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	MuscleZajac&
		operator=(const MuscleZajac &aActuator);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setControls(const double aX[]);
	virtual void getControls(double rX[]) const;
	virtual void setStates(const double aY[]);
	virtual void getStates(double rY[]) const;
	void setRiseTime(double aTime);
	double getRiseTime() const;
	void setFallTime(double aTime);
	double getFallTime() const;
	void setOptimalForce(double aForce);
	double getOptimalForce() const;
	void setOptimalFiberLength(double aLength);
	double getOptimalFiberLength() const;
	void setOptimalPennationAngle(double aAngle);
	double getOptimalPennationAngle() const;
	void setTendonSlackLength(double aLength);
	double getTendonSlackLength() const;
	void setMaxShorteningVelocity(double aVelocity);
	double getMaxShorteningVelocity() const;
	void setMslMass(double aMass);
	double getMslMass() const;
	void setShorteningVelocity(double aSpeed);
	void setActuatorLength(double aLength);	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void promoteControlsToStates(const double aX[],double aDT);
	virtual void computeActuation();
	virtual void computeStateDerivatives(double rDYDT[]);
	virtual double computeDFDT();
	virtual double computeDADT(double aTRise,double aTFall,double aX,double 	aA);
	virtual double ComputeMuscleForce(double act,double alength,double avelocity);
	virtual double ComputeActivation(double aForce, double aLength, double asvel);
			// Not sure about the argument list
	
        virtual double ComputeMsForce(double activ,  double aLength, double actsv);
	virtual double ComputeMuscleZero(double activ, double &rLength, double actlen, double actsv);
	virtual double ComputeMuscleLength(double activ, double ax, double bx, double tol,double actlen, double actsv );

	//Why change to static//
	
	static double ComputeForceLengthCurve(double aLmNorm);
	static double ComputeShorteningVelocity(double aFmNormIso,double aFmNorm);
	
	static double ComputeVelocityEffect(double aVNorm);



	// To be included
	//ComputeACtivationRate
//=============================================================================
};	// END of class MuscleZajac

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MuscleZajac_h__
