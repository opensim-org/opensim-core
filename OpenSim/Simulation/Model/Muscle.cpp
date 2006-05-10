// Muscle.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <math.h>
#include "Muscle.h"


//=============================================================================
// EXPORTED CONSTANTS
//=============================================================================


//=============================================================================
// INTERNAL CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================



//=============================================================================
// ACTUATOR DYNAMICS
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Estimate an new activation level given an initial activation level,
 * an excitation level, and a time interval.
 * The assumptions are that the excitation is constant over the interval and
 * that activation dynamics is represented as a pure exponential.
 * The equation for activation is
 *
 * 	at = x - (x-a0)*exp[-dt/tau]
 *
 * @param aTRise Activation rise time constant.
 * @param aTFall Activation fall time constant.
 * @param aA0 Starting value of activation.
 * @param aX Excitation value.
 * @param aDT Time interval over which activation is to change.
 * @return Estimated activation level.
 */
double Muscle::
EstimateActivation(double aTRise,double aTFall,double aA0,double aX,double aDT)
{
	// ERROR
	if(aDT<=0) {
		return(aA0);
	}

	// CHECK at==aA0
	if(aX==aA0) return(aA0);

	// DETERMINE TIME CONSTANT
	double tau;
	if(aX>=aA0) {
		tau = aTRise;
	} else {
		tau = aTFall;
	}

	// CHECK FOR ZERO TAU
	if(tau<=0) {
		printf("Muscle.EstimateActivation: ERROR- tau<=0.0\n");
		return(aA0);
	}

	// EXPONENTIAL
	double T = exp(-aDT/tau);

	// COMPUTE EXCITATION
	double a = aX - (aX-aA0)*T;

	return(a);
}
//_____________________________________________________________________________
/**
 * Invert the equation for activation dynamics in order to compute an
 * excitation value which will produce a given change in activation
 * over a given time interval.
 * The assumptions are that the excitation is constant over the interval and
 * that activation dynamics is represented as a pure exponential.
 * The equation which is inverted is
 *
 * 	at = x - (x-a0)*exp[-dt/tau]
 *
 * Parameters:
 * @param aTRise The rise time constant.
 * @param aTFall The fall time constant.
 * @param aA0 The starting value of activation
 * @param aA The final desired value of activation.
 * @param aDT The time interval over which a is to change.
 * @return Excitation that will achieve the desired activation.
 */
double Muscle::
InvertActivation(double aTRise,double aTFall,double aA0,double aA,double aDT)
{
	// ERROR
	if(aDT<=0) {
		printf("Muscle.invertActivation: ERROR- aDT<=0.0\n");
		return(aA);
	}

	// CHECK at==aA0
	if(aA==aA0) return(aA);

	// DETERMINE TIME CONSTANT
	double tau;
	if(aA>=aA0) {
		tau = aTRise;
	} else {
		tau = aTFall;
	}

	// CHECK FOR ZERO TAU
	if(tau<=0) {
		printf("Muscle.invertActivation: ERROR- tau<=0.0\n");
		return(aA);
	}

	// EXPONENTIAL
	double T = exp(-aDT/tau);

	// COMPUTE EXCITATION
	double x = (aA - aA0*T)/(1.0-T);

	return(x);
}
//_____________________________________________________________________________
/**
 * Compute the time derivative of an activation level given its excitation
 * signal, a rise-time, and a fall-time.
 * This method represents the rise or fall using a simple 1st order
 * differential equation which is linear in x and a.  The time constant is
 * chosen based on whether x is greater than or less than a.
 * 
 */
double Muscle::
DADT(double aTRise,double aTFall,double aX,double aA)
{
	// DETERMINE TIME CONSTANT
	double tau;
	if(aX>=aA) {
		tau = aTRise;
	} else {
		tau = aTFall;
	}

	// CHECK FOR ZERO TAU
	if(tau<=0) {
		printf("Muscle.dadt: ERROR- tau<=0.0\n");
		return(aX-aA);
	}

	// COMPUTE DERIVATIVE
	double dadt = (aX-aA)/tau;

	return(dadt);
}
//_____________________________________________________________________________
/**
 * Compute the time derivative of an activation level given its excitation
 * signal, a rise-time, and a fall-time.
 * This method represents the rise and fall using a 1st order differential
 * equation which is non-linear in x.  The advantange of this method is that
 * a single equation is used.  However, the equation is only valid if tFall
 * is mutch greater than tRise.
 */
double Muscle::
DADTNonlinear(double aTRise,double aTFall,double aX,double aA)
{
	double dadt = (aX*aX-aX*aA)/aTRise  + (aX-aA)/aTFall;
	return(dadt);
}




//=============================================================================
// MUSCLE MECHANICS AND DYNAMICS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the force in an actuator given its maximum force and activation
 * state.
 */
double Muscle::
f(double aFMax,double aA)
{
	double f = aA*aFMax;
	return(f);
}
