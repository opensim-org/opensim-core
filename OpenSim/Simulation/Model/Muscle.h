#ifndef _Muscle_h_
#define _Muscle_h_
// Muscle.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================

// INCLUDES
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <iostream>
#include <string>

//=============================================================================
//=============================================================================
/**
 * This class provides basic methods for activation and muscle-tendon dynamics.
 *
 */
namespace OpenSim { 

class RDSIMULATION_API Muscle 
{

//=============================================================================
// DATA
//=============================================================================
private:

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// ACTIVATION DYNAMICS
	//--------------------------------------------------------------------------
	static double
		EstimateActivation(double aTRise,double aTFall,double aA0,double aX,
		double aDT);
	static double
		InvertActivation(double aTRise,double aTFall,double aA0,double aA,
		double aDT);
	static double
		DADT(double aTRise,double aTFall,double aX,double aA);
	static double
		DADTNonlinear(double aTRise,double aTFall,double aX,double aA);

	//--------------------------------------------------------------------------
	// MUSCLE-TENDON DYNAMICS FOR AN IDEAL MUSCLE
	//--------------------------------------------------------------------------
	static double f(double aFMax,double aA);

//=============================================================================
};	// END class Muscle

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Muscle_h__
