#ifndef _IntegRKF_h_
#define _IntegRKF_h_
// IntegRKF.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/rdMath.h>
#include "Integrand.h"
#include "RKF.h"


// DLL SPECIFICATIONS FOR TEMPLATES
//template class OSIMSIMULATION_API Array<double>;


//=============================================================================
//=============================================================================
/**
 * A class for integrating the equations of motion of a dynamic system.
 *
 * This class performs the relatively high-level tasks during an integration
 * and implements the logic for when to reduce the integration step size or
 * increase it.  This class sits on top of class RKF which implements
 * the low-level numerics for computing the derivatives and taking the states
 * one step forward in time.  RKF means Runge-Kutta-Feldberg.  See RKF
 * for details.
 *
 * The user must supply a valid pointer to an Model on construction.
 */
namespace OpenSim { 

class OSIMSIMULATION_API IntegRKF
	: public RKF
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Status of the integration. */
	int _status;
	/** Number of integration steps successfully taken. */
	int _steps;
	/** Number of integration step trys. */
	int _trys;
	/** Maximum number of steps in an integration. */
	int _maxSteps;
	/** Flag for signaling a desired halt. */
	bool _halt;
	/** Minimum step size. */
	double _dtMin;
	/** Maximum step size. */
	double _dtMax;
	/** Flag to indicate whether or not specified integration time steps
	should be used.  The specified integration time steps are held in _tVec.
	If _tVec does not contain time steps appropriate for the integration,
	an exception is thrown. */
	bool _specifiedDT;
	/** Flag to indicate whether or not constant (fixed) integration time
	steps should be used.  The constant integration time step is set using
	setDT(). */
	bool _constantDT;
	/** Constant integration time step. */
	double _dt;
	/** Vector of integration time steps. */
	Array<double> _tArray;
	/** Vector of integration time step deltas. */
	Array<double> _dtArray;
	/** Name to be shown by the UI */
	static std::string _displayName;

//=============================================================================
// METHODS
//=============================================================================
public:
	IntegRKF(Integrand *aIntegrand,
		double aTol=1.0e-4,double aTolFine=-1.0);
	virtual ~IntegRKF();
private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	//	GET AND SET
	//--------------------------------------------------------------------------
	// MIN DT
	void setMinDT(double aMin);
	double getMinDT();
	// MIN DT
	void setMaxDT(double aMax);
	double getMaxDT();
	// MAX STEPS
	void setMaximumNumberOfSteps(int aMaxSteps);
	int getMaximumNumberOfSteps();
	// STATUS
	int getStatus();
	// SEPECIFIED TIME STEP
	void setUseSpecifiedDT(bool aTrueFalse);
	bool getUseSpecifiedDT() const;
	// CONSTANT TIME STEP
	void setUseConstantDT(bool aTrueFalse);
	bool getUseConstantDT() const;
	// DT
	void setDT(double aDT);
	double getDT() const;
	// DT VECTOR
	const Array<double>& getDTArray();
	void setDTArray(int aN,const double aDT[],double aTI=0.0);
	double getDTArrayDT(int aStep);
	void printDTArray(const char *aFileName=NULL);
	// TIME VECTOR
	const Array<double>& getTimeArray();
	double getTimeArrayTime(int aStep);
	int getTimeArrayStep(double aTime);
	void printTimeArray(const char *aFileName=NULL);
	void resetTimeAndDTArrays(double aTime);
	// NAME
	const std::string& toString() const;

	//--------------------------------------------------------------------------
	//	INTEGRATION
	//--------------------------------------------------------------------------
	bool integrate(double ti,double tf,double *y,double dtFirst=1.0e-3);

	//--------------------------------------------------------------------------
	//	INTERRUPT
	//--------------------------------------------------------------------------
	void halt();
	void clearHalt();
	bool checkHalt();

	//--------------------------------------------------------------------------
	//	UTILITY
	//--------------------------------------------------------------------------
	void printStepsAndTrys();

//=============================================================================
};	// END class IntegRKF

}; //namespace
//=============================================================================
//=============================================================================


#endif // __IntegRKF_h__
