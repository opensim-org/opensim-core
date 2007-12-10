// IntegRKF.cpp
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Exception.h>
#include "IntegRKF.h"


//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
std::string IntegRKF::_displayName = "RungeKuttaFeldberg56";


using namespace std;


//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructs an IntegRKF based on the specified model and integration
 * tolerance.
 */
IntegRKF::IntegRKF(Integrand *aIntegrand,double aTol,double aTolFine)
	: RKF(aIntegrand,aTol,aTolFine), _tArray(0.0), _dtArray(0.0)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
IntegRKF::~IntegRKF()
{
}


//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructs an IntegRKF based on the specified model and integration
 * tolerance.
 */
void IntegRKF::
setNull()
{
	_status = RKF_NORMAL;
	_steps = 0;
	_trys = 0;
	_maxSteps = 10000;
	_halt = false;
	_dtMax = 1.0;
	_dtMin = 1.0e-8;
	_specifiedDT = false;
	_constantDT = false;
	_dt = 1.0e-4;
	_tArray.setSize(0);
	_dtArray.setSize(0);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MINIMUM DT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum integration step size.
 */
void IntegRKF::
setMinDT(double aMin)
{
	_dtMin = aMin;
	if(_dtMin<0.0) _dtMin = 0.0;
}
//_____________________________________________________________________________
/**
 * Get the maximum number of integration steps.
 */
double IntegRKF::
getMinDT()
{
	return(_dtMin);
}

//-----------------------------------------------------------------------------
// MAXIMUM DT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum integration step size.
 */
void IntegRKF::
setMaxDT(double aMax)
{
	_dtMax = aMax;
	if(_dtMax<_dtMin) _dtMax = _dtMin;
}
//_____________________________________________________________________________
/**
 * Get the maximum number of integration steps.
 */
double IntegRKF::
getMaxDT()
{
	return(_dtMax);
}

//-----------------------------------------------------------------------------
// MAXIMUM NUMBER OF STEPS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum number of integration steps.
 */
void IntegRKF::
setMaximumNumberOfSteps(int aMaxSteps)
{
	if(aMaxSteps < 2) {
		_maxSteps = 2;
	} else {
		_maxSteps = aMaxSteps;
	}
}
//_____________________________________________________________________________
/**
 * Get the maximum number of integration steps.
 */
int IntegRKF::
getMaximumNumberOfSteps()
{
	return(_maxSteps);
}

//-----------------------------------------------------------------------------
// STATUS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the status of the integrator.
 */
int IntegRKF::
getStatus()
{
	return(_status);
}

//-----------------------------------------------------------------------------
// SPECIFIED DT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to take a specified sequence of deltas during an
 * integration. The time deltas are obtained from what's stored in the
 * vector dt vector (@see setDTVector()).  In order to execute an
 * integration in this manner, the sum of the deltas must cover any
 * requested integration interval.  If not, an exception will be thrown
 * at the beginning of an integration.
 *
 * @param aTrueFalse If true, a specified dt's will be used.
 * If set to false, a variable-step integration or a constant step integration
 * will be used.
 * When set to true, the flag used to indicate whether or not a constant
 * time step is used is set to false.
 *
 * @see setDTVector()
 * @see getUseConstantDT()
 */
void IntegRKF::
setUseSpecifiedDT(bool aTrueFalse)
{
	_specifiedDT = aTrueFalse;
	if(_specifiedDT==true) _constantDT = false;
}
//_____________________________________________________________________________
/**
 * Get whether or not to take a specified sequence of deltas during an
 * integration.
 * The time deltas are obtained from what's stored in the dt vector
 * (@see setDTVector()).  In order to execute an
 * integration in this manner, the sum of the deltas must cover any
 * requested integration interval.  If not, an exception will be thrown
 * at the beginning of an integration.
 *
 * @return If true, a specified time step will be used if possible.
 * If false, a variable-step integration will be performed or a constant
 * time step will be taken.
 *
 * @see getUseConstantDT()
 * @see getDT()
 * @see getTimeVector()
 */
bool IntegRKF::
getUseSpecifiedDT() const
{
	return(_specifiedDT);
}

//-----------------------------------------------------------------------------
// CONSTANT DT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to take a constant integration time step. The size of
 * the constant integration time step can be set using setDT().
 *
 * @param aTrueFalse If true, constant time steps are used.
 * When set to true, the flag used to indicate whether or not to take
 * specified time steps is set to false.
 * If set to false, a variable-step integration or a constant integration
 * time step will be used.
 *
 * @see setDT()
 * @see setUseSpecifiedDT()
 * @see getUseSpecifiedDT();
 */
void IntegRKF::
setUseConstantDT(bool aTrueFalse)
{
	_constantDT = aTrueFalse;
	if(_constantDT==true) _specifiedDT = false;
}
//_____________________________________________________________________________
/**
 * Get whether or not to use a constant integration time step. The
 * constant integration time step can be set using setDT().
 *
 * @return If true, constant time steps are used.  If false, either specified
 * or variable time steps are used.
 *
 * @see setDT()
 * @see getUseSpecifiedDTs();
 */
bool IntegRKF::
getUseConstantDT() const
{
	return(_constantDT);
}

//-----------------------------------------------------------------------------
// DT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the fixed time step to be taken when the integrator is set to take
 * a constant time step and when appropriate time steps are not available in
 * the time vector.
 *
 * @param aDT Fixed time step.
 *
 * @see setFixed()
 * @see getTimeVector()
 */
void IntegRKF::
setDT(double aDT)
{
	_dt = aDT;
	if(_dt<_dtMin) _dt = _dtMin;
}
//_____________________________________________________________________________
/**
 * Get the fixed time step to be taken when the integrator is set to take
 * a fixed time step and when appropriate time steps are not available in
 * the time vector.
 *
 * @return Fixed time step.
 *
 * @see setFixed()
 * @see getTimeVector()
 */
double IntegRKF::
getDT() const
{
	return(_dt);
}

//-----------------------------------------------------------------------------
// DT ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the time deltas used in the last integration.
 *
 * @return Constant reference to the dt array.
 */
const Array<double>& IntegRKF::
getDTArray()
{
	return(_dtArray);
}
//_____________________________________________________________________________
/**
 * Set the deltas held in the dt array.  These deltas will be used
 * if the integrator is set to take a specified set of deltas.  In order to
 * integrate using a specified set of deltas, the sum of deltas must cover
 * the requested integration time interval, otherwise an exception will be
 * thrown at the beginning of an integration.
 *
 * Note that the time vector is reconstructed in order to check that the
 * sum of the deltas covers a requested integration interval.
 *
 * @param aN Number of deltas.
 * @param aDT Array of deltas.
 * @param aTI Initial time.  If not specified, 0.0 is assumed.
 * @see getUseSpecifiedDT()
 */
void IntegRKF::
setDTArray(int aN,const double aDT[],double aTI)
{
	if(aN<=0) return;
	if(aDT==NULL) return;

	_dtArray.setSize(0);
	_dtArray.ensureCapacity(aN);
	_tArray.setSize(0);
	_tArray.ensureCapacity(aN+1);
	int i;
	for(_tArray.append(aTI),i=0;i<aN;i++) {
		_dtArray.append(aDT[i]);
		_tArray.append(_tArray.getLast()+aDT[i]);
	}
}
//_____________________________________________________________________________
/**
 * Get the delta used for a specified integration step.
 * For step aStep, the delta returned is the delta used to go from
 * step aStep to step aStep+1.
 *
 * @param aStep Index of the desired step.
 * @return Delta.  rdMath::NAN is returned on error.
 */
double IntegRKF::
getDTArrayDT(int aStep)
{
	if((aStep<0) || (aStep>=_dtArray.getSize())) {
		printf("IntegRKF.getDTArrayDT: ERR- invalid step.\n");
		return(rdMath::NAN);
	}

	return(_dtArray[aStep]);
}
//_____________________________________________________________________________
/**
 * Print the dt array.
 */
void IntegRKF::
printDTArray(const char *aFileName)
{
	// OPEN FILE
	FILE *fp;
	if(aFileName==NULL) {
		fp = stdout;
	} else {
		fp = fopen(aFileName,"w");
		if(fp==NULL) {
			printf("IntegRKF.printDTArray: unable to print to file %s.\n",
				aFileName);
			fp = stdout;
		}
	}

	// PRINT
	int i;
	fprintf(fp,"\n\ndt vector =\n");
	for(i=0;i<_dtArray.getSize();i++) {
		fprintf(fp,"%.16lf",_dtArray[i]);
		if(fp!=stdout) fprintf(fp,"\n");
		else fprintf(fp," ");
	}
	fprintf(fp,"\n");

	// CLOSE
	if(fp!=stdout) fclose(fp);
}

//-----------------------------------------------------------------------------
// TIME ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the sequence of time steps taken in the last integration.
 */
const Array<double>& IntegRKF::
getTimeArray()
{
	return(_tArray);
}
//_____________________________________________________________________________
/**
 * Get the integration step (index) that occured prior to or at 
 * a specified time.
 *
 * @param aTime Time of the integration step.
 * @return Step that occured prior to or at aTime.  0 is returned if there
 * is no such time stored.
 */
int IntegRKF::
getTimeArrayStep(double aTime)
{
	int step = _tArray.searchBinary(aTime);
	return(step);
}
//_____________________________________________________________________________
/**
 * Get the time of a specified integration step.
 *
 * @param aStep Index of the desired step.
 * @return Time of integration step aStep.  rdMath::NAN is returned on error.
 */
double IntegRKF::
getTimeArrayTime(int aStep)
{
	if((aStep<0) || (aStep>=_tArray.getSize())) {
		printf("IntegRKF.getTimeArrayTime: ERR- invalid step.\n");
		return(rdMath::NAN);
	}

	return(_tArray[aStep]);
}
//_____________________________________________________________________________
/**
 * Print the time array.
 *
 * @param aFileName Name of the file to which to print.  If the time array
 * cannot be written to a file of the specified name, the time array is
 * wirttent to standard out.
 */
void IntegRKF::
printTimeArray(const char *aFileName)
{
	// OPEN FILE
	FILE *fp;
	if(aFileName==NULL) {
		fp = stdout;
	} else {
		fp = fopen(aFileName,"w");
		if(fp==NULL) {
			printf("IntegRKF.printTimeArray: unable to print to file %s.\n",
				aFileName);
			fp = stdout;
		}
	}

	// PRINT
	int i;
	fprintf(fp,"\n\ntime vector =\n");
	for(i=0;i<_tArray.getSize();i++) {
		fprintf(fp,"%.16lf",_tArray[i]);
		if(fp!=stdout) fprintf(fp,"\n");
		else fprintf(fp," ");
	}
	fprintf(fp,"\n");

	// CLOSE
	if(fp!=stdout) fclose(fp);
}
//_____________________________________________________________________________
/**
 * Reset the time and dt arrays so that all times after the specified time
 * and the corresponding deltas are erased.
 *
 * @param aTime Time after which to erase the entries in the time and dt
 * vectors.
 */
void IntegRKF::
resetTimeAndDTArrays(double aTime)
{
	int size = getTimeArrayStep(aTime);
	_tArray.setSize(size+1);
	_dtArray.setSize(size);
}

//_____________________________________________________________________________
/**
 * Get the name of the integrator
 *
 * @return name of integrator
 */

const std::string& IntegRKF::
toString() const
{
	return (_displayName);
}

//=============================================================================
// INTERRUPT
//=============================================================================
//_____________________________________________________________________________
/**
 * Halt an integration.
 *
 * If an integration is pending or executing, the value of the interrupt
 * flag is set to true.
 */
void IntegRKF::halt()
{
		_halt = true;
}
//_____________________________________________________________________________
/**
 * Clear the halt flag.
 *
 * The value of the interrupt flag is set to false.
 */
void IntegRKF::clearHalt()
{
	_halt = false;
}
//_____________________________________________________________________________
/**
 * Check for a halt request.
 *
 * The value of the halt flag is simply returned.
 */
bool IntegRKF::checkHalt()
{
	return(_halt);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Print steps and trys.
 */
void IntegRKF::printStepsAndTrys()
{
	cout<<"IntegRKF:  steps="<<_steps<<"  trys="<<_trys<<endl;
}


//=============================================================================
// INTEGRATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Integrate the equations of motion from an initial time to a final
 * time.  The initial values of the states must already be set for the
 * model.
 *
 * @param ti Initial time.
 * @param tf Final time.
 * @param y States.
 * @param dtFirst Size of the first time step.
 * @return true on successful completion of the integration, false otherwise.
 * Use getStatus() to querry the nature of the error.
 */
bool IntegRKF::
integrate(double ti,double tf,double *y,double dtFirst)
{
	// CLEAR ANY INTERRUPT
	// Halts must arrive during an integration.
	clearHalt();

	// INITIALIZATIONS
	_steps = _trys = 0;
	_status = RKF_NORMAL;
	double t,dt,dtPrev;
	t=ti;
	dt=dtFirst;
	if(dt>_dtMax) dt = _dtMax;
	dtPrev=dt;

	// CHECK SPECIFIED DT STEPPING
	char tmp[Object::NAME_LENGTH];
	int tArrayStep = getTimeArrayStep(ti);
	if(_specifiedDT) {
		if(_tArray.getSize()<=0) {
			strcpy(tmp,"IntegRKF.integrate: ERR- specified dt stepping not");
			strcat(tmp,"possible-- empty time array.");
			throw( Exception((const char*)tmp) );
		}
		double first = _tArray[0];
		double last = _tArray.getLast();
		if((tArrayStep<0) || (ti<first) || (tf>last)) {
			strcpy(tmp,"IntegRKF.integrate: ERR- specified dt stepping not");
			strcat(tmp,"possible-- time array does not cover the requested");
			strcat(tmp," integration interval.");
			throw(Exception((const char*)tmp));
		}
	}

	// RECORD FIRST TIME STEP
	if(!_specifiedDT) {
		resetTimeAndDTArrays(t);
		if(_tArray.getSize()<=0) {
			_tArray.append(t);
		}
	}

	// SET FIRST DT
	if(_constantDT) {
		dt = _dt;
	} else if(_specifiedDT) {
		dt = _dtArray[tArrayStep];
	}
	if((t+dt)>tf) {
		dt = tf - t;
	}


	// INITIALIZE HOOK
	_integrand->initialize(_steps,dt,ti,tf,y);


	// TIME LOOP
	bool prnt = false;
	while(t<tf) {

		if(prnt) {
			cout<<"t="<<t<<endl;
		}

		// DT
		if(_constantDT) {
			dt = _dt;
		} else if(_specifiedDT) {
			dt = _dtArray[tArrayStep];
		}
		if((t+dt)>tf) {
			dt = tf - t;
		}

		// STEP
		if(_specifiedDT||_constantDT) {
			_status = stepFixed(dt,t,y);
			//_status = stepFixed(dt,t,_x,y);
		} else {
			_status = step(dt,t,y);
			//_status = step(dt,t,_x,y);
		}
		_trys++;

		// INTERPET STATUS
		// SUCCESSFUL INTEGRATION
		if((_status==RKF_NORMAL)||(_status==RKF_FINE)) {

			// INCREMENT TIME AND STEPS
			t += dt;
			dtPrev = dt;
			if(_specifiedDT) {
				tArrayStep++;
			} else {
				_tArray.append(t);
				_dtArray.append(dtPrev);
			}
			_steps++;

			// FINE ACCURACY- DOUBLE DT
			if((_status==RKF_FINE)&&(dt<_dtMax)) {
				dt*=2.0;
				if(dt>_dtMax) dt=_dtMax;
			}

			// PROCESS-AFTER-STEP HOOK
			_integrand->processAfterStep(_steps,dt,t,y);


		// POOR ACCURACY OR NOT A NUMBER- HALVE DT
		} else if((_status==RKF_POOR)||(_status==RKF_NAN)) {
			if(_status==RKF_NAN) {
				printf("IntegRKF.integrate:  NAN error encountered.\n");
			}
			dt *= 0.5;
			if(dt<_dtMin) {
				printf("IntegRKF.integrate:  dt below minimum allowed value.\n");
				break;
			}
	
		// ERROR
		} else if(_status==RKF_ERROR) {
			printf("IntegRKF.integrate:  Error encountered.\n");
			break;

		// DEFAULT
		} else {
			printf("IntegRKF.integrate:  unrecognized return status.\n");
		}

		// CHECK FOR TOO MANY STEPS
		if(_steps>_maxSteps) {
			printf("IntegRKF.integrate:  exceded the maximum number of allowed steps.\n");
			_status = RKF_TOO_MANY_STEPS;
			break;
		}

		// CHECK FOR INTERRUPT
		if(checkHalt()) break;
	}

	// CLEAR ANY INTERRUPT
	clearHalt();


	// FINALIZE HOOK
	_integrand->finalize(_steps,t,y);


	// RETURN
	if(_status<0) return(false);
	return(true);
}



