#ifndef _ModelIntegrandForactuators_h_
#define _ModelIntegrandForactuators_h_
// ModelIntegrandForActuators.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Simulation/Integrator/Integrand.h>
#include "ModelIntegrand.h"
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/Controller.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * This class makes AbstractModel into a valid Integrand.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class AbstractModel;

class OSIMSIMULATION_API ModelIntegrandForActuators : public ModelIntegrand
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Prescribed trajectories of the generalized coordinates. */
	FunctionSet *_qSet;
	/** Prescribed trajectories of the generalized speeds. */
	FunctionSet *_uSet;
	/** Correction to the generalized coordinates. */
	Array<double> _qCorrections;
	/** Correction to the generalized speeds. */
	Array<double> _uCorrections;
	/** Flag indicating whether or not to hold the coordinates and speeds
	constant. */
	bool _holdCoordinatesConstant;
	/** Time at which to hold the coordinates constant. */
	double _holdTime;

	// WORK VARIABLES
	/** Work array for holding the generalized coordinates. */
	Array<double> _qWork;
	/** Work array for holding the generalized speeds. */
	Array<double> _uWork;
	/** Array for holding the model states (as opposed to the integrated
	states). */
	Array<double> _yModel;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ModelIntegrandForActuators(AbstractModel *aModel);
	virtual ~ModelIntegrandForActuators();
private:
	void setNull();
public:

	//--------------------------------------------------------------------------
	// GET & SET
	//--------------------------------------------------------------------------
	// Size
	virtual int getSize() const;
	// Coordinate Trajectories
	virtual void setCoordinateTrajectories(FunctionSet *aSet);
	virtual FunctionSet* getCoordinateTrajectories();
	// Speed Trajectories
	virtual void setSpeedTrajectories(FunctionSet *aSet);
	virtual FunctionSet* getSpeedTrajectories();
	// Coordinate Corrections
	virtual void setCoordinateCorrections(const double aCorrections[]);
	virtual void getCoordinateCorrections(double rCorrections[]) const;
	// Speed Corrections
	virtual void setSpeedCorrections(const double aCorrections[]);
	virtual void getSpeedCorrections(double rCorrections[]) const;

	//--------------------------------------------------------------------------
	// CONVERT STATES
	//--------------------------------------------------------------------------
	void holdCoordinatesConstant(double t);
	void releaseCoordinates();
	void virtual
		convertStatesIntegrandToModel(double t,const double y[],double yModel[]);
	void virtual
		convertStatesModelToIntegrand(const double yModel[],double y[]) const;

	//--------------------------------------------------------------------------
	// INITIAL STATES
	//--------------------------------------------------------------------------
	virtual void setInitialStates(double ti,const double yi[]);
	virtual void getInitialStates(double yi[]) const;
	
	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual void compute(double t,double y[],double dydt[]);

	//--------------------------------------------------------------------------
	// HOOKS
	//--------------------------------------------------------------------------
	virtual void initialize(int step,double &dt,double ti,double tf,double y[]);
	virtual void processAfterStep(int step,double &dt,double t,double y[]);
	virtual void finalize(int step,double t,double y[]);

//=============================================================================
};	// END class ModelIntegrandForActuators

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __ModelIntegrandForActuators_h__
