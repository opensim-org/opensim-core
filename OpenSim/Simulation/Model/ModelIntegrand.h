#ifndef _ModelIntegrand_h_
#define _ModelIntegrand_h_
// ModelIntegrand.h
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
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/Integrator/Integrand.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/Controller.h>

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * This class makes Model into a valid Integrand.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class AbstractModel;

class RDSIMULATION_API ModelIntegrand : public Integrand
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Model. */
	AbstractModel *_model;
	/** Control set. */
	bool _ownsControlSet;
	ControlSet *_controlSet;
	/** Controller. */
	Controller *_controller;
	/** Storage for the controls. */
	Storage *_controlStore;
	/** Storage for the states. */
	Storage *_stateStore;
	/** Storage for the pseudostates. */
	Storage *_pseudoStore;

	// WORK VARIABLES
	/** Initial time of the integration. */
	double _ti;
	/** Final time of the integration. */
	double _tf;
	/** Previous time. */
	double _tPrev;
	/** Size of previous time step. */
	double _dtPrev;
	/** Array of control values at the current time. */
	Array<double> _x;
	/** Array of control values at the previous integration time step. */
	Array<double> _xPrev;
	/** Array of state values at the previous integration time step. */
	Array<double> _yPrev;
	/** Array of pseudo states. */
	Array<double> _yp;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ModelIntegrand(AbstractModel *aModel);
	virtual ~ModelIntegrand();
private:
	void setNull();
public:

	//--------------------------------------------------------------------------
	// GET & SET
	//--------------------------------------------------------------------------
	// Size
	virtual int getSize() const;
	// Model
	AbstractModel* getModel();
	// Control Set
	void setControlSet(const ControlSet &aControlSet);
	void setControlSetReference(ControlSet &aControlSet);
	ControlSet* getControlSet();
	// CONTROL STORAGE
	void setControlStorage(Storage *aStorage);
	Storage* getControlStorage();
	// STATE STORAGE
	void setStateStorage(Storage *aStorage);
	Storage* getStateStorage();
	// PSEUDO-STATE STORAGE
	void setPseudoStateStorage(Storage *aStorage);
	Storage* getPseudoStateStorage();
	// CONTROLLER
	void setController(Controller *aController);
	Controller* getController();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual ControlSet* constructControlSet() const;
	virtual void
		convertStatesIntegrandToModel(double t,const double y[],double yModel[]);
	virtual void
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
};	// END class ModelIntegrand

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __ModelIntegrand_h__
