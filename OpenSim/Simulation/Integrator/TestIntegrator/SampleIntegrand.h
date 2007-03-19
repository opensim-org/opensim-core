#ifndef _SampleIntegrand_h_
#define _SampleIntegrand_h_
// SampleIntegrand.h
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
#include <OpenSim/Simulation/Integrator/Integrand.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * This class demonstrates a sample implementation of an Integrand.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class SampleIntegrand : public Integrand
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Storage for the states. */
	Storage *_stateStore;

	// WORK VARIABLES
	/** Initial time of the integration. */
	double _ti;
	/** Final time of the integration. */
	double _tf;
	/** Previous time. */
	double _tPrev;
	/** Size of previous time step. */
	double _dtPrev;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SampleIntegrand();
	virtual ~SampleIntegrand();
private:
	void setNull();
public:

	//--------------------------------------------------------------------------
	// GET & SET
	//--------------------------------------------------------------------------
	// Size
	virtual int getSize() const;
	// STATE STORAGE
	void setStateStorage(Storage *aStorage);
	Storage* getStateStorage();

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual void compute(double t,double y[],double dydt[]);
	virtual void computeJacobian(double t,double y[],double *dydtdy);

	//--------------------------------------------------------------------------
	// HOOKS
	//--------------------------------------------------------------------------
	virtual void initialize(int step,double &dt,double ti,double tf,double y[]);
	virtual void processAfterStep(int step,double &dt,double t,double y[]);

//=============================================================================

};	// END class SampleIntegrand

}; //namespace

//=============================================================================
//=============================================================================

#endif  // __SampleIntegrand_h__
