#ifndef _SampleIntegrand_h_
#define _SampleIntegrand_h_
// SampleIntegrand.h
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
