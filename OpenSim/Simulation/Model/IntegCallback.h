#ifndef _IntegCallback_h_
#define _IntegCallback_h_
// IntegCallback.h
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

#include <OpenSim/Common/PropertyInt.h>
#include "Callback.h"


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying an integration callback.
 *
 * Integration callbacks are registered with a model and provide a
 * set of methods that the model calls at various stages of an integration.
 * The methods are begin(), which is called at the beginning of an
 * integration, step(), which is called after each successful integration
 * step, and end(), which is called at the completion of an integration.
 * These methods provide low-level access for performing analysis,
 * animating simulations, etc.
 *
 * On a final note, it is possible to register many integration callbacks
 * with a model and no attempt is made to ensure that the
 * the actions of registered callbacks are compatible.  Ensuring
 * compatibility is left to the user.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Model;

class OSIMSIMULATION_API IntegCallback : public Callback
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Step interval. */
	PropertyInt _stepIntervalProp;
	int &_stepInterval;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~IntegCallback();
	IntegCallback(Model *aModel=0);
	IntegCallback(const std::string &aFileName, bool aUpdateFromXMLNode = true);
	IntegCallback(const IntegCallback &aIntegCallback);
	virtual Object* copy() const;
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	IntegCallback& operator=(const IntegCallback &aIntegCallback);
#endif

private:
	void setNull();
	void setupProperties();

public:
	virtual bool proceed(int aStep=0);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setStepInterval(int aStepInterval);
	int getStepInterval() const;

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);

//=============================================================================
};// END of class IntegCallback
}; //namespace
//=============================================================================
//=============================================================================

#endif // __IntegCallback_h__


