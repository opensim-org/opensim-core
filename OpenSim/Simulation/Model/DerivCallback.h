#ifndef _DerivCallback_h_
#define _DerivCallback_h_
// DerivCallback.h
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include "Callback.h"


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying a derivative callback.
 *
 * Derivative callbacks are registered with a model and provide a
 * set of methods that the model calls during computation of the
 * model state derivatives.  The methods are set(), computeContact(),
 * applyContact(), computeActuation(), applyActuation(), and
 * computeDerivatives().  These methods provide low-level access for
 * making adjustments to model states and applied forces,
 * applying additional forces, and/or altering state derivatives.
 * Each of these methods is intended to be called FOLLOWING the 
 * corresponding action in AbstractModel::deriv(), although AbstractModel::deriv()
 * can be written to call these methods in any order or not at all.
 * 
 * Derivative callbacks are meant to make alterations to the "normal" model
 * of simulation either for analysis or to achieve some unusual effect.
 * Alternately, this class can be used to implement the entirety of the
 * AbstractModel::deriv() method, giving the user nearly complete control of how
 * derivatives are computed without making changes to the model.  To
 * accomplish this, the user may need to make a new derived model class
 * that overrides AbstractModel::deriv() with a new method that makes the
 * appropriate DerivCallback method calls.
 *
 * On a final note, it is possible to register many derivative callbacks
 * with a model and no attempt is made to ensure that the
 * the actions of registered callbacks are compatible.  Ensuring
 * compatibility is left to the user.
 *
 * @author Frank C. Anderson
 * @version 1.0
 * @see AbstractModel::deriv()
 */
namespace OpenSim { 

class AbstractModel;

class OSIMSIMULATION_API DerivCallback : public Callback
{

//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	DerivCallback(AbstractModel *aModel);
	virtual ~DerivCallback();
	DerivCallback(const DerivCallback &aDerivCallback);
private:
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	DerivCallback& operator=(const DerivCallback &aObject);

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		set(double aT,double *aX,double *aY);
	virtual void
		computeContact(double aT,double *aX,double *aY);
	virtual void
		applyContact(double aT,double *aX,double *aY);
	virtual void
		computeActuation(double aT,double *aX,double *aY);
	virtual void
		applyActuation(double aT,double *aX,double *aY);
	virtual void
		computeDerivatives(double aT,double *aX,double *aY,double *aDY);
	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void reset() { };
//=============================================================================
};	// END of class DerivCallback

}; //namespace
//=============================================================================
//=============================================================================

#endif // __DerivCallback_h__


