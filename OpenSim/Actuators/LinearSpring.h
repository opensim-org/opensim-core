#ifndef _LinearSpring_h_
#define _LinearSpring_h_
// LinearSpring.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/VectorFunction.h>
#include "osimActuatorsDLL.h"
#include "ForceApplier.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external forces during a
 * simulation.
 *
 * @author Frank C. Anderson, Saryn R. Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractBody;

class OSIMACTUATORS_API LinearSpring : public ForceApplier 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Vector function containing the target position of the point expressed
	in the global reference frame. */
	VectorFunction *_targetPosition;
	/** Vector function containing the target velocity of the point expressed
	in  the global reference frame. */
	VectorFunction *_targetVelocity;
	/** Function containing values for the time-dependent scaling factor. */
	Function *_scaleFunction;
	/** Scale factor that pre-multiplies the applied torque */
	double _scaleFactor;
	/** Stiffness. */
	SimTK::Vec3 _k;
	/** Damping. */
	SimTK::Vec3 _b;
	/** If the magnitude of the spring force is below this threshold, no spring
	force is applied. */
	double _threshold;

//=============================================================================
// METHODS
//=============================================================================
public:
	LinearSpring(Model *aModel, AbstractBody *aBody);	
	virtual ~LinearSpring();
private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setTargetPosition(VectorFunction* aTargetVelocity);
	VectorFunction* getTargetPosition() const;
	void setTargetVelocity(VectorFunction* aTargetVelocity);
	VectorFunction* getTargetVelocity() const;
	void setScaleFunction(Function* _scaleFunction);
	Function* getScaleFunction() const;
	void setScaleFactor(double aScaleFactor);
	double getScaleFactor();
	void setKValue(const SimTK::Vec3& aK);
	void getKValue(SimTK::Vec3& aK) const;
	void setBValue(const SimTK::Vec3& aB);
	void getBValue(SimTK::Vec3& aB) const;
	void setThreshold(double aThreshold);
	double getThreshold() const;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void
		computePointAndTargetFunctions(const Storage &aQStore,const Storage &aUStore,
		VectorFunction &aPGlobal);
	void
		computeTargetFunctions(const Storage &aQStoreForTarget,const Storage &aUStoreForTarget);

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		applyActuation(double aT,double *aX,double *aY);

//=============================================================================
};	// END of class LinearSpring

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __LinearSpring_h__
