#ifndef _LinearSpring_h_
#define _LinearSpring_h_
// LinearSpring.h
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
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/VectorFunction.h>
#include "CustomForce.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external forces during a
 * simulation.
 *
 * @author Frank C. Anderson, Saryn R. Goldberg, Ajay Seth
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class Body;

class OSIMSIMULATION_API LinearSpring : public CustomForce 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** VectorFunction containing points of force application (t,x,y,z). */
	VectorFunction* _pointFunction;
	/** VectorFunction containing force to be applied (t,x,y,z). */
	VectorFunction* _forceFunction;
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

	const Body &_body;

	/** Start time for the force. */
	double _startTime;
	/** End time for the force. */
	double _endTime;

//=============================================================================
// METHODS
//=============================================================================
public:
	LinearSpring(const Body &aBody, double start, double stop);	
	virtual ~LinearSpring();
	virtual Object* copy() const {throw Exception("LinearSping::copy() not implmented."); };

protected:
	virtual void setup(Model& aModel) {CustomForce::setup(aModel);} ;
	virtual void initState(SimTK::State& state) const {};

private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setTargetPosition(VectorFunction* aTargetVelocity);
	VectorFunction* getTargetPosition() const;
	void setPointFunction(VectorFunction* aPointFunction);
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
	void computePointAndTargetFunctions(SimTK::State &s, const Storage &aQStore,const Storage &aUStore,VectorFunction &aPGlobal);
	void computePointFunction(SimTK::State &s, const Storage &aQStore,const Storage &aUStore,VectorFunction &aPGlobal);
	void computeTargetFunctions(SimTK::State &s, const Storage &aQStoreForTarget,const Storage &aUStoreForTarget);

	//--------------------------------------------------------------------------
	// FORCE
	//--------------------------------------------------------------------------
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

//=============================================================================
};	// END of class LinearSpring

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __LinearSpring_h__
