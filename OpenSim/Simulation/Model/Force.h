#ifndef _Force_h_
#define _Force_h_
// Force.h
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/VectorFunction.h>
#include "AbstractActuator.h"


//=============================================================================
//=============================================================================
/**
 * A class that supports the application of a force between two bodies, BodyA
 * and BodyB.  This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim {

class AbstractBody;

class OSIMSIMULATION_API Force : public AbstractActuator
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** name of BodyA. */
	PropertyStr _propBodyAName;
	/** Point on BodyA expressed in the body-local frame at which the
	force is applied. */
	PropertyDblVec3 _propPointA;
	/** Unit vector expressed in the local frame of BodyA that
	specifies the direction a positive actuator force is applied to BodyA.
	(serialized) */
	PropertyDblVec3 _propUnitVectorA;
	/** name of BodyB. */
	PropertyStr _propBodyBName;
	/** Point on BodyB expressed in the body-local frame at which the
	force is applied. */
	PropertyDblVec3 _propPointB;
	/** Optimal force. */
	PropertyDbl _propOptimalForce;

	// REFERENCES
	std::string& _bodyAName;
	SimTK::Vec3 &_pA;
	SimTK::Vec3 &_uA;
	std::string& _bodyBName;
	SimTK::Vec3 &_pB;
	double &_optimalForce;

	AbstractBody *_bA;
	AbstractBody *_bB;

	/** Unit vector expressed in the local frame of BodyB that
	specifies the direction a positive actuator force is applied to BodyB. */
	SimTK::Vec3 _uB;
	/** Pointer to a vector function that contains coordinates of point on
	BodyA (express in the body-local frame) where force should be applied
	as a function of time. */
	VectorFunction *_pAFunction;
	/** Pointer to a vector function that contains coordinates of point on
	BodyB (express in the body-local frame) where force should be applied
	as a function of time. */
	VectorFunction *_pBFunction;
	/** Function containing values for the time-dependent force scaling factor. */
	Function* _scaleFunction;
	/** Scale factor that pre-multiplies the applied force */
	double _scaleFactor;

	/** Excitation (control 0). */
	double _excitation;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Force(const std::string &aBodyAName="",const std::string &aBodyBName="");
	Force(const Force &aForce);
	virtual ~Force();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Force& operator=(const Force &aForce);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// BODY A
	void setBodyA(AbstractBody* aBody);
	AbstractBody* getBodyA() const;
	// POINT A
	void setPointA(const SimTK::Vec3& aPoint);
	void getPointA(SimTK::Vec3& rPoint) const;
	// DIRECTION A
	void setForceDirectionA(const SimTK::Vec3& aDirection);
	void getForceDirectionA(SimTK::Vec3& rDirection) const;
	// BODY B
	void setBodyB(AbstractBody* aBody);
	AbstractBody* getBodyB() const;
	// POINT B
	void setPointB(const SimTK::Vec3& aPoint);
	void getPointB(SimTK::Vec3& rPoint) const;
	// DIRECTION B
	void getForceDirectionB(SimTK::Vec3& rDirection) const;
	// POINT A FUNCTION
	void setPointAFunction(VectorFunction* aVectorFunction);
	const VectorFunction* getPointAFunction() const;
	// POINT B FUNCTION
	void setPointBFunction(VectorFunction* aVectorFunction);
	const VectorFunction* getPointBFunction() const;
	// SCALE FACTOR
	void setScaleFunction(Function* _scaleFunction);
	Function* getScaleFunction() const;
	void setScaleFactor(double aScaleFactor);
	double getScaleFactor();
	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;
	// STRESS
	double getStress() const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void apply();

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeActuation();
	void computeForceDirectionForBodyB();
	void computeSpeed();
	void updatePointA();
	void updatePointB();

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;
	// Setup method to initialize Body references
	void setup(Model* aModel);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void computeLineOfAction(SimTK::Vec3& aLineOfAction) const;

	OPENSIM_DECLARE_DERIVED(Force, AbstractActuator);

//=============================================================================
};	// END of class Force

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Force_h__


