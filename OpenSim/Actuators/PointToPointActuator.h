#ifndef _PointToPointActuator_h_
#define _PointToPointActuator_h_
// PointToPointActuator.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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
 * Author: Matt DeMers
 */

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "SimTKsimbody.h"


//=============================================================================
//=============================================================================
/**
 * A class that implements a force actuator acting between two points on two bodies.
 * The direction of the force is along the line between the points, with a positive
 * value acting to exapnd the distance between them.  This actuator has no states; 
 * the control is simply the force to be applied to the model.
 *
 * @author Matt DeMers
 * @version 2.0
 */
namespace OpenSim { 

class Body;
class Model;

class OSIMACTUATORS_API PointToPointActuator : public Actuator
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Corresponding Body to which the force actuator is applied. */
    Body *_bodyA;

	/** Corresponding Body to which the equal and force torque is applied. */
    Body *_bodyB;

	// INTERNAL WORKING VARIABLES

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PointToPointActuator( std::string aBodyNameA="", std::string abodyNameB="");
	PointToPointActuator( const PointToPointActuator &aPointToPointActuator);
	virtual ~PointToPointActuator();
	virtual Object* copy() const;
	void copyData(const PointToPointActuator &aPointToPointActuator);
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	PointToPointActuator& operator=(const PointToPointActuator &aGenForce);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// GENERALIZED Body
	void setBodyA(Body* aBody);
	void setBodyB(Body* aBody);
	Body* getBodyA() const;
	Body* getBodyB() const;

	// Force points of application
	void setPointA(SimTK::Vec3 aPosition) { setPropertyValue("pointA", aPosition); } ;
	SimTK::Vec3 getPointA() const { return getPropertyValue<SimTK::Vec3>("pointA"); };
	void setPointB(SimTK::Vec3 aPosition) { setPropertyValue("pointB", aPosition); } ;
	SimTK::Vec3 getPointB() const { return getPropertyValue<SimTK::Vec3>("pointB"); };

	// flag for reference frame
	void setPointsAreGlobal(bool aBool) {setPropertyValue("points_are_global", aBool); };
	bool getPointsAreGlobal() {return getPropertyValue<bool>("points_are_global"); };

	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;
	// STRESS
#ifndef SWIG
	double getStress( const SimTK::State& s ) const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce( const SimTK::State& state, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	
	virtual double  computeActuation( const SimTK::State& s) const;

#endif
	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;

	// Setup method to initialize Body reference
	void setup(Model& aModel);
	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	OPENSIM_DECLARE_DERIVED(PointToPointActuator, Actuator);

//=============================================================================
};	// END of class PointToPointActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PointToPointActuator_h__


