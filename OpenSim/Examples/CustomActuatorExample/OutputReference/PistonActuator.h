#ifndef _PistonActuator_h_
#define _PistonActuator_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PistonActuator.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/*
 * Author: Matt DeMers
 */

#include <OpenSim/OpenSim.h>
/*
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Simulation/Model/CustomActuator.h>
#include "SimTKsimbody.h"
*/


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

class PistonActuator : public CustomActuator
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Name of Body to which the Body actuator is applied. */
	PropertyStr _propBodyNameA;

	/** Name of Body to which the equal and opposite torque is applied. */
	PropertyStr _propBodyNameB;

	/** Point of application on each body*/
	PropertyDblVec3 _propPointA;
	PropertyDblVec3 _propPointB;
	
	/** bool to indicate whether or not the points are expressed in global frame*/
	PropertyBool _propPointsAreGlobal;

	/** Optimal force. */
	PropertyDbl _propOptimalForce;

	// REFERENCES
	std::string& _bodyNameA;
	std::string& _bodyNameB;

	/** force points of application:  assumed to be expressed in the frame
	 *  of _bodyA  and _bodyB unless _pointsAreGlobal is true.  If _pointsAreGlobal is
	 *  true, _pointA and _pointB are assumed to be expressed in the ground body */
	SimTK::Vec3 &_pointA;
	SimTK::Vec3 &_pointB;
	bool &_pointsAreGlobal;
	
	/** Optimal force*/
	double &_optimalForce;

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
	PistonActuator( std::string aBodyNameA="", std::string abodyNameB="");
	PistonActuator( const PistonActuator &aPistonActuator);
	virtual ~PistonActuator();
	virtual Object* copy() const;
	void copyData(const PistonActuator &aPistonActuator);
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	PistonActuator& operator=(const PistonActuator &aGenForce);
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
	void setPointA(SimTK::Vec3 aPosition) { _pointA = aPosition; } ;
	SimTK::Vec3 getPointA() const { return _pointA; };
	void setPointB(SimTK::Vec3 aPosition) { _pointB = aPosition; } ;
	SimTK::Vec3 getPointB() const { return _pointB; };

	// flag for reference frame
	void setPointsAreGlobal(bool aBool) {_pointsAreGlobal = aBool; };
	bool getPointsAreGlobal() {return _pointsAreGlobal; };

	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;
	// STRESS
#ifndef SWIG
	double getStress( const SimTK::State& s ) const;

    // SIMTK STATE CACHE 
    virtual void initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model);
	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

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
	virtual void updateFromXMLNode();

	OPENSIM_DECLARE_DERIVED(PistonActuator, Actuator);

//=============================================================================
};	// END of class PistonActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PistonActuator_h__
