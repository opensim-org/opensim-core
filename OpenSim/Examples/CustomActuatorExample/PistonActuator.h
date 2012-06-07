#ifndef _PistonActuator_h_
#define _PistonActuator_h_
// PistonActuator.h
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

#include <OpenSim/OpenSim.h>

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

class PistonActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(PistonActuator, Actuator);

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
	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
							  SimTK::Vector& generalizedForces) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	
	virtual double  computeActuation( const SimTK::State& s) const;

#endif
	// Setup method to initialize Body reference
	void connectToModel(Model& aModel) OVERRIDE_11;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

//=============================================================================
};	// END of class PistonActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PistonActuator_h__
