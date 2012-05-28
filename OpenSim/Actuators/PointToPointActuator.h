#ifndef OPENSIM_POINT_TO_POINT_ACTUATOR_H_
#define OPENSIM_POINT_TO_POINT_ACTUATOR_H_
// PointToPointActuator.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009-12, Stanford University. All rights reserved. 
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

#include "Simbody.h"

namespace OpenSim { 

class Body;
class Model;

//=============================================================================
//                     POINT TO POINT ACTUATOR
//=============================================================================
/**
 * A class that implements a force actuator acting between two points on two bodies.
 * The direction of the force is along the line between the points, with a positive
 * value acting to exapnd the distance between them.  This actuator has no states; 
 * the control is simply the force to be applied to the model.
 *
 * @author Matt DeMers
 */
class OSIMACTUATORS_API PointToPointActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(PointToPointActuator, Actuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyA, std::string,
		"Name of Body to which the point-to-point actuator is applied.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyB, std::string,
		"Name of Body to which the equal and opposite force is applied.");
    /** The default is points_are_global=false. **/
    OpenSim_DECLARE_PROPERTY(points_are_global, bool, 
        "Interpret points in Ground frame if true; otherwise, corresponding "
        "body's frame.");
    /** The default location for pointA is bodyA's origin. **/
	OpenSim_DECLARE_PROPERTY(pointA, SimTK::Vec3,
        "Point of application on body A.");
    /** The default location for pointB is bodyB's origin. **/
	OpenSim_DECLARE_PROPERTY(pointB, SimTK::Vec3,
        "Point of application on body B.");
    /** The default for optimal force is 1. **/
	OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum force produced by this actuator when fully activated.");
	/**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor leaves body names unspecified. **/
    PointToPointActuator();
	/** Construct with specified body names. **/
	PointToPointActuator(const std::string& bodyNameA, 
                         const std::string& bodyNameB);
	
    /** Set the 'pointA' property to the supplied value; frame is interpreted
    according to the 'points_are_global' property. **/
	void setPointA(const SimTK::Vec3& pointAPos) 
    {   set_pointA(pointAPos); } ;
    /** Return the current value of the 'pointA' property. **/
	const SimTK::Vec3& getPointA() const 
    {   return get_pointA(); };
    /** Set the 'pointB' property to the supplied value; frame is interpreted
    according to the 'points_are_global' property. **/
	void setPointB(const SimTK::Vec3& pointBPos) 
    {   set_pointB(pointBPos); } ;
    /** Return the current value of the 'pointB' property. **/
	const SimTK::Vec3& getPointB() const 
    {   return get_pointB(); };

    /** Set the 'points_are_global' property that determines how to interpret
    the 'pointA' and 'pointB' location vectors: if not global (Ground frame) 
    then they are in the local frame of 'bodyA' and 'bodyB' respectively. **/
	void setPointsAreGlobal(bool isGlobal) 
    {   set_points_are_global(isGlobal); };
    /** Return the current value of the 'points_are_global' property. **/
	bool getPointsAreGlobal() const
    {   return get_points_are_global(); };

	/** Set the 'optimal_force' property. **/
	void setOptimalForce(double optimalForce)
    {   set_optimal_force(optimalForce); }
    /** Get the current value of the 'optimal_force' property. **/
	double getOptimalForce() const OVERRIDE_11 // Part of Actuator interface.
    {   return get_optimal_force(); }

    // default destructor, copy constructor, copy assignment

private:
	void constructProperties();

	// Set the bodies to which this actuator applies; setting these pointers
    // also sets the corresponding body name properties.
	void setBodyA(Body* bodyp);
	void setBodyB(Body* bodyp);
    Body* getBodyA() const {return _bodyA;}
    Body* getBodyB() const {return _bodyB;}    

	//--------------------------------------------------------------------------
	// Implement Force interface
	//--------------------------------------------------------------------------
	void computeForce(const SimTK::State& state, 
					  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
					  SimTK::Vector& mobilityForces) const OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement Actuator interface (also see getOptimalForce() above)
	//--------------------------------------------------------------------------	
	double computeActuation( const SimTK::State& s) const OVERRIDE_11;
	// Return the stress, defined as abs(force/optimal_force).
	double getStress( const SimTK::State& s ) const OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement ModelComponent interface
	//--------------------------------------------------------------------------
	// Setup method initializes Body reference pointers to match the names.
	void setup(Model& aModel) OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement Object interface.
	//--------------------------------------------------------------------------
	void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        OVERRIDE_11;
//=============================================================================
// DATA
//=============================================================================
    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

	// The bodies on which this point-to-point actuator acts.
    SimTK::ReferencePtr<Body> _bodyA, _bodyB;

//=============================================================================
};	// END of class PointToPointActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PointToPointActuator_h__


