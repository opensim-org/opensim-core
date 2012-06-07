#ifndef OPENSIM_COORDINATE_ACTUATOR_H_
#define OPENSIM_COORDINATE_ACTUATOR_H_
// CoordinateActuator.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005-12, Stanford University. All rights reserved. 
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


#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "Simbody.h"


namespace OpenSim { 

class Coordinate;
class ForceSet;
class Model;

//==============================================================================
//                           COORDINATE ACTUATOR
//==============================================================================
/**
 * A class that supports the application of a coordinate actuator to a model.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Frank C. Anderson
 */
class OSIMACTUATORS_API CoordinateActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateActuator, Actuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_OPTIONAL_PROPERTY(coordinate, std::string,
		"Name of the generalized coordinate to which the actuator applies.");
	OpenSim_DECLARE_PROPERTY(optimal_force, double,
		"The maximum generalized force produced by this actuator.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves coordinate name unspecified, or you can
    provide it. **/
	explicit CoordinateActuator(const std::string& coordinateName="");

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

	/** Set the 'optimal_force' property. **/
	void setOptimalForce(double optimalForce);
    /** Get the current setting of the 'optimal_force' property. **/
	double getOptimalForce() const OVERRIDE_11; // part of Actuator interface

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static ForceSet* CreateForceSetOfCoordinateActuatorsForModel(const SimTK::State& s, Model& aModel,double aOptimalForce = 1,bool aIncludeLockedAndConstrainedCoordinates = true);

	bool isCoordinateValid() const;
	double getSpeed( const SimTK::State& s) const;

    /** Set the reference pointer to point to the given Coordinate and set
    the 'coordinate' name property also. **/
    void setCoordinate(Coordinate* aCoordinate);
    /** Get a pointer to the Coordinate to which this actuator refers. **/
	Coordinate* getCoordinate() const;

//==============================================================================
// PRIVATE
//==============================================================================
private:
	//--------------------------------------------------------------------------
	// Implement Force interface
	//--------------------------------------------------------------------------
	void computeForce(const SimTK::State& state, 
					  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
					  SimTK::Vector& mobilityForces) const OVERRIDE_11;


	//--------------------------------------------------------------------------
	// Implement Actuator interface (also see getOptimalForce() above)
	//--------------------------------------------------------------------------
	double  computeActuation( const SimTK::State& s) const OVERRIDE_11;
	// Return the stress, defined as abs(force/optimal_force).
	double getStress( const SimTK::State& s ) const OVERRIDE_11;


	//--------------------------------------------------------------------------
	// Implement ModelComponent interface
	//--------------------------------------------------------------------------
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement Object interface.
	//--------------------------------------------------------------------------
	void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        OVERRIDE_11;

	void setNull();
	void constructProperties();


    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

	// Corresponding generalized coordinate to which the coordinate actuator
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;
//==============================================================================
};	// END of class CoordinateActuator

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_COORDINATE_ACTUATOR_H_


