#ifndef OPENSIM_SPRING_GENERALIZED_FORCE_H_
#define OPENSIM_SPRING_GENERALIZED_FORCE_H_
// SpringGeneralizedForce.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
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

//==============================================================================
// INCLUDES
//==============================================================================
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

//==============================================================================
//==============================================================================
/**
 * An force that exerts a generalized force based on spring-like
 * characteristics (stiffness and viscosity).  
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 2.0
 */
namespace OpenSim {

class Coordinate;

class OSIMACTUATORS_API SpringGeneralizedForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(SpringGeneralizedForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_OPTIONAL_PROPERTY(coordinate, std::string,
		"Name of the coordinate to which this force is applied.");
	OpenSim_DECLARE_PROPERTY(stiffness, double,
		"Spring stiffness.");
	OpenSim_DECLARE_PROPERTY(rest_length, double,
		"Coordinate value at which spring produces no force.");
	OpenSim_DECLARE_PROPERTY(viscosity, double,
		"Damping constant.");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** This serves as default constructor or you can specify the coordinate
    name. A name of "" is treated as though unspecified. **/
	SpringGeneralizedForce(const std::string& coordinateName="");

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	// STIFFNESS
	void setStiffness(double aStiffness);
	double getStiffness() const;
	// REST LENGTH
	void setRestLength(double aRestLength);
	double getRestLength() const;
	// VISCOSITY
	void setViscosity(double aViscosity);
	double getViscosity() const;
	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
protected:
    // Force interface.
	void computeForce(  const SimTK::State& state, 
					    SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
					    SimTK::Vector& mobilityForces) const OVERRIDE_11;
	
    // ModelComponent interface.
    void createSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	// Setup method to initialize coordinate reference
	void setup(Model& model) OVERRIDE_11;

private:
	void setNull();
	void constructProperties();
	double computeForceMagnitude(const SimTK::State& s) const;

	// Set the Coordinate pointer, and set the corresponding name property
    // to match.
	void setCoordinate(Coordinate* coordinate);
	Coordinate* getCoordinate() const;

    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

    // Corresponding generalized coordinate to which the coordinate actuator
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;

	//==============================================================================
};	// END of class SpringGeneralizedForce

}; //namespace
//==============================================================================
//==============================================================================


#endif // OPENSIM_SPRING_GENERALIZED_FORCE_H_
