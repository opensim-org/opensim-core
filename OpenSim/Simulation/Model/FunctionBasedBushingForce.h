#ifndef OPENSIM_FUNCTION_BASED_BUSHING_FORCE_H_
#define OPENSIM_FUNCTION_BASED_BUSHING_FORCE_H_

// FunctionBasedBushingForce.h
// Author: Matt DeMers
/*
 * Copyright (c) 2010, Stanford University. All rights reserved. 
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


// INCLUDE
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include "Force.h"
#include <OpenSim/Common/osimCommon.h>

namespace OpenSim {

//==============================================================================
//                             FUNCTION BASED BUSHING FORCE
//==============================================================================
/**
 * A class implementing a Bushing Force driven by functions relating forces to 
 * deviations.
 * A Bushing Force is the force increasing due to deviation between two frames. 
 * One can think of the Bushing as being composed of 3 linear and 3 torsional
 * spring-dampers, which act along or about the bushing frames. Orientations
 * are measured as x-y-z body-fixed Euler rotations, which are treated as
 * though they were uncoupled. That makes this bushing model suitable only for
 * relatively small relative orientation between the frames.
 * The underlying Force in Simbody is a SimtK::Force::LinearBushing.
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API FunctionBasedBushingForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(FunctionBasedBushingForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body_1, std::string,
		"One of the two bodies connected by the bushing.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body_2, std::string,
		"The other of the two bodies connected by the bushing.");
	OpenSim_DECLARE_PROPERTY(location_body_1, SimTK::Vec3,
		"Location of bushing frame on body 1.");
	OpenSim_DECLARE_PROPERTY(orientation_body_1, SimTK::Vec3,
		"Orientation of bushing frame in body 1 as x-y-z, body fixed Euler rotations.");
	OpenSim_DECLARE_PROPERTY(location_body_2, SimTK::Vec3,
		"Location of bushing frame on body 2.");
	OpenSim_DECLARE_PROPERTY(orientation_body_2, SimTK::Vec3,
		"Orientation of bushing frame in body 2 as x-y-z, body fixed Euler rotations.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(m_xx_function, Function,
        "Function defining the contribution of theta_x deflection to the moment about body_2's x axis.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(m_yy_function, Function,
        "Function defining the contribution of theta_y deflection to the moment about body_2's y axis.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(m_zz_function, Function,
        "Function defining the contribution of theta_z deflection to the moment about body_2's z axis.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(f_xx_function, Function,
        "Function defining the contribution of x deflection to the force in the x direction.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(f_yy_function, Function,
        "Function defining the contribution of y deflection to the force in the y direction.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(f_zz_function, Function,
        "Function defining the contribution of z deflection to the force in the z direction.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(moment_visual_scale, double,
        "Value multiplying the bushing moments before displaying the moment vector");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(force_visual_scale, double,
        "Value multiplying the bushing forces before displaying the force vector");
    // To do:  Delete these stiffness properties
    //--------------------------------------------------------
	OpenSim_DECLARE_PROPERTY(rotational_stiffness, SimTK::Vec3,
		"Stiffness parameters resisting relative rotation.");
	OpenSim_DECLARE_PROPERTY(translational_stiffness, SimTK::Vec3,
		"Stiffness parameters resisting relative translation.");
    //-------------------------------------------------------
	OpenSim_DECLARE_PROPERTY(rotational_damping, SimTK::Vec3,
		"Damping parameters resisting relative angular velocity.");
	OpenSim_DECLARE_PROPERTY(translational_damping, SimTK::Vec3,
		"Damping parameters resisting relative translational velocity.");
    /**@}**/

protected:
    /** how to display the bushing */
	VisibleObject _displayer;
private:
	// underlying SimTK system elements
	// the mobilized bodies involved
	const SimTK::MobilizedBody *_b1;
	const SimTK::MobilizedBody *_b2;
	// The bushing frames affixed to the mobilized bodies
	SimTK::Transform _inb1;
	SimTK::Transform _inb2;
    

public:
//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor leaves bodies unspecified, sets the bushing frames
    to be at their body origins, and sets all bushing parameters to zero. **/
	FunctionBasedBushingForce();
    /** This convenience constructor defines and sets the bushing frames on 
    each body, and sets all bushing functions to zero.  **/
    FunctionBasedBushingForce(const std::string& body1Name, 
                 const SimTK::Vec3& point1, 
                 const SimTK::Vec3& orientation1,
		         const std::string& body2Name, 
                 const SimTK::Vec3& point2, 
                 const SimTK::Vec3& orientation2);
    /** This convenience constructor defines a bushing that behaves like a
    primitive bushing.  Stiffnesses are used to define linear functions for
    force deflection profiles.**/
	FunctionBasedBushingForce(const std::string& body1Name, 
                 const SimTK::Vec3& point1, 
                 const SimTK::Vec3& orientation1,
		         const std::string& body2Name, 
                 const SimTK::Vec3& point2, 
                 const SimTK::Vec3& orientation2,
				 const SimTK::Vec3& transStiffness, 
                 const SimTK::Vec3& rotStiffness, 
                 const SimTK::Vec3& transDamping, 
                 const SimTK::Vec3& rotDamping);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

	/** Set the name of the Body that will serve as body 1 for this bushing. **/
	void setBody1ByName(const std::string& aBodyName);
    /** Set the location and orientation (optional) for bushing frame on 
    body 1. **/
	void setBody1BushingLocation(const SimTK::Vec3& location, 
                                 const SimTK::Vec3& orientation=SimTK::Vec3(0));
	/** Set the name of the Body that will serve as body 2 for this bushing. **/
	void setBody2ByName(const std::string& aBodyName);
    /** Set the location and orientation (optional) for bushing frame on 
    body 2. **/
	void setBody2BushingLocation(const SimTK::Vec3& location, 
                                 const SimTK::Vec3& orientation=SimTK::Vec3(0));
    void setMomentVisualScale(double scale) {set_moment_visual_scale(scale);}
    void setForceVisualScale(double scale) {set_force_visual_scale(scale);}
	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	/** Compute the deflection (spatial separation) of the two frames connected
	    by the bushing force. Angualar displacement expressed in Euler angles.
		The force and potential energy are determined by the deflection.  */
	virtual SimTK::Vec6 computeDeflection(const SimTK::State& s) const;

	/** Compute the bushing force contribution to the system and add in to appropriate
	  * bodyForce and/or system generalizedForce. 
	  */
	virtual void computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

    /** Potential energy is determined by the elastic energy storage of the bushing.
	    In spatial terms, U = ~dq*[K]*dq, with K and dq defined above. */
	virtual double computePotentialEnergy(const SimTK::State& s) const;

    

	//--------------------------------------------------------------------------
	// Reporting
	//--------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) 
     * to be reported.
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

protected:
    //--------------------------------------------------------------------------
    // Visual support in SimTK visualizer
    // -------------------------------------------------------------------------
    void FunctionBasedBushingForce::generateDecorations(
        bool fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const;
    void ComputeForcesAtBushing(const SimTK::State& state, 
                                SimTK::SpatialVec& forces_on_M_in_ground, 
                                SimTK::SpatialVec& forces_on_F_in_ground) const;

private:
	//--------------------------------------------------------------------------
	// Implement ModelComponent interface.
	//--------------------------------------------------------------------------
	void connectToModel(Model& aModel) OVERRIDE_11;
	// Create a SimTK::Force::LinarBushing which implements this FunctionBasedBushingForce.
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    //--------------------------------------------------------------------------
	// Visible Object Support for Java Gui
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);
	virtual void updateGeometry(const SimTK::State& s);
	void setNull();
	void constructProperties();

//==============================================================================
};	// END of class FunctionBasedBushingForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FUNCTION_BASED_BUSHING_FORCE_H_


