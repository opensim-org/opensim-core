#ifndef __CoupledBushingForce_h__
#define __CoupledBushingForce_h__

// CoupledBushingForce.h
// Author: Ajay Seth
/*
 * Copyright (c) 2011, Stanford University. All rights reserved. 
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
#include "osimPluginDLL.h"
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Simulation/Model/Force.h>

namespace OpenSim {

//=================================================================================
//=================================================================================
/**
 * A class implementing a Coupled Bushing Force.
 * A Couple Bushing Force is the force proportional to the deviation of two frames, 
 * where components of the resulting BodyForce are coupled to all any or all 
 * deviations, such that the general stiffness and damping matrices are 6x6.
 *
 * Deviations of a frame1 on body1 and another (frame2) on body2 are represents in
 * termsof x-y-z Euler angle sequence and x,y,z position of frame2 in frame1.
 * Damping is applied to the relative angular velocity and linear velocity of
 * frame2 in frame1.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMPLUGIN_API CoupledBushingForce : public Force  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Specify first of two bodies held together by the bushing. */
	PropertyStr _body1NameProp;
	std::string& _body1Name;

	/** Specify second of two bodies held by the bushing force. */
	PropertyStr _body2NameProp;
	std::string& _body2Name;

	/** Location of the bushing frame1 in first body specified in body1 reference frame. */
	PropertyDblVec3 _locationInBody1Prop;
	SimTK::Vec3& _locationInBody1;

	/** Orientation of the bushing frame1 axes on body1 specified in body1's
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationInBody1Prop;
	SimTK::Vec3& _orientationInBody1;

	/** Location of bushing frame2 in second body specified in body2 reference frame. */
	PropertyDblVec3 _locationInBody2Prop;
	SimTK::Vec3& _locationInBody2;

	/** Orientation of bushing frame2 axes on body2 specified in body2's
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationInBody2Prop;
	SimTK::Vec3& _orientationInBody2;

	/** Stiffness of the bushing related to Euler XYZ body-fixed angular and
	    translationals deviations that express frame2 in frame1. Force is zero
		when frames are coincident and aligned. */
	PropertyDblVec6 _stiffnessMatrixRow1Prop;
	PropertyDblVec6 _stiffnessMatrixRow2Prop;
	PropertyDblVec6 _stiffnessMatrixRow3Prop;
	PropertyDblVec6 _stiffnessMatrixRow4Prop;
	PropertyDblVec6 _stiffnessMatrixRow5Prop;
	PropertyDblVec6 _stiffnessMatrixRow6Prop;
	SimTK::Mat66 _stiffnessMatrix;

	/** Damping of the bushing related to XYZ angular and
	    translational speeds that express frame2 in frame1.  */
	PropertyDblVec6 _dampingMatrixRow1Prop;
	PropertyDblVec6 _dampingMatrixRow2Prop;
	PropertyDblVec6 _dampingMatrixRow3Prop;
	PropertyDblVec6 _dampingMatrixRow4Prop;
	PropertyDblVec6 _dampingMatrixRow5Prop;
	PropertyDblVec6 _dampingMatrixRow6Prop;
	SimTK::Mat66 _dampingMatrix;

private:
	// underlying SimTK system elements
	// the mobilized bodies involved
	const SimTK::MobilizedBody *_b1;
	const SimTK::MobilizedBody *_b2;
	// The bushing frames affixed to the mobilized bodies
	SimTK::Transform _inb1;
	SimTK::Transform _inb2;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	CoupledBushingForce();
	CoupledBushingForce( std::string body1Name, SimTK::Vec3 point1, SimTK::Vec3 orientation1,
		          std::string body2Name, SimTK::Vec3 point2, SimTK::Vec3 orientation2,
				  SimTK::Mat66 stiffnessMat, SimTK::Mat66 dampingMat);
	CoupledBushingForce(const CoupledBushingForce &aForce);
	virtual ~CoupledBushingForce();
	virtual Object* copy() const;
	CoupledBushingForce& operator=(const CoupledBushingForce &aForce);
	void copyData(const CoupledBushingForce &aForce);

	//SET 
	void setBody1ByName(std::string aBodyName);
	void setBody1BushingLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));
	void setBody2ByName(std::string aBodyName);
	void setBody2BushingLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));


	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	/** Compute the deflection (spatial separation) of the two frames connected
	    by the bushing force. Angualar displacement expressed in Euler angles.
		The force and potential energy are determined by the deflection.  */
	virtual SimTK::Vec6 computeDeflection(const SimTK::State& s) const;

	/** Compute the bushing force contribution to the system and add in to appropriate
	  * bodyForce and/or system generalizedForce. The bushing force is [K]*dq + [D]*dqdot
	  * where, [K] is the spatial 6dof stiffness matrix between the two frames 
	           dq is the deflection in body spatial coordinates with rotations in Euler angles
	  *        [D] is the spatial 6dof damping matrix opposing the velocity between the frames
	  *        dqdot is the relative spatial velocity of the two frames
	  * CoupledBushingForce implementation based SimTK::Force::LinearBushing
	  * developed and implemented by Michael Sherman.
	  */
	virtual void computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

    /** Potential energy is determined by the elastic energy storage of the bushing.
	    In spatial terms, U = ~dq*[K]*dq, with K and dq defined above. */
	virtual double computePotentialEnergy(const SimTK::State& s) const;

	//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

protected:
	virtual void setup(Model& aModel);
	/**
	 * Create a SimTK::Force which implements this CoupledBushingForce as part of the SimTK::MultibodySystem.
	 */
	virtual void createSystem(SimTK::MultibodySystem& system) const;


private:
	void setNull();
	void setupProperties();
	void constructMatricesFromProperties();
	void updatePropertiesFromMatrices();

//=============================================================================
};	// END of class CoupledBushingForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __CoupledBushingForce_h__


