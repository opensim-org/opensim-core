#ifndef __CoupledBushingForce_h__
#define __CoupledBushingForce_h__
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  CoupledBushingForce.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


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
class OSIMPLUGIN_API CoupledBushingForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(CoupledBushingForce, Force);

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
	void connectToModel(Model& aModel) override;
	/**
	 * Create a SimTK::Force which implements this CoupledBushingForce as part of the SimTK::MultibodySystem.
	 */
	void extendAddToSystem(SimTK::MultibodySystem& system) const override;


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


