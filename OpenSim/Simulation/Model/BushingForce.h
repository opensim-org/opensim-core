#ifndef __BushingForce_h__
#define __BushingForce_h__

// BushingForce.h
// Author: Ajay Seth
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

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Bushing Force.
 * A Bushing Force is the force proportional to the deviation of two frames. 
 * One can think of the Bushing as being composed of 3 linear and 3 torsional
 * spring-dampers, which act along or about the bushing frames. 
 * The underlying Force in Simbody is a SimtK::Force::LinearBushing
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API BushingForce : public Force  
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

	/** Location of the bushing in first body specified in body1 reference frame. */
	PropertyDblVec3 _locationInBody1Prop;
	SimTK::Vec3& _locationInBody1;

	/** Orientation of the bushing axes on body1 specified in body1's
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationInBody1Prop;
	SimTK::Vec3& _orientationInBody1;

	/** Location of the bushing in second body specified in body2 reference frame. */
	PropertyDblVec3 _locationInBody2Prop;
	SimTK::Vec3& _locationInBody2;

	/** Orientation of the bushing axes on body2 specified in body2's
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationInBody2Prop;
	SimTK::Vec3& _orientationInBody2;

	/** Stiffness of the bushing related to Euler XYZ body-fixed rotation angles 
	    used to express the orientation. */
	PropertyDblVec3 _rotStiffnessProp;
	SimTK::Vec3& _rotStiffness;

	/** Stiffness of the bushing related to XYZ translations between frames   */
	PropertyDblVec3 _transStiffnessProp;
	SimTK::Vec3& _transStiffness;

	/** Damping of the bushing related to Euler XYZ body-fixed angular speeds */ 
	PropertyDblVec3 _rotDampingProp;
	SimTK::Vec3& _rotDamping;

	/** Damping of the bushing related to XYZ translation speeds between frames   */
	PropertyDblVec3 _transDampingProp;
	SimTK::Vec3& _transDamping;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	BushingForce();
	BushingForce( std::string body1Name, SimTK::Vec3 point1, SimTK::Vec3 orientation1,
		          std::string body2Name, SimTK::Vec3 point2, SimTK::Vec3 orientation2,
				  SimTK::Vec3 transStiffness, SimTK::Vec3 rotStiffness, SimTK::Vec3 transDamping, SimTK::Vec3 rotDamping );
	BushingForce(const BushingForce &aForce);
	virtual ~BushingForce();
	virtual Object* copy() const;
	BushingForce& operator=(const BushingForce &aForce);
	void copyData(const BushingForce &aForce);

	//SET 
	void setBody1ByName(std::string aBodyName);
	void setBody1BushingLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));
	void setBody2ByName(std::string aBodyName);
	void setBody2BushingLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));

	/** Potential energy is determine by the elastic energy storage of the bushing. */
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
	 * Create a SimTK::Force::LinarBushing which implements this BushingForce.
	 */
	virtual void createSystem(SimTK::MultibodySystem& system) const;


private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class BushingForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __BushingForce_h__


