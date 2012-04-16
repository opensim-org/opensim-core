#ifndef __PointOnLineConstraint_h__
#define __PointOnLineConstraint_h__

// PointOnLineConstraint.h
// Author: Samuel Hamner
/*
 * Copyright (c) 2009, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include "Constraint.h"
#include "Body.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Point On Line Constraint.  The underlying Constraint 
 * in Simbody is a Constraint::PointOnLine
 *
 * @author Samuel Hamner
 * @version 1.0
 */
class OSIMSIMULATION_API PointOnLineConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(PointOnLineConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Specify the body on which the line is defined */
	PropertyStr _lineBodyNameProp;
	std::string& _lineBodyName;

	/** Direction of the line in line body specified in the line body frame. */
	PropertyDblVec3 _lineDirectionProp;
	SimTK::Vec3& _lineDirection;

	/** Specify the default point on the line in the line body frame. */
	PropertyDblVec3 _pointOnLineProp;
	SimTK::Vec3& _pointOnLine;

	/** Specify the follower body constrained to the line. */
	PropertyStr _followerBodyNameProp;
	std::string& _followerBodyName;

	/** Specify the  point on the follower bocy constrained to the line in 
	the follower body reference frame. */
	PropertyDblVec3 _pointOnFollowerProp;
	SimTK::Vec3& _pointOnFollower;

	/** Line body */
	Body *_lineBody;

	/** Follower body */
	Body *_followerBody;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	PointOnLineConstraint();
	PointOnLineConstraint(const PointOnLineConstraint &aConstraint);
	PointOnLineConstraint(OpenSim::Body& lineBody, SimTK::Vec3 lineDirection, SimTK::Vec3 pointOnLine,
		OpenSim::Body& followerBody, SimTK::Vec3 followerPoint);
	virtual ~PointOnLineConstraint();

	PointOnLineConstraint& operator=(const PointOnLineConstraint &aConstraint);
	void copyData(const PointOnLineConstraint &aConstraint);

	//SET 
	void setLineBodyByName(std::string aBodyName);
	void setFollowerBodyByName(std::string aBodyName);
	void setLineDirection(SimTK::Vec3 direction);
	void setPointOnLine(SimTK::Vec3 point);
	void setPointOnFollower(SimTK::Vec3 point);

protected:
	virtual void setup(Model& aModel);
	/**
	 * Create a SimTK::Constraint::PointOnLine which implements this constraint.
	 */
	virtual void createSystem(SimTK::MultibodySystem& system) const;


private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class PointOnLineConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PointOnLineConstraint_h__


