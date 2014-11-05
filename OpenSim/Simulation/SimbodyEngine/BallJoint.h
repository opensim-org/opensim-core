#ifndef OPENSIM_BALL_JOINT_H_ 
#define OPENSIM_BALL_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  BallJoint.h                            *
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
#include "Joint.h"


namespace OpenSim {

class Model;

/** 

<P> A class implementing a Ball joint. The underlying implementation is Simbody is a
Mobilized::Ball. A modeling option allows the joint to use a Euler sequence. </P>

\image html small_ballJoint.gif

<strong>Python Example</strong> 

\code{.py}

## Assumes that you have created two bodies 'block1' and 'block2'
## Generate joint specific properties

import opensim as osim

locationInParent    = modeling.Vec3(-0.05, -0.35, -0)
orientationInParent = modeling.Vec3(0,0,0)
locationInChild     = modeling.Vec3(-0.2,0.4,-0.2)
orientationInChild  = modeling.Vec3(0,1.5,0)

joint1  = modeling.BallJoint('bloclJoint', 
                                ,block1, 
                                ,locInParent
                                ,oriInParent
                                ,block2
                                ,locInChild
                                ,oriInChild
                                ,0)
block1.setJoint(joint1)
model.addBody(block2)

\endcode

@author Ajay Seth
@version 1.0

*/

class OSIMSIMULATION_API BallJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(BallJoint, Joint);

private:
	static const int _numMobilities = 3;
//=============================================================================
// DATA
//=============================================================================


//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	BallJoint();
	// convenience constructor
	BallJoint(const std::string &name, const OpenSim::Body& parent, 
		const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
		const OpenSim::Body& body, 
		const SimTK::Vec3& locationInBody, const SimTK::Vec3& orientationInBody,
				/*bool useEulerAngles=true,*/ bool reverse=false);

	virtual ~BallJoint();

	int numCoordinates() const override {return _numMobilities;} ;

protected:
    // ModelComponent interface.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void initStateFromProperties(SimTK::State& s) const override;
    void setPropertiesFromState(const SimTK::State& state) override;

//=============================================================================
};	// END of class BallJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_BALL_JOINT_H_


