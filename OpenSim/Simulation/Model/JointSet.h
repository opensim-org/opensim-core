#ifndef __JointSet_h__
#define __JointSet_h__
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  JointSet.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include <map>
#include <vector>

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of joints.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class OSIMSIMULATION_API JointSet :	public ModelComponentSet<Joint> {
OpenSim_DECLARE_CONCRETE_OBJECT(JointSet, ModelComponentSet<Joint>);

private:
	void setNull();
    void addToSystemForOneJoint(SimTK::MultibodySystem& system, int jointIndex, 
                                 const std::map<Body*, int>& bodyMap, 
                                 std::vector<bool>& hasProcessed) const;
    // This overrides the ModelComponentSet method to enforce ordering.
    void invokeAddToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
public:
	JointSet();
	JointSet(Model& model);
	JointSet(const JointSet& aJointSet);
	~JointSet(void);
	/**
	* Populate this flat list of Joints given a Model that has been setup
	*/
	void populate(Model& aModel);

	// Somehow the following function is not exported from base template
    JointSet(Model& model, const std::string &aFileName, 
             bool aUpdateFromXMLNode = true)
    :   Super(model, aFileName, aUpdateFromXMLNode) {}

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	JointSet& operator=(const JointSet &aJointSet);
#endif
	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------
	void scale(const ScaleSet& aScaleSet);
//=============================================================================
};	// END of class JointSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __JointSet_h__
