#ifndef __JointSet_h__
#define __JointSet_h__

// JointSet.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

class OSIMSIMULATION_API JointSet :	public ModelComponentSet<Joint>
{
private:
	void setNull();
    void createSystemForOneJoint(SimTK::MultibodySystem& system, int jointIndex, const std::map<Body*, int>& bodyMap, std::vector<bool>& hasProcessed) const;
protected:
    void createSystem(SimTK::MultibodySystem& system) const;
public:
	JointSet();
	JointSet(Model& model);
	JointSet(const JointSet& aJointSet);
	~JointSet(void);
	void setup(Model& aModel);
	// Somehow the following function is not exported from base template
    JointSet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode = true) :
        ModelComponentSet<Joint>(model, aFileName, aUpdateFromXMLNode)
    {
    }
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
