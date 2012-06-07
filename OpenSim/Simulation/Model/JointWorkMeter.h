#ifndef OPENSIM_JOINT_WORK_METER_H_
#define OPENSIM_JOINT_WORK_METER_H_

// JointWorkMeter.h
// Author: Ajay Seth
/*
 * Copyright (c)  2012, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

namespace OpenSim {

class Model;

//==============================================================================
//                           JOINT WORK METER
//==============================================================================
/**
 * JointWorkMeter is a ModelComponent for computing the Work generated(+)
 * or dissipated by a Joint (with prescribed motion).
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API JointWorkMeter : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(JointWorkMeter, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(joint_name, std::string,
		"The joint name whos work use will be calculated.");
	OpenSim_DECLARE_PROPERTY(initial_joint_work, double,
		"The initial value for the work.");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	// default 
	JointWorkMeter();
	// convenience
	explicit JointWorkMeter(const Joint &joint, double initialWork=0);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

	double getWork(const SimTK::State& state) const;

protected:
	// Model component interface
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	void initStateFromProperties(SimTK::State& state) const OVERRIDE_11;
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;
	int getNumStateVariables() const  OVERRIDE_11 { return 1; }	
	SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const
         OVERRIDE_11;

private:
	void setNull();
	void constructProperties();

//=============================================================================
// DATA
//=============================================================================
    // A ReferencePtr is automatically initialize to null at construction,
    // and reset to null after copy construction or copy assignment. This is
    // set in the connectToModel() method.
	SimTK::ReferencePtr<Joint> _joint;
//=============================================================================
};	// END of class JointWorkMeter
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_JOINT_WORK_METER_H_


