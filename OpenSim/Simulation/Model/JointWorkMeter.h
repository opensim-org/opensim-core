#ifndef OPENSIM_JOINT_WORK_METER_H
#define OPENSIM_JOINT_WORK_METER_H

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
//==============================================================================
/**
 * JointWorkMeter is a ModelComponent for computing the Work generated(+)
 * or dissipated by a Joint (with prescribed motion).
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API JointWorkMeter : public ModelComponent  
{
//=============================================================================
// DATA
//=============================================================================

protected:

	Joint* _joint;

//=============================================================================
// METHODS
//=============================================================================
//--------------------------------------------------------------------------
// CONSTRUCTION
//--------------------------------------------------------------------------
public:
	// default 
	JointWorkMeter();
	// convenience
	JointWorkMeter(const Joint &joint, double initialWork=0);
	// copy
	JointWorkMeter(const JointWorkMeter &aActuatorWorkMeter);
	~JointWorkMeter();
	Object* copy() const;

#ifndef SWIG
	JointWorkMeter& operator=(const JointWorkMeter &aActuatorWorkMeter);
#endif

	OPENSIM_DECLARE_DERIVED(JointWorkMeter, Object);

	double getWork(const SimTK::State& state) const;

protected:
	// Model component interface
	void setup(Model& aModel);
	void createSystem(SimTK::MultibodySystem& system) const;
	void initState(SimTK::State& state) const;
    void setDefaultsFromState(const SimTK::State& state);
	int getNumStateVariables() const { return 1; };
	
	SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class JointWorkMeter
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_JOINT_WORK_METER_H


