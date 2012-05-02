#ifndef OPENSIM_ACTUATOR_WORK_METER_H_
#define OPENSIM_ACTUATOR_WORK_METER_H_

// ActuatorWorkMeter.h
// Author: Ajay Seth
/*
 * Copyright (c)  2011-12, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

class Model;

//==============================================================================
//                         ACTUATOR WORK METER
//==============================================================================
/**
 * ActuatorWorkMeter is a ModelComponent for computing the Work generated(+)
 * or dissipated by an Actuator.
 * Specific WorkMeters for specific actuators like muscles can be derived from 
 * this class. 
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API ActuatorWorkMeter : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(ActuatorWorkMeter, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(actuator_name, std::string,
		"The name of the actuator whose work use will be calculated.");
    OpenSim_DECLARE_PROPERTY(initial_actuator_work, double,
        "Initial value for work; normally zero.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------

	/** Default constructor **/
    ActuatorWorkMeter();
	/** Convenience constructor **/
	explicit ActuatorWorkMeter(const Actuator& actuator, double initialWork=0);
    
    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

	virtual double getWork(const SimTK::State& state) const;

protected:
	// Model component interface
	virtual void setup(Model& aModel);
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual void initState(SimTK::State& state) const;
    virtual void setDefaultsFromState(const SimTK::State& state);
	virtual int getNumStateVariables() const { return 1; };
	
	SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;

private:
	void setNull();
	void constructProperties();

//==============================================================================
// DATA
//==============================================================================
	// A ReferencePtr is initialized to zero on construction, and also after
    // copy construction and copy assignment. A value is assigned in setup().
    SimTK::ReferencePtr<Actuator> _actuator;

//==============================================================================
};	// END of class ActuatorWorkMeter
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ACTUATOR_WORK_METER_H_


