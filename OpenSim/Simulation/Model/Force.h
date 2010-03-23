#ifndef __Force_h__
#define __Force_h__
// Force.h
// Author: Peter Eastman, Ajay Seth
/*
 * Copyright (c)  2009 Stanford University. All rights reserved. 
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
#include "OpenSim/Simulation/osimSimulationDLL.h"
#include "OpenSim/Common/Object.h"
#include <OpenSim/Common/PropertyBool.h>
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include <SimTKsimbody.h>

namespace OpenSim {

class Model;

/**
 * This abstract class represents a force applied to bodies or generalized coordinates during a simulation.
 * Each subclass represents a different type of force.  The actual force computation is done by a SimTK::Force,
 * which is created by createSystem().
 *
 * @author Peter Eastman
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Force : public ModelComponent
{
	protected:
	/** Flag indicating whether the force is disabled or not.  Disabled 
	means that the force is not active in subsequent dynamics realizations. */
	PropertyBool _isDisabledProp;

	/** ID for the force in Simbody. */
	SimTK::ForceIndex _index;

//=============================================================================
// METHODS
//=============================================================================
public:
	Force();
	Force(const Force &aForce);
	virtual ~Force();

	/**
	 * deserialization from XML, necessary so that derived classes can (de)serialize
	 */
	Force(DOMElement* aNode): ModelComponent(aNode) {setNull(); setupProperties(); };
	
	Force& operator=(const Force &aForce);
	void copyData(const Force &aForce);
	virtual Object* copy() const;

	/**
	 * Subclasses may optionally override this method to perform setup.
	 */
    virtual void setup(Model& model);
	virtual void initState(SimTK::State& state) const;
    virtual void setDefaultsFromState(const SimTK::State& state);
	/**
	 * Subclasses may optionally override this method to perform setup after initSystem() has been called.
	 */
	 virtual void postInit(Model& model) { }
    /**
     * Get the number of state variables allocated by this force.  The default implementation
     * returns 0.  Subclasses that allocate state variables must override it.
     */
    virtual int getNumStateVariables() const;
    /**
     * Get the name of a state variable allocated by this force.  The default implementation
     * throws an exception, so subclasses that allocate state variables must override it.
     *
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     */
    virtual std::string getStateVariableName(int index) const;
    /**
     * Get the value of a state variable allocated by this force.  The default implementation
     * throws an exception, so subclasses that allocate state variables must override it.
     *
     * @param state   the State for which to get the value
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     */
    virtual double getStateVariable(const SimTK::State& state, int index) const;
    /**
     * Set the value of a state variable allocated by this force.  The default implementation
     * throws an exception, so subclasses that allocate state variables must override it.
     *
     * @param state   the State for which to set the value
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     * @param value   the value to set
     */
    virtual void setStateVariable(SimTK::State& state, int index, double value) const;


	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const {
		return OpenSim::Array<std::string>();
	}
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
		return OpenSim::Array<double>();
	};

	/** return if the Force is disabled or not */
	virtual bool isDisabled(const SimTK::State &s) const;
	/** Set the Force as disabled (true) or not (false)*/
	virtual void setDisabled(SimTK::State &s, bool disabled);

private:

	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class Force
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Force_h__


