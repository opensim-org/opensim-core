#ifndef BLOCK_COMPONENT_H
#define BLOCK_COMPONENT_H

#include <Simbody.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "ComponentInput.h"

namespace OpenSim {

/** 
* An abstract class for modeling the input/output relationship of a component.
*
*	@author Dimitar Stanev
*/
class BlockComponent : public ModelComponent {
	OpenSim_DECLARE_ABSTRACT_OBJECT(BlockComponent, ModelComponent);
public:

	OpenSim_DECLARE_PROPERTY(gain, double,
		"The gain of the transfer function");
	OpenSim_DECLARE_PROPERTY(delay, double,
		"The delay of the transfer function");
	OpenSim_DECLARE_PROPERTY(type, std::string,
		"Type of the transfer function");

	static const std::string CACHE_OUTPUT;

	BlockComponent();

	BlockComponent(const std::string& name);

	~BlockComponent();


	/************************************************************************/
	/* Getters/setters                                                      */
	/************************************************************************/
	
	double getGain() const;
	void setGain(double g);

	double getDelay() const;
	void setDelay(double d);

	std::string getType() const;
	void setType(std::string t);

	/**
	* Getter and setter for the array of excitatory components.
	*/
	OpenSim::Array<std::string> getExcitatory() const;
	void setExcitatory(OpenSim::Array<std::string> &e);

	/**
	* Getter and setter for the array of inhibitory components.
	*/
	OpenSim::Array<std::string> getInhibitory() const;
	void setInhibitory(OpenSim::Array<std::string> &i);

	/************************************************************************/
	/* Interface                                                            */
	/************************************************************************/

	/**
	* Gets the transfer function input.
	*/
	double getInput(const SimTK::State& s, bool delayed = true) const;

	/**
	* Sets the transfer function input and if desirable invalidates the output 
	* cache.
	*/
	void setInput(const SimTK::State& s, const double x,
		bool toInvalidate = false) const;

	/**
	* Gets the transfer function output from the cache variable.
	*/
	double getOutput(const SimTK::State& s) const;
	

protected:

	/**
	 *	Input handler with delay buffer and event trigger.
	 */
	ComponentInput* m_input;	

	/**
	* Array of excitatory and inhibitory components
	*/
	OpenSim::Array<std::string> m_excitatory, m_inhibitory;

	/************************************************************************/
	/* Must be implemented by subclass                                      */
	/************************************************************************/

	/**
	* Calculates the output of the system based on the state and must be 
	* implemented by the components which have state equations. It uses the cache.
	*/
	virtual void calculateOutput(const SimTK::State& s, double& data) const = 0;

	/**
	* It must be implemented in order to have access to subsystem and must be 
	* called to initialize the state variables.
	*/
	virtual void extendAddToSystem(SimTK::MultibodySystem& system) const;

	/**
	* Initializes data members.
	*/
	virtual void setNull();

	/**
	* Initializes property values, do use the auto generated set functions.
	*/
	virtual void constructProperties();

	/************************************************************************/
	/* Implemented                                                          */
	/************************************************************************/

	/**
	* Computes the total contribution of all excitatory and inhibitory
	* components to the input. Used if a component has many input signals.
	*/
	double computeExcitatoryInhibitoryCommand(const SimTK::State& s) const;


}; // end of class
}  // end of namespace OpenSim

#endif 
