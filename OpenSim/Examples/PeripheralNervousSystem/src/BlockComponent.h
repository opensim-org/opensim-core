#ifndef BLOCK_COMPONENT_H
#define BLOCK_COMPONENT_H

#include <Simbody.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Model.h>

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
	static const std::string INPUT;
	static const std::string OUTPUT;

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
	* Gets the transfer function output from the cache variable.
	*/
	double getComponentOutput(const SimTK::State& s) const;

protected:


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

	/**
	* Initializes input output connectors.
	*/
	virtual void constructInputOutput() const = 0;

	

private:
	/**
	*	Contains the non-delayed input.
	*/
	mutable SimTK::Measure_<double>::Constant m_buffer;

	/**
	*	Contains the delayed input.
	*/
	SimTK::Measure_<double>::Delay m_delay_buffer;

	



}; // end of class
}  // end of namespace OpenSim

#endif 
