#ifndef CONSTANT_INPUT_H
#define CONSTANT_INPUT_H

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

/**
* Implements a constant input function.
*
* @author Dimitar Stanev
*/
class ConstantInput : public ModelComponent {
	OpenSim_DECLARE_CONCRETE_OBJECT(ConstantInput, ModelComponent);
public:

	ConstantInput(double amplitude);

	~ConstantInput();

	OpenSim_DECLARE_PROPERTY(amplitude, double,
		"The amplitude of the component");

	static const std::string OUTPUT;

protected:

	double getValue(const SimTK::State& s) const;

private:

	/**
	* Initializes property values, do use the auto generated set functions.
	*/
	virtual void constructProperties();

	/**
	* Constructs output of the component
	*/
	virtual void constructOutputs() const;

}; // end of class 
}; // end of namespace OpenSim

#endif