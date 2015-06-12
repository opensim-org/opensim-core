#ifndef COMPONENT_INPUT_H
#define COMPONENT_INPUT_H

#include "OpenSim/OpenSim.h"
#include "Simbody.h"
#include <SimTKcommon/internal/EventHandler.h>


namespace OpenSim {

	/**
	 *	This class implements a delayed input buffer, that invalidates the
	 *  component's output cache if changes to the input are detected by an
	 *  event triggered handler. The event trigger is used because the 
	 *  integrator may take large steps and may not detect the exact time
	 *  when the input to the component changed.
	 *
	 *  @author Dimitar Stanev
	 */
class ComponentInput : public SimTK::TriggeredEventHandler {

public:
	ComponentInput(OpenSim::Model* model, std::string component, double delay);

	~ComponentInput();

	/**
	 *	Gets the delayed or not delayed input.
	 */
	double getInput(const SimTK::State& s, bool delayed = true) const;
	
	/**
	 *	Sets the input, which is inserted into circular buffer to produce the 
	 *  desired delay.
	 */
	void setInput(const SimTK::State& s, const double x) const;
	
	/**
	 *	Sets the buffer delay.
	 */
	void setDelay(double d);

protected:

	/**
	 *	Checks if the input has changed and triggers an event.
	 */
	virtual SimTK::Real getValue(const SimTK::State&) const OVERRIDE_11;

	/**
	 *	Handles the event of changing in the input.
	 */
	virtual void handleEvent(
		SimTK::State& s, SimTK::Real accuracy, bool& shouldTerminate)
		const OVERRIDE_11;

private:

	OpenSim::Model* m_model;

	/**
	 *	Contains the non-delayed input.
	 */
	SimTK::Measure_<double>::Constant m_input;

	/**
	 *	Contains the delayed input.
	 */
	SimTK::Measure_<double>::Delay m_delay_input;

	/**
	 *	The input delay.
	 */
	double m_delay;
	
	/**
	 *	Used for flat accessing to the component.
	 */
	std::string m_component;

}; // end of class 
}; // end of namespace OpenSim

#endif