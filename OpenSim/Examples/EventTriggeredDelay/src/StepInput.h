#ifndef STEP_INPUT_H
#define STEP_INPUT_H

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

/**
* Implements a step response function.
*
* @author Dimitar Stanev
*/
class StepInput : public ModelComponent {
	OpenSim_DECLARE_CONCRETE_OBJECT(StepInput, ModelComponent);
public:

	StepInput(double amplitude, double eventDelay);

	~StepInput();

	OpenSim_DECLARE_PROPERTY(amplitude, double,
		"The amplitude of the component");
	OpenSim_DECLARE_PROPERTY(delay, double,
		"The delay of step event");

	static const std::string OUTPUT;

	void setEventEnabled();

protected:

	double getValue(const SimTK::State& s) const;

	virtual void extendAddToSystem(SimTK::MultibodySystem& system)
		const override;
	
private:

	bool m_event_enabled;

	virtual void setNull();

	/**
	* Initializes property values, do use the auto generated set functions.
	*/
	virtual void constructProperties();

	/**
	* Constructs output of the component
	*/
	virtual void constructOutputs() const;

}; // end of class 

class StepEvent : public SimTK::ScheduledEventHandler
{

public:

	StepEvent(double eventDelay, StepInput* stepEventReference)
		: SimTK::ScheduledEventHandler(), m_event_delay(eventDelay),
		m_step_event_reference(stepEventReference)
	{

	}

	~StepEvent(){};

	SimTK::Real getNextEventTime(const SimTK::State& s, bool includeCurrentTime) const
	{
		return m_event_delay;
	}

	void handleEvent(SimTK::State& s, SimTK::Real accuracy, bool& shouldTerminate) const
	{
		m_step_event_reference->setEventEnabled();
	}

private:

	mutable SimTK::ReferencePtr<StepInput> m_step_event_reference;

	double m_event_delay;

};

}; // end of namespace OpenSim

#endif