#ifndef STEP_EVENT_H
#define STEP_EVENT_H

#include "OpenSim/OpenSim.h"
#include "Simbody.h"
#include <SimTKcommon/internal/EventHandler.h>


namespace OpenSim {

	/**
	 *	Implements a step function.
	 */
class StepEvent : public SimTK::ScheduledEventHandler, public OpenSim::ModelComponent {
	OpenSim_DECLARE_CONCRETE_OBJECT(StepEvent, ModelComponent);

public:

	StepEvent(double amplitude, double eventTime, OpenSim::Model* model,
		std::string component, std::string input);

	~StepEvent();

protected:

	virtual SimTK::Real getNextEventTime(const SimTK::State& s,
		bool includeCurrentTime) const override;

	/**
	 *	Handles the event.
	 */
	virtual void handleEvent(
		SimTK::State& s, SimTK::Real accuracy, bool& shouldTerminate)
		const override;

private:

	double computeValue(const SimTK::State& s) const;

	void constructInputOutput() const;

	OpenSim::Model* m_model;

	/**
	 *	Used for flat accessing to the component.
	 */
	std::string m_component;

	std::string m_input;

	/**
	 *	Time to apply the step response.
	 */
	double m_event_time;

	/**
	 *	Amplitude of the step response.
	 */
	double m_amplitude;

	bool m_enabled = 0;

}; // end of class 
}; // end of namespace OpenSim

#endif