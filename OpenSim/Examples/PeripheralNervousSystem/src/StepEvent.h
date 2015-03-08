#ifndef STEP_EVENT_H
#define STEP_EVENT_H

#include "OpenSim/OpenSim.h"
#include "Simbody.h"
#include <SimTKcommon/internal/EventHandler.h>


namespace OpenSim {

	/**
	 *	Implements a step function.
	 */
class StepEvent : public SimTK::ScheduledEventHandler {

public:

	StepEvent(double amplitude, double eventTime, OpenSim::Model* model,
		std::string component);

	~StepEvent();

protected:

	virtual SimTK::Real getNextEventTime(const SimTK::State& s,
		bool includeCurrentTime) const OVERRIDE_11;

	/**
	 *	Handles the event.
	 */
	virtual void handleEvent(
		SimTK::State& s, SimTK::Real accuracy, bool& shouldTerminate)
		const OVERRIDE_11;

private:

	OpenSim::Model* m_model;

	/**
	 *	Used for flat accessing to the component.
	 */
	std::string m_component;

	/**
	 *	Time to apply the step response.
	 */
	double m_event_time;

	/**
	 *	Amplitude of the step response.
	 */
	double m_amplitude;

}; // end of class 
}; // end of namespace OpenSim

#endif