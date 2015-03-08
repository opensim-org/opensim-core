
#include "StepEvent.h"

#include "BlockComponent.h"
#include <OpenSim/Simulation/Model/ComponentSet.h>


using namespace std;
using namespace OpenSim;
using namespace SimTK;

StepEvent::StepEvent(double amplitude, double eventTime, Model* model, 
	string component)
	: ScheduledEventHandler(), m_amplitude(amplitude), m_event_time(eventTime),
	m_model(model), m_component(component)
{

}

StepEvent::~StepEvent()
{
	
}

Real StepEvent::getNextEventTime(const State& s, bool includeCurrentTime) const
{
	return m_event_time;
}

void StepEvent::handleEvent(State& s, Real accuracy, bool& shouldTerminate) const
{
	((BlockComponent &) m_model->updMiscModelComponentSet().get(m_component)).
		setInput(s, m_amplitude, false);
}

