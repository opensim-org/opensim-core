
#include "StepEvent.h"

#include "BlockComponent.h"
#include <OpenSim/Simulation/Model/ComponentSet.h>


using namespace std;
using namespace OpenSim;
using namespace SimTK;

StepEvent::StepEvent(double amplitude, double eventTime, Model* model, 
	string component, string input)
	: ScheduledEventHandler(), m_amplitude(amplitude), m_event_time(eventTime),
	m_model(model), m_component(component), m_input(input)
{
	constructInputOutput();
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
	StepEvent* self = const_cast<StepEvent *>(this);

	self->m_enabled = true;

	m_model->updComponent(m_component).
		markCacheVariableInvalid(s, BlockComponent::CACHE_OUTPUT);
}

double StepEvent::computeValue(const SimTK::State& s) const
{
	if (m_enabled)
	{
		return m_amplitude;
	} 
	else
	{
		return 0;
	}
}

void StepEvent::constructInputOutput() const
{
	StepEvent* self = const_cast<StepEvent *>(this);

	self->constructOutput<double>("OUTPUT", &StepEvent::computeValue, Stage::Time);
	m_model->updComponent(m_component).getInput(m_input).connect(getOutput("OUTPUT"));
}

