
#include "ComponentInput.h"

#include "BlockComponent.h"
#include <OpenSim/Simulation/Model/ComponentSet.h>


using namespace std;
using namespace OpenSim;
using namespace SimTK;

ComponentInput::ComponentInput(Model* model, string component, double delay)
	: TriggeredEventHandler(Stage::Time), m_model(model), m_component(component),
	m_delay(delay)
{

	m_input = Measure_<double>::Constant(
		m_model->updDefaultSubsystem(), 0);

	m_delay_input = Measure_<double>::Delay(
		m_model->updDefaultSubsystem(), m_input, m_delay);
	m_delay_input.setDefaultValue(0);

	getTriggerInfo().setTriggerOnRisingSignTransition(true);
	getTriggerInfo().setTriggerOnFallingSignTransition(false);
}

ComponentInput::~ComponentInput()
{
	
}

SimTK::Real ComponentInput::getValue(const SimTK::State& s) const
{
	if (m_delay_input.getValue(s) == m_input.getValue(s))
	{
		return -1;
	}
	else
	{
		return 1;
	}
}

void ComponentInput::handleEvent(State& s, Real accuracy, bool& shouldTerminate) const
{
	((BlockComponent &)m_model->updMiscModelComponentSet().get(m_component)).
		markCacheVariableInvalid(s, BlockComponent::CACHE_OUTPUT);
}

double ComponentInput::getInput(const State& s, bool delayed) const
{
	if (delayed)
	{
		return m_delay_input.getValue(s);
	}
	else
	{
		return m_input.getValue(s);
	}
}


void ComponentInput::setInput(const State& s, const double x) const
{
	ComponentInput* self = const_cast<ComponentInput *>(this);

	self->m_input.setValue(x);
}

void ComponentInput::setDelay(double d)
{
	m_delay_input.setDelay(d);
}