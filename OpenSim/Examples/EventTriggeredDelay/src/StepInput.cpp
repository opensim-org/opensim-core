#include "StepInput.h"


using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string StepInput::OUTPUT = "output";

StepInput::StepInput(double amplitude, double eventDelay)
{
	setNull();
	constructProperties();
	constructOutputs();

	set_amplitude(amplitude);
	set_delay(eventDelay);
}

StepInput::~StepInput()
{
	
}

void StepInput::setEventEnabled()
{
	m_event_enabled = true;
}

double StepInput::getValue(const SimTK::State& s) const
{
	if (m_event_enabled)
	{
		return get_amplitude();
	}
	else
	{
		return 0;
	}

}

void StepInput::extendAddToSystem(MultibodySystem& system) const
{
	Super::extendAddToSystem(system);
	
	StepInput* self = const_cast<StepInput *>(this);
	StepEvent* step_event = new StepEvent(get_delay(), self);
	system.addEventHandler(step_event);
}


void StepInput::setNull()
{
	setAuthors("Dimitar Stanev");

	m_event_enabled = false;
}

void StepInput::constructProperties()
{
	constructProperty_amplitude(1);
	constructProperty_delay(0);
}


void StepInput::constructOutputs() const
{
	StepInput* self = const_cast<StepInput *>(this);

	self->constructOutput<double>(OUTPUT, &StepInput::getValue, Stage::Time);
}