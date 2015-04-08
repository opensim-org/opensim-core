#include "DelaySignal.h"


using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string DelaySignal::OUTPUT = "output";
const string DelaySignal::INPUT = "input";


DelaySignal::DelaySignal(double delay, int dependsOn)
{
	constructProperties();

	set_delay(delay);
	set_stage_dependence(dependsOn);

	constructInputs();
	constructOutputs();

}

DelaySignal::~DelaySignal()
{

}

double DelaySignal::getValue(const SimTK::State& s) const
{
	return m_delay.getValue(s);
}

void DelaySignal::extendAddToSystem(MultibodySystem& system) const
{
	Super::extendAddToSystem(system);

	DelaySignal* self = const_cast<DelaySignal *>(this);

	self->m_delay = Measure_<double>::Delay(
		system.updDefaultSubsystem(), 
		InputMeasure<double>(system.updDefaultSubsystem(), 
			*static_cast<const Input<double>*>(&getInput("input"))),
		get_delay());

	self->m_delay_trigger = new DelayEvenTrigger(self, Stage(get_stage_dependence() + 1));
	system.addEventHandler(m_delay_trigger);
}

void DelaySignal::constructProperties()
{
	constructProperty_delay(0);
	constructProperty_stage_dependence(Stage::Report);
}

void DelaySignal::constructInputs() const
{
	DelaySignal* self = const_cast<DelaySignal *>(this);

	self->constructInput<double>(INPUT, Stage(get_stage_dependence()));
}

void DelaySignal::constructOutputs() const
{
	DelaySignal* self = const_cast<DelaySignal *>(this);

	self->constructOutput<double>(OUTPUT,
		&DelaySignal::getValue, Stage(get_stage_dependence()));
}