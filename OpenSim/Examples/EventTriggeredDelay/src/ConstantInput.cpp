#include "ConstantInput.h"


using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string ConstantInput::OUTPUT = "output";

ConstantInput::ConstantInput(double amplitude)
{
	constructProperties();
	constructOutputs();

	set_amplitude(amplitude);
}

ConstantInput::~ConstantInput()
{
	
}

double ConstantInput::getValue(const SimTK::State& s) const
{
	return get_amplitude();
}

void ConstantInput::constructProperties()
{
	constructProperty_amplitude(1);
}


void ConstantInput::constructOutputs() const
{
	ConstantInput* self = const_cast<ConstantInput *>(this);

	self->constructOutput<double>(OUTPUT,
		&ConstantInput::getValue, Stage::Time);
}