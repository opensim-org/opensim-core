
#include "BlockComponent.h"

#include <OpenSim/Simulation/Model/ComponentSet.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string BlockComponent::CACHE_OUTPUT = "cache_output";
const string BlockComponent::INPUT = "component_input";
const string BlockComponent::OUTPUT = "component_output";


BlockComponent::BlockComponent()
{
	setNull();
	constructProperties();
	setName("default_TransferFunction");
}

BlockComponent::BlockComponent(const string& name)
{
	setNull();
	constructProperties();
	setName(name);
}

BlockComponent::~BlockComponent()
{
	
}

/************************************************************************/
/* Getter/setter                                                        */
/************************************************************************/

double BlockComponent::getGain() const
{
	return get_gain();
}
   
void BlockComponent::setGain(double g)
{
	set_gain(g);
}

double BlockComponent::getDelay() const
{
	return get_delay();
}
	
void BlockComponent::setDelay(double d)
{
	set_delay(d);

	BlockComponent* self = const_cast<BlockComponent *>(this);
	self->m_delay_buffer.setDelay(d);
}

string BlockComponent::getType() const
{
	return get_type();
}

void BlockComponent::setType(std::string t)
{
	set_type(t);
}

/************************************************************************/
/* Interface                                                            */
/************************************************************************/

double BlockComponent::getComponentOutput(const State& s) const
{
	if (!isCacheVariableValid(s, CACHE_OUTPUT))
	{

		double& value = updCacheVariableValue<double>(s, CACHE_OUTPUT);
		calculateOutput(s, value);
		markCacheVariableValid(s, CACHE_OUTPUT);

		m_buffer.setValue(value);

		return m_delay_buffer.getValue(s);
	}
	return getCacheVariableValue<double>(s, CACHE_OUTPUT);
}

/************************************************************************/
/* Must be implemented by subclass                                      */
/************************************************************************/

void BlockComponent::extendAddToSystem(MultibodySystem& system) const
{
	Super::extendAddToSystem(system);

	double value = 0;
	addCacheVariable(CACHE_OUTPUT, value, Stage::Dynamics);

	
	m_buffer = Measure_<double>::Constant(
		_model->updDefaultSubsystem(), 0);

	BlockComponent* self = const_cast<BlockComponent *>(this);
	self->m_delay_buffer = Measure_<double>::Delay(
		_model->updDefaultSubsystem(), m_buffer, getDelay());
	self->m_delay_buffer.setDefaultValue(0);
}

void BlockComponent::setNull()
{
	setAuthors("Dimitar Stanev");
}

void BlockComponent::constructProperties()
{
	constructProperty_gain(1);
	constructProperty_delay(0);
	constructProperty_type("TF");
}


