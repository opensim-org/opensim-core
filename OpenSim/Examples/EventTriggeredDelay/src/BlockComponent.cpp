
#include "BlockComponent.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string BlockComponent::CACHE_OUTPUT = "component_output";
const string BlockComponent::OUTPUT = "output";
const string BlockComponent::INPUT = "input";

BlockComponent::BlockComponent()
{
	setNull();
	constructProperties();
	constructInputs();
	constructOutputs();
	setName("default_TransferFunction");
}

BlockComponent::BlockComponent(const string& name)
{
	setNull();
	constructProperties();
	constructInputs();
	constructOutputs();
	setName(name);
}

BlockComponent::~BlockComponent()
{
	//no need to delete m_input
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

		return value;
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

void BlockComponent::constructInputs() const
{
	BlockComponent* self = const_cast<BlockComponent *>(this);

	self->constructInput<double>(INPUT, Stage::Time);//not used	
}

void BlockComponent::constructOutputs() const
{
	BlockComponent* self = const_cast<BlockComponent *>(this);

	self->constructOutput<double>(OUTPUT,
		&BlockComponent::getComponentOutput, Stage::Dynamics);
}




