
#include "BlockComponent.h"

#include <OpenSim/Simulation/Model/ComponentSet.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string BlockComponent::CACHE_OUTPUT = "component_output";


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
	m_input->setDelay(d);
}

string BlockComponent::getType() const
{
	return get_type();
}

void BlockComponent::setType(std::string t)
{
	set_type(t);
}

Array<string> BlockComponent::getExcitatory() const
{
	return m_excitatory;
}

void BlockComponent::setExcitatory(Array<string> &e)
{
	m_excitatory = e;
}

Array<string> BlockComponent::getInhibitory() const
{
	return m_inhibitory;
}
	
void BlockComponent::setInhibitory(Array<string> &i)
{
	m_inhibitory = i;
}

/************************************************************************/
/* Interface                                                            */
/************************************************************************/

double BlockComponent::getInput(const State& s, bool delayed) const
{
	return m_input->getInput(s, delayed);
}


void BlockComponent::setInput(const State& s, const double x,
	bool toInvalidate) const
{
	m_input->setInput(s, x);

	if (toInvalidate)
	{
		markCacheVariableInvalid(s, CACHE_OUTPUT);
	}
}

double BlockComponent::getOutput(const State& s) const
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

	BlockComponent* self = const_cast<BlockComponent *>(this);

	double value = 0;
	addCacheVariable(CACHE_OUTPUT, value, Stage::Dynamics);

	self->m_input = new ComponentInput(_model, getName(), get_delay());
	_model->updDefaultSubsystem().addEventHandler(m_input);
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

/************************************************************************/
/* Implemented                                                          */
/************************************************************************/

double BlockComponent::computeExcitatoryInhibitoryCommand(
	const State& s) const
{
	ComponentSet& component_set = _model->updMiscModelComponentSet();
	double feedback = 0;

	//cout << "Time: " << s.getTime() << " " <<getName() << endl;

	for(int i = 0;i < m_excitatory.getSize();++i)
	{
		BlockComponent& component = 
			(BlockComponent&) component_set.get(m_excitatory[i]);

		feedback += component.getOutput(s);
	}

	for(int i = 0;i < m_inhibitory.getSize();++i)
	{
		BlockComponent& component = 
			(BlockComponent&) component_set.get(m_inhibitory[i]);

		feedback -= component.getOutput(s);
	}

	return feedback;
}



