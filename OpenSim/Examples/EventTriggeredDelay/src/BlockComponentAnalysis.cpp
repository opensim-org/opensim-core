#include "BlockComponentAnalysis.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>

#include "BlockComponent.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

BlockComponentAnalysis::BlockComponentAnalysis(Model* model) 
	: Analysis(model)
{
	constructDescription();
	constructColumnLabels();
	setupStorage();
}

void BlockComponentAnalysis::setModel(Model& aModel)
{
	Super::setModel(aModel);

	constructDescription();
	constructColumnLabels();
	setupStorage();
}

int BlockComponentAnalysis::begin(SimTK::State& s)
{
	if(!proceed()) return(0);

	m_storage.reset(s.getTime());

	int status = 0;
	if(m_storage.getSize()<=0) 
	{
		status = record(s);
	}

	return(status);
}

int BlockComponentAnalysis::step(const SimTK::State& s, int stepNumber)
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}

int BlockComponentAnalysis::end(SimTK::State& s)
{
	if(!proceed()) return(0);

	record(s);

	return(0);
}

int BlockComponentAnalysis::print(const std::string &path)
{
	m_storage.print(path);

	return(0);
}

int BlockComponentAnalysis::record(const SimTK::State& s)
{
	_model->getMultibodySystem().realize(s, SimTK::Stage::Report);

	//append storage
	Array<double> data;
	ComponentSet& component_set = _model->updMiscModelComponentSet();
	for (int i = 0; i<component_set.getSize(); i++)
	{
		if (dynamic_cast<BlockComponent*>(&component_set[i]) == NULL)
		{
			continue;
		}

		data.append(((BlockComponent&) component_set[i]).
			getInputValue<double>(s, BlockComponent::INPUT));

#ifdef USE_SO_COMPONENT_INPUT
		if (((BlockComponent &) component_set[i]).getType() == "SO")
		{
			data.append(component_set[i].getInputValue<double>(s,
				SpindleOrgan::MUSCLE_LENGTH));
			data.append(component_set[i].getInputValue<double>(s,
				SpindleOrgan::MUSCLE_VELOCITY));
		}
#endif

		/*if (((BlockComponent &) component_set[i]).getType() == "MN")
		{
			if (component_set[i].getInput(Motoneuron::GTO_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					Motoneuron::GTO_INPUT));
			} 
			else
			{
				data.append(0);
			}

			if (component_set[i].getInput(Motoneuron::SO_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					Motoneuron::SO_INPUT));
			}
			else
			{
				data.append(0);
			}

			if (component_set[i].getInput(Motoneuron::RC_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					Motoneuron::RC_INPUT));
			}
			else
			{
				data.append(0);
			}

			if (component_set[i].getInput(Motoneuron::OIN_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					Motoneuron::OIN_INPUT));
			}
			else
			{
				data.append(0);
			}
		}

		if (((BlockComponent &) component_set[i]).getType() == "RC")
		{
			if (component_set[i].getInput(RenshawCell::ORC_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					RenshawCell::ORC_INPUT));
			}
			else
			{
				data.append(0);
			}
		}

		if (((BlockComponent &) component_set[i]).getType() == "IN")
		{
			if (component_set[i].getInput(Interneuron::SO_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					Interneuron::SO_INPUT));
			}
			else
			{
				data.append(0);
			}

			if (component_set[i].getInput(Interneuron::RC_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					Interneuron::RC_INPUT));
			}
			else
			{
				data.append(0);
			}

			if (component_set[i].getInput(Interneuron::OIN_INPUT).isConnected())
			{
				data.append(component_set[i].getInputValue<double>(s,
					Interneuron::OIN_INPUT));
			}
			else
			{
				data.append(0);
			}
		}*/

		data.append(((BlockComponent&) component_set[i]).getOutputValue<double>(s, 
			BlockComponent::OUTPUT));
	}

	m_storage.append(s.getTime(), data.size(), &data[0]);

	return(0);
}

void BlockComponentAnalysis::constructDescription()
{
	string descrip;

	descrip = "\nThis file contains the record of custom variables\n";
	descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
	if (getInDegrees()) {
		descrip += "\nAngles are in degrees.";
	}
	else 
	{
		descrip += "\nAngles are in radians.";
	}
	descrip += "\n\n";

	setDescription(descrip);
}

void BlockComponentAnalysis::constructColumnLabels()
{
	if (_model == NULL) return;

	Array<string> labels;
	labels.append("time");

	ComponentSet& component_set = _model->updMiscModelComponentSet();
	for (int i = 0; i<component_set.getSize(); i++)
	{
		if (dynamic_cast<BlockComponent*>(&component_set[i]) == NULL)
		{
			continue;
		}
		
		labels.append(component_set[i].getName() + "_" + BlockComponent::INPUT);

#ifdef USE_SO_COMPONENT_INPUT
		if (((BlockComponent &)component_set[i]).getType() == "SO")
		{
			labels.append(component_set[i].getName() + "_" +
				((SpindleOrgan &) component_set[i]).MUSCLE_LENGTH);
			labels.append(component_set[i].getName() +  "_" +
				((SpindleOrgan &) component_set[i]).MUSCLE_VELOCITY);
		}
#endif

		//if (((BlockComponent &) component_set[i]).getType() == "MN")
		//{
		//	labels.append(component_set[i].getName() + "_" +
		//		((Motoneuron &) component_set[i]).GTO_INPUT);
		//	labels.append(component_set[i].getName() + "_" +
		//		((Motoneuron &) component_set[i]).SO_INPUT);
		//	labels.append(component_set[i].getName() + "_" +
		//		((Motoneuron &) component_set[i]).RC_INPUT);
		//	labels.append(component_set[i].getName() + "_" +
		//		((Motoneuron &) component_set[i]).OIN_INPUT);
		//}

		//if (((BlockComponent &) component_set[i]).getType() == "RC")
		//{
		//	labels.append(component_set[i].getName() + "_" +
		//		((RenshawCell &) component_set[i]).ORC_INPUT);
		//}

		//if (((BlockComponent &) component_set[i]).getType() == "IN")
		//{
		//	labels.append(component_set[i].getName() + "_" +
		//		((Interneuron &) component_set[i]).SO_INPUT);
		//	labels.append(component_set[i].getName() + "_" +
		//		((Interneuron &) component_set[i]).RC_INPUT);
		//	labels.append(component_set[i].getName() + "_" +
		//		((Interneuron &) component_set[i]).OIN_INPUT);
		//}

		labels.append(component_set[i].getName() + "_" + BlockComponent::OUTPUT);
	}

	setColumnLabels(labels);
}

void BlockComponentAnalysis::setupStorage()
{
	m_storage.reset(0);
	m_storage.setName("custom_analysis");
	m_storage.setDescription(getDescription());
	m_storage.setColumnLabels(getColumnLabels());
}

int BlockComponentAnalysis::printResults(const string &aBaseName, 
	const string &aDir, double aDT,	const string &aExtension)
{
	Storage::printResult(&m_storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

	return(0);
}
