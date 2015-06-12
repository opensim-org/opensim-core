#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>

#include "BlockComponentAnalysis.h"
#include "BlockComponent.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

BlockComponentAnalysis::BlockComponentAnalysis() 
	: Analysis()
{
	setNull();
	constructProperties();
}

BlockComponentAnalysis::BlockComponentAnalysis(Model *model) 
	: Analysis(model)
{
	setNull();
	constructProperties();

	constructDescription();
	constructColumnLabels();
	setupStorage();
}


/**
 * Set the model for which this analysis is to be run.
 *
 * Sometimes the model on which an analysis should be run is not available
 * at the time an analysis is created.  Or, you might want to change the
 * model.  This method is used to set the model on which the analysis is
 * to be run.
 *
 * @param aModel Model pointer
 */
void BlockComponentAnalysis::setModel(Model& aModel)
{
	// SET THE MODEL IN THE BASE CLASS
	Super::setModel(aModel);

	// UPDATE VARIABLES IN THIS CLASS
	constructDescription();
	constructColumnLabels();
	setupStorage();
}

/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int BlockComponentAnalysis::begin(SimTK::State& s)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	m_storage.reset(s.getTime());

	// RECORD
	int status = 0;
	if(m_storage.getSize()<=0) {
		status = record(s);
	}

	return(status);
}

/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int BlockComponentAnalysis::step(const SimTK::State& s, int stepNumber)
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}

/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
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

/**
* Compute and record the results.
*
* This method, for the purpose of example, records the position and
* orientation of each body in the model.  You will need to customize it
* to perform your analysis.
*
* @param aT Current time in the simulation.
* @param aX Current values of the controls.
* @param aY Current values of the states: includes generalized coords and speeds
*/
int BlockComponentAnalysis::record(const SimTK::State& s)
{
	_model->getMultibodySystem().realize(s, SimTK::Stage::Report);

	//append storage
	Array<double> data;
	ComponentSet& component_set = _model->updMiscModelComponentSet();
	for (int i = 0; i<component_set.getSize(); i++)
	{
		data.append(((BlockComponent&) component_set[i]).
			getInputValue<double>(s, BlockComponent::INPUT));
		data.append(((BlockComponent&) component_set[i]).
			getOutputValue<double>(s, BlockComponent::OUTPUT));
	}

	m_storage.append(s.getTime(), data.size(), &data[0]);

	return(0);
}

void BlockComponentAnalysis::constructDescription()
{
	string descrip;

	descrip = "\nThis file contains tibia displacement and acting force.\n";
	descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
	if (getInDegrees()) {
		descrip += "\nAngles are in degrees.";
	}
	else {
		descrip += "\nAngles are in radians.";
	}
	descrip += "\n\n";

	setDescription(descrip);
}

/**
* Construct column labels for the output results.
*
* For analyses that run during a simulation, the first column is almost
* always time.  For the purpose of example, the code below adds labels
* appropriate for recording the translation and orientation of each
* body in the model.
*
* This method needs to be called as necessary to update the column labels.
*/
void BlockComponentAnalysis::constructColumnLabels()
{
	if (_model == NULL) return;

	Array<string> labels;
	labels.append("time");

	ComponentSet& component_set = _model->updMiscModelComponentSet();
	for (int i = 0; i<component_set.getSize(); i++)
	{
		labels.append(component_set[i].getName() + "_input");
		labels.append(component_set[i].getName() + "_output");
	}

	setColumnLabels(labels);
}

/**
* Set up storage objects.
*
* In general, the storage objects in your analysis are used to record
* the results of your analysis and write them to file.  You will often
* have a number of storage objects, each for recording a different
* kind of result.
*/
void BlockComponentAnalysis::setupStorage()
{
	m_storage.reset(0);
	m_storage.setName("custom_analysis");
	m_storage.setDescription(getDescription());
	m_storage.setColumnLabels(getColumnLabels());
}

/**
* Print results.
*
* The file names are constructed as
* aDir + "/" + aBaseName + "_" + ComponentName + aExtension
*
* @param aDir Directory in which the results reside.
* @param aBaseName Base file name.
* @param aDT Desired time interval between adjacent storage vectors.  Linear
* interpolation is used to print the data out at the desired interval.
* @param aExtension File extension.
*
* @return 0 on success, -1 on error.
*/
int BlockComponentAnalysis::printResults(const string &aBaseName, const string &aDir, double aDT,
	const string &aExtension)
{
	Storage::printResult(&m_storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

	return(0);
}

void BlockComponentAnalysis::setNull()
{

}

void BlockComponentAnalysis::constructProperties()
{

}