#ifndef BLOCK_COMPONENT_ANALYSIS_H
#define BLOCK_COMPONENT_ANALYSIS_H

#include <OpenSim/Simulation/Model/Analysis.h>

namespace OpenSim { 

/**
*	Keeps track of the input/output of each BlockComponents.
*
* @author Dimitar Stanev
*/
class BlockComponentAnalysis : public OpenSim::Analysis 
{
	OpenSim_DECLARE_CONCRETE_OBJECT(OpenSim::BlockComponentAnalysis, OpenSim::Analysis);
public:

	BlockComponentAnalysis(OpenSim::Model* model);

	
	virtual void setModel(OpenSim::Model& aModel);

	virtual int begin(SimTK::State& s);

	virtual int step(const SimTK::State& s, int stepNumber);

	virtual int	end(SimTK::State& s);


	virtual int print(const std::string &path);
	
	virtual int printResults(
		const std::string &aBaseName, const std::string &aDir="",
		double aDT=-1.0, const std::string &aExtension=".sto");


protected:

	int record(const SimTK::State& s);

private:

	OpenSim::Storage m_storage;

	void constructDescription();

	void constructColumnLabels();

	void setupStorage();

}; // end of class 
}; // end of namespace 


#endif
