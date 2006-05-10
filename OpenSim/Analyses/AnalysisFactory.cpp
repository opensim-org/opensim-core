// AnalysisFactory
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author:  
 */
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "AnalysisFactory.h"
#include "Actuation.h"
#include "Kinematics.h"
#include "IndAcc.h"
#include "GeneralizedForces.h"



using namespace OpenSim;
using namespace std;
//=============================================================================
// STATICS
//=============================================================================
AnalysisFactory *AnalysisFactory::_factoryInstance=0;
std::string AnalysisFactory::_displayName = "Analysis Manager";

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * clears the list of available analyses
 */
AnalysisFactory::AnalysisFactory(Model *aModel)
{
	_analysesList.setSize(0);
	Analysis *analysis = new Actuation(aModel);
	analysis->setOn(false);
	registerAnalysis(analysis);
	analysis = new Kinematics(aModel);
	analysis->setOn(false);
	registerAnalysis(analysis);
	analysis = new GeneralizedForces(aModel);
	analysis->setOn(false);
	registerAnalysis(analysis);
}

//_____________________________________________________________________________
/**
 * Destructor.
 *
 * erases the list of available analyses and frees up memory used by them
 */
AnalysisFactory::~AnalysisFactory()
{
}
//_____________________________________________________________________________
/**
 * getInstance
 *
 * Method to obtain the sole instance of AnalysisFactory. Uses lazy evaluation
 * so that the AnalysisFactory is created only on demand. If we decide to have 
 * more than one instance of the AnalysisFactory this's the place to change.
 * Clients of this class get access to AnalysisFactory only through the call
 * to the static method AnalysisFactory::getInstance()
 */
AnalysisFactory*	AnalysisFactory::getInstance(Model *aModel)
{
	if (_factoryInstance==0){
		_factoryInstance = new AnalysisFactory(aModel);
	}
	return _factoryInstance;
}
//_____________________________________________________________________________
/**
 * createAnalysis
 *
 * @parm aAnalysisName is the name of the analysis to construct
 *
 * @return a pointer to a newly instantiated Analysis with name
 * equal to aAnalysisName or 0 if the analysis is not found.
 * 
 * CALLER IS REPONSIBLE FOR FREEING UP RETURNED ANALYSIS
 */
Analysis*  AnalysisFactory::createAnalysis(const string &aAnalysisType) const
{
	bool found = false;
	Analysis *retAnalysis=0;

	for(int i=0; i < _analysesList.getSize() && !found; i++){
		const Analysis *nextAnalysis = _analysesList.get(i);
		if (nextAnalysis->getType()==aAnalysisType){
			found=true;
			retAnalysis = (Analysis *)nextAnalysis->copy();
		}
	}
	return retAnalysis;
}

//_____________________________________________________________________________
/**
 * analysisExists
 *
 * @parm aAnalysisName is the Name of the analysis to construct
 *
 * @return a pointer to a newly instantiated Analysis with name
 * (case insitive) equal to aAnalysisName or 0 if the analysis is not found.
 * 
 */
bool  AnalysisFactory::analysisExists(const std::string &aAnalysisName) const
{
	bool found = false;

	for(int i=0; i < _analysesList.getSize() && !found; i++){
		found=(_analysesList[i]->getName()==aAnalysisName);
	}
	return found;
}


//_____________________________________________________________________________
/**
 * registerAnalysis registers an instance of an Analysis for future use by
 * the framework.
 *
 * @parm aAnalysis pointer to analysis to be registered
 *
 * @return bool on success of registration, false otherwise. Main reason registration 
 * can fail is if the name already exists
 */
bool AnalysisFactory::registerAnalysis(Analysis *aAnalysis)
{
	const std::string& analysisName = aAnalysis->getType();
	bool success=false;
	if (analysisExists(analysisName)==false){
		_analysesList.append(aAnalysis);
		success=true;
	}
	return success;
}
//_____________________________________________________________________________
/**
 * getRegisteredAnalyses returns an const ref to the set of Analysis registered with
 * the factory.
 *
 *
 * Users should not change the list directly, they should use the registeration 
 * mechanism instead (that's why the return value is const)
 */
const ArrayPtrs<Analysis>& AnalysisFactory::
getRegisteredAnalyses() const
{
	return _analysesList;
}
//_____________________________________________________________________________
/**
 * Get the name of the factory
 *
 * @return name of AnalysisFactory
 */

const std::string& AnalysisFactory::
toString() const
{
	return (_displayName);
}
