// StatesReporter.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Simulation/Model/Model.h>
#include "StatesReporter.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
StatesReporter::~StatesReporter()
{
}
//_____________________________________________________________________________
/**
 * Construct an StatesReporter object for recording the states of the model during a simulation.
 *
 * @param aModel Model for which the states are to be recorded.
 */
StatesReporter::StatesReporter(Model *aModel) :
	Analysis(aModel),
	_statesStore(1000,"ModelStates")
{
	// NULL
	setNull();

	// DESCRIPTION
	constructDescription();
}
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
StatesReporter::StatesReporter(const std::string &aFileName):
Analysis(aFileName, false),
_statesStore(1000,"ModelStates")
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// DESCRIPTION
	constructDescription();

	// STORAGE
	setupStorage();
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
StatesReporter::StatesReporter(const StatesReporter &aStatesReporter):
Analysis(aStatesReporter),
_statesStore(aStatesReporter._statesStore)
{
	setNull();
	// COPY TYPE AND NAME
	*this = aStatesReporter;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* StatesReporter::copy() const
{
	StatesReporter *object = new StatesReporter(*this);
	return(object);

}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void StatesReporter::
setNull()
{
	// TYPE
	setType("StatesReporter");
	// NAME
	setName("StatesReporter");
}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
StatesReporter& StatesReporter::operator=(const StatesReporter &aStatesReporter)
{
	// BASE CLASS
	Analysis::operator=(aStatesReporter);

	// STORAGE
	setupStorage();

	return (*this);
}
//_____________________________________________________________________________
/**
 * Allocate storage for the forces.
 */
void StatesReporter::
setupStorage()
{
	// ACCELERATIONS
	_statesStore.setDescription(getDescription());
	// Keep references o all storages in a list for uniform access from GUI
	_storageList.append(&_statesStore);
	_storageList.setMemoryOwner(false);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the StatesReporter files.
 */
void StatesReporter::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis file contains the states of a model ");
	strcat(descrip,"during a simulation.\n");
	strcat(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
	if(getInDegrees()) {
		strcat(descrip,"\nAngles are in degrees.");
	} else {
		strcat(descrip,"\nAngles are in radians.");
	}
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the column labels for the StatesReporter storage files.
 */
void StatesReporter::
constructColumnLabels()
{
	if (_model)
	{
		// ASSIGN
		Array<string> columnLabels;
		_model->getStateNames(columnLabels);
		columnLabels.insert(0, "time");		
		_statesStore.setColumnLabels(columnLabels);
	}
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the StatesReporter quantities.
 */
int StatesReporter::
record(const SimTK::State& s)
{
 
	if(_model==NULL) return(-1);

	// MAKE SURE ALL StatesReporter QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity );

	StateVector nextRow = StateVector(s.getTime());
	OpenSim::Array<double> StateValues;
	_model->getStateValues(s, StateValues);
	nextRow.getData().append(StateValues);
	_statesStore.append(nextRow);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param state system State
 * @param aStep Step number of the integration.
 *
 * @return -1 on error, 0 otherwise.
 */
int StatesReporter::
begin( SimTK::State& s)
{
	if(!proceed()) return(0);
	// LABELS
	constructColumnLabels();
	// RESET STORAGE
	_statesStore.reset(s.getTime());

	// RECORD
	int status = 0;
	if(_statesStore.getSize()<=0) {
		status = record(s);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called 
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param state System state
 *
 * @return -1 on error, 0 otherwise.
 */
int StatesReporter::
step(const SimTK::State& s, int stepNumber )
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration 
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 * @param state System state
 *
 * @return -1 on error, 0 otherwise.
 */
int StatesReporter::
end( SimTK::State& s )
{
	if (!proceed()) return 0;

	record(s);

	return(0);
}




//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
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
int StatesReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("StatesReporter.printResults: Off- not printing.\n");
		return(0);
	}

	std::string prefix=aBaseName+"_"+getName()+"_";
	Storage::printResult(&_statesStore, prefix+"states", aDir, aDT, aExtension);

	return(0);
}
