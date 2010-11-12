// ForceReporter.cpp
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
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include "ForceReporter.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ForceReporter::~ForceReporter()
{
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an ForceReporter object for recording the Forces exerted on a model during a simulation.
 *
 * @param aModel Model for which the Forces are to be recorded.
 */
ForceReporter::ForceReporter(Model *aModel) : 	Analysis(aModel),
	_forceStore(1000,"ModelForces"),
	_includeConstraintForces(_includeConstraintForcesProp.getValueBool())
{
	// NULL
	setNull();

	// DESCRIPTION
	constructDescription();

	// STORAGE
	allocateStorage();

	// CHECK MODEL
	if(_model==NULL) return;
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
ForceReporter::ForceReporter(const std::string &aFileName): Analysis(aFileName, false),
	_forceStore(1000,"ModelForces"),
	_includeConstraintForces(_includeConstraintForcesProp.getValueBool())
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// DESCRIPTION
	constructDescription();

	// STORAGE
	allocateStorage();
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
ForceReporter::ForceReporter(const ForceReporter &aForceReporter):
	Analysis(aForceReporter),
	_forceStore(aForceReporter._forceStore),
	_includeConstraintForces(_includeConstraintForcesProp.getValueBool())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aForceReporter;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* ForceReporter::copy() const
{
	ForceReporter *object = new ForceReporter(*this);
	return(object);

}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void ForceReporter::setNull()
{
	// TYPE
	setType("ForceReporter");
	// NAME
	setName("ForceReporter");

	_includeConstraintForcesProp.setComment("Flag indicating whether to include forces due to constraints.");
	_includeConstraintForcesProp.setName("include_constraint_forces");
	_includeConstraintForcesProp.setValue(false);
	_propertySet.append( &_includeConstraintForcesProp );
}


//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
ForceReporter& ForceReporter::operator=(const ForceReporter &aForceReporter)
{
	// BASE CLASS
	Analysis::operator=(aForceReporter);

	// STORAGE
	deleteStorage();
	allocateStorage();

	_includeConstraintForces = aForceReporter._includeConstraintForces;

	return (*this);
}
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void ForceReporter::setModel(Model& aModel)
{
	// BASE CLASS
	Analysis::setModel(aModel);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the forces.
 */
void ForceReporter::allocateStorage()
{
	// ACCELERATIONS
	_forceStore.setDescription(getDescription());
	// Keep references o all storages in a list for uniform access from GUI
	_storageList.append(&_forceStore);
	_storageList.setMemoryOwner(false);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the ForceReporter files.
 */
void ForceReporter::constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis file contains the forces exerted on a model ");
	strcat(descrip,"during a simulation.\n");

	strcat(descrip,"\nA force is a generalized force, meaning that");
	strcat(descrip," it can be either a force (N) or a torque (Nm).\n");

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
 * Construct the column labels for the ForceReporter storage files.
 */
void ForceReporter::constructColumnLabels()
{
	if (_model)
	{
		// ASSIGN
		Array<string> columnLabels;
		columnLabels.append("time");
		int nf=_model->getForceSet().getSize();
		for(int i=0;i<nf;i++) {
			// If body force we need to record six values for torque+force
			// If muscle we record one scalar
			Array<string> forceLabels = _model->getForceSet().get(i).getRecordLabels();
			// If prescribed force we need to record point, 
			columnLabels.append(forceLabels);
		}
		if(_includeConstraintForces){
			int nc=_model->getConstraintSet().getSize();
			for(int i=0;i<nc;i++) {
				// Ask constraint how many columns and their names it reports
				Array<string> forceLabels = _model->getConstraintSet().get(i).getRecordLabels();
				// If prescribed force we need to record point, 
				columnLabels.append(forceLabels);
			}
		}
		_forceStore.setColumnLabels(columnLabels);
	}
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void ForceReporter::deleteStorage()
{
	//if(_forceStore!=NULL) { delete _forceStore;  _forceStore=NULL; }
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the ForceReporter quantities.
 */
int ForceReporter::record(const SimTK::State& s)
{
	if(_model==NULL) return(-1);

	// MAKE SURE ALL ForceReporter QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics );

	StateVector nextRow = StateVector(s.getTime());

	// NUMBER OF Forces
	const ForceSet& forces = _model->getForceSet(); // This does not contain gravity
	int nf = forces.getSize();

	for(int i=0;i<nf;i++) {
		// If body force we need to record six values for torque+force
		// If muscle we record one scalar
		OpenSim::Force& nextForce = (OpenSim::Force&)forces[i];
		Array<double> values = nextForce.getRecordValues(s);
		nextRow.getData().append(values);
	}

	if(_includeConstraintForces){
		// NUMBER OF Constraints
		const ConstraintSet& constraints = _model->getConstraintSet(); // This does not contain gravity
		int nc = constraints.getSize();

		for(int i=0;i<nc;i++) {
			OpenSim::Constraint& nextConstraint = (OpenSim::Constraint&)constraints[i];
			Array<double> values = nextConstraint.getRecordValues(s);
			nextRow.getData().append(values);
		}
	}
	_forceStore.append(nextRow);

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
int ForceReporter::
begin(SimTK::State& s)
{
	if(!proceed()) return(0);

	tidyForceNames();
	// LABELS
	constructColumnLabels();
	// RESET STORAGE
	_forceStore.reset(s.getTime());

	// RECORD
	int status = 0;
	if(_forceStore.getSize()<=0) {
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
int ForceReporter::
step(const SimTK::State& s, int stepNumber )
{
	if(!proceed( stepNumber )) return(0);

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
int ForceReporter::
end(SimTK::State& s )
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
int ForceReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("ForceReporter.printResults: Off- not printing.\n");
		return(0);
	}

	std::string prefix=aBaseName+"_"+getName()+"_";
	Storage::printResult(&_forceStore, prefix+"forces", aDir, aDT, aExtension);

	return(0);
}


void ForceReporter::tidyForceNames()
{
	OpenSim::Array<string> forceNames("");
	const ForceSet& forces = _model->getForceSet(); // This does not contain gravity
	forces.getNames(forceNames);
	// Make sure names are unique and non-empty. If empty assign names AutoForcexxx
	int nf = forceNames.getSize();
	for(int i=0; i<nf; i++){
		int j=1;
		if (forceNames[i]==""){
			bool validNameFound=false;
			char pad[100];
			// Make up a candidate name
			sprintf(pad, "%s%d", forces.get(i).getType().c_str(), j++);
			while(!validNameFound){
				validNameFound = (forceNames.findIndex(string(pad))== -1);
				if (!validNameFound){
					sprintf(pad, "Force%d", j++);
				}
			}
			string newName(pad);
			_model->updForceSet()[i].setName(newName);
			cout << "Changing blank name for force to " << newName << endl;
		}
	}
}
