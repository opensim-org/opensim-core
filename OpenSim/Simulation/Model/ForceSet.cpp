// ForceSet.cpp
// Authors: Frank C. Anderson, Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include <algorithm>
#include "ForceSet.h"
#include "Model.h"
#include "Muscle.h"
#include "OpenSimForceSubsystem.h"
#include <OpenSim/Simulation/Control/Controller.h>
#include "SimTKsimbody.h"
#include <OpenSim/Simulation/Model/PrescribedForce.h>

using namespace std;
using namespace OpenSim;

#ifndef SWIG
template class OSIMSIMULATION_API ModelComponentSet<Force>;
#endif


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ForceSet::~ForceSet()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ForceSet::ForceSet():
_dataFileName(_dataFileNameProp.getValueStr())
{
	setNull();
}

ForceSet::ForceSet(Model& model) : 
ModelComponentSet<Force>(model),
_dataFileName(_dataFileNameProp.getValueStr())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct an actuator set from file.
 *
 * @param aFileName Name of the file.
 */
ForceSet::ForceSet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode) :
	ModelComponentSet<Force>(model, aFileName, false),
	_dataFileName(_dataFileNameProp.getValueStr())
{
	setNull();

	if(aUpdateFromXMLNode) updateFromXMLNode();
	// removeInvalidObjects();
	setModel(model);
	setupFromXML();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForceSet ForceSet to be copied.
 */
ForceSet::ForceSet(const ForceSet &aForceSet) :
	ModelComponentSet<Force>(aForceSet),
	_dataFileName(_dataFileNameProp.getValueStr())
{
	setNull();

	// Class Members
	copyData(aForceSet);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this ForceSet to their null values.
 */
void ForceSet::setNull()
{
	// TYPE
	setType("ForceSet");
	// NAME
	//setName("ForceSet");

	// PROPERTIES
	setupSerializedMembers();

	_actuators.setMemoryOwner(false);
}

//_____________________________________________________________________________
/**
 * Copy this ForceSet and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ForceSet.
 */
Object* ForceSet::copy() const
{
	ForceSet *actSet = new ForceSet(*this);
	return(actSet);
}

//_____________________________________________________________________________
/**
 * Copy the member variables of the ForceSet.
 *
 * @param aAbsForceSet actuator set to be copied
 */
void ForceSet::copyData(const ForceSet &aAbsForceSet)
{
    // ACTUATORS
    _actuators = aAbsForceSet._actuators;
	_actuators.setMemoryOwner(false);
	_dataFileName = aAbsForceSet._dataFileName;
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ForceSet::
setupSerializedMembers()
{
	_dataFileNameProp.setName("datafile");
	_dataFileName="";
	_propertySet.append(&_dataFileNameProp);

}

void ForceSet::setup(Model& aModel)
{
	// BASE CLASS
	Set<Force>::setup();
	_model = &aModel;

    // INDICES
	updateActuators();
	for (int i = 0; i < getSize(); i++) {
		get(i).setup(aModel);
    }
}

void ForceSet::postInit(Model& aModel)
{
	for (int i = 0; i < getSize(); i++)
		get(i).postInit(aModel);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
ForceSet& ForceSet::
operator=(const ForceSet &aAbsForceSet)
{
	// BASE CLASS
	Set<Force>::operator=(aAbsForceSet);

	// Class Members
	copyData(aAbsForceSet);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// ACTUATOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Remove an actuator from the actuator set.
 *
 * @param aIndex Index of the actuator to be removed.
 * @return True if the remove was successful; false otherwise.
 */
bool ForceSet::
remove(int aIndex)
{
	bool success = Set<Force>::remove(aIndex);

	updateActuators();

	return(success);
}

//_____________________________________________________________________________
/**
 * Append an actuator on to the set.  A copy of the specified actuator
 * is not made.
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aActuator Pointer to the actuator to be appended.
 * @return True if successful; false otherwise.
 */
bool ForceSet::
append(Force *aActuator)
{
	bool success = ModelComponentSet<Force>::append(aActuator);


	if((success)&&(_model!=NULL)) {
		aActuator->setup(*_model);
		updateActuators();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Append actuators from an actuator set to this set.  Copies of the actuators are not made.
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aForceSet The set of actuators to be appended.
 * @param aAllowDuplicateNames If true, all actuators will be appended; If false, don't append actuators whose
 * name already exists in this model's actuator set.
 * @return True if successful; false otherwise.
 */
bool ForceSet::
append(ForceSet &aForceSet, bool aAllowDuplicateNames)
{
	bool success = true;
	for(int i=0;i<aForceSet.getSize() && success;i++) {
		bool nameExists = false;
		if(!aAllowDuplicateNames) {
			std::string name = aForceSet.get(i).getName();
			for(int j=0;j<getSize();j++) {
				if(get(j).getName() == name) {
					nameExists = true;
					break;
				}
			}
		}
		if(!nameExists) {
			if(!ModelComponentSet<Force>::append(&aForceSet.get(i))) success = false;
			// individual actuators keep pointers to model as well!! 
			aForceSet.get(i).setup(*_model);
		}
	}

	if(success) {
		updateActuators();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Set the actuator at an index.  A copy of the specified actuator is NOT made.
 * The actuator previously set a the index is removed (and deleted).
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aIndex Array index where the actuator is to be stored.  aIndex
 * should be in the range 0 <= aIndex <= getSize();
 * @param aActuator Pointer to the actuator to be set.
 * @return True if successful; false otherwise.
 */
bool ForceSet::
set(int aIndex,Force *aActuator)
{
	bool success = ModelComponentSet<Force>::set(aIndex,aActuator);

	if(success) {
		updateActuators();
	}

	return(success);
}

bool ForceSet::
insert(int aIndex, Force *aForce)
{
	bool success = ModelComponentSet<Force>::insert(aIndex, aForce);

	if(success) {
		updateActuators();
	}

	return(success);
}

//_____________________________________________________________________________
/**
 * Get the list of Actuators.
 */
const Set<Actuator>& ForceSet::
getActuators() const
{
    return _actuators;
}
Set<Actuator>& ForceSet::
updActuators() 
{
    return _actuators;
}
//_____________________________________________________________________________
/**
 * Rebuild the list of Actuators.
 */
void ForceSet::
updateActuators()
{
    _actuators.setSize(0);
    for (int i = 0; i < getSize(); ++i)
    {
        Actuator* act = dynamic_cast<Actuator*>(&get(i));
        if (act != NULL)  _actuators.append(act);
    }
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________

//_____________________________________________________________________________
/**
 * Compute a set of equlibirium states. Based on each actuator's curent set of
 * states, each actuator alters those states to satisfy some notion of
 * equilibrium. Each actuator is in charge of what it considers to be
 * equilibirum. For example, given a muscle activation, compute muscle fiber
 * length that is consistent with that activation level.
 */
void ForceSet::
computeEquilibrium(SimTK::State& s)
{
	int i;
	int size = getSize();
	Actuator *act;
	for(i=0;i<size;i++) {
		act = dynamic_cast<Actuator *>(&get(i));
		if(act) {
			act->computeActuation(s);
			act->computeEquilibrium(s);
		}
	}
}

/**
 *  Compute the time derivatives of the states that characterize the actuators.
 *   
 *   @param rDY Array of state derivatives.
 **/
void ForceSet::
computeStateDerivatives(const SimTK::State& s)
{
    int i;
    Actuator *act;
    for(i=0;i<getSize();i++) {
        act = dynamic_cast<Actuator *>(&get(i));
        if(act!=NULL) act->computeStateDerivatives( s );
    }
}


//_____________________________________________________________________________
/**
 * Get the names of the states of the actuators.
 *
 * @param rNames Array of names.
 */
void ForceSet::
getStateVariableNames(OpenSim::Array<std::string> &rNames) const
{
	for(int i=0;i<getSize();i++) {
		Actuator *act = dynamic_cast<Actuator*>(&get(i)); 
       
		if(act) {
            for( int j=0;j<act->getNumStateVariables(); j++ )rNames.append(act->getStateVariableName(j));
        }
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that all actuators are valid.
 */
bool ForceSet::
check() const
{
	bool status=true;

	// LOOP THROUGH ACTUATORS
	Actuator *act;
	int size = getSize();
	for(int i=0;i<size;i++) {
		act = dynamic_cast<Actuator *>(&get(i));
		if(!act) continue;
		if(!act->check()) status = false;
	}

	return(status);
}

//_____________________________________________________________________________
/**
 * Create a set of prescribed forces from a file.
 * Assumptions:
 *  1. instance variable _dataFileName is set already
 *  2. Sizes of the three arrays startForceColumns, bodyNames, columnCount is identical
 *  3. columnCount is 
 *						9: Force, Point, Torque 
 *						6: Force, Point
 */
void ForceSet::createForcesFromFile(const std::string& fileName,
									Array<std::string>& startForceColumns, 
									Array<int>& columnCount,
									Array<std::string>& bodyNames)
{
	_dataFileName=fileName;
	assert(_dataFileName!="");

	Storage kineticsStore(_dataFileName);
	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;

	assert(startForceColumns.getSize()==columnCount.getSize());
	assert(startForceColumns.getSize()==bodyNames.getSize());
	double *t=0;
	kineticsStore.getTimeColumn(t);
	const Array<string>& lbls=kineticsStore.getColumnLabels();

	if (startForceColumns.getSize()==0){	
		// User didn't specify,
		// We'll assume 9 columns for force, point, torque
		int nColumns = lbls.getSize()-1;
		int nForces = nColumns/9;
		for(int i=0; i<nForces; i++){
			startForceColumns.append(lbls[i*9+1]);
			columnCount.append(9);
			bodyNames.append("ground");
		}
	}
	// Make sure that column names are unique, otherwise assert
	// Sort names and then do one pass for more intelligent name finding.
	// This need to be made more efficient using some stl sorting implementation
	bool duplicateIsFound = false;
	std::string duplicateName="";
	for(int i=0; i< startForceColumns.getSize()-1 && !duplicateIsFound; i++){
		for(int j=i+1; j< startForceColumns.getSize()-1 && !duplicateIsFound; i++){
			duplicateIsFound = (startForceColumns.get(i)==startForceColumns.get(j));
			if (duplicateIsFound) duplicateName = startForceColumns.get(i);
		}
	}
	if (duplicateIsFound){
		string msg = "Create forces from file "+_dataFileName+", duplicate column "
			+duplicateName+" found.\nOperation is aborted, please have unique column names and retry.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	{
		double *column1=0;
		double *column2=0;
		double *column3=0;

		for(int i=0; i< startForceColumns.getSize(); i++){
			std::string labelX = startForceColumns[i];
			int storageIndex=lbls.findIndex(labelX);

			kineticsStore.getDataColumn(labelX, column1);
			std::string labelY = lbls[storageIndex+1];
			kineticsStore.getDataColumn(labelY, column2);
			std::string labelZ = lbls[storageIndex+2];
			kineticsStore.getDataColumn(labelZ, column3);

			PrescribedForce* pf= new PrescribedForce();
			pf->setBodyName(bodyNames[i]);
			// If columnCount is 3 then create the forces and we're done
			NaturalCubicSpline* spline1= new NaturalCubicSpline(forceSize, t, column1, labelX);
			NaturalCubicSpline* spline2= new NaturalCubicSpline(forceSize, t, column2, labelY);
			NaturalCubicSpline* spline3= new NaturalCubicSpline(forceSize, t, column3, labelZ);
			delete column1, column2, column3;
			column1 = column2 = column3 = 0;
			//
			pf->setForceFunctions(spline1, spline2, spline3);	// Copies of these functions are made 

			if (columnCount[i]>=6){
				std::string labelX = lbls[storageIndex+3];
				kineticsStore.getDataColumn(labelX, column1);
				std::string labelY = lbls[storageIndex+4];
				kineticsStore.getDataColumn(labelY, column2);
				std::string labelZ = lbls[storageIndex+5];
				kineticsStore.getDataColumn(labelZ, column3);

				// If columnCount is 3 then create the forces and we're done
				NaturalCubicSpline* spline1= new NaturalCubicSpline(forceSize, t, column1, labelX);
				NaturalCubicSpline* spline2= new NaturalCubicSpline(forceSize, t, column2, labelY);
				NaturalCubicSpline* spline3= new NaturalCubicSpline(forceSize, t, column3, labelZ);

				delete column1, column2, column3;
				column1 = column2 = column3 = 0;
				pf->setPointFunctions(spline1, spline2, spline3); 
			}
			if (columnCount[i]==9){
				std::string labelX = lbls[storageIndex+6];
				kineticsStore.getDataColumn(labelX, column1);
				std::string labelY = lbls[storageIndex+7];
				kineticsStore.getDataColumn(labelY, column2);
				std::string labelZ = lbls[storageIndex+8];
				kineticsStore.getDataColumn(labelZ, column3);

				// If columnCount is 3 then create the forces and we're done
				NaturalCubicSpline* spline1= new NaturalCubicSpline(forceSize, t, column1, labelX);
				NaturalCubicSpline* spline2= new NaturalCubicSpline(forceSize, t, column2, labelY);
				NaturalCubicSpline* spline3= new NaturalCubicSpline(forceSize, t, column3, labelZ);

				delete column1, column2, column3;
				column1 = column2 = column3 = 0;
				pf->setTorqueFunctions(spline1, spline2, spline3); 
			}
			append(pf);
		}
	}
	print("externalForces.xml");
}
