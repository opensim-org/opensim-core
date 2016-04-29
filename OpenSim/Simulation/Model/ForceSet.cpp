/* -------------------------------------------------------------------------- *
 *                           OpenSim:  ForceSet.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Jack Middleton                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


//=============================================================================
// INCLUDES
//=============================================================================



using namespace std;
using namespace OpenSim;

#ifndef SWIG
namespace OpenSim {
template class OSIMSIMULATION_API ModelComponentSet<Force>;
}
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
ForceSet::ForceSet()
{
    setNull();
}

ForceSet::ForceSet(Model& model) : 
ModelComponentSet<Force>(model)
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
    ModelComponentSet<Force>(model, aFileName, false)
{
    setNull();

    if(aUpdateFromXMLNode)
        updateFromXMLDocument();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForceSet ForceSet to be copied.
 */
ForceSet::ForceSet(const ForceSet &aForceSet) :
    ModelComponentSet<Force>(aForceSet)
{
    setNull();

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
    setAuthors("Ajay Seth, Jack Middleton");

    // PROPERTIES
    setupSerializedMembers();

    _actuators.setMemoryOwner(false);

    _muscles.setMemoryOwner(false);
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ForceSet::setupSerializedMembers()
{
}

void ForceSet::invokeConnectToModel(Model& aModel)
{
    // BASE CLASS
    Super::invokeConnectToModel(aModel);

    updateActuators();
    updateMuscles();
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
ForceSet& ForceSet::operator=(const ForceSet &aAbsForceSet)
{
    // BASE CLASS
    Set<Force>::operator=(aAbsForceSet);

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
bool ForceSet::remove(int aIndex)
{
    bool success = Set<Force>::remove(aIndex);

    updateActuators();
    updateMuscles();

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
append(Force *aForce)
{
    bool success = ModelComponentSet<Force>::adoptAndAppend(aForce);

    if((success)&&(&updModel()!=NULL)) {
        updateActuators();
        updateMuscles();
    }

    return(success);
}
//_____________________________________________________________________________
/**
 * Append an actuator on to the set.  A copy of the specified actuator
 * is made.
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aActuator reference to the actuator to be appended.
 * @return True if successful; false otherwise.
 */
bool ForceSet::
append(Force &aForce)
{
    bool success = ModelComponentSet<Force>::cloneAndAppend(aForce);


    if ((success) && (&getModel() != NULL)) {
        updateActuators();
        updateMuscles();
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
bool ForceSet::append(ForceSet &aForceSet, bool aAllowDuplicateNames)
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
            if(!ModelComponentSet<Force>::adoptAndAppend(&aForceSet.get(i))) 
                success = false;
        }
    }

    if(success) {
        updateActuators();
        updateMuscles();
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
bool ForceSet::set(int aIndex,Force *aActuator)
{
    bool success = ModelComponentSet<Force>::set(aIndex,aActuator);

    if(success) {
        updateActuators();
        updateMuscles();
    }

    return(success);
}

bool ForceSet::insert(int aIndex, Force *aForce)
{
    bool success = ModelComponentSet<Force>::insert(aIndex, aForce);

    if(success) {
        updateActuators();
        updateMuscles();
    }

    return(success);
}

//_____________________________________________________________________________
/**
 * Get the list of Actuators.
 */
const Set<Actuator>& ForceSet::getActuators() const
{
    return _actuators;
}

Set<Actuator>& ForceSet::updActuators() 
{
    updateActuators();
    return _actuators;
}
//_____________________________________________________________________________
/**
 * Rebuild the list of Actuators.
 */
void ForceSet::updateActuators()
{
    _actuators.setSize(0);
    for (int i = 0; i < getSize(); ++i)
    {
        Actuator* act = dynamic_cast<Actuator*>(&get(i));
        if (act != NULL)  _actuators.adoptAndAppend(act);
    }
}

//=============================================================================
//_____________________________________________________________________________
/**
 * Get the list of Muscles.
 */
const Set<Muscle>& ForceSet::getMuscles() const
{
    return _muscles;
}
Set<Muscle>& ForceSet::updMuscles() 
{
    updateMuscles();
    return _muscles;
}
//_____________________________________________________________________________
/**
 * Rebuild the list of Muscles.
 */
void ForceSet::updateMuscles()
{
    _muscles.setSize(0);
    for (int i = 0; i < getSize(); ++i)
    {
        Muscle* m = dynamic_cast<Muscle*>(&get(i));
        if (m != NULL)  _muscles.adoptAndAppend(m);
    }
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________


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
        ScalarActuator *act = dynamic_cast<ScalarActuator*>(&get(i));
       
        if(act) {
            rNames.append(act->getStateVariableNames());
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
    ScalarActuator *act;
    int size = getSize();
    for(int i=0;i<size;i++) {
        act = dynamic_cast<ScalarActuator *>(&get(i));
        if(!act) continue;
    }

    return(status);
}
