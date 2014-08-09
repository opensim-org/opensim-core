/* -------------------------------------------------------------------------- *
 *                         OpenSim:  DynamicsTool.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "DynamicsTool.h"
#include <string>
#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Storage.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
DynamicsTool::~DynamicsTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
DynamicsTool::DynamicsTool() : Tool(),
    _modelFileName(_modelFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblVec()),
    _excludedForces(_excludedForcesProp.getValueStrArray()),
    _externalLoadsFileName(_externalLoadsFileNameProp.getValueStr())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
DynamicsTool::DynamicsTool(const string &aFileName, bool aLoadModel) :
    Tool(aFileName, false),
    _modelFileName(_modelFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblVec()),
    _excludedForces(_excludedForcesProp.getValueStrArray()),
    _externalLoadsFileName(_externalLoadsFileNameProp.getValueStr())
{
    setNull();
    updateFromXMLDocument();

    if(aLoadModel) {
        //loadModel(aFileName);
    }
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTool Object to be copied.

 */
DynamicsTool::DynamicsTool(const DynamicsTool &aTool) : Tool(aTool),
    _modelFileName(_modelFileNameProp.getValueStr()),
    _timeRange(_timeRangeProp.getValueDblVec()),
    _excludedForces(_excludedForcesProp.getValueStrArray()),
    _externalLoadsFileName(_externalLoadsFileNameProp.getValueStr())
{
    setNull();
    *this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void DynamicsTool::setNull()
{
    setupProperties();
    _model = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void DynamicsTool::setupProperties()
{
    _modelFileNameProp.setComment("Name of the .osim file used to construct a model.");
    _modelFileNameProp.setName("model_file");
    _propertySet.append( &_modelFileNameProp );

    SimTK::Vec2  defaultTimeRange(-std::numeric_limits<SimTK::Real>::infinity(), std::numeric_limits<SimTK::Real>::infinity());
    _timeRangeProp.setComment("Time range over which the inverse dynamics problem is solved.");
    _timeRangeProp.setName("time_range");
    _timeRangeProp.setValue(defaultTimeRange);
    _propertySet.append(&_timeRangeProp);

    _excludedForcesProp.setComment("List of forces by individual or grouping name (e.g. All, actuators, muscles, ...)"
                                   " to be excluded when computing model dynamics.");
    _excludedForcesProp.setName("forces_to_exclude");
    _propertySet.append(&_excludedForcesProp);

    string comment = "XML file (.xml) containing the external loads applied to the model as a set of ExternalForce(s).";
    _externalLoadsFileNameProp.setComment(comment);
    _externalLoadsFileNameProp.setName("external_loads_file");
    _propertySet.append( &_externalLoadsFileNameProp );

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
DynamicsTool& DynamicsTool::operator=(const DynamicsTool &aTool)
{
    // BASE CLASS
    Tool::operator=(aTool);

    // MEMBER VARIABLES
    _modelFileName = aTool._modelFileName;
    _timeRange = aTool._timeRange;
    _excludedForces = aTool._excludedForces;
    _externalLoadsFileName = aTool._externalLoadsFileName;

    return(*this);
}

/** Modify model to exclude specified forces by disabling those identified by name or group */
void DynamicsTool::disableModelForces(Model &model, SimTK::State &s, const Array<std::string> &forcesByNameOrGroup)
{
    ForceSet &modelForces = model.updForceSet();
    Array<string> groupNames;
    modelForces.getGroupNames(groupNames);

    /* The search for inidividual group or force names IS case-sensitive BUT keywords are not*/
    for(int i=0; i<forcesByNameOrGroup.getSize(); i++) {
        //Check for kewords first starting with ALL
        if(IO::Uppercase(forcesByNameOrGroup[i]) == "ALL") {
            for(int i=0; i<modelForces.getSize(); i++) {
                modelForces[i].setDisabled(s, true);
            }
            return;
        }
        if(IO::Uppercase(forcesByNameOrGroup[i]) == "ACTUATORS") {
            Set<Actuator> &acts = model.updActuators();
            for(int i=0; i<acts.getSize(); i++) {
                acts[i].setDisabled(s, true);
            }
            continue;
        }
        if(IO::Uppercase(forcesByNameOrGroup[i]) == "MUSCLES") {
            Set<Muscle> &muscles = model.updMuscles();
            for(int i=0; i<muscles.getSize(); i++) {
                muscles[i].setDisabled(s, true);
            }
            continue;
        }

        // index result when a name is not a force or group name for forces in the model
        int k = -1;
        // It is possible to set ACTUATORS and/or MUSCLES and other forces by name or group
        // So check what else is in the list.
        // see if name is a group name first
        if(groupNames.getSize() > 0) {
            k = groupNames.findIndex(forcesByNameOrGroup[i]);
            if(k > -1) { //found
                const ObjectGroup* group = modelForces.getGroup(k);
                Array<Object*> members = group->getMembers();
                for(int j=0; j<members.getSize(); j++)
                    ((Force *)(members[j]))->setDisabled(s, true);
            }
        } //otherwise, check for individual forces
        else {
            k = modelForces.getIndex(forcesByNameOrGroup[i]);
            if(k > -1) { //found
                modelForces[k].setDisabled(s, true);
            }
        }
        // No force or group was found
        if(k < 0)
            cout << "\nWARNING: Tool could not find force or group named '" << forcesByNameOrGroup[i] << "' to be excluded." << endl;

    }
}

bool DynamicsTool::createExternalLoads( const string& aExternalLoadsFileName, Model& aModel, const Storage *loadKinematics)
{
    if(aExternalLoadsFileName==""||aExternalLoadsFileName=="Unassigned") {
        cout<<"No external loads will be applied (external loads file not specified)."<<endl;
        return false;
    }

    // This is required so that the references to other files inside ExternalLoads file are interpretted
    // as relative paths
    std::string savedCwd = IO::getCwd();
    IO::chDir(IO::getParentDirectory(aExternalLoadsFileName));
    // Create external forces
    try {
        _externalLoads = ExternalLoads(aModel, aExternalLoadsFileName);
    }
    catch (const Exception& ex) {
        // Important to catch exceptions here so we can restore current working directory...
        // And then we can rethrow the exception
        cout << "Error: failed to construct ExternalLoads from file " << aExternalLoadsFileName
             << ". Please make sure the file exists and that it contains an ExternalLoads object or create a fresh one." << endl;
        if(getDocument()) IO::chDir(savedCwd);
        throw(ex);
    }
    _externalLoads.setMemoryOwner(false);
    _externalLoads.invokeConnectToModel(aModel);

    string loadKinematicsFileName = _externalLoads.getExternalLoadsModelKinematicsFileName();

    const Storage *loadKinematicsForPointTransformation = NULL;

    //If the the Tool is already loading the storage allow it to pass it in for use rather than reloading and processing
    if(loadKinematics && loadKinematics->getName() == loadKinematicsFileName) {
        loadKinematicsForPointTransformation = loadKinematics;
    }
    else {
        IO::TrimLeadingWhitespace(loadKinematicsFileName);
        Storage *temp = NULL;
        // fine if there are no kinematics as long as it was not assigned
        if(!(loadKinematicsFileName == "") && !(loadKinematicsFileName == "Unassigned")) {
            temp = new Storage(loadKinematicsFileName);
            if(!temp) {
                IO::chDir(savedCwd);
                throw Exception("DynamicsTool: could not find external loads kinematics file '"+loadKinematicsFileName+"'.");
            }
        }
        // if loading the data, do whatever filtering operations are also specified
        if(temp && _externalLoads.getLowpassCutoffFrequencyForLoadKinematics() >= 0) {
            cout<<"\n\nLow-pass filtering coordinates data with a cutoff frequency of "<<_externalLoads.getLowpassCutoffFrequencyForLoadKinematics()<<"."<<endl;
            temp->pad(temp->getSize()/2);
            temp->lowpassIIR(_externalLoads.getLowpassCutoffFrequencyForLoadKinematics());
        }
        loadKinematicsForPointTransformation = temp;
    }

    // if load kinematics for performing re-expressing the point of application is provided
    // then perform the transformations
    if(loadKinematicsForPointTransformation) {
        SimTK::State& s = aModel.initSystem();

        // Form complete storage so that the kinematics match the state labels/ordering
        Storage *qStore=NULL;
        Storage *uStore=NULL;
        aModel.getSimbodyEngine().formCompleteStorages(s, *loadKinematicsForPointTransformation,qStore,uStore);
        // qStore should be in radians
        if (qStore->isInDegrees()) {
            aModel.getSimbodyEngine().convertDegreesToRadians(*qStore);
        }
        _externalLoads.transformPointsExpressedInGroundToAppliedBodies(*qStore, _timeRange[0], _timeRange[1]);
        delete qStore;
        delete uStore;
    }

    // Add external loads to the set of all model forces
    for(int i=0; i<_externalLoads.getSize(); ++i) {
        aModel.updForceSet().adoptAndAppend(&_externalLoads[i]);
    }

    if(!loadKinematics)
        delete loadKinematics;

    IO::chDir(savedCwd);
    return(true);
}
