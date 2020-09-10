/* -------------------------------------------------------------------------- *
 *                         OpenSim:  DynamicsTool.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>
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

    _excludedForcesProp.setComment(
            "List of forces by individual or grouping name "
            "(e.g. All, actuators, muscles, ...)"
            " to be excluded when computing model dynamics. "
            "'All' also excludes external loads added "
            "via 'external_loads_file'.");
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

    /* The search for individual group or force names IS case-sensitive BUT keywords are not*/
    for(int i=0; i<forcesByNameOrGroup.getSize(); i++){
        //Check for keywords first starting with ALL
        if(IO::Uppercase(forcesByNameOrGroup[i]) == "ALL"){
            for(int i=0; i<modelForces.getSize(); i++){
                modelForces[i].setAppliesForce(s, false);
            }
            return;
        }
        if(IO::Uppercase(forcesByNameOrGroup[i]) == "ACTUATORS"){
            Set<Actuator> &acts = model.updActuators();
            for(int i=0; i<acts.getSize(); i++){
                acts[i].setAppliesForce(s, false);
            }
            continue;
        }
        if(IO::Uppercase(forcesByNameOrGroup[i]) == "MUSCLES"){
            Set<Muscle> &muscles = model.updMuscles();
            for(int i=0; i<muscles.getSize(); i++){
                muscles[i].setAppliesForce(s, false);
            }
            continue;
        }

        // index result when a name is not a force or group name for forces in the model
        int k = -1;  
        // It is possible to set ACTUATORS and/or MUSCLES and other forces by name or group
        // So check what else is in the list.
        // see if name is a group name first    
        if(groupNames.getSize() > 0){
            k = groupNames.findIndex(forcesByNameOrGroup[i]);
            if(k > -1){ //found
                const ObjectGroup* group = modelForces.getGroup(k);
                Array<const Object*> members = group->getMembers();
                for(int j=0; j<members.getSize(); j++)
                    ((Force *)(members[j]))->setAppliesForce(s, false);
            }
        } //otherwise, check for individual forces
        else{
            k = modelForces.getIndex(forcesByNameOrGroup[i]);
            if(k > -1){ //found
                modelForces[k].setAppliesForce(s, false);
            }
        }
        // No force or group was found
        if(k < 0) {
            log_warn("Could not find force or group named '{}' to be excluded.", 
                forcesByNameOrGroup[i]);
        }
    }
}


// NOTE: The implementation here should be verbatim that of AbstractTool::
// createExternalLoads to ensure consistent behavior of Tools in the GUI
// TODO: Unify the code bases.
bool DynamicsTool::createExternalLoads( const string& aExternalLoadsFileName,
                                        Model& aModel)
{
    if(aExternalLoadsFileName == "" || aExternalLoadsFileName == "Unassigned") {
        log_info("No external loads will be applied (external loads file not "
                 "specified).");
        return false;
    }

    Model copyModel = aModel;
    // speedup realize position calculations by removing all force elements
    // including muscles whose path calculations are most intensive
    copyModel.updForceSet().clearAndDestroy();
    copyModel.updControllerSet().clearAndDestroy();

    // Create external forces
    ExternalLoads* externalLoads = nullptr;
    try {
        externalLoads = new ExternalLoads(aExternalLoadsFileName, true);
        copyModel.addModelComponent(externalLoads);
    }
    catch (const Exception &ex) {
        // Important to catch exceptions here so we can restore current working directory...
        // And then we can re-throw the exception
        log_error("Failed to construct ExternalLoads from file '{}'. Please "
                  "make sure the file exists and that it contains an "
                  "ExternalLoads object or create a fresh one.", 
            aExternalLoadsFileName);
        throw(ex);
    }

    //Now add the ExternalLoads (transformed or not) to the Model to be analyzed
    ExternalLoads* exLoadsClone = externalLoads->clone();
    aModel.addModelComponent(exLoadsClone);

    // copy over created external loads to the external loads owned by the tool
    _externalLoads = *externalLoads;
    // tool holds on to a reference of the external loads in the model so it can
    // be removed afterwards
    _modelExternalLoads = exLoadsClone;

    return true;
}

void DynamicsTool::removeExternalLoadsFromModel()
{
    // If ExternalLoads were added to the model by the Tool, then remove them
    if (modelHasExternalLoads()) {
        _model->updMiscModelComponentSet().remove(_modelExternalLoads.release());
    }
}
