/* -------------------------------------------------------------------------- *
 *                        OpenSim:  JointReaction.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers, Ajay Seth                                       *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "JointReaction.h"

using namespace OpenSim;
using namespace std;
using namespace SimTK;


//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 * Delete any variables allocated using the "new" operator.  You will not
 * necessarily have any of these.
 */
JointReaction::~JointReaction()
{

}
//_____________________________________________________________________________
/**
 * Construct a JointReaction instance.
 *
 * @param aModel Model for which the analysis is to be run.
 */
JointReaction::JointReaction(Model *aModel) :
    Analysis(aModel),
    _forcesFileName(_forcesFileNameProp.getValueStr()),
    _jointNames(_jointNamesProp.getValueStrArray()),
    _onBody(_onBodyProp.getValueStrArray()),
    _inFrame(_inFrameProp.getValueStrArray())
{
    setNull();
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
JointReaction::JointReaction(const std::string &aFileName):
    Analysis(aFileName, false),
    _forcesFileName(_forcesFileNameProp.getValueStr()),
    _jointNames(_jointNamesProp.getValueStrArray()),
    _onBody(_onBodyProp.getValueStrArray()),
    _inFrame(_inFrameProp.getValueStrArray())
{
    setNull();

    // Serialize from XML
    updateFromXMLDocument();

    /* The rest will be done by setModel().
    // CONSTRUCT DESCRIPTION AND LABELS
    constructDescription();
    updateBodiesToRecord();
    constructColumnLabels();

    // STORAGE
    allocateStorage();
    */
}

// Copy constructor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
JointReaction::JointReaction(const JointReaction &aJointReaction):
    Analysis(aJointReaction),
    _forcesFileName(_forcesFileNameProp.getValueStr()),
    _jointNames(_jointNamesProp.getValueStrArray()),
    _onBody(_onBodyProp.getValueStrArray()),
    _inFrame(_inFrameProp.getValueStrArray())
{
    setNull();
    // COPY TYPE AND NAME
    *this = aJointReaction;
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
JointReaction& JointReaction::
operator=(const JointReaction &aJointReaction)
{
    // Base Class
    Analysis::operator=(aJointReaction);

    // Member Variables
    _forcesFileName = aJointReaction._forcesFileName;
    _jointNames = aJointReaction._jointNames;
    _onBody = aJointReaction._onBody;
    _inFrame = aJointReaction._inFrame;
    _useForceStorage = aJointReaction._useForceStorage;
    _storeActuation = NULL;
    return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void JointReaction::
setNull()
{
    setAuthors("Matt S. DeMers, Ajay Seth");
    setupProperties();

    // Property Default Values that are set if the associated fields are
    // omitted from the setup file.
    _forcesFileName = "";
    _useForceStorage = false;
    _jointNames.setSize(1);
    _jointNames[0] = "ALL";
    _onBody.setSize(1);
    _onBody[0]= "child";
    _inFrame.setSize(1);
    _inFrame[0] = "ground";

    _storeActuation = NULL;

}
//_____________________________________________________________________________
/**
 * Set up the properties for the analysis.
 *
 * The name give to each property is the tag that will be used in the XML
 * file.  The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set.  Once added, they can be
 * read in and written to file.
 */
void JointReaction::
setupProperties()
{

    _forcesFileNameProp.setName("forces_file");
    _forcesFileNameProp.setComment("The name of a file containing forces "
        "storage. If a file name is provided, the forces for all actuators "
        "will be applied according to values specified in the forces_file "
        "instead of being computed from the states. This option should be "
        "used to calculate joint reactions from static optimization results.");
    _propertySet.append(&_forcesFileNameProp);

    _jointNamesProp.setName("joint_names");
    _jointNamesProp.setComment("Names of the joints on which to perform the "
        "analysis. The key word 'All' indicates that the analysis should be "
        "performed for all joints.");
    _propertySet.append(&_jointNamesProp);

    _onBodyProp.setName("apply_on_bodies");
    _onBodyProp.setComment("Choice of body ('parent' or 'child') for which "
        "the reaction loads are calculated. Child body is default. The array "
        "must either have one entry or the same number of entries as joints "
        "specified above. If the array has one entry only, that selection "
        "is applied to all chosen joints.");
    _propertySet.append(&_onBodyProp);

    _inFrameProp.setName("express_in_frame");
    _inFrameProp.setComment("Names of frames in which the calculated "
        "reactions are expressed, or the keyword 'child' or 'parent' to "
        "indicate the joint's 'child' or 'parent' Frame. "
        "ground is default. If a Frame named 'child' or "
        "'parent' exists and the keyword 'child' or 'parent' is used, "
        "the analysis will use that Frame. The array must "
        "either have one entry or the same number of entries as joints "
        "specified above. If the array has one entry only, "
        "that selection is applied to all chosen joints.");
    _propertySet.append(&_inFrameProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Setup the ReactionList which controls the joints included in the analysis,
 * the bodies the loads are applied to, and the frames the loads are expressed
 * in.
 */
void JointReaction::setupReactionList()
{
    /* check length of property arrays.  if one is empty, set it to default*/
    if(_jointNames.getSize() == 0) {
        log_info("No joints are specified in joint_names. Setting to ALL");
        _jointNames.setSize(1);
        _jointNames[0] = "ALL";}
    if(_onBody.getSize() == 0) {
        log_info("No bodies are specified in apply_on_bodies. Setting to child");
        _onBody.setSize(1);
        _onBody[0] = "child";}
    if(_inFrame.getSize() == 0) {
        log_info("No bodies are specified in express_in_frame. Setting to ground");
        _inFrame.setSize(1);
        _inFrame[0] = "ground";}

    /* get the joint set and  body set from the dynamics engine model*/
    const JointSet& jointSet = _model->getJointSet();
    int numJoints = jointSet.getSize();

    /* check if jointNames is specified to "ALL".  if yes, setup to 
    *  compute reactions loads for all joints*/
    std::string firstNameEntry = _jointNames.get(0);
    // convert to upper case
    std::transform(firstNameEntry.begin(),firstNameEntry.end(),firstNameEntry.begin(), ::toupper);
    if (firstNameEntry == "ALL") {
        _jointNames.setSize(numJoints);
        for(int i=0;i<numJoints;i++) {
            Joint& joint = jointSet.get(i);
            _jointNames.set(i, joint.getName());
        }
    }
    int numJointNames = _jointNames.getSize();
    /* check that the _onBody Array and _inFrame Array are either the same length as 
    *  the _jointNames Array or are of length 1.  If not, set lengths to default of 1 and 
    *  set the values so that all reactions will be reported on the child body, expressed  
    *  in the ground frame.*/
    if (_onBody.getSize() == 1);
    else if (_onBody.getSize() != numJointNames)
        OPENSIM_THROW(InvalidArgument,
            "apply_on_bodies list is neither of length 1 nor the same length as indicated by joint_names.");

    if (_inFrame.getSize() == 1);
    else if (_inFrame.getSize() != numJointNames)
        OPENSIM_THROW(InvalidArgument,
            "express_in_frame list is neither of length 1 nor the same length as indicated by joint_names.");
    
    /* setup the JointReactionKey and, for valid joint names, determine and set the 
    *  reactionIndex, onBodyIndex, and inFrameIndex of each JointReactionKey */
    _reactionList.setSize(0);
    int index = -1;
    for (int i = 0; i < _jointNames.getSize(); ++i) {
        JointReactionKey currentKey;
        index = jointSet.getIndex(_jointNames[i], index + 1);
        if (index > -1) { // found the Joint in the model
            // Add joint to JointReactionKey 
            const Joint& joint = jointSet[index];
            currentKey.joint = &joint;

            // Want joint reaction applied to child or parent?
            std::string appliedOnName = "child";
            if (_onBody.size()){
                appliedOnName = (i < _onBody.size()) ? _onBody[i] : _onBody[0];
            }

            // convert to lowercase
            std::transform(appliedOnName.begin(), appliedOnName.end(),
                appliedOnName.begin(), ::tolower);
            // determine if user wants reaction on child or parent
            OPENSIM_THROW_IF(appliedOnName != "child" && appliedOnName != "parent",
                             InvalidArgument,
                             "'apply_on_bodies' must be either 'child' or 'parent'");
            currentKey.isAppliedOnChild = (appliedOnName == "child");
            currentKey.appliedOnBody = currentKey.isAppliedOnChild ?
                &joint.getChildFrame().findBaseFrame() : &joint.getParentFrame().findBaseFrame();

            // set frame that reactions are expressed in
            std::string expressedIn = "ground";
            if (_inFrame.size()) {
                expressedIn = (i < _inFrame.size()) ? _inFrame[i] : _inFrame[0];
            }

            if (_model->hasComponent<Frame>(expressedIn))
                currentKey.expressedInFrame =
                    &_model->getComponent<Frame>(expressedIn);
            else if (_model->getBodySet().hasComponent<Frame>(expressedIn))
                currentKey.expressedInFrame =
                    &_model->getBodySet().getComponent<Frame>(expressedIn);
            else if (_model->getJointSet().hasComponent<Frame>(expressedIn))
                currentKey.expressedInFrame =
                    &_model->getJointSet().getComponent<Frame>(expressedIn);
            else {
                std::transform(expressedIn.begin(), expressedIn.end(), expressedIn.begin(), ::tolower);
                if (expressedIn == "child")
                    currentKey.expressedInFrame = &joint.getChildFrame().findBaseFrame();
                else if (expressedIn == "parent")
                    currentKey.expressedInFrame = &joint.getParentFrame().findBaseFrame();
                else {
                    OPENSIM_THROW(InvalidArgument,
                                  "'express_in_frame' must either be a Frame "
                                  "name or the keyword 'child' or 'parent'.")
                }
            }
            
            _reactionList.append(currentKey);
        }
        else {
            log_warn("'{}' is not a valid joint. Ignoring this entry.",
                     _jointNames[i]);
        }
    }
}
    



//_____________________________________________________________________________
/**
 * Construct a description for the joint reaction loads files.
 */
void JointReaction::
constructDescription()
{
    string descrip;

    descrip = "\nThis file contains the reaction forces and moments\n";
    descrip += "applied to the specified body of a joint pair and expressed  ";
    descrip += "in the specified reference frame.\n";
    descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";

    setDescription(descrip);
    
}

//_____________________________________________________________________________
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
void JointReaction::
constructColumnLabels()
{
    if(_model==NULL) return;

    Array<string> labels;
    labels.append("time");

    int numOutputJoints = _reactionList.getSize();
    //  For each joint listed in _reactionList, append 3 column labels for forces
    //  and 3 column labels for moments.
    for(int i=0; i<numOutputJoints; ++i) {
        std::string jointName = _reactionList[i].joint->getName();
        std::string onBodyName = _reactionList[i].appliedOnBody->getName();
        std::string inFrameName = _reactionList[i].expressedInFrame->getName();
        std::string labelRoot = jointName + "_on_" + onBodyName + "_in_" + inFrameName;
        labels.append(labelRoot + "_fx");
        labels.append(labelRoot + "_fy");
        labels.append(labelRoot + "_fz");
        labels.append(labelRoot + "_mx");
        labels.append(labelRoot + "_my");
        labels.append(labelRoot + "_mz");
        labels.append(labelRoot + "_px");
        labels.append(labelRoot + "_py");
        labels.append(labelRoot + "_pz");
    }

    setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Load actuation storage from file.
 *
 * If called, this method sets _storeActuation to the
 * forces data in _forcesFileName
 */
void JointReaction::loadForcesFromFile()
{
    delete _storeActuation; _storeActuation = NULL;
    // check if the forces storage file name is valid and, if so, load the file into storage
    if(_forcesFileNameProp.isValidFileName()) {
        
        log_info("Loading actuator forces from file {}.", _forcesFileName);
        _storeActuation = new Storage(_forcesFileName);
        int storeSize = _storeActuation->getSmallestNumberOfStates();

        log_info("Found {} actuator forces with time stamps ranging from {}"
                 "to {}.", storeSize, _storeActuation->getFirstTime(),
                 _storeActuation->getLastTime());

        // check if actuator set and forces file have the same actuators
        bool _containsAllActuators = true;
        int actuatorSetSize = _model->getActuators().getSize();
        if(actuatorSetSize > storeSize){
            log_warn("The forces file does not contain enough actuators.");
            _containsAllActuators = false;
        }
        else {
            for(int actuatorIndex=0;actuatorIndex<actuatorSetSize;actuatorIndex++)
            {
                std::string actuatorName = _model->getActuators().get(actuatorIndex).getName();
                int storageIndex = _storeActuation->getStateIndex(actuatorName,0);
                if(storageIndex == -1) {
                    log_warn("The actuator '{}' was not found in the forces "
                             "file.",
                            actuatorName);

                    _containsAllActuators = false;
                }
            }
        }

        if(_containsAllActuators) {
            if(storeSize> actuatorSetSize) {
                log_warn("The forces file contains actuators that are not in "
                         "the model's actuator set.");
            }
            _useForceStorage = true;
            log_warn("Ignoring fiber lengths and activations from the states "
                     "since {} is also set. Actuator forces will be "
                     "constructed from {}.",
                    _forcesFileNameProp.getName(), _forcesFileName);
        }
        else {
            _useForceStorage = false;
            log_info("Actuator forces will be constructed from the states.");
        }
    }

    else {
        log_warn("'{}' is not a valid file name. Actuator forces will be "
                 "constructed from the states.",
                _forcesFileNameProp.getName());
        _useForceStorage = false;
    }
}

//_____________________________________________________________________________
/**
 * Set up storage objects.
 *
 * The storage objects in the analysis are used to record
 * the results of the analysis and write them to file.  
 */
void JointReaction::
setupStorage()
{
    // Reaction Loads
    _storeReactionLoads.reset(0);
    _storeReactionLoads.setName("Joint Reaction Loads");
    _storeReactionLoads.setDescription(getDescription());
    _storeReactionLoads.setColumnLabels(getColumnLabels());

    // Actuator forces - if a forces file is specified, load the forces storage data to _storeActuation
    if(!(_forcesFileName == "")) loadForcesFromFile();

}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
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
void JointReaction::
setModel(Model& aModel)
{
    // SET THE MODEL IN THE BASE CLASS
    Analysis::setModel(aModel);

    // UPDATE VARIABLES IN THIS CLASS
    setupReactionList();
    constructDescription();
    constructColumnLabels();

    int numJoints = _reactionList.getSize();
    // set size of working array of loads.  Each load has 3 components each
    // for force, moment, and point of application
    _Loads.setSize(9*numJoints);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and record the results.
 *
 * This method computes the reaction loads at all joints in the model, then
 * truncates the results to contain only the loads at the requested joints
 * and finally, if necessary, modifies the loads to be acting on the specified
 * body and expressed in the specified frame
 *
 * @param aT Current time in the simulation.
 * @param aX Current values of the controls.
 * @param aY Current values of the states.
 */
int JointReaction::
record(const SimTK::State& s)
{
    /** if a forces file is specified replace the computed actuation with the 
        forces from storage.*/
    SimTK::State s_analysis = s;

    _model->updMultibodySystem().realize(s_analysis, s.getSystemStage());
    if(_useForceStorage){
        const auto& actuatorSet = _model->getActuators();
        int nA = actuatorSet.getSize();
        Array<double> forces(0,nA);
        _storeActuation->getDataAtTime(s.getTime(),nA,forces);
        int storageIndex = -1;
        for(int actuatorIndex=0;actuatorIndex<nA;actuatorIndex++)
        {
            //Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(actuatorIndex));
            std::string actuatorName = actuatorSet.get(actuatorIndex).getName();
            storageIndex = _storeActuation->getStateIndex(actuatorName, 0);
            if(storageIndex == -1){
                log_warn("The actuator '{}' was not found in the forces file.",
                        actuatorName);
                break;
            }
            const ScalarActuator* act = dynamic_cast<const ScalarActuator*>(&actuatorSet[actuatorIndex]);
            if (act){
                act->overrideActuation(s_analysis, true);
                act->setOverrideActuation(s_analysis, forces[storageIndex]);
            }
        }
    }
    // VARIABLES
    const Ground& ground = _model->getGround();

    _model->realizeAcceleration(s_analysis);

    /* retrieved desired joint reactions, convert to desired bodies, and convert
    *  to desired reference frames*/
    int numOutputJoints = _reactionList.getSize();
    Vector_<Vec3> forcesVec(numOutputJoints), momentsVec(numOutputJoints), pointsVec(numOutputJoints);
    for(int i=0; i<numOutputJoints; i++) {
        JointReactionKey currentKey = _reactionList[i];
        const Joint& joint = *currentKey.joint;
        const Frame& expressedInBody = *currentKey.expressedInFrame;
        SpatialVec jointReaction;
        Vec3 pointOfApplication;
        
        // check if the load requested is on the parent or child
        if(!currentKey.isAppliedOnChild){
            jointReaction = joint.calcReactionOnParentExpressedInGround(s_analysis);

            // find the point of application in immediate parent frame, then
            // transform to the base frame of the parent (expressedInBody)
            Vec3 parentLocationInGlobal = joint.getParentFrame().getTransformInGround(s_analysis).p();
            pointOfApplication = 
                ground.findStationLocationInAnotherFrame(s_analysis, parentLocationInGlobal, expressedInBody);
        }
        else{
            jointReaction = joint.calcReactionOnChildExpressedInGround(s_analysis);

            // find the point of application in immediate child frame, then
            // transform to the base frame of the child (expressedInBody)
            Vec3 childLocationInGlobal = joint.getChildFrame().getTransformInGround(s_analysis).p();
            pointOfApplication =
                ground.findStationLocationInAnotherFrame(s_analysis, childLocationInGlobal, expressedInBody);
        }

        // transform SpatialVec of reaction forces and moments to the
        // requested base frame (expressedInBody)
        Vec3 force = ground.expressVectorInAnotherFrame(s_analysis, jointReaction[1], expressedInBody);
        Vec3 moment = ground.expressVectorInAnotherFrame(s_analysis, jointReaction[0], expressedInBody);

        /* place results in the truncated loads vectors*/
        forcesVec[i] = force;
        momentsVec[i] = moment;
        pointsVec[i] = pointOfApplication;
    }

    /* fill out row construction array*/
    for(int i=0;i<numOutputJoints;i++) {
        int I = 9*i;
        for(int j=0;j<3;j++) {
            _Loads[I+j] = forcesVec[i][j];
            _Loads[I+j+3] = momentsVec[i][j];
            _Loads[I+j+6] = pointsVec[i][j];
        }
    }
    /* Write the reaction data to storage*/
    _storeReactionLoads.append(s.getTime(),_Loads.getSize(),&_Loads[0]);

    return 0;
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the beginning of an integration 
 *
 * @param s reference to the current state
 *
 * @return -1 on error, 0 otherwise.
 */
int JointReaction::
begin(const SimTK::State& s)
{
    if(!proceed()) return(0);
    // Read forces file here rather than during initialization
    setupStorage();

    // RESET STORAGE
    _storeReactionLoads.reset(s.getTime());

    // RECORD
    int status = 0;
    if(_storeReactionLoads.getSize()<=0) {
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
 *
 * @param s reference to the current stateaClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int JointReaction::
step( const SimTK::State& s, int stepNumber)
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
 * @param s reference to the current state
 *
 * @return -1 on error, 0 otherwise.
 */
int JointReaction::
end(const SimTK::State&s)
{
    if(!proceed()) return(0);

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
int JointReaction::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    // Reaction Loads
    Storage::printResult(&_storeReactionLoads,
            aBaseName + "_" + getName() + "_ReactionLoads", aDir, aDT,
            aExtension);

    return(0);
}


