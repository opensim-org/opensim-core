/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ExternalLoads.cpp                         *
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
#include "ExternalLoads.h"
#include "Model.h"
#include "BodySet.h"
#include <OpenSim/Simulation/Model/PrescribedForce.h>
#include <OpenSim/Common/IO.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ExternalLoads::~ExternalLoads()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ExternalLoads::ExternalLoads():
_dataFileName(_dataFileNameProp.getValueStr())
{
    setNull();
}

ExternalLoads::ExternalLoads(const std::string &fileName, bool updateFromXMLNode) :
    Super(fileName, false),
    _dataFileName(_dataFileNameProp.getValueStr()),
    _loadedFromFile(fileName)
{
    setNull();

    if(updateFromXMLNode)
        updateFromXMLDocument();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param otherExternalLoads ExternalLoads to be copied.
 */
ExternalLoads::ExternalLoads(const ExternalLoads &otherExternalLoads) :
    ModelComponentSet<ExternalForce>(otherExternalLoads),
    _dataFileName(_dataFileNameProp.getValueStr())
{
    // copy the document over, because it's used during `extendFinalizeConnections`
    // to figure out where the associated motion file (#3926)
    if (auto* document = otherExternalLoads.getDocument()) {
        setDocument(std::make_unique<XMLDocument>(*document).release());
    }

    setNull();

    // Class Members
    copyData(otherExternalLoads);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this ExternalLoads to their null values.
 */
void ExternalLoads::setNull()
{
    setAuthors("Ajay Seth, Ayman Habib");

    // PROPERTIES
    setupSerializedMembers();
    _storages.clear();
}


//_____________________________________________________________________________
/**
 * Copy the member variables of the ExternalLoads.
 *
 * @param otherExternalLoads actuator set to be copied
 */
void ExternalLoads::copyData(const ExternalLoads &aAbsExternalLoads)
{
    // ACTUATORS
    _dataFileName = aAbsExternalLoads._dataFileName;
    _storages = aAbsExternalLoads._storages;
    _loadedFromFile = aAbsExternalLoads._loadedFromFile;
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ExternalLoads::setupSerializedMembers()
{
    string comment;
    _dataFileNameProp.setName("datafile");
    _dataFileName="";
    comment =   "Storage file (.sto) containing (3) components of force and/or torque and point of application."
                "Note: this file overrides the data source specified by the individual external forces if specified.";
    _dataFileNameProp.setComment(comment);
    _propertySet.append(&_dataFileNameProp);
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
ExternalLoads& ExternalLoads::operator=(const ExternalLoads &otherExternalLoads)
{
    // BASE CLASS
    Super::operator=(otherExternalLoads);

    // Class Members
    copyData(otherExternalLoads);

    return(*this);
}

void ExternalLoads::extendConnectToModel(Model& aModel)
{
    // BASE CLASS
    Super::extendConnectToModel(aModel);

    Storage *forceData = nullptr;
    auto loadDataFromDirectoryAdjacentToFile =
        [this, &forceData](const std::string& filepath) {
            // Change working directory the ExternalLoads location
            auto cwd = IO::CwdChanger::changeToParentOf(filepath);
            try {
                forceData = new Storage(this->_dataFileName);
            }
            catch (const std::exception &ex) {
                log_error("Failed to read ExternalLoads data file '{}'.",
                        this->_dataFileName);
                if (this->getDocument()) {
                    cwd.restore();
                } else {
                    cwd.stay();
                }
                throw(ex);
            }
    };
    if (_dataFileName.length() > 0) {
        if(IO::FileExists(_dataFileName))
            forceData = new Storage(_dataFileName);
        else if(getDocument()) { // ExternalLoads constructed from file
            loadDataFromDirectoryAdjacentToFile(getDocumentFileName());
        }
        else if (!_loadedFromFile.empty()) {
            // Might be dealing with a copy of an ExternalLoads constructed
            // from file.
            loadDataFromDirectoryAdjacentToFile(_loadedFromFile);
        }
        else {
            // Cannot find the data file and do not have an ExternalLoads (XML)
            // document to test if file is in its directory.
            throw Exception("Error: unable to read ExternalLoads data file '" +
                _dataFileName + "'.");
        }

        for (int i = 0; i < getSize(); ++i)
            get(i).setDataSource(*forceData);

        // add loaded storage into list of storages for later garbage collection
        _storages.push_back(shared_ptr<Storage>(forceData));
    }
}

//-----------------------------------------------------------------------------
// RE-EXPRESS POINT DATA 
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Re-express the position of the point of application for all ExternalForces
 * in this collection of ExternalLoads, such that all points expressed in
 * ground are transformed (re-expressed) in the appliedToBody of the External-
 * Force. The pointExpressedInBody is correspondingly updated to be the
 * appliedToBody as well.
 * Note: If the point is not expressed in ground to begin with, it is not
 * re-expressed. If the ExternalForce does not specify a point of application
 * (body force or torque) it remains untouched.
 *
 * @param kinematics Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 */
void ExternalLoads::transformPointsExpressedInGroundToAppliedBodies(
    const Storage &kinematics, double startTime, double endTime)
{
    std::vector<ExternalForce*> transformedForces;
    for(int i=0; i<getSize(); ++i){
        ExternalForce *transformedExf = transformPointExpressedInGroundToAppliedBody(get(i), kinematics, startTime, endTime);
        transformedForces.push_back(transformedExf);
    }
    // Once we've transformed the forces (done with computation),
    // then replace them in the Set
    for (int i = 0; i < (int)transformedForces.size(); ++i) {
        ExternalForce *transformedExf = transformedForces[i];
        if (transformedExf) {
            set(i, transformedExf);
        }
    }

    if (transformedForces.size())
        _dataFileName = "";
}

ExternalForce* ExternalLoads::transformPointExpressedInGroundToAppliedBody(
    const ExternalForce &exForce, const Storage &kinematics,
    double startTime, double endTime)
{
    if(!hasModel() || !getModel().isValidSystem()) // no model and no system underneath, cannot proceed
        throw Exception("ExternalLoads::transformPointExpressedInGroundToAppliedBody() requires a model with a valid system."); 

    if(!exForce._specifiesPoint){ // The external force does not apply a force to a point
        log_warn("ExternalLoads: ExternalForce '{}' does not specify a point of application.",
            exForce.getName());
        return NULL;
    }

    if (exForce.getPointExpressedInBodyName() != getModel().getGround().getName()){
        log_warn("ExternalLoads: ExternalForce '{}' is not expressed in ground "
                 "and will not be transformed.",
                exForce.getName());
        return NULL;
    }

    if (exForce.getAppliedToBodyName() == getModel().getGround().getName()){
        log_warn("ExternalLoads: ExternalForce '{}' is applied to a point on ground and will not be transformed.",
            exForce.getName());
        return NULL;
    }

    int nq = getModel().getNumCoordinates();
    int nt = kinematics.getSize();

    int startIndex=0;
    int lastIndex=nt-1;
    int findex =0;

    if (nt > 0){
        if (startTime!= -SimTK::Infinity){  // Start time was actually specified.
            // since we are interpolating relevant data, make sure we don't
            // truncate user specified time by starting one index back
            findex = kinematics.findIndex(startTime)-1; 
            startIndex = findex >= 0 ? findex : 0;
        }
        if (endTime!= SimTK::Infinity){ // Final time was actually specified.
            // don't truncate user specified time range by ending one index later
            findex = kinematics.findIndex(endTime) + 1;
            lastIndex = findex <= lastIndex ? findex : lastIndex;
        }
    }
    else{
        log_warn("ExternalLoads: Specified load kinematics contains no "
                 "coordinate values. "
                 "Point of force application cannot be transformed.");
        return NULL;
    }

    nt = lastIndex-startIndex+1;

    // Construct a new storage to contain the re-expressed point data for the 
    // new external force.
    Storage *newDataSource = new Storage(nt);
    Array<string> labels;
    labels.append("time");

    const string &forceIdentifier = exForce.getForceIdentifier();
    const string &pointIdentifier = exForce.getPointIdentifier();
    const string &torqueIdentifier = exForce.getTorqueIdentifier();

    labels.append(forceIdentifier + ".x");
    labels.append(forceIdentifier + ".y");
    labels.append(forceIdentifier + ".z");
    labels.append(pointIdentifier + ".x");
    labels.append(pointIdentifier + ".y");
    labels.append(pointIdentifier + ".z");
    if(exForce._appliesTorque){
        labels.append(torqueIdentifier + ".x");
        labels.append(torqueIdentifier + ".y");
        labels.append(torqueIdentifier + ".z");
    }

    newDataSource->setColumnLabels(labels);
    int ncols = labels.getSize()-1; // time treated separately when appended to storage

    SimTK::Vector datarow(ncols, SimTK::NaN);

    double time = 0;
    Array<double> Q(0.0,nq);

    Vec3 pGround(SimTK::NaN);
    Vec3 pAppliedBody(SimTK::NaN);
    Vec3 force(SimTK::NaN); 
    Vec3 torque(SimTK::NaN);
    
    // Checked that we had a model with a valid system, so get its working state
    SimTK::State& s = updModel().updWorkingState();

    // get from (ground) and to (applied) bodies 
    const Ground& ground = getModel().getGround();
    const Body& appliedToBody = getModel().getBodySet().get(exForce.getAppliedToBodyName());

    /*std::map<int, int>      coordinatesToColumns;
    // create a map entry for each coordinate 0 to nq-1, the contents of which would be -1 if not found in the file otherwise Q index
    for (int qi=0; qi < nq; qi++) coordinatesToColumns[qi] = -1;
    const Array<string>& kinLabels = kinematics.getColumnLabels();

    for (int qj = 0; qj < nq; qj++) {
        Coordinate& coord = _model->getCoordinateSet().get(qj);
        int idx = kinLabels.findIndex(coord.getName());
        if (idx!= -1) coordinatesToColumns[qj] = idx-1; // Since time is not accounted for
    }*/
    for(int i=startIndex; i<=lastIndex; ++i) {
        // transform data on an instant-by-instant basis
        kinematics.getTime(i, time);
        kinematics.getData(i, nq, &Q[0]);

        // Set the coordinates values in the state in order to position the model according to specified kinematics
        for (int j = 0; j < nq; j++) {
            Coordinate& coord = getModel().getCoordinateSet().get(j);
            coord.setValue(s, Q[j], j==nq-1);
            /*if (coordinatesToColumns[j]!=-1)
                coord.setValue(s, Q[coordinatesToColumns[j]], j==nq-1);*/
        }

        // get force data
        force = exForce.getForceAtTime(time);
        if(exForce._appliesTorque)
            torque = exForce.getTorqueAtTime(time);
        
        // get the untransformed point expressed in ground in the ExternalForce specified in  ground (check made above)
        pGround = exForce.getPointAtTime(time);
        pAppliedBody = ground.findStationLocationInAnotherFrame(s, pGround, appliedToBody);

        // populate the force data for this instant in time
        for(int j =0; j<3; ++j){
            datarow[j] = force[j];
            datarow[j+3] = pAppliedBody[j];
            if(exForce._appliesTorque)
                datarow[j+6] = torque[j];
        }

        newDataSource->append(time, datarow); 
    }

    // assign a name to the new data source
    newDataSource->setName(exForce.getDataSourceName() + "_transformedP");

    ExternalForce *exF_transformedPoint = exForce.clone();
    exF_transformedPoint->setName(exForce.getName()+"_transformedP");
    exF_transformedPoint->setPointExpressedInBodyName(exForce.getAppliedToBodyName());
    exF_transformedPoint->setDataSource(*newDataSource);

    _storages.push_back(shared_ptr<Storage>(newDataSource));

    newDataSource->print(exForce.getName()+"_NewDataSource_TransformedP.sto");

    return exF_transformedPoint;
}

//-----------------------------------------------------------------------------
// UPDATE FROM OLDER VERSION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ExternalLoads::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if ( documentVersion < 20301){
        log_debug("Updating ExternalLoad object to latest format...");
        _dataFileName="";
        SimTK::Xml::element_iterator dataFileElementIter =aNode.element_begin("datafile");
        if(dataFileElementIter!=aNode.element_end()) {
                    // Could still be empty or whiteSpace
            SimTK::String transcoded = dataFileElementIter->getValueAs<SimTK::String>();
                    if (transcoded.length()>0)
                _dataFileName =transcoded;
        }
        SimTK::Xml::element_iterator kinFileNode = aNode.element_begin("external_loads_model_kinematics_file");
        if (kinFileNode != aNode.element_end()){
            SimTK::String transcoded = kinFileNode->getValueAs<SimTK::String>();
                    if (transcoded.length()>0)
                        log_warn("ExternalLoads: external_loads_model_kinematics_file option is not supported anymore."
                            "Results may change.");
        }
        SimTK::Xml::element_iterator kinFilterNode = aNode.element_begin("lowpass_cutoff_frequency_for_load_kinematics");
        if (kinFilterNode != aNode.element_end()){
            // This is now unnecessary since we dropped supoprt for external_loads_model_kinematics_file
                // _lowpassCutoffFrequencyForLoadKinematics = kinFilterNode->getValueAs<double>();
            }
            bool changeWorkingDir = false;
            std::string savedCwd;
            // Change to directory of Document
            if(!ifstream(_dataFileName.c_str(), ios_base::in).good()) {
            string msg =
                    "Object: Could not open file " + _dataFileName+ "IO. It may not exist or you don't have permission to read it.";
            log_error(msg);
            // Try switching to directory of setup file before aborting
            if (getDocument()) {
                savedCwd = IO::getCwd();
                IO::chDir(IO::getParentDirectory(getDocument()->getFileName()));
                changeWorkingDir = true;
            }
                if(!ifstream(_dataFileName.c_str(), ios_base::in).good()) {
                    if(changeWorkingDir) IO::chDir(savedCwd);
            throw Exception(msg,__FILE__,__LINE__);
                }
            }
            Storage* dataSource = new Storage(_dataFileName, true);
            if (!dataSource->makeStorageLabelsUnique()){
                log_info("Making labels unique in storage file {}", _dataFileName);
                dataSource = new Storage(_dataFileName);
                dataSource->makeStorageLabelsUnique();
                dataSource->print(_dataFileName);
            }
            if(changeWorkingDir) IO::chDir(savedCwd);
            
            const Array<string> &labels = dataSource->getColumnLabels();
            // Populate data file and other things that haven't changed
            // Create Set of Forces from this XML node, which we
            // then reassign to an ExternalForce and add to ExternalLoads
            Set<PrescribedForce> oldForces(getDocument()->getFileName(), true);
            for(int i=0; i< oldForces.getSize(); i++){
                PrescribedForce& oldPrescribedForce = oldForces.get(i);
                ExternalForce* newExternalForce = new ExternalForce();
                newExternalForce->setName(oldPrescribedForce.getName());
                // In 4.0, PrescribedForce's body_name became a relative path
                // to the body; we need to pull off just the body name.
                std::string bodyName = oldPrescribedForce.getBodyName();
                const auto slashLoc = bodyName.rfind('/');
                if (slashLoc != std::string::npos)
                    bodyName = bodyName.substr(slashLoc + 1);
                newExternalForce->setAppliedToBodyName(bodyName);
                newExternalForce->setPointExpressedInBodyName("ground");
                newExternalForce->setForceExpressedInBodyName("ground");
                // Reconstruct function names and use these to extract the identifier(s)
                OpenSim::Array<std::string> aFunctionNames;
                oldPrescribedForce.getForceFunctionNames(aFunctionNames);
                // Get names from force functions
                newExternalForce->setForceIdentifier(createIdentifier(aFunctionNames, labels));
                aFunctionNames.setSize(0);
                oldPrescribedForce.getPointFunctionNames(aFunctionNames);
                newExternalForce->setPointIdentifier(createIdentifier(aFunctionNames, labels));
                aFunctionNames.setSize(0);
                // Now the torques
                oldPrescribedForce.getTorqueFunctionNames(aFunctionNames);
                newExternalForce->setTorqueIdentifier(createIdentifier(aFunctionNames, labels));
                //newExternalForce->setDataSourceName("");
                adoptAndAppend(newExternalForce);
            }
            delete dataSource;
        }
        else {
            // Warn on removed external_loads_kinematics_specification
            SimTK::Xml::element_iterator kinFileNode =
                    aNode.element_begin("external_loads_model_kinematics_file");
            if (kinFileNode != aNode.element_end()) {
                SimTK::String transcoded =
                        kinFileNode->getValueAs<SimTK::String>();
                if (transcoded.length() > 0)
                    log_warn("ExternalLoades: "
                             "external_loads_model_kinematics_file "
                             "option is not supported anymore. Results may "
                             "change.");
            }
        }
        // Call base class now assuming _node has been corrected for current version
        ModelComponentSet<ExternalForce>::updateFromXMLNode(aNode, versionNumber);
}
/**
 * Helper function to recover Identifier based on the conventions used in earlier versions before identifiers were introduced
 */
std::string ExternalLoads::createIdentifier(OpenSim::Array<std::string>&oldFunctionNames, const Array<std::string>& labels)
{
    if (oldFunctionNames.getSize()==0) return "";
    // There was at least one function that could be specified by name or #
    std::string firstFuncName = oldFunctionNames[0];
    string fullIdentifier=firstFuncName;
    if (firstFuncName.c_str()[0]=='#'){ // it's really a number, may imply the storage has duplicate labels
        int columnOrder=-1;
        sscanf(firstFuncName.c_str(), "#%d", &columnOrder);
        fullIdentifier = labels[columnOrder];
    }
    return fullIdentifier.substr(0, fullIdentifier.length()-1);
    
}
