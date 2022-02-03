/* -------------------------------------------------------------------------- *
 *                     OpenSim:  InverseDynamicsTool.cpp                      *
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
#include "InverseDynamicsTool.h"

#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Stopwatch.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

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
InverseDynamicsTool::~InverseDynamicsTool()
{
    if (_coordinateValues) {
        delete _coordinateValues; _coordinateValues=NULL; 
    }
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InverseDynamicsTool::InverseDynamicsTool() : DynamicsTool(),
    _coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _outputGenForceFileName(_outputGenForceFileNameProp.getValueStr()),
    _jointsForReportingBodyForces(_jointsForReportingBodyForcesProp.getValueStrArray()),
    _outputBodyForcesAtJointsFileName(_outputBodyForcesAtJointsFileNameProp.getValueStr())
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
InverseDynamicsTool::InverseDynamicsTool(const string &aFileName, bool aLoadModel) :
    DynamicsTool(aFileName, false),
    _coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _outputGenForceFileName(_outputGenForceFileNameProp.getValueStr()),
    _jointsForReportingBodyForces(_jointsForReportingBodyForcesProp.getValueStrArray()),
    _outputBodyForcesAtJointsFileName(_outputBodyForcesAtJointsFileNameProp.getValueStr())
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
InverseDynamicsTool::InverseDynamicsTool(const InverseDynamicsTool &aTool) :
    DynamicsTool(aTool),
    _coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _outputGenForceFileName(_outputGenForceFileNameProp.getValueStr()),
    _jointsForReportingBodyForces(_jointsForReportingBodyForcesProp.getValueStrArray()),
    _outputBodyForcesAtJointsFileName(_outputBodyForcesAtJointsFileNameProp.getValueStr())
{
    setNull();
    *this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InverseDynamicsTool::setNull()
{
    setupProperties();
    _model = NULL;
    _lowpassCutoffFrequency = -1.0;
    _coordinateValues = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InverseDynamicsTool::setupProperties()
{
    _coordinatesFileNameProp.setComment("The name of the file containing coordinate data. Can be a motion (.mot) or a states (.sto) file.");
    _coordinatesFileNameProp.setName("coordinates_file");
    _propertySet.append(&_coordinatesFileNameProp);

    string comment = "Low-pass cut-off frequency for filtering the coordinates_file data (currently does not apply to states_file or speeds_file). "
                 "A negative value results in no filtering. The default value is -1.0, so no filtering.";
    _lowpassCutoffFrequencyProp.setComment(comment);
    _lowpassCutoffFrequencyProp.setName("lowpass_cutoff_frequency_for_coordinates");
    _propertySet.append( &_lowpassCutoffFrequencyProp );

    string genForceComment =
            "Name of the storage file (.sto) to which the generalized forces "
            "are written. Only a filename should be specified here (not a "
            "full path); the file will appear in the location provided in the "
            "results_directory property.";
    _outputGenForceFileNameProp.setComment(genForceComment);
    _outputGenForceFileNameProp.setName("output_gen_force_file");
    _outputGenForceFileNameProp.setValue("inverse_dynamics.sto");
    _propertySet.append(&_outputGenForceFileNameProp);

    _jointsForReportingBodyForcesProp.setComment("List of joints (keyword All, for all joints)"
        " to report body forces acting at the joint frame expressed in ground.");
    _jointsForReportingBodyForcesProp.setName("joints_to_report_body_forces");
    _propertySet.append(&_jointsForReportingBodyForcesProp);

    _outputBodyForcesAtJointsFileNameProp.setComment("Name of the storage file (.sto) to which the body forces at specified joints are written.");
    _outputBodyForcesAtJointsFileNameProp.setName("output_body_forces_file");
    _outputBodyForcesAtJointsFileNameProp.setValue("body_forces_at_joints.sto");
    _propertySet.append(&_outputBodyForcesAtJointsFileNameProp);
}

//_____________________________________________________________________________
/**
 * Register InverseDynamicsTool and any Object types it may employ internally.
 */
void InverseDynamicsTool::registerTypes()
{
    Object::registerType(InverseDynamicsTool());
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
InverseDynamicsTool& InverseDynamicsTool::
operator=(const InverseDynamicsTool &aTool)
{
    // BASE CLASS
    DynamicsTool::operator=(aTool);

    // MEMBER VARIABLES
    _modelFileName = aTool._modelFileName;
    _coordinatesFileName = aTool._coordinatesFileName;
    _lowpassCutoffFrequency = aTool._lowpassCutoffFrequency;
    _outputGenForceFileName = aTool._outputGenForceFileName;
    _outputBodyForcesAtJointsFileName = aTool._outputBodyForcesAtJointsFileName;
    _coordinateValues = NULL;

    return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================

void InverseDynamicsTool::setCoordinateValues(const OpenSim::Storage& aStorage)
{
    if (_coordinateValues) delete _coordinateValues;
    _coordinateValues = new Storage(aStorage);
    _coordinatesFileName = "";
}


/** Build the list of Joints for computing and reporting equivalent body forces */
void InverseDynamicsTool::getJointsByName(Model &model, const Array<std::string> &jointNames, JointSet &joints) const
{   
    const JointSet &modelJoints = model.getJointSet();
    Array<string> groupNames;
    modelJoints.getGroupNames(groupNames);

    /* The search for individual group or force names IS case-sensitive BUT keywords are not*/
    for(int i=0; i<jointNames.getSize();  ++i){
        //Check for keywords first starting with ALL
        if(IO::Uppercase(jointNames[i]) == "ALL"){
            for(int j=0; j<modelJoints.getSize(); ++j){
                joints.adoptAndAppend(&modelJoints[j]);
            }
            break;
        } 
        
        int k = modelJoints.getIndex(jointNames[i]);
        if (k >= 0){
            joints.adoptAndAppend(&modelJoints[k]);
        } else {
            log_warn("InverseDynamicsTool could not find Joint named '{}' to "
                    "report body forces.", jointNames[i]);
        }
    }
    joints.setMemoryOwner(false);
}


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the inverse Dynamics tool.
 */
bool InverseDynamicsTool::run()
{
    bool success = false;
    bool modelFromFile=true;
    try{
        //Load and create the indicated model
        if (!_model) {
            OPENSIM_THROW_IF_FRMOBJ(_modelFileName.empty(), Exception,
                "No model filename was provided.")

            _model = new Model(_modelFileName);
        }
        else
            modelFromFile = false;

        _model->finalizeFromProperties();
        _model->printBasicInfo();

        log_info("Running tool {}...", getName());
        // Do the maneuver to change then restore working directory 
        // so that the parsing code behaves properly if called from a different directory.
        auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

        /*bool externalLoads = */createExternalLoads(_externalLoadsFileName, *_model);
        // Initialize the model's underlying computational system and get its default state.
        SimTK::State& s = _model->initSystem();

        // OpenSim::Coordinates represent degrees of freedom for a model.
        // Each Coordinate's value and speed maps to an index
        // in the model's underlying SimTK::State (value to a slot in the
        // State's q, and speed to a slot in the State's u).
        // So we need to map each OpenSim::Coordinate value and speed to the
        // corresponding SimTK::State's q and u indices, respectively.
        auto coords = _model->getCoordinatesInMultibodyTreeOrder();
        int nq = s.getNQ();
        int nu = s.getNU();
        int nCoords = (int)coords.size();
        int intUnusedSlot = -1;

        // Create a vector mapCoordinateToQ whose i'th element provides
        // the index in vector 'coords' that corresponds with the i'th 'q' value.
        // Do the same for mapCoordinateToU, which tracks the order for
        // each 'u' value.
        auto coordMap = createSystemYIndexMap(*_model);
        std::vector<int> mapCoordinateToQ(nq, intUnusedSlot);
        std::vector<int> mapCoordinateToU(nu, intUnusedSlot);
        for (const auto& c : coordMap) {
            // SimTK::State layout is [q u z] 
            // (i.e., all "q"s first then "u"s second).
            if (c.second < nq + nu) { 
                std::string svName = c.first;
                
                // The state names corresponding to q's and u's will be of 
                // the form:
                // /jointset/(joint)/(coordinate)/(value or speed).
                // So the corresponding coordinate name is second from the end.
                ComponentPath svPath(svName);
                std::string lastPathElement = svPath.getComponentName();
                std::string coordName = svPath.getSubcomponentNameAtLevel(
                        svPath.getNumPathLevels() - 2);

                for (int i = 0; i < nCoords; ++i) {
                    if (coordName == coords[i]->getName()) {
                        if (lastPathElement == "value") {
                            mapCoordinateToQ[c.second] = i;
                            break;
                        }
                        
                        // Shift State/System indices by nq since u's follow q's
                        else if (lastPathElement == "speed") {
                            mapCoordinateToU[c.second - nq] = i;
                            break;
                        }
                        
                        else {
                            throw Exception("Last element in state variable "
                                            " name " + svName + " is neither "
                                            "'value' nor 'speed'");
                        }
                    }
                    if (i == nCoords - 1) {
                        throw Exception("Coordinate " + coordName + 
                                " not found in model.");
                    }
                }

            }
        }

        // Make sure that order of coordFunctions (which define splines for 
        // State's q's) is in the same order as the State's q order.
        // Also make a new vector (coordinatesToSpeedsIndexMap) that, for each
        // u in the State, gives the corresponding index in q (which is same
        // order as  coordFunctions). This accounts for cases where qdot != u.
        FunctionSet coordFunctions;
        coordFunctions.ensureCapacity(nq);
        std::vector<int> coordinatesToSpeedsIndexMap(nu, intUnusedSlot);

        if (loadCoordinateValues()){
            if(_lowpassCutoffFrequency>=0) {
                log_info("Low-pass filtering coordinates data with a cutoff "
                         "frequency of {}...", _lowpassCutoffFrequency);
                _coordinateValues->pad(_coordinateValues->getSize()/2);
                _coordinateValues->lowpassIIR(_lowpassCutoffFrequency);
            }
            // Convert degrees to radian if indicated
            if(_coordinateValues->isInDegrees()){
                _model->getSimbodyEngine().convertDegreesToRadians(*_coordinateValues);
            }
            // Create differentiable splines of the coordinate data
            GCVSplineSet coordSplines(5, _coordinateValues);

            // Functions must correspond to model coordinates.
            // Solver needs the order of Function's to be the same as order
            // in State's q's.
            for (int i = 0; i < nq; i++) {
                int coordInd = mapCoordinateToQ[i];

                // unused q slot
                if (coordInd == intUnusedSlot) {
                    coordFunctions.insert(i, new Constant(0));
                    continue;
                }

                const Coordinate& coord = *coords[coordInd];
                if (coordSplines.contains(coord.getName())) {
                    coordFunctions.insert(i, coordSplines.get(coord.getName()));
                }
                else{
                    coordFunctions.insert(i,new Constant(coord.getDefaultValue()));
                    log_info("InverseDynamicsTool: coordinate file does not "
                             "contain coordinate '{}'. Assuming default value.",
                            coord.getName());   
                }

                // Fill in coordinatesToSpeedsIndexMap as we go along to make
                // sure we know which function corresponds to State's u's.
                for (int j = 0; j < nu; ++j) {
                    if (mapCoordinateToU[j] == coordInd) { 
                        coordinatesToSpeedsIndexMap[j] = i;
                    }
                }
            }
            if(coordFunctions.getSize() > nq){
                coordFunctions.setSize(nq);
            }
        }
        else{
            throw Exception("InverseDynamicsTool: no coordinate file found, "
                " or setCoordinateValues() was not called.");
        }

        // Exclude user-specified forces from the dynamics for this analysis
        disableModelForces(*_model, s, _excludedForces);

        double first_time = _coordinateValues->getFirstTime();
        double last_time = _coordinateValues->getLastTime();

        // Determine the starting and final time for the Tool by comparing to what data is available
        double start_time = ( first_time > _timeRange[0]) ? first_time : _timeRange[0];
        double final_time = ( last_time < _timeRange[1]) ? last_time : _timeRange[1];
        int start_index = _coordinateValues->findIndex(start_time);
        int final_index = _coordinateValues->findIndex(final_time);

        // create the solver given the input data
        InverseDynamicsSolver ivdSolver(*_model);

        Stopwatch watch;

        int nt = final_index-start_index+1;
        
        Array_<double> times(nt, 0.0);
        for(int i=0; i<nt; i++){
            times[i]=_coordinateValues->getStateVector(start_index+i)->getTime();
        }

        // Preallocate results
        Array_<Vector> genForceTraj(nt, Vector(nCoords, 0.0));

        // solve for the trajectory of generalized forces that correspond to the 
        // coordinate trajectories provided
        ivdSolver.solve(s, coordFunctions, coordinatesToSpeedsIndexMap, times,
                genForceTraj);
        success = true;

        log_info("InverseDynamicsTool: {} time frames in {}.", nt, 
            watch.getElapsedTimeFormatted());
    
        JointSet jointsForEquivalentBodyForces;
        getJointsByName(*_model, _jointsForReportingBodyForces, jointsForEquivalentBodyForces);
        int nj = jointsForEquivalentBodyForces.getSize();

        // Generalized forces from ID Solver are in MultibodyTree order and not
        // necessarily in the order of the Coordinates in the Model.
        // We can get the Coordinates in Tree order from the Model.
        Array<string> labels("time", nCoords + 1);
        for (int i = 0; i < nCoords; i++) {
            labels[i+1] = coords[i]->getName();
            labels[i+1] += (coords[i]->getMotionType() == Coordinate::Rotational) ? 
                "_moment" : "_force";
        }

        Array<string> body_force_labels("time", 6*nj+1);
        string XYZ = "XYZ";
        for(int i=0; i<nj; i++){
            string joint_body_label = jointsForEquivalentBodyForces[i].getName()+"_";
            joint_body_label += jointsForEquivalentBodyForces[i].getChildFrame().getName();
            for(int k=0; k<3; ++k){
                body_force_labels[6*i+k+1] =  joint_body_label+"_F"+XYZ[k]; //first label is time
                body_force_labels[6*i+k+3+1] =  joint_body_label+"_M"+XYZ[k];
            }
        }

        Storage genForceResults(nt);
        Storage bodyForcesResults(nt);
        SpatialVec equivalentBodyForceAtJoint;

        for(int i=0; i<nt; i++){
            StateVector
                genForceVec(times[i], genForceTraj[i]);
            genForceResults.append(genForceVec);

            // if there are joints requested for equivalent body forces then calculate them
            if(nj>0){
                Vector forces(6*nj, 0.0);
                StateVector bodyForcesVec(times[i],
                                          SimTK::Vector_<double>(6*nj,
                                                                 &forces[0]));

                s.updTime() = times[i];
                Vector &q = s.updQ();
                Vector &u = s.updU();

                // Account for cases where qdot != u with coordinatesToSpeedsIndexMap
                for( int j = 0; j < nq; ++j) {
                    q[j] = coordFunctions.evaluate(j, 0, times[i]);
                }
                for (int j = 0; j < nu; ++j) {
                    u[j] = coordFunctions.evaluate(
                            coordinatesToSpeedsIndexMap[j], 1, times[i]);
                }

            
                for(int j=0; j<nj; ++j){
                    equivalentBodyForceAtJoint = jointsForEquivalentBodyForces[j].calcEquivalentSpatialForce(s, genForceTraj[i]);
                    for(int k=0; k<3; ++k){
                        // body force components
                        bodyForcesVec.setDataValue(6*j+k, equivalentBodyForceAtJoint[1][k]); 
                        // body torque components
                        bodyForcesVec.setDataValue(6*j+k+3, equivalentBodyForceAtJoint[0][k]);
                    }
                }
                bodyForcesResults.append(bodyForcesVec);

            }
        }

        genForceResults.setColumnLabels(labels);
        genForceResults.setName("Inverse Dynamics Generalized Forces");

        IO::makeDir(getResultsDir());
        Storage::printResult(&genForceResults, _outputGenForceFileName, getResultsDir(), -1, ".sto");
        cwd.restore();

        // if body forces to be reported for specified joints
        if(nj >0){
            bodyForcesResults.setColumnLabels(body_force_labels);
            bodyForcesResults.setName("Inverse Dynamics Body Forces at Specified Joints");

            IO::makeDir(getResultsDir());
            Storage::printResult(&bodyForcesResults, _outputBodyForcesAtJointsFileName, getResultsDir(), -1, ".sto");
        }

        removeExternalLoadsFromModel();
    }
    catch (const OpenSim::Exception& ex) {
        log_error("InverseDynamicsTool Failed: {}", ex.what());
        throw (Exception("InverseDynamicsTool Failed, please see messages window for details..."));
    }

    if (modelFromFile) delete _model;
    return success;
}

bool InverseDynamicsTool::loadCoordinateValues()
{
    if (_coordinateValues!= NULL) // Coordinates has been set from GUI
        return true;
    // Try constructing coordinates from specified file
    if(_coordinatesFileName != "" && _coordinatesFileName != "Unassigned"){
            _coordinateValues = new Storage(_coordinatesFileName);
            _coordinateValues->setName(_coordinatesFileName);
            return true;
    }
    return false;
}
/* Handle reading older formats/Versioning */
void InverseDynamicsTool::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if ( documentVersion < XMLDocument::getLatestVersion()){
        std::string newFileName = getDocumentFileName();
        if (documentVersion < 20300){
            std::string origFilename = getDocumentFileName();
            newFileName=IO::replaceSubstring(newFileName, ".xml", "_v23.xml");
            log_info("Old version setup file encountered. Converting to new "
                     "file '{}'...", newFileName);
            SimTK::Xml::Document doc = SimTK::Xml::Document(origFilename);
            doc.writeToFile(newFileName);
        }
        /*if (documentVersion < 20201) {
            AnalyzeTool updateAnalyzeTool(newFileName, false);
            updateAnalyzeTool.print(newFileName);
        }*/
        if (documentVersion < 20202){
            // get filename and use SimTK::Xml to parse it
            SimTK::Xml::Document doc = SimTK::Xml::Document(newFileName);
            Xml::Element oldRoot = doc.getRootElement();
            SimTK::Xml::Document newDoc;
            string prefix = "";
            if (oldRoot.getElementTag()=="AnalyzeTool"){
                // Make OpenSimDocument node and copy root underneath it
                newDoc.getRootElement().setElementTag("OpenSimDocument");
                newDoc.getRootElement().setAttributeValue("Version", "20201");
                prefix = oldRoot.getRequiredAttributeValueAs<string>("name");
                // Move all children of root to toolNode
                newDoc.getRootElement().insertNodeAfter(newDoc.getRootElement().node_end(), oldRoot.clone());
            }
            else
                newDoc = doc;
            Xml::Element root = newDoc.getRootElement();
            if (root.getElementTag()=="OpenSimDocument"){
                int curVersion = root.getRequiredAttributeValueAs<int>("Version");
                if (curVersion <= 20201) root.setAttributeValue("Version", "20300");
                Xml::element_iterator iterTool(root.element_begin("AnalyzeTool"));
                iterTool->setElementTag("InverseDynamicsTool");
                prefix = iterTool->getRequiredAttributeValueAs<string>("name");
                // Remove children <output_precision>, <initial_time>, <final_time>
                Xml::element_iterator initTimeIter(iterTool->element_begin("initial_time"));
                double tool_initial_time = initTimeIter->getValueAs<double>();
                if (initTimeIter->isValid()) iterTool->eraseNode(initTimeIter);
                Xml::element_iterator finalTimeIter(iterTool->element_begin("final_time"));
                double tool_final_time = finalTimeIter->getValueAs<double>();
                if (finalTimeIter->isValid()) iterTool->eraseNode(finalTimeIter);
                Xml::element_iterator precisionIter(iterTool->element_begin("output_precision"));
                if (precisionIter->isValid()) iterTool->eraseNode(precisionIter);
                bool use_model_forces=false;
                // Handle missing or uninitialized values after parsing the old InverseDynamics "Analysis"
                // Find Analyses underneath it AnalyzeTool
                Xml::element_iterator iterAnalysisSet(iterTool->element_begin("AnalysisSet"));
                Xml::element_iterator iterObjects(iterAnalysisSet->element_begin("objects"));
                Xml::element_iterator iterAnalysis(iterObjects->element_begin("InverseDynamics"));
                if (iterAnalysis!= iterObjects->element_end()){
                    // move children to top level
                    Xml::element_iterator p = iterAnalysis->element_begin();
                    //std::vector<std::string> deprecated({"on", "in_degrees", "step_interval"});
                    for (; p!= iterAnalysis->element_end(); ++p) {
                        // skip <on>, <step_interval>, <in_degrees>
                        if (p->getElementTag()=="on" ||
                            p->getElementTag()=="in_degrees" ||
                            p->getElementTag()=="step_interval" ||
                            p->getElementTag()=="start_time" ||
                            p->getElementTag()=="end_time")
                            continue;
                        else if (p->getElementTag()=="use_model_force_set"){
                            String use_model_forcesStr = p->getValueAs<String>();
                            use_model_forces = (use_model_forcesStr=="true");
                        }
                        else
                            iterTool->insertNodeAfter( iterTool->node_end(), p->clone());
                    }
                    // insert elements for "forces_to_exclude" & "time_range"
                    std::ostringstream stream;
                    stream << tool_initial_time << " " << tool_final_time;
                    iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("time_range", stream.str()));
                    iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("forces_to_exclude", use_model_forces?"":"Muscles"));
                    iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("output_gen_force_file", prefix+"_InverseDynamics.sto"));
                    iterTool->insertNodeAfter( iterTool->node_end(), Xml::Element("coordinates_in_degrees", "true"));
                    iterTool->eraseNode(iterAnalysisSet);
                }
                newDoc.writeToFile(newFileName);
                setDocument(new XMLDocument(newFileName));
                aNode = updDocument()->getRootDataElement();
            }

        }
    }
    Object::updateFromXMLNode(aNode, versionNumber);
}
