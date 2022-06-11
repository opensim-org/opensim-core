/* -------------------------------------------------------------------------- *
 *                    OpenSim:  InverseKinematicsTool.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "InverseKinematicsTool.h"

#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"
#include "IKTaskSet.h"

#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Stopwatch.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/Model/Model.h>

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
InverseKinematicsTool::~InverseKinematicsTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InverseKinematicsTool::InverseKinematicsTool() : InverseKinematicsToolBase() {
    constructProperties();
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
InverseKinematicsTool::InverseKinematicsTool(const string &aFileName, bool aLoadModel) : 
    InverseKinematicsToolBase(aFileName, aLoadModel) {
    constructProperties();
    updateFromXMLDocument();
}

//_____________________________________________________________________________
/**
 * Construct properties
 */
void InverseKinematicsTool::constructProperties()
{
    constructProperty_IKTaskSet(IKTaskSet());
    constructProperty_marker_file("");
    constructProperty_coordinate_file("");
    constructProperty_report_marker_locations(false);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the inverse kinematics tool.
 */
bool InverseKinematicsTool::run()
{
    bool success = false;
    bool modelFromFile=true;
    std::unique_ptr<Kinematics> kinematicsReporter(new Kinematics());
    try{
        //Load and create the indicated model
        if (_model.empty()) { 
            OPENSIM_THROW_IF_FRMOBJ(get_model_file().empty(), Exception,
                    "No model filename was provided.");
            _model.reset(new Model(get_model_file())); 
        }
        else
            modelFromFile = false;

        // although newly loaded model will be finalized
        // there is no guarantee that the _model has not been edited/modified
        _model->finalizeFromProperties();
        _model->printBasicInfo();

        // Do the maneuver to change then restore working directory so that the
        // parsing code behaves properly if called from a different directory.
        auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

        // Define reporter for output
        kinematicsReporter->setRecordAccelerations(false);
        kinematicsReporter->setInDegrees(true);
        _model->addAnalysis(kinematicsReporter.get());

        log_info("Running tool {}.", getName());

        // Get the trial name to label data written to files
        string trialName = getName();

        // Initialize the model's underlying system and get its default state.
        SimTK::State& s = _model->initSystem();

        //Convert old Tasks to references for assembly and tracking
        MarkersReference markersReference;
        SimTK::Array_<CoordinateReference> coordinateReferences;
        // populate the references according to the setting of this Tool
        populateReferences(markersReference, coordinateReferences);

        // Determine the start time, if the provided time range is not 
        // specified then use time from marker reference.
        // Adjust the time range for the tool if the provided range exceeds
        // that of the marker data.
        SimTK::Vec2 markersValidTimeRange = markersReference.getValidTimeRange();
        double start_time = (markersValidTimeRange[0] > get_time_range(0)) ?
            markersValidTimeRange[0] : get_time_range(0);
        double final_time = (markersValidTimeRange[1] < get_time_range(1)) ?
            markersValidTimeRange[1] : get_time_range(1);

        SimTK_ASSERT2_ALWAYS(final_time >= start_time,
            "InverseKinematicsTool final time (%f) is before start time (%f).",
            final_time, start_time);

        const auto& markersTable = markersReference.getMarkerTable();
        const int start_ix = int(
            markersTable.getNearestRowIndexForTime(start_time) );
        const int final_ix = int(
            markersTable.getNearestRowIndexForTime(final_time) );
        const int Nframes = final_ix - start_ix + 1;
        const auto& times = markersTable.getIndependentColumn();

        // create the solver given the input data
        InverseKinematicsSolver ikSolver(*_model, make_shared<MarkersReference>(markersReference),
            coordinateReferences, get_constraint_weight());
        ikSolver.setAccuracy(get_accuracy());
        s.updTime() = times[start_ix];
        ikSolver.assemble(s);
        kinematicsReporter->begin(s);

        AnalysisSet& analysisSet = _model->updAnalysisSet();
        analysisSet.begin(s);
        // Get the actual number of markers the Solver is using, which
        // can be fewer than the number of references if there isn't a
        // corresponding model marker for each reference.
        int nm = ikSolver.getNumMarkersInUse();
        SimTK::Array_<double> squaredMarkerErrors(nm, 0.0);
        SimTK::Array_<Vec3> markerLocations(nm, Vec3(0));
        
        Storage *modelMarkerLocations = get_report_marker_locations() ?
            new Storage(Nframes, "ModelMarkerLocations") : nullptr;
        Storage *modelMarkerErrors = get_report_errors() ? 
            new Storage(Nframes, "ModelMarkerErrors") : nullptr;

        Stopwatch watch;

        for (int i = start_ix; i <= final_ix; ++i) {
            s.updTime() = times[i];
            ikSolver.track(s);
            // show progress line every 1000 frames so users see progress
            if (std::remainder(i - start_ix, 1000) == 0 && i != start_ix)
                log_info("Solved {} frame(s)...", i - start_ix);
            if(get_report_errors()){
                Array<double> markerErrors(0.0, 3);
                double totalSquaredMarkerError = 0.0;
                double maxSquaredMarkerError = 0.0;
                int worst = -1;

                ikSolver.computeCurrentSquaredMarkerErrors(squaredMarkerErrors);
                for(int j=0; j<nm; ++j){
                    totalSquaredMarkerError += squaredMarkerErrors[j];
                    if(squaredMarkerErrors[j] > maxSquaredMarkerError){
                        maxSquaredMarkerError = squaredMarkerErrors[j];
                        worst = j;
                    }
                }

                double rms = nm > 0 ? sqrt(totalSquaredMarkerError / nm) : 0;
                markerErrors.set(0, totalSquaredMarkerError); 
                markerErrors.set(1, rms);
                markerErrors.set(2, sqrt(maxSquaredMarkerError));
                modelMarkerErrors->append(s.getTime(), 3, &markerErrors[0]);

                log_info("Frame {} (t = {}):\t total squared error = {}, "
                         "marker error: RMS = {}, max = {} ({})", 
                    i, s.getTime(), totalSquaredMarkerError, rms,
                    sqrt(maxSquaredMarkerError), 
                    ikSolver.getMarkerNameForIndex(worst));
            }

            if(get_report_marker_locations()){
                ikSolver.computeCurrentMarkerLocations(markerLocations);
                Array<double> locations(0.0, 3*nm);
                for(int j=0; j<nm; ++j){
                    for(int k=0; k<3; ++k)
                        locations.set(3*j+k, markerLocations[j][k]);
                }

                modelMarkerLocations->append(s.getTime(), 3*nm, &locations[0]);

            }

            kinematicsReporter->step(s, i);
            analysisSet.step(s, i);
        }

        // Do the maneuver to change then restore working directory 
        // so that output files are saved to same folder as setup file.
        if (get_output_motion_file() != "" &&
                get_output_motion_file() != "Unassigned") {
            kinematicsReporter->getPositionStorage()->print(
                    get_output_motion_file());
        }
        // Remove the analysis we added to the model, do not delete as 
        // the unique_ptr takes care of that automatically
        _model->removeAnalysis(kinematicsReporter.get(), false);

        if (modelMarkerErrors) {
            Array<string> labels("", 4);
            labels[0] = "time";
            labels[1] = "total_squared_error";
            labels[2] = "marker_error_RMS";
            labels[3] = "marker_error_max";

            modelMarkerErrors->setColumnLabels(labels);
            modelMarkerErrors->setName("Model Marker Errors from IK");

            IO::makeDir(getResultsDir());
            string errorFileName = trialName + "_ik_marker_errors";
            Storage::printResult(modelMarkerErrors, errorFileName,
                                 getResultsDir(), -1, ".sto");

            delete modelMarkerErrors;
        }

        if(modelMarkerLocations){
            Array<string> labels("", 3*nm+1);
            labels[0] = "time";
            Array<string> XYZ("", 3*nm);
            XYZ[0] = "_tx"; XYZ[1] = "_ty"; XYZ[2] = "_tz";

            for(int j=0; j<nm; ++j){
                for(int k=0; k<3; ++k)
                    labels.set(3*j+k+1, ikSolver.getMarkerNameForIndex(j)+XYZ[k]);
            }
            modelMarkerLocations->setColumnLabels(labels);
            modelMarkerLocations->setName("Model Marker Locations from IK");
    
            IO::makeDir(getResultsDir());
            string markerFileName = trialName + "_ik_model_marker_locations";
            Storage::printResult(modelMarkerLocations, markerFileName,
                                 getResultsDir(), -1, ".sto");

            delete modelMarkerLocations;
        }

        success = true;

        log_info("InverseKinematicsTool completed {} frames in {}.", Nframes,
            watch.getElapsedTimeFormatted());
    }
    catch (const std::exception& ex) {
        log_error("InverseKinematicsTool Failed: {}", ex.what());
        // If failure happened after kinematicsReporter was added, make sure to cleanup
        if (kinematicsReporter!= nullptr)
            _model->removeAnalysis(kinematicsReporter.get());
        throw (Exception("InverseKinematicsTool Failed, "
            "please see messages window for details..."));
    }

    if (modelFromFile) { 
        log_debug("Deleting Model {} at end of IK.run", _model->getName());
        delete _model.get();
        _model.reset();
    }

    return success;
}

// Handle conversion from older format
void InverseKinematicsTool::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if ( versionNumber < XMLDocument::getLatestVersion()){
        std::string newFileName = getDocumentFileName();
        if (versionNumber < 20300){
            std::string origFilename = getDocumentFileName();
            newFileName=IO::replaceSubstring(newFileName, ".xml", "_v23.xml");
            log_info("Old version setup file encountered. Converting to new "
                     "file '{}'.", newFileName);
            SimTK::Xml::Document doc = SimTK::Xml::Document(origFilename);
            doc.writeToFile(newFileName);
        }
        if (versionNumber <= 20201){
            // get filename and use SimTK::Xml to parse it
            SimTK::Xml::Document doc = SimTK::Xml::Document(newFileName);
            Xml::Element root = doc.getRootElement();
            if (root.getElementTag()=="OpenSimDocument"){
                int curVersion = root.getRequiredAttributeValueAs<int>("Version");
                if (curVersion <= 20201) root.setAttributeValue("Version", "20300");
                Xml::element_iterator iter(root.element_begin("IKTool"));
                iter->setElementTag("InverseKinematicsTool");
                Xml::element_iterator toolIter(iter->element_begin("IKTrialSet"));
                // No optimizer_algorithm specification anymore
                Xml::element_iterator optIter(iter->element_begin("optimizer_algorithm"));
                if (optIter!= iter->element_end())
                    iter->eraseNode(optIter);

                Xml::element_iterator objIter(toolIter->element_begin("objects")); 
                Xml::element_iterator trialIter(objIter->element_begin("IKTrial")); 
                // Move children of (*trialIter) to root
                Xml::node_iterator p = trialIter->node_begin();
                for (; p!= trialIter->node_end(); ++p) {
                    iter->insertNodeAfter( iter->node_end(), p->clone());
                }
                iter->insertNodeAfter( iter->node_end(), Xml::Element("constraint_weight", "20.0"));
                iter->insertNodeAfter( iter->node_end(), Xml::Element("accuracy", "1e-4"));
                // erase node for IKTrialSet
                iter->eraseNode(toolIter);  
                Xml::Document newDocument;
                Xml::Element docElement= newDocument.getRootElement();
                docElement.setAttributeValue("Version", "20300");
                docElement.setElementTag("OpenSimDocument");
                // Copy all children of root to newRoot
                docElement.insertNodeAfter(docElement.node_end(), iter->clone());
                newDocument.writeToFile(newFileName);
                setDocument(new XMLDocument(newFileName));
                aNode = updDocument()->getRootDataElement();
            }
            else { 
                if (root.getElementTag()=="IKTool"){
                    root.setElementTag("InverseKinematicsTool");
                    Xml::element_iterator toolIter(root.element_begin("IKTrialSet"));
                    if (toolIter== root.element_end())
                        throw (Exception("Old IKTool setup file doesn't have required IKTrialSet element.. Aborting"));
                    // No optimizer_algorithm specification anymore
                    Xml::element_iterator optIter(root.element_begin("optimizer_algorithm"));
                    if (optIter!= root.element_end())
                    root.eraseNode(optIter);

                    Xml::element_iterator objIter(toolIter->element_begin("objects")); 
                    Xml::element_iterator trialIter(objIter->element_begin("IKTrial")); 
                    // Move children of (*trialIter) to root
                    Xml::node_iterator p = trialIter->node_begin();
                    for (; p!= trialIter->node_end(); ++p) {
                        root.insertNodeAfter( root.node_end(), p->clone());
                    }
                    root.insertNodeAfter( root.node_end(), Xml::Element("constraint_weight", "20.0"));
                    root.insertNodeAfter( root.node_end(), Xml::Element("accuracy", "1e-5"));
                    // erase node for IKTrialSet
                    root.eraseNode(toolIter);
                    
                    // Create an OpenSimDocument node and move root inside it
                    Xml::Document newDocument;
                    Xml::Element docElement= newDocument.getRootElement();
                    docElement.setAttributeValue("Version", "20300");
                    docElement.setElementTag("OpenSimDocument");
                    // Copy all children of root to newRoot
                    docElement.insertNodeAfter(docElement.node_end(), doc.getRootElement().clone());
                    newDocument.writeToFile(newFileName);
                    setDocument(new XMLDocument(newFileName));
                    aNode = updDocument()->getRootDataElement();
                }
            }
        }
    }
    Object::updateFromXMLNode(aNode, versionNumber);
}

void InverseKinematicsTool::populateReferences(MarkersReference& markersReference,
    SimTK::Array_<CoordinateReference>&coordinateReferences) const
{
    FunctionSet *coordFunctions = NULL;
    // Load the coordinate data
    // bool haveCoordinateFile = false;
    if (get_coordinate_file() != "" && get_coordinate_file() != "Unassigned") {
        Storage coordinateValues(get_coordinate_file());
        // Convert degrees to radian (TODO: this needs to have a check that the storage is, in fact, in degrees!)
        _model->getSimbodyEngine().convertDegreesToRadians(coordinateValues);
        // haveCoordinateFile = true;
        coordFunctions = new GCVSplineSet(5, &coordinateValues);
    }

    Set<MarkerWeight> markerWeights;
    // Loop through old "IKTaskSet" and assign weights to the coordinate and marker references
    // For coordinates, create the functions for coordinate reference values
    int index = 0;
    for (int i = 0; i < get_IKTaskSet().getSize(); i++) {
        if (!get_IKTaskSet()[i].getApply()) continue;
        if (IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&get_IKTaskSet()[i])) {
            CoordinateReference *coordRef = NULL;
            if (coordTask->getValueType() == IKCoordinateTask::FromFile) {
                if (!coordFunctions)
                    throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");

                index = coordFunctions->getIndex(coordTask->getName(), index);
                if (index >= 0) {
                    coordRef = new CoordinateReference(coordTask->getName(), coordFunctions->get(index));
                }
            }
            else if ((coordTask->getValueType() == IKCoordinateTask::ManualValue)) {
                Constant reference(Constant(coordTask->getValue()));
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }
            else { // assume it should be held at its default value
                double value = _model->getCoordinateSet().get(coordTask->getName()).getDefaultValue();
                Constant reference = Constant(value);
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }

            if (coordRef == NULL)
                throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");
            else
                coordRef->setWeight(coordTask->getWeight());

            coordinateReferences.push_back(*coordRef);
        }
        else if (IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&get_IKTaskSet()[i])) {
            if (markerTask->getApply()) {
                // Only track markers that have a task and it is "applied"
                markerWeights.adoptAndAppend(
                    new MarkerWeight(markerTask->getName(), markerTask->getWeight()));
            }
        }
    }

    //Read in the marker data file and set the weights for associated markers.
    //Markers in the model and the marker file but not in the markerWeights are
    //ignored
    markersReference.initializeFromMarkersFile(get_marker_file(), markerWeights);
}


