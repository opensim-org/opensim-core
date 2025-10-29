/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Model.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ayman Habib, Ajay Seth,          *
 *            Michael Sherman                                                 *
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

#include "Model.h"

#include "Actuator.h"
#include "AnalysisSet.h"
#include "BodySet.h"
#include "ComponentSet.h"
#include "ContactGeometrySet.h"
#include "ControllerSet.h"
#include "CoordinateSet.h"
#include "ForceSet.h"
#include "Ligament.h"
#include "MarkerSet.h"
#include "ProbeSet.h"
#include "SimTKcommon/internal/SystemGuts.h"

#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Logger.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/AssemblySolver.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/Model/StationDefinedFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;
using namespace OpenSim;

namespace {
    template<typename TConcreteComponent>
    TConcreteComponent* MultibodyGraphMakerPtrCast(void* p)
    {
        // this helper function exists to try and add a little bit of
        // runtime safety checking to a `void*` downcast, to try and
        // downgrade what would otherwise be runtime segfaults (#3299)
        // into runtime exceptions (which can be caught, logged, etc.)

        if (!p)
        {
            return nullptr;
        }

        TConcreteComponent* casted = dynamic_cast<TConcreteComponent*>(static_cast<Component*>(p));

        if (!casted)
        {
            OPENSIM_THROW(OpenSim::Exception, "Failed to convert a pointer from a SimTK::MultibodyGraphMaker to the desired type. This is a known bug that can happen when trying to assemble models that have incorrect topologies (see opensim-core issue #3299). You might need to change/fix your model's topology, or post a comment on that issue.");
        }

        return casted;
    }

    // helper: internal class used by the depth-first topology sorter
    struct MarkedFrameRef final {

        explicit MarkedFrameRef(std::reference_wrapper<Frame> ref_) :
            ref{ref_}
        {}

        friend bool operator==(const MarkedFrameRef& lhs, const Frame& rhs)
        {
            return &lhs.ref.get() == &rhs;
        }

        std::reference_wrapper<Frame> ref;
        bool permanentlyMarked = false;
        bool temporarilyMarked = false;
    };

    // helper: the recursive `visit` step in a depth-first topology sort
    //
    // it recursively visits a `node` (marked frame) by attempting to traverse
    // to its parent (PhysicalOffsetFrame/StationDefinedFrame).
    void RecursivelyVisitNode(
        OpenSim::Component const& root,
        std::vector<std::reference_wrapper<Frame>>& sorted,
        std::vector<MarkedFrameRef>& nodes,
        MarkedFrameRef& node)
    {
        if (node.permanentlyMarked) {
            return;
        }
        if (node.temporarilyMarked) {
            OPENSIM_THROW(PhysicalOffsetFramesFormLoop, root, node.ref.get().getName());
        }

        // recurse
        node.temporarilyMarked = true;
        if (auto* pof = dynamic_cast<PhysicalOffsetFrame*>(&node.ref.get())) {
            auto const it = std::find(nodes.begin(), nodes.end(), pof->getParentFrame());
            if (it != nodes.end()) {
                RecursivelyVisitNode(root, sorted, nodes, *it);
            }
        }
        else if (auto* sdf = dynamic_cast<StationDefinedFrame*>(&node.ref.get())) {
            auto const it = std::find(nodes.begin(), nodes.end(), sdf->findBaseFrame());
            if (it != nodes.end()) {
                RecursivelyVisitNode(root, sorted, nodes, *it);
            }
        }
        node.temporarilyMarked = false;

        // finish with this node
        node.permanentlyMarked = true;
        sorted.push_back(node.ref);
    }

    // helper: extract `Frame`s under the given `root` in topological order from
    // "least dependent" to "most dependent"
    //
    // throws if a graph cycle is detected between the frames (this method should
    // be called after all graph cycles have been broken)
    std::vector<std::reference_wrapper<Frame>> TopologicallySortedFrames(Component& root)
    {
        // perform a depth-first search to perform the topological sort, see:
        //
        // - https://en.wikipedia.org/wiki/Topological_sorting
        // - https://en.wikipedia.org/wiki/Depth-first_search

        // collect all frames into `MarkedFrame` references
        std::vector<MarkedFrameRef> markedFrameRefs;
        for (auto& frame : root.updComponentList<Frame>()) {
            markedFrameRefs.emplace_back(frame);
        }

        std::vector<std::reference_wrapper<Frame>> sortedFrames;
        sortedFrames.reserve(markedFrameRefs.size());

        for (auto& node : markedFrameRefs) {
            RecursivelyVisitNode(root, sortedFrames, markedFrameRefs, node);
        }

        return sortedFrames;
    }
}

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor.
 */
Model::Model() : ModelComponent(),
    _fileName("Unassigned"),
    _analysisSet(AnalysisSet()),
    _coordinateSet(CoordinateSet()),
    _workingState(),
    _useVisualizer(false),
    _allControllersEnabled(true)
{
    constructProperties();
    setNull();
    finalizeFromProperties();
}
//_____________________________________________________________________________
/*
 * Constructor from an XML file
 */
Model::Model(const string &aFileName) :
    ModelComponent(aFileName, false),
    _fileName("Unassigned"),
    _analysisSet(AnalysisSet()),
    _coordinateSet(CoordinateSet()),
    _workingState(),
    _useVisualizer(false),
    _allControllersEnabled(true)
{
    constructProperties();
    setNull();
    updateFromXMLDocument();
    // Check is done below because only Model files have migration issues, version is not available until
    // updateFromXMLDocument is called. Fixes core issue #2395
    OPENSIM_THROW_IF(getDocument()->getDocumentVersion() < 10901,
        Exception,
        "Model file " + aFileName + " is using unsupported file format"
        ". Please open model and save it in OpenSim version 3.3 to upgrade.");

    _fileName = aFileName;
    log_info("Loaded model {} from file {}", getName(), getInputFileName());

    try {
        finalizeFromProperties();
    }
    catch(const InvalidPropertyValue& err) {
        log_error("Model was unable to finalizeFromProperties."
                  "Update the model file and reload OR update the property and "
                  "call finalizeFromProperties() on the model."
                  "(details: {}).",
                err.what());
    }
}

Model* Model::clone() const
{
    // Invoke default copy constructor.
    Model* clone = new Model(*this);

    try {
        clone->finalizeFromProperties();
    }
    catch (const InvalidPropertyValue& err) {
        log_error(
                "clone() was unable to finalizeFromProperties."
                "Update the model and call clone() again OR update the clone's "
                " property and call finalizeFromProperties() on it. (details: {}).",
                err.what());
    }

    return clone;
}

//_____________________________________________________________________________
/*
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
/*virtual*/
void Model::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if (versionNumber < XMLDocument::getLatestVersion()){
        log_info("Updating Model file from {} to latest format...",
                versionNumber);
        // Version has to be 1.6 or later, otherwise assert
        if (versionNumber == 10600){
            // Get node for DynamicsEngine
            SimTK::Xml::element_iterator engIter =
                aNode.element_begin("DynamicsEngine");
            //Get node for SimbodyEngine
            if (engIter != aNode.element_end()){
                SimTK::Xml::element_iterator simbodyEngIter =
                    engIter->element_begin("SimbodyEngine");
                // Move all Children of simbodyEngineNode to be children of _node
                // we'll keep inserting before enginesNode then remove it;
                SimTK::Array_<SimTK::Xml::Element> elts =
                    simbodyEngIter->getAllElements();
                while (elts.size() != 0){
                    // get first child and move it to Model
                    aNode.insertNodeAfter(aNode.element_end(),
                        simbodyEngIter->removeNode(simbodyEngIter->element_begin()));
                    elts = simbodyEngIter->getAllElements();
                }
                engIter->eraseNode(simbodyEngIter);
            }
            // Now handling the rename of ActuatorSet to ForceSet
            XMLDocument::renameChildNode(aNode, "ActuatorSet", "ForceSet");
        }
        if (versionNumber < 30501) {
            // Create JointSet node after BodySet under <OpenSimDocument>/<Model>
            SimTK::Xml::Element jointSetElement("JointSet");
            SimTK::Xml::Element jointObjects("objects");
            jointSetElement.insertNodeBefore(jointSetElement.element_begin(), jointObjects);
            SimTK::Xml::element_iterator bodySetNode = aNode.element_begin("BodySet");
            aNode.insertNodeAfter(bodySetNode, jointSetElement);
            // Cycle through Bodies and move their Joint nodes under the Model's JointSet
            SimTK::Xml::element_iterator  objects_node =
                bodySetNode->element_begin("objects");
            SimTK::Xml::element_iterator bodyIter =
                objects_node->element_begin("Body");
            for (; bodyIter != objects_node->element_end(); ++bodyIter) {
                std::string body_name = bodyIter->getOptionalAttributeValue("name");
                if (body_name == "ground") {
                    SimTK::Xml::Element newGroundElement("Ground");
                    newGroundElement.setAttributeValue("name", "ground");

                    SimTK::Xml::element_iterator geometryIter =
                        bodyIter->element_begin("geometry");
                    if (geometryIter != bodyIter->element_end()) {
                        SimTK::Xml::Element cloneOfGeomety = geometryIter->clone();
                        newGroundElement.insertNodeAfter(newGroundElement.node_end(),
                            cloneOfGeomety);
                    }

                    SimTK::Xml::element_iterator visObjIter =
                        bodyIter->element_begin("VisibleObject");
                    if (visObjIter != bodyIter->element_end()) {
                        SimTK::Xml::Element cloneOfVisObj = visObjIter->clone();
                        newGroundElement.insertNodeAfter(newGroundElement.node_end(),
                            cloneOfVisObj);
                    }

                    SimTK::Xml::element_iterator wrapSetIter =
                        bodyIter->element_begin("WrapObjectSet");
                    if (wrapSetIter != bodyIter->element_end()) {
                        SimTK::Xml::Element cloneOfWrapSet = wrapSetIter->clone();
                        newGroundElement.insertNodeAfter(newGroundElement.node_end(),
                            cloneOfWrapSet);
                    }

                    SimTK::String test;
                    newGroundElement.writeToString(test);

                    objects_node->eraseNode(bodyIter);

                    aNode.insertNodeBefore(bodySetNode, newGroundElement);
                    break;
                }
            }
            bodyIter = objects_node->element_begin("Body");
            for (; bodyIter != objects_node->element_end(); ++bodyIter) {
                std::string body_name = bodyIter->getOptionalAttributeValue("name");
                SimTK::Xml::element_iterator  joint_node =
                    bodyIter->element_begin("Joint");
                if (joint_node->element_begin() != joint_node->element_end()){
                    SimTK::Xml::Element detach_joint_node = joint_node->clone();
                    SimTK::Xml::element_iterator concreteJointNode =
                        detach_joint_node.element_begin();
                    detach_joint_node.removeNode(concreteJointNode);
                    SimTK::Xml::element_iterator parentBodyElement =
                        concreteJointNode->element_begin("parent_body");
                    SimTK::String parent_name = "ground";
                    parentBodyElement->getValueAs<SimTK::String>(parent_name);

                    // In version 30501, this Joint is 1 level deep (in the
                    // model's JointSet), and the Bodies are necessarily 1
                    // level deep (in the model's BodySet). Prepend "../" to
                    // get the correct relative path.
                    std::string parent_frame = parent_name;
                    if (!parent_frame.empty())
                        parent_frame = "../" + parent_frame;
                    XMLDocument::addConnector(*concreteJointNode,
                        "Connector_PhysicalFrame_", "parent_frame",
                        parent_frame);
                    std::string child_frame = body_name;
                    if (!child_frame.empty())
                        child_frame = "../" + child_frame;
                    XMLDocument::addConnector(*concreteJointNode,
                        "Connector_PhysicalFrame_", "child_frame",
                        child_frame);
                    concreteJointNode->eraseNode(parentBodyElement);
                    jointObjects.insertNodeAfter(jointObjects.node_end(),
                        *concreteJointNode);
                    detach_joint_node.clearOrphan();
                }
                bodyIter->eraseNode(joint_node);
            }
        }
        if (versionNumber < 30512) {
            // FrameSet was removed from Model as of 30512 and this update
            // is responsible for moving the Frames in the FrameSet to
            // the Model's list of components.
            SimTK::Xml::element_iterator componentsNode =
                aNode.element_begin("components");
            SimTK::Xml::element_iterator framesNode =
                aNode.element_begin("FrameSet");

            // If no FrameSet nothing to be done
            if (framesNode != aNode.element_end()) {
                if (componentsNode == aNode.element_end()) {
                    // if no 'components' list element, create one and insert it
                    SimTK::Xml::Element componentsElement("components");
                    aNode.insertNodeBefore(framesNode, componentsElement);
                }

                componentsNode = aNode.element_begin("components");

                SimTK::Xml::element_iterator  objects_node =
                    framesNode->element_begin("objects");

                SimTK::Xml::element_iterator frameIter =
                    objects_node->element_begin();
                for (; frameIter != objects_node->element_end(); ++frameIter) {
                    SimTK::Xml::Element cloneOfFrame = frameIter->clone();
                    componentsNode->insertNodeAfter(componentsNode->node_end(),
                        cloneOfFrame);
                }
                // now delete the FrameSet
                framesNode->getParentElement().eraseNode(framesNode);
            }
        }
    }
     // Call base class now assuming _node has been corrected for current version
     Super::updateFromXMLNode(aNode, versionNumber);
     setDefaultProperties();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the values of all data members to an appropriate "null" value.
 */
void Model::setNull()
{
    _useVisualizer = false;
    _allControllersEnabled = true;

    _validationLog="";

    _analysisSet.setMemoryOwner(false);
}

void Model::constructProperties()
{
    constructProperty_assembly_accuracy(1e-9);

    constructProperty_ground(Ground());

    constructProperty_gravity(SimTK::Vec3(0.0, -9.80665, 0.0));

    constructProperty_credits("Frank Anderson, Peter Loan, Ayman Habib, Ajay Seth, Michael Sherman");

    constructProperty_publications("List of publications related to model...");

    constructProperty_length_units("meters");
    _lengthUnits = Units::Meters;

    constructProperty_force_units("N");
    _forceUnits = Units::Newtons;

    BodySet bodies;
    bodies.setName(IO::Lowercase(bodies.getConcreteClassName()));
    constructProperty_BodySet(bodies);

    JointSet joints;
    joints.setName(IO::Lowercase(joints.getConcreteClassName()));
    constructProperty_JointSet(joints);

    ControllerSet controllers;
    controllers.setName(IO::Lowercase(controllers.getConcreteClassName()));
    constructProperty_ControllerSet(controllers);

    ConstraintSet constraints;
    constraints.setName(IO::Lowercase(constraints.getConcreteClassName()));
    constructProperty_ConstraintSet(constraints);

    ForceSet forces;
    forces.setName(IO::Lowercase(forces.getConcreteClassName()));
    constructProperty_ForceSet(forces);

    MarkerSet markers;
    markers.setName(IO::Lowercase(markers.getConcreteClassName()));
    constructProperty_MarkerSet(markers);

    ContactGeometrySet contacts;
    contacts.setName(IO::Lowercase(contacts.getConcreteClassName()));
    constructProperty_ContactGeometrySet(contacts);

    ProbeSet probes;
    probes.setName(IO::Lowercase(probes.getConcreteClassName()));
    constructProperty_ProbeSet(probes);

    ComponentSet miscComponents;
    miscComponents.setName(IO::Lowercase(miscComponents.getConcreteClassName()));
    constructProperty_ComponentSet(miscComponents);

    ModelVisualPreferences mvps;
    mvps.setName(IO::Lowercase(mvps.getConcreteClassName()));
    constructProperty_ModelVisualPreferences(mvps);
}

// Append to the Model's validation log
void Model::appendToValidationLog(const std::string& note) {
    _validationLog.append(note);
}

//------------------------------------------------------------------------------
//                                BUILD SYSTEM
//------------------------------------------------------------------------------
// Perform some final checks on the Model, wire up all its components, and then
// build a computational System for it.
void Model::buildSystem() {
    // Finish connecting up the Model.
    setup();

    // Create the computational System representing this Model.
    createMultibodySystem();

    // Create a Visualizer for this Model if one has been requested. This adds
    // necessary elements to the System. Doesn't initialize geometry yet.
    if (getUseVisualizer())
        _modelViz.reset(new ModelVisualizer(*this));
}


//------------------------------------------------------------------------------
//                            INITIALIZE STATE
//------------------------------------------------------------------------------
// Requires that buildSystem() has already been called.
SimTK::State& Model::initializeState() {
    if (!hasSystem())
        throw Exception("Model::initializeState(): call buildSystem() first.");

    // This tells Simbody to finalize the System.
    getMultibodySystem().invalidateSystemTopologyCache();
    getMultibodySystem().realizeTopology();

    // Set the model's operating state (internal member variable) to the
    // default state that is stored inside the System.
    _workingState = getMultibodySystem().getDefaultState();

    // Set the Simbody modeling option that tells any joints that use
    // quaternions to use Euler angles instead.
    _matter->setUseEulerAngles(_workingState, true);

    // Process the modified modeling option.
    getMultibodySystem().realizeModel(_workingState);

    // Invoke the ModelComponent interface for initializing the state.
    initStateFromProperties(_workingState);

    // Realize instance variables that may have been set above. This
    // means floating point parameters such as mass properties and
    // geometry placements are frozen.
    getMultibodySystem().realize(_workingState, SimTK::Stage::Instance);

    // Realize the initial configuration in preparation for assembly. This
    // initial configuration does not necessarily satisfy constraints.
    getMultibodySystem().realize(_workingState, SimTK::Stage::Position);

    // Reset (initialize) all underlying Probe SimTK::Measures
    for (int i=0; i<getProbeSet().getSize(); ++i)
        getProbeSet().get(i).reset(_workingState);

    // Do the assembly
    createAssemblySolver(_workingState);
    assemble(_workingState);
    // We can now collect up all the fixed geometry, which needs full configuration.
    if (getUseVisualizer())
        _modelViz->collectFixedGeometry(_workingState);

    return _workingState;
}


SimTK::State& Model::updWorkingState()
{
    if (!isValidSystem())
        throw Exception("Model::updWorkingState(): call initializeState() first.");

    return _workingState;
}


const SimTK::State& Model::getWorkingState() const
{
    if (!isValidSystem())
        throw Exception("Model::getWorkingState(): call initializeState() first.");

    return _workingState;
}


void Model::assemble(SimTK::State& s, const Coordinate *coord, double weight)
{

    bool constrained = false;
    const CoordinateSet &coords = getCoordinateSet();
    for(int i=0; i<coords.getSize(); ++i){
        constrained = constrained || coords[i].isConstrained(s);
    }

    // Don't bother assembling if the model has no constraints
    if(get_ConstraintSet().getSize()< 1){
        // just realize the current state to position
        getMultibodySystem().realize(s, SimTK::Stage::Position);

        // if a coordinate is locked or prescribed, then project will suffice
        if(constrained){
            // correct position constraint violations due to prescribed motion
            getMultibodySystem().projectQ(s, 1e-10);

            // Have a new working configuration so should realize to velocity
            getMultibodySystem().realize(s, SimTK::Stage::Velocity);
            // correct velocity constraint violations due to prescribed motion
            getMultibodySystem().projectU(s, 1e-10);
        }
        return;
    }

    if (!_assemblySolver){
        createAssemblySolver(s);
    }
    const SimTK::Array_<CoordinateReference>& coordRefs =
            _assemblySolver->getCoordinateReferences();

    for(unsigned int i=0; i<coordRefs.size(); i++){
        const string &coordName = coordRefs[i].getName();
        Coordinate& c = _coordinateSet.get(coordName);
        _assemblySolver->updateCoordinateReference(coordName, c.getValue(s));
    }

    if(coord) // use specified weighting for coordinate being set
        _assemblySolver->updateCoordinateReference(coord->getName(), coord->getValue(s), weight);


    try{
        // Try to track first with model satisfying the constraints exactly.
        _assemblySolver->track(s);
    }
    catch (const std::exception&)    {
        try{
            // Otherwise try to do a full-blown assemble
            _assemblySolver->assemble(s);
        }
        catch (const std::exception& ex){
            // Constraints are probably infeasible so try again relaxing constraints
            log_error("Model unable to assemble: {}."
                      "Model relaxing constraints and trying again.",
                    ex.what());
            try{
                // Try to satisfy with constraints as errors weighted heavily.
                _assemblySolver->setConstraintWeight(20.0);
                _assemblySolver->assemble(s);
            }
            catch (const std::exception& ex){
                log_error("Model unable to assemble with relaxed constraints: {}",
                    ex.what());
            }
        }
    }

    // Have a new working configuration so should realize to velocity
    getMultibodySystem().realize(s, SimTK::Stage::Velocity);
}

void Model::invalidateSystem()
{
    if (_system)
        _system->getSystemGuts().invalidateSystemTopologyCache();
}

bool Model::isValidSystem() const
{
    if (_system)
        return _system->systemTopologyHasBeenRealized();
    else
        return false;
}


//_____________________________________________________________________________
/**
 * Create the multibody system.
 *
 */
void Model::createMultibodySystem()
{
    // We must reset these unique_ptr's before deleting the System (through
    // reset()), since deleting the System puts a null handle pointer inside
    // the subsystems (since System deletes the subsystems).
    _matter.reset();
    _forceSubsystem.reset();
    _contactSubsystem.reset();
    _cableSubsystem.reset();
    // create system
    _system.reset(new SimTK::MultibodySystem);
    _matter.reset(new SimTK::SimbodyMatterSubsystem(*_system));
    _forceSubsystem.reset(new SimTK::GeneralForceSubsystem(*_system));
    _contactSubsystem.reset(new SimTK::GeneralContactSubsystem(*_system));
    _cableSubsystem.reset(new SimTK::CableSubsystem(*_system));

    // create gravity force, a direction is needed even if magnitude=0 for
    // PotentialEnergy purposes.
    double magnitude = get_gravity().norm();
    SimTK::UnitVec3 direction = magnitude==0 ? SimTK::UnitVec3(0,-1,0) : SimTK::UnitVec3(get_gravity()/magnitude);
    _gravityForce.reset(new SimTK::Force::Gravity(*_forceSubsystem, *_matter,
                direction, magnitude));

    addToSystem(*_system);
}


std::vector<SimTK::ReferencePtr<const Coordinate>>
    Model::getCoordinatesInMultibodyTreeOrder() const
{
    OPENSIM_THROW_IF_FRMOBJ(!isValidSystem(), Exception,
        "Cannot order Coordinates without a valid MultibodySystem.");

    int nc = getNumCoordinates();
    auto coordinates = getComponentList<Coordinate>();

    std::vector<SimTK::ReferencePtr<const Coordinate>>
        coordinatesInTreeOrder(nc,
            SimTK::ReferencePtr<const Coordinate>(*coordinates.begin()));

    // We have a valid MultibodySystem underlying the Coordinates
    const SimTK::State& s = getWorkingState();
    SimTK_ASSERT_ALWAYS(nc <= s.getNQ(),
        "Number of Coordinates exceeds the number of generalized coordinates in "
        "the underlying MultibodySystem.");

    auto& matter = getSystem().getMatterSubsystem();

    int cnt = 0;

    for (auto& coord : coordinates) {
        auto mbix = coord.getBodyIndex();
        auto mqix = coord.getMobilizerQIndex();

        int cix = matter.getMobilizedBody(mbix).getFirstUIndex(s) + mqix;

        SimTK_ASSERT_ALWAYS(cix < nc, "Index exceeds the number of Coordinates "
            "in this Model.");

        coordinatesInTreeOrder.at(cix).reset(&coord);
        cnt++;
    }

    SimTK_ASSERT_ALWAYS(cnt == nc,
        "The number of ordered Coordinates does not correspond to the number of "
        "Coordinates in the Model's CoordinateSet.");

    return coordinatesInTreeOrder;
}

std::string Model::getWarningMesssageForMotionTypeInconsistency() const
{
    std::string message;

    auto enumToString = [](Coordinate::MotionType mt)->std::string {
        switch (mt) {
        case Coordinate::MotionType::Rotational :
            return "Rotational";
        case Coordinate::MotionType::Translational :
            return "Translational";
        case Coordinate::MotionType::Coupled :
            return "Coupled";
        default:
            return "Undefined";
        }
    };

    auto coordinates = getComponentList<Coordinate>();
    for (auto& coord : coordinates) {
        const Coordinate::MotionType oldMotionType =
            coord.getUserSpecifiedMotionTypePriorTo40();
        const Coordinate::MotionType motionType = coord.getMotionType();

        if( (oldMotionType != Coordinate::MotionType::Undefined ) &&
            (oldMotionType != motionType) ){
            message += "Coordinate '" + coord.getName() +
                "' was labeled as '" + enumToString(oldMotionType) +
                "' but was found to be '" + enumToString(motionType) + "' based on the joint definition.\n";
        }
    }

    // We have a reason to provide a warning. Add more details about the model
    // and how to resolve future issues.
    if (message.size()) {
        message = "\nModel '" + getName() + "' has inconsistencies:\n" + message;
        message +=
            "You must update any motion files you generated in versions prior to 4.0. You can:\n"
            "  (1) Run the updatePre40KinematicsFilesFor40MotionType() utility (in the scripting shell) OR\n"
            "  (2) Re-run the Inverse Kinematics Tool with the updated model in 4.0.\n"
            "In versions prior to 4.0, we allowed some Coupled coordinates to be incorrectly\n"
            "labeled as Rotational. This leads to incorrect motion when playing back a pre-4.0\n"
            "motion file (.mot or .sto in degrees) and incorrect inverse dynamics and\n"
            "static optimization results.";
    }

    return message;
}

void Model::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    // wipe-out the existing System: make sure to wipe subsystems first
    _matter.reset();
    _forceSubsystem.reset();
    _contactSubsystem.reset();
    _cableSubsystem.reset();

    _system.reset();  // after wiping all subsystems

    if(getForceSet().getSize()>0)
    {
        ForceSet &fs = updForceSet();
        // Update internal subsets of the ForceSet
        fs.updActuators();
        fs.updMuscles();
    }

    std::string warn = getWarningMesssageForMotionTypeInconsistency();
    appendToValidationLog(warn);

    updCoordinateSet().populate(*this);
}

void Model::createMultibodyTree()
{
    // building the system for the first time, need to tell
    // multibodyTree builder what joints are available
    _multibodyTree.clearGraph();
    _multibodyTree.setWeldJointTypeName("WeldJoint");
    _multibodyTree.setFreeJointTypeName("FreeJoint");

    ArrayPtrs<OpenSim::Joint> availablJointTypes;
    Object::getRegisteredObjectsOfGivenType<OpenSim::Joint>(availablJointTypes);
    for (int i = 0; i< availablJointTypes.getSize(); i++) {
        OpenSim::Joint* jt = availablJointTypes[i];
        if ((jt->getConcreteClassName() == "WeldJoint") ||
            (jt->getConcreteClassName() == "FreeJoint")) {
            continue;
        }
        else {
            _multibodyTree.addJointType(
                availablJointTypes[i]->getConcreteClassName(),
                availablJointTypes[i]->numCoordinates(), false);
        }
    }

    Ground& ground = updGround();

    // assemble a multibody tree according to the PhysicalFrames in the
    // OpenSim model, which include Ground and Bodies
    _multibodyTree.addBody(ground.getAbsolutePathString(),
                           0, false, &ground);

    auto bodies = getComponentList<Body>();
    for (auto& body : bodies) {
        _multibodyTree.addBody( body.getAbsolutePathString(),
                                body.getMass(), false,
                                const_cast<Body*>(&body) );
    }

    std::vector<SimTK::ReferencePtr<Joint>> joints;
    for (auto& joint : updComponentList<Joint>()) {
        joints.push_back(SimTK::ReferencePtr<Joint>(&joint));
    }

    // Complete multibody tree description by indicating how (mobilized)
    // "bodies" are connected by joints.
    for (auto& joint : joints) {
        std::string name = joint->getAbsolutePathString();
        IO::TrimWhitespace(name);

        // Currently we need to take a first pass at connecting the joints
        // in order to ask the joint for the frames that they attach to and
        // to determine their underlying base (physical) frames.
        joint->finalizeConnections(*this);

        // Verify that the underlying frames are also connected so we can
        // traverse to the base frame and get its name. This allows the
        // (offset) frames to satisfy the sockets of Joint to be added
        // to a Body, for example, and not just joint itself.
        const PhysicalFrame& parent = joint->getParentFrame();
        const PhysicalFrame& child = joint->getChildFrame();
        const_cast<PhysicalFrame&>(parent).finalizeConnections(*this);
        const_cast<PhysicalFrame&>(child).finalizeConnections(*this);

        OPENSIM_THROW_IF(&parent.findBaseFrame() == &child.findBaseFrame(),
            JointFramesHaveSameBaseFrame, getName(),
            parent.getName(), child.getName(), child.findBaseFrame().getName());

        // Use joints to define the underlying multibody tree
        _multibodyTree.addJoint(name,
            joint->getConcreteClassName(),
            // Multibody tree builder only cares about bodies not intermediate
            // frames that joints actually connect to.
            parent.findBaseFrame().getAbsolutePathString(),
            child.findBaseFrame().getAbsolutePathString(),
            false,
            joint.get());
    }
}

void Model::extendConnectToModel(Model &model)
{
    Super::extendConnectToModel(model);

    if (&model != this){
        log_info("Model:: {} is being connected to model {}.", getName(),
                model.getName());
        // if part of another Model, that Model is in charge
        // of creating a valid Multibody tree that includes
        // Components of this Model.
        return;
    }

    // Create the Multibody tree according to the components that
    // form this model.
    createMultibodyTree();

    // generate the graph of the Multibody tree to determine the order in
    // which subcomponents will be added to the MultibodySystem (in addToSystem)
    _multibodyTree.generateGraph();
    //_multibodyTree.dumpGraph(cout);
    //cout << endl;

    // Ground is the first component of any MultibodySystem
    Ground& ground = updGround();
    setNextSubcomponentInSystem(ground);

    // The JointSet of the Model is only being manipulated for consistency with
    // Tool expectations. TODO fix Tools and remove
    JointSet& joints = upd_JointSet();

    bool isMemoryOwner = joints.getMemoryOwner();
    //Temporarily set owner ship to false so we
    //can swap to rearrange order of the joints
    joints.setMemoryOwner(false);

    // Run through all the mobilizers in the multibody tree, adding
    // a joint in the correct sequence. Also add massless bodies,
    // loop closure constraints, etc... to form the valid tree.
    for (int m = 0; m < _multibodyTree.getNumMobilizers(); ++m) {
        // Get a mobilizer from the tree, then extract its corresponding
        // joint and bodies. Note that these should have equivalents in OpenSim.
        const SimTK::MultibodyGraphMaker::Mobilizer& mob =
                _multibodyTree.getMobilizer(m);

        if (mob.isSlaveMobilizer()){
            // add the slave body and joint
            Body* outbMaster = MultibodyGraphMakerPtrCast<Body>(mob.getOutboardMasterBodyRef());
            Joint* useJoint = MultibodyGraphMakerPtrCast<Joint>(mob.getJointRef());
            Body* outb = MultibodyGraphMakerPtrCast<Body>(mob.getOutboardBodyRef());

            // the joint must be added to the system next
            setNextSubcomponentInSystem(*useJoint);

            if (!outb) {
                outb = outbMaster->addSlave();
                useJoint->setSlaveBodyForChild(*outb);
                SimTK::Transform o(SimTK::Vec3(0));
                //Now add the constraints that weld the slave to the master at the
                // body origin
                std::string pathName = outb->getAbsolutePathString();
                WeldConstraint* weld = new WeldConstraint(outb->getName()+"_weld",
                                                          *outbMaster, o, *outb, o);

                // include within adopted list of owned components
                adoptSubcomponent(weld);
                setNextSubcomponentInSystem(*weld);
            }
        }

        if (mob.isAddedBaseMobilizer()){
            // create and add the base joint to enable these dofs
            Body* child = MultibodyGraphMakerPtrCast<Body>(mob.getOutboardBodyRef());
            log_warn("Body '{}' not connected by a Joint."
                "A FreeJoint will be added to connect it to ground.",
                child->getName());
            Ground* ground = MultibodyGraphMakerPtrCast<Ground>(mob.getInboardBodyRef());

            std::string jname = "free_" + child->getName();
            SimTK::Vec3 zeroVec(0.0);
            Joint* free = new FreeJoint(jname, *ground, *child);
            free->isReversed = mob.isReversedFromJoint();
            // TODO: Joints are currently required to be in the JointSet
            // When the reordering of Joints is eliminated (see following else block)
            // this limitation can be removed and the free joint adopted as in
            // internal subcomponent (similar to the weld constraint above)
            addJoint(free);
            setNextSubcomponentInSystem(*free);
        }
        else{
            // Update the directionality of the joint according to tree's
            // preferential direction
            MultibodyGraphMakerPtrCast<Joint>(mob.getJointRef())->isReversed =
                mob.isReversedFromJoint();

            // order the joint components in the order of the multibody tree
            Joint* joint = MultibodyGraphMakerPtrCast<Joint>(mob.getJointRef());
            setNextSubcomponentInSystem(*joint);


        }
    }
    joints.setMemoryOwner(isMemoryOwner);

    // Add the loop joints if any.
    for (int lcx = 0; lcx < _multibodyTree.getNumLoopConstraints(); ++lcx) {
        const SimTK::MultibodyGraphMaker::LoopConstraint& loop =
                _multibodyTree.getLoopConstraint(lcx);

        Joint& joint = *MultibodyGraphMakerPtrCast<Joint>(loop.getJointRef());
        Body&  parent = *MultibodyGraphMakerPtrCast<Body>(loop.getParentBodyRef());
        Body&  child = *MultibodyGraphMakerPtrCast<Body>(loop.getChildBodyRef());

        if (joint.getConcreteClassName() == "WeldJoint") {
            WeldConstraint* weld = new WeldConstraint( joint.getName()+"_Loop",
                parent, joint.getParentFrame().findTransformInBaseFrame(),
                child, joint.getChildFrame().findTransformInBaseFrame());
            adoptSubcomponent(weld);
            setNextSubcomponentInSystem(*weld);
        }
        else if (joint.getConcreteClassName() == "BallJoint") {
            PointConstraint* point = new PointConstraint(
                parent, joint.getParentFrame().findTransformInBaseFrame().p(),
                child, joint.getChildFrame().findTransformInBaseFrame().p());
            point->setName(joint.getName() + "_Loop");
            adoptSubcomponent(point);
            setNextSubcomponentInSystem(*point);
        }
        else if (joint.getConcreteClassName() == "FreeJoint") {
            // A "free" loop constraint is no constraint at all so we can
            // just ignore it. It might be more convenient if there were
            // a 0-constraint Constraint::Free, just as there is a 0-mobility
            // MobilizedBody::Weld.
        }
        else
            throw std::runtime_error(
            "Unrecognized loop constraint type '" + joint.getConcreteClassName() + "'.");
    }

    // special case: topologically sort dependent frames
    //
    // `PhysicalOffsetFrame`/`StationDefinedFrame` are dependent on their
    // parent/base `Frame` being added to the Simbody system before them, and
    // those `Frame`s might, themselves, be `PhysicalOffsetFrames`, and so on
    //
    // so collect all the frames in the model up and topologically sort them
    // from "least-dependent"/"closer to ground" to "depends on other frames"
    for (Frame& frame : TopologicallySortedFrames(*this)) {
        if (dynamic_cast<PhysicalOffsetFrame*>(&frame) ||
            dynamic_cast<StationDefinedFrame*>(&frame)) {

            // finalizing connections here ensures that the sockets bake
            // correct paths (e.g. `/ground` vs. `../../../ground`)
            frame.finalizeConnections(*this);

            setNextSubcomponentInSystem(frame);
        }
    }

    // and everything else including Forces, Controllers, etc...
    // belonging to the model
    auto components = getImmediateSubcomponents();
    for (const auto& comp : components) {
        setNextSubcomponentInSystem(comp.getRef());
    }

    // Populate the model's list of Coordinates as references into
    // the individual Joints.
    updCoordinateSet().populate(*this);
    updForceSet().setupGroups();
    updControllerSet().setActuators(updActuators());

    // TODO: Get rid of the SimbodyEngine
    updSimbodyEngine().connectSimbodyEngineToModel(*this);

    // Now that the model is fully connected, cache controllers
    // locally to reduce the runtime cost of Model::computeControls()
    this->_enabledControllers.clear();
    for (const Controller& controller : getComponentList<Controller>()) {
        if (controller.isEnabled()) {
            this->_enabledControllers.emplace_back(controller);
        }
    }
}


// ModelComponent interface enables this model to be a subcomponent of another
// model. In that case, it adds itself to the parent model's system.
void Model::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    Model *mutableThis = const_cast<Model *>(this);

    //Analyses are not Components so add them after legit
    //Components have been wired-up correctly.
    mutableThis->updAnalysisSet().setModel(*mutableThis);

    // Reset the vector of all controls' defaults
    mutableThis->_defaultControls.resize(0);

    // Create the shared cache that will hold all model controls
    // This must be created before Actuator.extendAddToSystem() since Actuator
    // will append its "slots" and retain its index by accessing this cached
    // Vector. Value depends on velocity and invalidates dynamics BUT should not
    // trigger re-computation of the controls which are necessary for dynamics
    SimTK::Measure_<SimTK::Vector>::Result modelControls(
            system.updDefaultSubsystem(), SimTK::Stage::Velocity,
            SimTK::Stage::Acceleration);

    mutableThis->_modelControlsIndex = modelControls.getSubsystemMeasureIndex();
}


// Add any Component derived from ModelComponent to the Model
void Model::addModelComponent(ModelComponent* component)
{
    if(component){
        upd_ComponentSet().adoptAndAppend(component);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*component);
    }
}

// Add a body to the Model
void Model::addBody(OpenSim::Body* body)
{
    if (body){
        updBodySet().adoptAndAppend(body);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*body);
    }
}

// Add a Marker to the Model
void Model::addMarker(OpenSim::Marker* marker)
{
    if (marker){
        updMarkerSet().adoptAndAppend(marker);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*marker);
    }
}

// Add a joint to the Model
void Model::addJoint(Joint* joint)
{
    if (joint){
        updJointSet().adoptAndAppend(joint);
        finalizeFromProperties();
        updCoordinateSet().populate(*this);
        prependComponentPathToConnecteePath(*joint);
    }
}

// Add a constraint to the Model
void Model::addConstraint(OpenSim::Constraint *constraint)
{
    if(constraint){
        updConstraintSet().adoptAndAppend(constraint);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*constraint);
    }
}

// Add a force to the Model
void Model::addForce(OpenSim::Force *force)
{
    if(force){
        updForceSet().adoptAndAppend(force);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*force);
    }
}

// Add a probe to the Model
void Model::addProbe(OpenSim::Probe *probe)
{
    if(probe){
        updProbeSet().adoptAndAppend(probe);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*probe);
    }
}

// Remove a probe from the Model. Probe will be deleted since model owns it
void Model::removeProbe(OpenSim::Probe *aProbe)
{
    clearConnections();
    updProbeSet().remove(aProbe);
    finalizeFromProperties();
}

// Add a contact geometry to the Model
void Model::addContactGeometry(OpenSim::ContactGeometry *contactGeometry)
{
    if (contactGeometry) {
        updContactGeometrySet().adoptAndAppend(contactGeometry);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*contactGeometry);
    }
}

// Add a controller to the Model
void Model::addController(Controller *controller)
{
    if (controller) {
        updControllerSet().adoptAndAppend(controller);
        finalizeFromProperties();
        prependComponentPathToConnecteePath(*controller);
    }
}
//_____________________________________________________________________________
/* Perform some setup functions that happen after the
 * object has been deserialized. This method is
 * not yet designed to be called after a model has been
 * copied.
 */
void Model::setup()
{
    // finalize the model and its subcomponents from its properties
    // automatically marks properties that are Components as subcomponents
    finalizeFromProperties();
    //now connect the Model and all its subcomponents all up
    finalizeConnections();
}

//_____________________________________________________________________________
/* Perform some clean up functions that are normally done from the destructor
 * however this gives the GUI a way to proactively do the cleaning without
 * waiting for garbage collection to do the actual cleanup.
 */
void Model::cleanup()
{
    clearConnections();
    upd_ForceSet().setSize(0);
}

void Model::setDefaultProperties()
{
    // Initialize the length and force units from the strings specified in the model file.
    // If they were not specified, use meters and Newtons.
    _lengthUnits = Units(get_length_units());
    _forceUnits = Units(get_force_units());
}

void Model::extendInitStateFromProperties(SimTK::State& state) const
{
    Super::extendInitStateFromProperties(state);
    // Allocate the size and default values for controls
    // Actuators will have a const view into the cache
    SimTK::Measure_<SimTK::Vector>::Result controlsCache =
            SimTK::Measure_<SimTK::Vector>::Result::getAs(
                    updSystem().updDefaultSubsystem().getMeasure(
                            _modelControlsIndex));
    controlsCache.updValue(state).resize(_defaultControls.size());
    controlsCache.updValue(state) = _defaultControls;
}

void Model::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);
}

void Model::generateDecorations
       (bool                                        fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
{
    if (!hints.isVisualizationEnabled()) return;
    ComponentList<const Component> allComps = getComponentList();
    ComponentList<Component>::const_iterator iter = allComps.begin();
    while (iter != allComps.end()){
        //std::string cn = iter->getConcreteClassName();
        //std::cout << cn << ":" << iter->getName() << std::endl;
        iter->generateDecorations(fixed, hints, state, appendToThis);
        iter++;
    }
}

void Model::equilibrateMuscles(SimTK::State& state) {
    getMultibodySystem().realize(state, SimTK::Stage::Velocity);

    bool failed = false;
    string errorMsg = "";

    auto muscles = getComponentList<Muscle>();

    for (auto& muscle : muscles) {
        if (muscle.appliesForce(state)){
            try{
                muscle.computeEquilibrium(state);
            }
            catch (const std::exception& e) {
                if(!failed){ // haven't failed to equilibrate other muscles yet
                    errorMsg = e.what();
                    failed = true;
                }
                // just because one muscle failed to equilibrate doesn't mean
                // it isn't still useful to have remaining muscles equilibrate
                // in an analysis, for example, we might not be reporting about
                // all muscles, so continue with the rest.
                continue;
            }
        }
    }

    if(failed) // Notify the caller of the failure to equilibrate
        throw Exception("Model::equilibrateMuscles() "+errorMsg, __FILE__, __LINE__);
}

//=============================================================================
// GRAVITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the gravity vector in the global frame.
 *
 * @return the XYZ gravity vector in the global frame is returned here.
 */
SimTK::Vec3 Model::getGravity() const
{
    if(_gravityForce)
        return _gravityForce->getDefaultGravityVector();

    return get_gravity();
}
//_____________________________________________________________________________
/**
 * Set the gravity vector in the global frame.
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
bool Model::setGravity(const SimTK::Vec3& aGrav)
{
    upd_gravity() = aGrav;

    if(_gravityForce)
        _gravityForce->setDefaultGravityVector(aGrav);

    return true;
}


//=============================================================================
// NUMBERS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of markers in the model.
 *
 * @return Number of markers.
 */
int Model::getNumMarkers() const
{
    return get_MarkerSet().getSize();
}
/**
 * Get the number of ContactGeometry objects in the model.
 *
 * @return Number of ContactGeometries.
 */
int Model::getNumContactGeometries() const
{
    return get_ContactGeometrySet().getSize();
}

/**
 * Get the number of Muscle state variables in the model.
 *
 * @return Number of MuscleStates.
 */

int Model::getNumMuscleStates() const {

    int n = 0;
    for(int i=0;i<get_ForceSet().getSize();i++){
        Muscle *mus = dynamic_cast<Muscle*>( &get_ForceSet().get(i) );
        if(mus!=NULL) {
            n += mus->getNumStateVariables();
        }
    }
    return(n);
}

/**
 * Get the number of Probe state variables in the model.
 *
 * @return Number of MuscleStates.
 */

int Model::getNumProbeStates() const {

    int n = 0;
    for(int i=0;i<get_ProbeSet().getSize();i++){
        Probe *p = dynamic_cast<Probe*>( &get_ProbeSet().get(i) );
        if(p!=NULL) {
            n += p->getNumInternalMeasureStates();
        }
    }
    return(n);
}

//_____________________________________________________________________________
/**
 * Get the total number of bodies in the model.
 *
 * @return Number of bodies.
 */
int Model::getNumBodies() const
{
    return  getBodySet().getSize();
}

//_____________________________________________________________________________
/**
 * Get the total number of joints in the model.
 *
 * @return Number of joints.
 */
int Model::getNumJoints() const
{
    return  getJointSet().getSize();
}
//_____________________________________________________________________________
/**
 * Get the total number of coordinates in the model.
 *
 * @return Number of coordinates.
 */
int Model::getNumCoordinates() const
{
    return _coordinateSet.getSize();
}

/**
 * Get the total number of coordinates = number of speeds in the model.
 *
 * @return Number of coordinates.
 */
int Model::getNumSpeeds() const
{
    return _coordinateSet.getSize();
}

/**
* Get the total number of constraints in the model.
* @return Number of constraints.
*/
int Model::getNumConstraints() const
{
    return get_ConstraintSet().getSize();
}


/**
 * Get the total number of probes in the model.
 *
 * @return Number of probes.
 */
int Model::getNumProbes() const
{
    return get_ProbeSet().getSize();
}

//_____________________________________________________________________________
/**
 * Get the subset of Forces in the model which are actuators
 *
 * @return The set of Actuators
 */
const Set<Actuator>& Model::getActuators() const
{
    return get_ForceSet().getActuators();
}
Set<Actuator>& Model::updActuators()
{
    return upd_ForceSet().updActuators();
}

//_____________________________________________________________________________
/**
 * Get the subset of Forces in the model which are muscles
 *
 * @return The set of Muscles
 */
const Set<Muscle>& Model::getMuscles() const
{
    return get_ForceSet().getMuscles();
}
Set<Muscle>& Model::updMuscles()
{
    return upd_ForceSet().updMuscles();
}

//_____________________________________________________________________________
/**
 * Get the number of analyses in the model.
 *
 * @return The number of analyses
 */
int Model::getNumAnalyses() const
{
    return _analysisSet.getSize();
}

//_____________________________________________________________________________

//=============================================================================
// TIME NORMALIZATION CONSTANT
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// STATES
//=============================================================================
//_____________________________________________________________________________





//_____________________________________________________________________________
/**
 * Add an analysis to the model.
 *
 * @param aAnalysis pointer to the analysis to add
 */
void Model::addAnalysis(Analysis *aAnalysis)
{
    if (aAnalysis )
    {
//      aAnalysis->setModel(this);
        _analysisSet.adoptAndAppend(aAnalysis);
    }
}
//_____________________________________________________________________________
/**
 * Remove an analysis from the model
 *
 * @param aAnalysis Pointer to the analysis to remove.
 * If deleteIt is true (default) the Analysis object itself is destroyed
 * else only removed from the list which is the desired behavior when the Analysis
 * is created from the GUI.
 */
void Model::removeAnalysis(Analysis *aAnalysis, bool deleteIt)
{
    // CHECK FOR NULL
    if(aAnalysis==NULL) {
        log_error("Model.removeAnalysis:  NULL analysis");
    }
    if (!deleteIt){
        bool saveStatus = _analysisSet.getMemoryOwner();
        _analysisSet.setMemoryOwner(false);
        _analysisSet.remove(aAnalysis);
        _analysisSet.setMemoryOwner(saveStatus);
    }
    else
        _analysisSet.remove(aAnalysis);
}

//_____________________________________________________________________________
/**
 * Remove a controller from the model
 *
 * @param aController Pointer to the controller to remove.
 */
void Model::removeController(Controller *aController)
{
    // CHECK FOR NULL
    if(aController==NULL) {
        log_error("Model.removeController:  NULL controller.");
    }

    upd_ControllerSet().remove(aController);
}



//==============================================================================
// OPERATIONS
//==============================================================================

//------------------------------------------------------------------------------
// SCALE
//------------------------------------------------------------------------------
bool Model::scale(SimTK::State& s, const ScaleSet& scaleSet,
                  bool preserveMassDist, double finalMass)
{
    // Save the model's current pose.
    SimTK::Vector savedConfiguration = s.getY();

    // Put the model in a default pose so that GeometryPath lengths can be
    // computed and stored. These lengths will be required for adjusting
    // properties after the rest of the model has been scaled.
    applyDefaultConfiguration(s);

    // Call preScale() on each ModelComponent owned by the model to store
    // GeometryPath lengths (and perform any other necessary computations).
    for (ModelComponent& comp : updComponentList<ModelComponent>())
        comp.preScale(s, scaleSet);

    // Call scale() on each ModelComponent owned by the model. Each
    // ModelComponent is responsible for scaling itself. All scaling operations
    // are performed here except scaling inertial properties of bodies, which is
    // done below.
    for (ModelComponent& comp : updComponentList<ModelComponent>())
        comp.scale(s, scaleSet);

    // Scale the inertial properties of bodies. If "preserve mass distribution"
    // is true, then the masses are not scaled (but inertias are still updated).
    for (Body& body : updComponentList<Body>())
        body.scaleInertialProperties(scaleSet, !preserveMassDist);

    // When bodies are scaled, the properties of the model are changed. The
    // general rule is that you MUST recreate and initialize the system when
    // properties of the model change. We must do that here or we will be
    // querying a stale system (e.g., wrong body properties!).
    s = initSystem();

    // Now that the masses of the individual bodies have been scaled (if
    // preserveMassDist == false), get the total mass and compare it to
    // finalMass in order to determine how much to scale the body masses again,
    // so that the total model mass comes out to finalMass.
    if (finalMass > 0.0)
    {
        const double mass = getTotalMass(s);
        if (mass > 0.0)
        {
            const double factor = finalMass / mass;
            for (Body& body : updComponentList<Body>())
                body.scaleMass(factor);

            // Recreate the system and update the state after updating masses.
            s = initSystem();

            // Ensure the final model mass is correct.
            const double newMass = getTotalMass(s);
            const double normDiffMass = abs(finalMass - newMass) / finalMass;
            if (normDiffMass > SimTK::SignificantReal) {
                throw Exception("Model::scale() scaled model mass does not match specified subject mass.");
            }
        }
    }

    // Call postScale() on all ModelComponents owned by the model so that
    // components like muscles, ligaments, and path springs can update their
    // properties based on their new path length.
    for (ModelComponent& comp : updComponentList<ModelComponent>())
        comp.postScale(s, scaleSet);

    // Changed the model after scaling path actuators. Have to recreate system!
    s = initSystem();

    // Put the model back in its original pose.
    s.updY() = savedConfiguration;
    getMultibodySystem().realize( s, SimTK::Stage::Velocity );

    return true;
}


//=============================================================================
// PRINT
//=============================================================================
void Model::printBasicInfo(std::ostream& aOStream) const
{
    OPENSIM_THROW_IF_FRMOBJ(!isObjectUpToDateWithProperties(), Exception,
        "Model::finalizeFromProperties() must be called first.");

    std::stringstream ss;
    ss
        << "\n               MODEL: " << getName()
        << "\n         coordinates: " << countNumComponents<Coordinate>()
        << "\n              forces: " << countNumComponents<Force>()
        << "\n           actuators: " << countNumComponents<Actuator>()
        << "\n             muscles: " << countNumComponents<Muscle>()
        << "\n            analyses: " << getNumAnalyses()
        << "\n              probes: " << countNumComponents<Probe>()
        << "\n              bodies: " << countNumComponents<Body>()
        << "\n              joints: " << countNumComponents<Joint>()
        << "\n         constraints: " << countNumComponents<Constraint>()
        << "\n             markers: " << countNumComponents<Marker>()
        << "\n         controllers: " << countNumComponents<Controller>()
        << "\n  contact geometries: " << countNumComponents<ContactGeometry>()
        << "\nmisc modelcomponents: " << getMiscModelComponentSet().getSize();

    if (aOStream.rdbuf() == std::cout.rdbuf()) {
        log_cout(ss.str());
    } else {
        aOStream << ss.str() << std::endl;
    }
}

void Model::printDetailedInfo(const SimTK::State& s, std::ostream& aOStream) const
{
    std::stringstream ss;

    ss << "MODEL: " << getName() << std::endl;

    ss
        << "\nnumStates = "      << s.getNY()
        << "\nnumCoordinates = " << countNumComponents<Coordinate>()
        << "\nnumSpeeds = "      << countNumComponents<Coordinate>()
        << "\nnumActuators = "   << countNumComponents<Actuator>()
        << "\nnumBodies = "      << countNumComponents<Body>()
        << "\nnumConstraints = " << countNumComponents<Constraint>()
        << "\nnumProbes = "      << countNumComponents<Probe>()
        << std::endl;

    ss << "\nANALYSES (total: " << getNumAnalyses() << ")" << std::endl;
    for (int i = 0; i < _analysisSet.getSize(); ++i)
        ss << "analysis[" << i << "] = " << _analysisSet.get(i).getName()
                 << std::endl;

    ss << "\nBODIES (total: " << countNumComponents<Body>() << ")"
             << std::endl;
    unsigned bodyNum = 0u;
    for (const auto& body : getComponentList<Body>()) {
        const auto inertia = body.getInertia();
        ss << "body[" << bodyNum << "] = " << body.getName() << ". "
            << "mass: " << body.get_mass()
            << "\n              moments of inertia:  " << inertia.getMoments()
            << "\n              products of inertia: " << inertia.getProducts()
            << std::endl;
        ++bodyNum;
    }

    ss << "\nJOINTS (total: " << countNumComponents<Joint>() << ")"
             << std::endl;
    unsigned jointNum = 0u;
    for (const auto& joint : getComponentList<Joint>()) {
        ss << "joint[" << jointNum << "] = " << joint.getName() << "."
                 << " parent: " << joint.getParentFrame().getName()
                 << ", child: " << joint.getChildFrame().getName() << std::endl;
        ++jointNum;
    }

    ss << "\nACTUATORS (total: " << countNumComponents<Actuator>() << ")"
             << std::endl;
    unsigned actuatorNum = 0u;
    for (const auto& actuator : getComponentList<Actuator>()) {
        ss << "actuator[" << actuatorNum << "] = " << actuator.getName()
                 << std::endl;
        ++actuatorNum;
    }

    /*
    int n;
    ss<<"MODEL: "<<getName()<<std::endl;

    n = getNumBodies();
    ss<<"\nBODIES ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) ss<<"body["<<i<<"] = "<<getFrameName(i)<<std::endl;

    n = getNQ();
    ss<<"\nGENERALIZED COORDINATES ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) ss<<"q["<<i<<"] = "<<getCoordinateName(i)<<std::endl;

    n = getNU();
    ss<<"\nGENERALIZED SPEEDS ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) ss<<"u["<<i<<"] = "<<getSpeedName(i)<<std::endl;

    n = getNA();
    ss<<"\nACTUATORS ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) ss<<"actuator["<<i<<"] = "<<getActuatorName(i)<<std::endl;

    n = getNP();
    ss<<"\nCONTACTS ("<<n<<")" << std::endl;
    */

    Array<string> stateNames = getStateVariableNames();
    ss << "\nSTATES (total: " << stateNames.getSize() << ")" << std::endl;
    for (int i = 0; i < stateNames.getSize(); ++i)
        ss << "y[" << i << "] = " << stateNames[i] << std::endl;

    if (aOStream.rdbuf() == std::cout.rdbuf()) {
        log_cout(ss.str());
    } else {
        aOStream << ss.str() << std::endl;
    }
}

//--------------------------------------------------------------------------
// CONFIGURATION
//--------------------------------------------------------------------------

//_____________________________________________________________________________
/**
 * Apply the default configuration to the model.  This means setting the
 * generalized coordinates and speeds to their default values.
 */
void Model::applyDefaultConfiguration(SimTK::State& s)
{
    int i;

    // Coordinates
    int ncoords = getCoordinateSet().getSize();

    for(i=0; i<ncoords; i++) {
        Coordinate& coord = getCoordinateSet().get(i);
        coord.setValue(s, coord.getDefaultValue(), false);
        coord.setSpeedValue(s, coord.getDefaultSpeedValue());
    }

    // Satisfy the constraints.
    assemble(s);
}

//_____________________________________________________________________________
/**
 * createAssemblySolver anew, old solver is deleted first. This should be invoked whenever changes in locks and/or constraint status
 * that has the potential to affect model assembly.
 */
void Model::createAssemblySolver(const SimTK::State& s)
{
    // Allocate on stack and pass to AssemblySolver to make working copy.
    SimTK::Array_<CoordinateReference> coordsToTrack;

    for(int i=0; i<getNumCoordinates(); ++i){
        // Iff a coordinate is dependent on other coordinates for its value,
        // do not give it a reference/goal.
        if(!_coordinateSet[i].isDependent(s)){
            Constant reference(_coordinateSet[i].getValue(s));
            CoordinateReference coordRef(_coordinateSet[i].getName(), reference);
            coordsToTrack.push_back(coordRef);
        }
    }

    // Use the assembler to generate the initial pose from Coordinate defaults
    // that also satisfies the constraints. AssemblySolver makes copy of
    // coordsToTrack
    _assemblySolver.reset(new AssemblySolver(*this, coordsToTrack));
    _assemblySolver->setConstraintWeight(SimTK::Infinity);
    _assemblySolver->setAccuracy(get_assembly_accuracy());
}

void Model::updateAssemblyConditions(SimTK::State& s)
{
    createAssemblySolver(s);
}
//--------------------------------------------------------------------------
// MARKERS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Write an XML file of all the markers in the model.
 *
 * @param aFileName the name of the file to create
 */
void Model::writeMarkerFile(const string& aFileName)
{
    upd_MarkerSet().print(aFileName);
}


//_____________________________________________________________________________
void Model::updateMarkerSet(MarkerSet& newMarkerSet)
{
    for (int i = 0; i < newMarkerSet.getSize(); i++) {
        Marker& updatingMarker = newMarkerSet.get(i);

        /* If there is already a marker in the model with that name,
         * replace it with the updating marker.
         */
        if (updMarkerSet().contains(updatingMarker.getName())) {
            Marker& modelMarker = updMarkerSet().get(updatingMarker.getName());
            // Delete the marker from the model and add the updating one
            upd_MarkerSet().remove(&modelMarker);
        }
        // append the marker to the model's Set
        addMarker(updatingMarker.clone());
    }

    log_info("Updated markers in model {}", getName());
}

//_____________________________________________________________________________
/**
 * Remove all markers from the model that are not in the passed-in list.
 *
 * @param aMarkerNames array of marker names not to be deleted
 * @return Number of markers deleted
 *
 * @Todo_AYMAN make sure visuals adjust as well
 */
int Model::deleteUnusedMarkers(const OpenSim::Array<string>& aMarkerNames)
{
    int i, numDeleted = 0;

    for (i = 0; i < get_MarkerSet().getSize(); )
    {
        int index = aMarkerNames.findIndex(get_MarkerSet().get(i).getName());
        if (index < 0)
        {
            // Delete the marker, but don't increment i or else you'll
            // skip over the marker right after the deleted one.
            upd_MarkerSet().remove(i);
            numDeleted++;
        }
        else
        {
            i++;
        }
    }

    log_info("Deleted {} unused markers from model {}.", numDeleted, getName());

    return numDeleted;
}

/**
 ** Get a flat list of Joints contained in the model
 **
 **/
JointSet& Model::updJointSet()
{
    return upd_JointSet();
}

const JointSet& Model::getJointSet() const
{
    return get_JointSet();
}

const Ground& Model::getGround() const
{
    return get_ground();
}

Ground& Model::updGround()
{
    return upd_ground();
}

//--------------------------------------------------------------------------
// CONTROLS
//--------------------------------------------------------------------------
/** Get the number of controls for this the model.
 * Throws an exception if called before Model::initSystem()  */
int Model::getNumControls() const
{
    if(!_system){
        throw Exception("Model::getNumControls() requires an initialized Model./n"
            "Prior Model::initSystem() required.");
    }

    return _defaultControls.size();
}

/** Update the controls for this the model at a given state.
 * Throws an exception if called before Model::initSystem() */
SimTK::Vector& Model::updControls(const SimTK::State& s) const {
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::updControls() requires an initialized Model./n"
            "Prior call to Model::initSystem() is required.");
    }

    // direct the system shared cache
    SimTK::Measure_<SimTK::Vector>::Result controlsCache =
            SimTK::Measure_<SimTK::Vector>::Result::getAs(
                    _system->updDefaultSubsystem().getMeasure(
                            _modelControlsIndex));
    return controlsCache.updValue(s);
}

void Model::markControlsAsValid(const SimTK::State& s) const
{
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::markControlsAsValid() requires an initialized Model./n"
            "Prior call to Model::initSystem() is required.");
    }

    SimTK::Measure_<SimTK::Vector>::Result controlsCache =
            SimTK::Measure_<SimTK::Vector>::Result::getAs(
                    _system->updDefaultSubsystem().getMeasure(
                            _modelControlsIndex));
    controlsCache.markAsValid(s);
}

void Model::markControlsAsInvalid(const SimTK::State& s) const
{
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::markControlsAsInvalid() requires an initialized Model./n"
            "Prior call to Model::initSystem() is required.");
    }

    SimTK::Measure_<SimTK::Vector>::Result controlsCache =
            SimTK::Measure_<SimTK::Vector>::Result::getAs(
                    _system->updDefaultSubsystem().getMeasure(
                            _modelControlsIndex));
    controlsCache.markAsNotValid(s);
}

void Model::setControls(const SimTK::State& s, const SimTK::Vector& controls) const
{
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::setControls() requires an initialized Model./n"
            "Prior call to Model::initSystem() is required.");
    }

    // direct the system shared cache
    SimTK::Measure_<SimTK::Vector>::Result controlsCache =
            SimTK::Measure_<SimTK::Vector>::Result::getAs(
                    _system->updDefaultSubsystem().getMeasure(
                            _modelControlsIndex));
    controlsCache.setValue(s, controls);

    // Make sure to re-realize dynamics to make sure controls can affect forces
    // and not just derivatives
    if (s.getSystemStage() == SimTK::Stage::Dynamics)
        s.invalidateAllCacheAtOrAbove(SimTK::Stage::Dynamics);
}

/** Const access to controls does not invalidate dynamics */
const SimTK::Vector& Model::getControls(const SimTK::State& s) const {
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::getControls() requires an initialized Model./n"
            "Prior call to Model::initSystem() is required.");
    }

    // direct the system shared cache
    SimTK::Measure_<SimTK::Vector>::Result controlsCache =
            SimTK::Measure_<SimTK::Vector>::Result::getAs(
                    _system->updDefaultSubsystem().getMeasure(
                            _modelControlsIndex));

    if(!controlsCache.isValid(s)){
        // Always reset controls to their default values before computing controls
        // since default behavior is for controllers to "addInControls" so there should be valid
        // values to begin with.
        controlsCache.updValue(s) = _defaultControls;
        computeControls(s, controlsCache.updValue(s));
        controlsCache.markAsValid(s);
    }

    return controlsCache.getValue(s);
}

/** Compute the controls the model */
void Model::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
    if (!this->getAllControllersEnabled()) {
        return;
    }

    for (const Controller& controller : this->_enabledControllers) {
        controller.computeControls(s, controls);
    }
}


/** Get a flag indicating if the model needs controls to operate its actuators */
bool Model::isControlled() const
{
    bool isControlled = getActuators().getSize() > 0;

    return isControlled;
}

const ControllerSet& Model::getControllerSet() const{
    return(get_ControllerSet());
}
ControllerSet& Model::updControllerSet() {
    return(upd_ControllerSet());
}
void Model::storeControls( const SimTK::State& s, int step ) {
    upd_ControllerSet().storeControls(s, step);
    return;
}
void Model::printControlStorage(const string& fileName ) const {
    get_ControllerSet().printControlStorage(fileName);
}

TimeSeriesTable Model::getControlsTable() const {
    return get_ControllerSet().getControlTable();
}

bool Model::getAllControllersEnabled() const{
  return( _allControllersEnabled );
}
void Model::setAllControllersEnabled( bool enabled ) {
    _allControllersEnabled = enabled;
}

void Model::formStateStorage(const Storage& originalStorage,
                             Storage& statesStorage,
                             bool warnUnspecifiedStates) const
{
    Array<string> rStateNames = getStateVariableNames();
    int numStates = getNumStateVariables();
    // make sure same size, otherwise warn
    if (originalStorage.getSmallestNumberOfStates() != rStateNames.getSize() && warnUnspecifiedStates){
        log_warn("Number of columns does not match in formStateStorage. Found {}"
            " Expected  {}.",
            originalStorage.getSmallestNumberOfStates(), rStateNames.getSize());
    }

    OPENSIM_THROW_IF_FRMOBJ(originalStorage.isInDegrees(), Exception,
        "Input Storage must have values for Coordinates in meters or radians (not degrees).\n"
        "Please convert input Storage to meters and/or degrees first.");

    // when the state value is not found in the storage use its default value in the State
    SimTK::Vector defaultStateValues = getStateVariableValues(getWorkingState());

    // Create a list with entry for each desiredName telling which column in originalStorage has the data
    Array<int> mapColumns(-1, rStateNames.getSize());
    for(int i=0; i< rStateNames.getSize(); i++){
        // the index is -1 if not found, >=1 otherwise since time has index 0 by defn.
        int fix = originalStorage.getStateIndex(rStateNames[i]);
        mapColumns[i] = fix;
        if (fix==-1 && warnUnspecifiedStates){
            log_warn("Column {} not found by Model::formStateStorage(). "
                "Assuming its default value of {}",
                rStateNames[i], defaultStateValues[i]);
        }
    }
    // Now cycle through each state (row of Storage) and form the Model consistent
    // order for the state values
    double assignedValue = SimTK::NaN;
    for (int row =0; row< originalStorage.getSize(); row++){
        StateVector* originalVec = originalStorage.getStateVector(row);
        StateVector stateVec{originalVec->getTime()};
        stateVec.getData().setSize(numStates);
        for(int column=0; column< numStates; column++) {
            if (mapColumns[column] != -1)
                originalVec->getDataValue(mapColumns[column], assignedValue);
            else
                assignedValue = defaultStateValues[column];

            stateVec.setDataValue(column, assignedValue);
        }
        statesStorage.append(stateVec);
    }
    rStateNames.insert(0, "time");
    statesStorage.setColumnLabels(rStateNames);
}

/**
 * Model::formStateStorage is intended to take any storage and populate qStorage.
 * stateStorage is supposed to be a Storage with labels identical to those obtained by calling
 * Model::getStateNames(). Columns/entries found in the "originalStorage" are copied to the
 * output qStorage. Entries not found are populated with 0s.
 */
void Model::formQStorage(const Storage& originalStorage, Storage& qStorage) {

    int nq = getWorkingState().getNQ();
    Array<string> qNames;
    getCoordinateSet().getNames(qNames);


    int* mapColumns = new int[qNames.getSize()];
    for(int i=0; i< nq; i++){
        // the index is -1 if not found, >=1 otherwise since time has index 0 by defn.
        mapColumns[i] = originalStorage.getColumnLabels().findIndex(qNames[i]);
        if (mapColumns[i] == -1)
            log_warn("Column {} not found in formQStorage, assuming 0.",
                    qNames[i]);
    }


    // Now cycle through and shuffle each
    for (int row =0; row< originalStorage.getSize(); row++){
        StateVector* originalVec = originalStorage.getStateVector(row);
        StateVector* stateVec = new StateVector(originalVec->getTime());
        stateVec->getData().setSize(nq);  // default value 0f 0.
        for(int column=0; column< nq; column++){
            double valueInOriginalStorage=0.0;
            if (mapColumns[column]!=-1)
                originalVec->getDataValue(mapColumns[column]-1, valueInOriginalStorage);

            stateVec->setDataValue(column, valueInOriginalStorage);
        }
        qStorage.append(*stateVec);
    }
    qNames.insert(0, "time");

    qStorage.setColumnLabels(qNames);
    // Since we're copying data from a Storage file, keep the inDegrees flag consistent
    qStorage.setInDegrees(originalStorage.isInDegrees());
}
void Model::disownAllComponents()
{
    updMiscModelComponentSet().setMemoryOwner(false);
    updBodySet().setMemoryOwner(false);
    updJointSet().setMemoryOwner(false);
    updConstraintSet().setMemoryOwner(false);
    updForceSet().setMemoryOwner(false);
    updContactGeometrySet().setMemoryOwner(false);
    updControllerSet().setMemoryOwner(false);
    updAnalysisSet().setMemoryOwner(false);
    updMarkerSet().setMemoryOwner(false);
    updProbeSet().setMemoryOwner(false);
}

void Model::overrideAllActuators( SimTK::State& s, bool flag) {
     Set<Actuator>& as = updActuators();

     for(int i=0;i<as.getSize();i++) {
         ScalarActuator* act = dynamic_cast<ScalarActuator*>(&as[i]);
         act->overrideActuation(s, flag);
     }

}

const Object& Model::getObjectByTypeAndName(const std::string& typeString, const std::string& nameString) {
    if (typeString=="Body")
        return getBodySet().get(nameString);
    else if (typeString == "Joint")
        return getJointSet().get(nameString);
    else if (typeString=="Force")
        return getForceSet().get(nameString);
    else if (typeString=="Constraint")
        return getConstraintSet().get(nameString);
    else if (typeString=="Coordinate")
        return getCoordinateSet().get(nameString);
    else if (typeString=="Marker")
        return getMarkerSet().get(nameString);
    else if (typeString=="Controller")
        return getControllerSet().get(nameString);
    else if (typeString == "Probe")
        return getProbeSet().get(nameString);
    throw Exception("Model::getObjectByTypeAndName: no object of type " + typeString +
        " and name "+nameString+" was found in the model.");

}

//------------------------------------------------------------------------------
//          REALIZE THE SYSTEM TO THE REQUIRED COMPUTATIONAL STAGE
//------------------------------------------------------------------------------
void Model::realizeTime(const SimTK::State& state) const {
    getSystem().realize(state, SimTK::Stage::Time);
}

void Model::realizePosition(const SimTK::State& state) const {
    getSystem().realize(state, SimTK::Stage::Position);
}

void Model::realizeVelocity(const SimTK::State& state) const {
    getSystem().realize(state, SimTK::Stage::Velocity);
}

void Model::realizeDynamics(const SimTK::State& state) const {
    getSystem().realize(state, SimTK::Stage::Dynamics);
}

void Model::realizeAcceleration(const SimTK::State& state) const {
    getSystem().realize(state, SimTK::Stage::Acceleration);
}

void Model::realizeReport(const SimTK::State& state) const {
    getSystem().realize(state, SimTK::Stage::Report);
}

//------------------------------------------------------------------------------
// Subsystem computations
//------------------------------------------------------------------------------
void Model::computeStateVariableDerivatives(const SimTK::State &s) const
{
    realizeAcceleration(s);
}

double Model::getTotalMass(const SimTK::State &s) const
{
    return getMatterSubsystem().calcSystemMass(s);
}

SimTK::Inertia Model::getInertiaAboutMassCenter(const SimTK::State &s) const
{
    SimTK::Inertia inertia =
            getMatterSubsystem().calcSystemCentralInertiaInGround(s);

    return inertia;
}

SimTK::Vec3 Model::calcMassCenterPosition(const SimTK::State &s) const
{
    getMultibodySystem().realize(s, SimTK::Stage::Position);
    return getMatterSubsystem().calcSystemMassCenterLocationInGround(s);
}

SimTK::Vec3 Model::calcMassCenterVelocity(const SimTK::State &s) const
{
    getMultibodySystem().realize(s, SimTK::Stage::Velocity);
    return getMatterSubsystem().calcSystemMassCenterVelocityInGround(s);
}

SimTK::Vec3 Model::calcMassCenterAcceleration(const SimTK::State &s) const
{
    getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    return getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s);
}

SimTK::SpatialVec Model::calcMomentum(const SimTK::State &s) const
{
    getMultibodySystem().realize(s, SimTK::Stage::Velocity);
    return getMatterSubsystem().calcSystemCentralMomentum(s);
}

SimTK::Vec3 Model::calcAngularMomentum(const SimTK::State& s) const {
    return calcMomentum(s).get(0);
}

SimTK::Vec3 Model::calcLinearMomentum(const SimTK::State& s) const {
    return calcMomentum(s).get(1);
}
