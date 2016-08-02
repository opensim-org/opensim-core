/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Model.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlLinearNode.h>
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/AssemblySolver.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/CoordinateReference.h>

#include "SimTKcommon/internal/SystemGuts.h"

#include "Model.h"

#include "Muscle.h"
#include "Ligament.h"
#include "CoordinateSet.h"
#include "FrameSet.h"
#include "BodySet.h"
#include "AnalysisSet.h"
#include "ForceSet.h"
#include "ControllerSet.h"
#include "Analysis.h"
#include "ForceAdapter.h"
#include "Actuator.h"
#include "MarkerSet.h"
#include "ContactGeometrySet.h"
#include "ProbeSet.h"
#include "ComponentSet.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
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
/**
 * Constructor from an XML file
 */
Model::Model(const string &aFileName, const bool finalize) :
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

    if (finalize) {
        finalizeFromProperties();
    }

    _fileName = aFileName;
    cout << "Loaded model " << getName() << " from file " << getInputFileName() << endl;
}

//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
/*virtual*/
void Model::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if (versionNumber < XMLDocument::getLatestVersion()){
        cout << "Updating Model file from " << versionNumber << " to latest format..." << endl;
        // Version has to be 1.6 or later, otherwise assert
        if (versionNumber == 10600){
            // Get node for DynamicsEngine
            SimTK::Xml::element_iterator engIter = aNode.element_begin("DynamicsEngine");
            //Get node for SimbodyEngine
            if (engIter != aNode.element_end()){
                SimTK::Xml::element_iterator simbodyEngIter = engIter->element_begin("SimbodyEngine");
                // Move all Children of simbodyEngineNode to be children of _node
                // we'll keep inserting before enginesNode then remove it;
                SimTK::Array_<SimTK::Xml::Element> elts = simbodyEngIter->getAllElements();
                while (elts.size() != 0){
                    // get first child and move it to Model
                    aNode.insertNodeAfter(aNode.element_end(), simbodyEngIter->removeNode(simbodyEngIter->element_begin()));
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
            SimTK::Xml::element_iterator  objects_node = bodySetNode->element_begin("objects");
            SimTK::Xml::element_iterator bodyIter = objects_node->element_begin("Body");
            for (; bodyIter != objects_node->element_end(); ++bodyIter) {
                std::string body_name = bodyIter->getOptionalAttributeValue("name");
                if (body_name == "ground") {
                    SimTK::Xml::Element newGroundElement("Ground");
                    newGroundElement.setAttributeValue("name", "ground");

                    SimTK::Xml::element_iterator geometryIter = bodyIter->element_begin("geometry");
                    if (geometryIter != bodyIter->element_end()) {
                        SimTK::Xml::Element cloneOfGeomety = geometryIter->clone();
                        newGroundElement.insertNodeAfter(newGroundElement.node_end(), cloneOfGeomety);
                    }

                    SimTK::Xml::element_iterator visObjIter = bodyIter->element_begin("VisibleObject");
                    SimTK::Xml::Element cloneOfVisObj = visObjIter->clone();
                    newGroundElement.insertNodeAfter(newGroundElement.node_end(), cloneOfVisObj);

                    SimTK::Xml::element_iterator wrapSetIter = bodyIter->element_begin("WrapObjectSet");
                    SimTK::Xml::Element cloneOfWrapSet = wrapSetIter->clone();
                    newGroundElement.insertNodeAfter(newGroundElement.node_end(), cloneOfWrapSet);

                    String test;
                    newGroundElement.writeToString(test);

                    objects_node->eraseNode(bodyIter);

                    aNode.insertNodeBefore(bodySetNode, newGroundElement);
                    break;
                }
            }
            bodyIter = objects_node->element_begin("Body");
            for (; bodyIter != objects_node->element_end(); ++bodyIter) {
                std::string body_name = bodyIter->getOptionalAttributeValue("name");
                SimTK::Xml::element_iterator  joint_node = bodyIter->element_begin("Joint");
                if (joint_node->element_begin() != joint_node->element_end()){
                    SimTK::Xml::Element detach_joint_node = joint_node->clone();
                    SimTK::Xml::element_iterator concreteJointNode = detach_joint_node.element_begin();
                    detach_joint_node.removeNode(concreteJointNode);
                    SimTK::Xml::element_iterator parentBodyElement = concreteJointNode->element_begin("parent_body");
                    SimTK::String parent_name = "ground";
                    parentBodyElement->getValueAs<SimTK::String>(parent_name);
                    //cout << "Processing Joint " << concreteJointNode->getElementTag() << "Parent body " << parent_name << std::endl;
                    XMLDocument::addConnector(*concreteJointNode, "Connector_PhysicalFrame_", "parent_frame", parent_name);
                    XMLDocument::addConnector(*concreteJointNode, "Connector_PhysicalFrame_", "child_frame", body_name);
                    concreteJointNode->eraseNode(parentBodyElement);
                    jointObjects.insertNodeAfter(jointObjects.node_end(), *concreteJointNode);
                    detach_joint_node.clearOrphan();
                }
                bodyIter->eraseNode(joint_node);
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
    constructProperty_BodySet(bodies);

    FrameSet frames;
    constructProperty_FrameSet(frames);

    JointSet joints;
    constructProperty_JointSet(joints);

    ControllerSet controllerSet;
    constructProperty_ControllerSet(controllerSet);

    ConstraintSet constraintSet;
    constructProperty_ConstraintSet(constraintSet);

    ForceSet forceSet;
    constructProperty_ForceSet(forceSet);

    MarkerSet markerSet;
    constructProperty_MarkerSet(markerSet);

    ContactGeometrySet contactGeometrySet;
    constructProperty_ContactGeometrySet(contactGeometrySet);

    ComponentSet componentSet;
    constructProperty_ComponentSet(componentSet);

    ProbeSet probeSet;
    constructProperty_ProbeSet(probeSet);

    ModelVisualPreferences md;
    constructProperty_ModelVisualPreferences(md);
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
    if (!_system) 
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
    getMultibodySystem().realize(_workingState, Stage::Instance);

    // Realize the initial configuration in preparation for assembly. This
    // initial configuration does not necessarily satisfy constraints.
    getMultibodySystem().realize(_workingState, Stage::Position);

    // Reset (initialize) all underlying Probe SimTK::Measures
    for (int i=0; i<getProbeSet().getSize(); ++i)
        getProbeSet().get(i).reset(_workingState);

    // Reset the controller's storage
    upd_ControllerSet().constructStorage();
    
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
        getMultibodySystem().realize(s, Stage::Position);

        // if a coordinate is locked or prescribed, then project will suffice
        if(constrained){
            // correct position constraint violations due to prescribed motion
            getMultibodySystem().projectQ(s, 1e-10);

            // Have a new working configuration so should realize to velocity
            getMultibodySystem().realize(s, Stage::Velocity);
            // correct velocity constraint violations due to prescribed motion
            getMultibodySystem().projectU(s, 1e-10);
        }
        return;
    }

    if (!_assemblySolver){
        createAssemblySolver(s);
    }
    const Array_<CoordinateReference>& coordRefs = _assemblySolver->getCoordinateReferences();

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
            cout << "Model unable to assemble: " << ex.what() << endl;
            cout << "Model relaxing constraints and trying again." << endl;

            try{
                // Try to satisfy with constraints as errors weighted heavily.
                _assemblySolver->setConstraintWeight(20.0);
                _assemblySolver->assemble(s);
            }
            catch (const std::exception& ex){
                cout << "Model unable to assemble with relaxed constraints: " << ex.what() << endl;
            }
        }
    }

    // Have a new working configuration so should realize to velocity
    getMultibodySystem().realize(s, Stage::Velocity);

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
    // create system
    _system.reset(new SimTK::MultibodySystem);
    _matter.reset(new SimTK::SimbodyMatterSubsystem(*_system));
    _forceSubsystem.reset(new SimTK::GeneralForceSubsystem(*_system));
    _contactSubsystem.reset(new SimTK::GeneralContactSubsystem(*_system));

    // create gravity force, a direction is needed even if magnitude=0 for
    // PotentialEnergy purposes.
    double magnitude = get_gravity().norm();
    SimTK::UnitVec3 direction = magnitude==0 ? SimTK::UnitVec3(0,-1,0) : SimTK::UnitVec3(get_gravity()/magnitude);
    _gravityForce.reset(new SimTK::Force::Gravity(*_forceSubsystem, *_matter,
                direction, magnitude));

    addToSystem(*_system);
}


void Model::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    if(getForceSet().getSize()>0)
    {
        ForceSet &fs = updForceSet();
        // Update internal subsets of the ForceSet
        fs.updActuators();
        fs.updMuscles();
    }

    if (getValidationLog().size() > 0) {
        cout << "The following Errors/Warnings were encountered interpreting properties of the model. " <<
            getValidationLog() << endl;
    }

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
    _multibodyTree.addBody(ground.getName(), 0, false, &ground);

    auto bodies = getComponentList<Body>();
    for (auto& body : bodies) {
        _multibodyTree.addBody( body.getName(), body.getMass(), false,
                                const_cast<Body*>(&body) );
    }

    // Complete multibody tree description by indicating how "bodies" are
    // connected by joints.
    auto joints = getComponentList<Joint>();
    for (auto& joint : joints) {
        std::string name = joint.getName();
        IO::TrimWhitespace(name);

        if (name.empty()) {
            name = joint.getParentFrame().getName() + "_to_" +
                    joint.getChildFrame().getName();
        }

        // Currently we need to take a first pass at connecting the joints
        // in order to ask the joint for the frames that they attach to and
        // to determine their underlying base (physical) frames.
        auto& mutableJoint = const_cast<Joint&>(joint);
        mutableJoint.connect(*this);
        // hack to make sure underlying Frame is also connected so it can 
        // traverse to the base frame and get its name. This allows the
        // (offset) frames to satisfy the connectors of Joint to be added
        // to a Body, for example, and not just joint itself.
        // TODO: try to create the multibody tree later when components
        // can already be expected to be connected then traverse those
        // relationships to create the multibody tree. -aseth
        const_cast<PhysicalFrame&>(joint.getParentFrame()).connect(*this);
        const_cast<PhysicalFrame&>(joint.getChildFrame()).connect(*this);

        // Use joints to define the underlying multibody tree
        _multibodyTree.addJoint(name,
            joint.getConcreteClassName(),
            // Multibody tree builder only cares about bodies not intermediate
            // frames that joints actually connect to.
            joint.getParentFrame().findBaseFrame().getName(),
            joint.getChildFrame().findBaseFrame().getName(),
            false,
            &mutableJoint);
    }
}

void Model::extendConnectToModel(Model &model)
{
    Super::extendConnectToModel(model);

    if (&model != this){
        cout << "Model::" << getName() <<
            " is being connected to model " <<
            model.getName() << "." << endl;
    }

    createMultibodyTree();

    // Model is connected so build the Multibody tree to represent it
    _multibodyTree.generateGraph();
    //_multibodyTree.dumpGraph(cout);
    //cout << endl;

    JointSet& joints = upd_JointSet();
    BodySet& bodies = upd_BodySet();
    int nb = bodies.getSize();

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
        const MultibodyGraphMaker::Mobilizer& mob 
            = _multibodyTree.getMobilizer(m);

        if (mob.isSlaveMobilizer()){
            // add the slave body and joint
            Body* outbMaster = static_cast<Body*>(mob.getOutboardMasterBodyRef());
            //Body* inb = static_cast<Body*>(mob.getInboardBodyRef());
            Joint* useJoint = static_cast<Joint*>(mob.getJointRef());
            Body* outb = static_cast<Body*>(mob.getOutboardBodyRef());

            if (!outb) {
                outb = outbMaster->addSlave();
                useJoint->setSlaveBodyForChild(*outb);
                SimTK::Transform o(SimTK::Vec3(0));
                //Now add the constraints that weld the slave to the master at the 
                // body origin
                std::string pathName = outb->getFullPathName();
                WeldConstraint* weld = new WeldConstraint(outb->getName()+"_weld",
                                                          *outbMaster, o, *outb, o);

                // include within adopted list of owned components
                adoptSubcomponent(weld);
            }
        }

        if (mob.isAddedBaseMobilizer()){
            // create and add the base joint to enable these dofs
            Body* child = static_cast<Body*>(mob.getOutboardBodyRef());
            cout << "Body '" << child->getName() << "' not connected by a Joint.\n"
                << "A FreeJoint will be added to connect it to ground." << endl;
            Body* ground = static_cast<Body*>(mob.getInboardBodyRef());

            // Verify that this is an orphan and it was assigned to ground
            assert(*ground == getGround());

            std::string jname = "free_" + child->getName();
            SimTK::Vec3 zeroVec(0.0);
            Joint* free = new FreeJoint(jname, *ground, *child);
            free->upd_reverse() = mob.isReversedFromJoint();
            addJoint(free);
        }
        else{
            Component* compToMoveOut = _propertySubcomponents.at(m+nb).get();
            // reorder the joint components in the order of the multibody tree
            Joint* jointToSwap = static_cast<Joint*>(mob.getJointRef());
            auto it = std::find(_propertySubcomponents.begin(),
                                _propertySubcomponents.end(), 
                                SimTK::ReferencePtr<Component>(jointToSwap));
            if (it != _propertySubcomponents.end()){
                // Only if the joint is not in the correct sequence the swap
                if (compToMoveOut != jointToSwap){
                    _propertySubcomponents[m + nb].reset(jointToSwap);
                    it->reset(compToMoveOut);
                }
            }
            int jx = joints.getIndex(jointToSwap, m);
            //if in the set but not already in the right order
            if ((jx >= 0) && (jx != m)) {
                // perform a move to put the joint in tree order
                // this is necessary ONLY because some tools assume that the
                // order or joints and specifically coordinates is the
                // order of the mobility (generalized) forces.
                // IDTool, StaticOptimization and RRA for example will fail.
                // TODO: when the tools are fixed/removed remove this as well.
                jointToSwap = &joints.get(jx);
                joints.set(jx, &joints.get(m));
                joints.set(m, jointToSwap);
            }
            // Update the directionality of the joint according to tree's
            // preferential direction
            static_cast<Joint*>(mob.getJointRef())->upd_reverse() =
                mob.isReversedFromJoint();
        }
    }
    joints.setMemoryOwner(isMemoryOwner);

    // Add the loop joints if any.
    for (int lcx = 0; lcx < _multibodyTree.getNumLoopConstraints(); ++lcx) {
        const MultibodyGraphMaker::LoopConstraint& loop =
            _multibodyTree.getLoopConstraint(lcx);

        Joint& joint = *(Joint*)loop.getJointRef();
        Body&  parent = *(Body*)loop.getParentBodyRef();
        Body&  child = *(Body*)loop.getChildBodyRef();

        if (joint.getConcreteClassName() == "WeldJoint") {
            WeldConstraint* weld = new WeldConstraint( joint.getName()+"_Loop",
                parent, joint.getParentFrame().findTransformInBaseFrame(),
                child, joint.getChildFrame().findTransformInBaseFrame());
            adoptSubcomponent(weld);
        }
        else if (joint.getConcreteClassName() == "BallJoint") {
            PointConstraint* point = new PointConstraint(
                parent, joint.getParentFrame().findTransformInBaseFrame().p(),
                child, joint.getChildFrame().findTransformInBaseFrame().p());
            point->setName(joint.getName() + "_Loop");
            adoptSubcomponent(point);
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

    // Reorder coordinates in order of the underlying mobilities
    updCoordinateSet().populate(*this);
    updForceSet().setupGroups();
    updControllerSet().setActuators(updActuators());

    // TODO: Get rid of the SimbodyEngine
    updSimbodyEngine().connectSimbodyEngineToModel(*this);
}


// ModelComponent interface enables this model to be treated as a subcomponent of another model by 
// creating components in its system.
void Model::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    Model *mutableThis = const_cast<Model *>(this);

    // Ensure Ground is added before all other Components
    getGround().addToSystem(system);
    //Analyses are not Components so add them after legit 
    //Components have been wired-up correctly.
    mutableThis->updAnalysisSet().setModel(*mutableThis);

    // Reset the vector of all controls' defaults
    mutableThis->_defaultControls.resize(0);

    // Create the shared cache that will hold all model controls
    // This must be created before Actuator.extendAddToSystem() since Actuator will append 
    // its "slots" and retain its index by accessing this cached Vector
    // value depends on velocity and invalidates dynamics BUT should not trigger
    // re-computation of the controls which are necessary for dynamics
    Measure_<Vector>::Result modelControls(_system->updDefaultSubsystem(),
        Stage::Velocity, Stage::Acceleration);

    mutableThis->_modelControlsIndex = modelControls.getSubsystemMeasureIndex();
}

// TODO so that
/*
void Model::extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystemAfterSubcomponents(system);
    // TODO determine if we have any state variables with an implicit form.
    
    // TODO would rather do s.getNY() but can't do that till after
    // realizeTopology().
    // TODO what if Simbody has state variables that OpenSim doesn't know about?
    const int numStateVars = getNumStateVariables();
    Vector nans(numStateVars, SimTK::NaN);
    nans.lockShape(); // TODO where is the right place for this?
    
    // This Measure is basically a discrete state variable.
    // Acceleration cache is invalid if this is updated, since acceleration-
    // dependent quantities should be computed with uDotGuess.
    // Dynamics stage must also be invalidated, since residuals cache var
    // depends on Dynamics stage.
    // TODO what should default value be? what size should it have?
    Measure_<Vector>::Variable yDotGuess(_system->updDefaultSubsystem(),
        Stage::Dynamics, nans);
    _yDotGuessIndex = yDotGuess.getSubsystemMeasureIndex();
    
    // TODO how to set size of lambdaGuess? shouldn't be numStateVars.
    Measure_<Vector>::Variable lambdaGuess(_system->updDefaultSubsystem(),
        Stage::Dynamics, Vector(0));
    _lambdaGuessIndex = lambdaGuess.getSubsystemMeasureIndex();
    
    // Storing the residual.
    // We can only compute the residual once realized to Dynamics, since
    // we will need to apply forces. Residual does not depend on accelerations.
    // TODO this would change if realizeAcceleration() simply uses uDotGuess.
    // None of the acceleration-level calculations depend on the residual
    // (nothing should depend on the residual) so there is nothing to
    // invalidate when the residual is changed (invalidated = Infinity).
    // TODO do we depend on Acceleration stage, for constraint forces?
    Measure_<Vector>::Result implicitResidual(_system->updDefaultSubsystem(),
        Stage::Dynamics, Stage::Infinity);
    implicitResidual.setDefaultValue(nans);
    _implicitResidualIndex = implicitResidual.getSubsystemMeasureIndex();
}
*/

/*
const SimTK::Vector& Model::getImplicitResiduals(const SimTK::State& state) const {
    OPENSIM_THROW_IF_FRMOBJ(!_system || !_implicitResidualIndex.isValid(),
        Exception, "Prior call to Model::initSystem() is required.");
    // TODO must make sure we are realized to Dynamics.
    auto measure = Measure_<Vector>::Result::getAs(
            _system->getDefaultSubsystem().getMeasure(_implicitResidualIndex));
    
    // Multibody states.
    

    if (!measure.isValid(state)) {
        getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
       
        // TODO do all of this in extendComputeImplicitResiduals().
        
        // Multibody states.
        // =================
        if (state.getNQ() > 0) { // Are there multibody states?
            const auto& yDotGuess = getYDotGuess(state);
            const auto& lambdaGuess = getMultipliersGuess(state);
            SimTK::Vector& residuals = measure.updValue(state);
            
            // qdot - N u
            // ----------
            // Get a view into YDotGuess[0:nq].
            const auto& qDotGuess = yDotGuess(0, state.getNQ());
            VectorView qResiduals = residuals(0, state.getNQ());
            qResiduals = state.getQDot() - qDotGuess;
            // TODO do we want to use QDot here?? I think so, otherwise
            // we are recomputing something that should already be cached for us.
            // The alternatives:
            // TODO getMatterSubsystem().calcQDot(...)
            // TODO getMatterSubsystem().multiplybyN(state, state.getU(), qResiduals);
            
            // M u_dot - f
            // -----------
            // Get a view into YDotGuess[nq:nq+nu].
            const auto& uDotGuess = yDotGuess(state.getNQ(), state.getNU());
            InverseDynamicsSolver idSolver(*this);
            // TODO improve InverseDynamicsSolver to take a lambda.
            VectorView uResiduals = residuals(state.getNQ(), state.getNU());
            // TODO is there an unnecessary copy here, and is it expensive?
            uResiduals = idSolver.solve(state, uDotGuess, lambdaGuess);
        }
    
        // Auxiliary states.
        // =================
        // TODO should put this in a separate "realizeImplicitResidual"?
        // TODO perhaps this is not the best way to invoke the actual calculation
        // of residuals.
        for (const auto& comp : getComponentList()) {
            comp.computeImplicitResiduals(state);
        }
        
        measure.markAsValid(state); // TODO
    }

    return measure.getValue(state);
}
*/
// TODO based on conversations with Sherm and Brad, we should not compute
// constraint errors here; users can compute them on their own if they'd like.
// But different methods may require different ways of solving the constraint
// errors, so we shouldn't try to do it here.

void Model::calcImplicitResiduals(const SimTK::State& state,
        const SimTK::Vector& yDotGuess, const SimTK::Vector& lambdaGuess,
        SimTK::Vector& residuals) const {
    
    // Check sizes of yDotGuess and lambdaGuess.
    OPENSIM_THROW_IF_FRMOBJ(   yDotGuess.size() != 0
                            && yDotGuess.size() != state.getNY(),
        Exception, "Length of yDotGuess argument was "
                   + std::to_string(yDotGuess.size())
                   + " but should have been either zero or the same as the"
                   + " number of mobilities nu="
                   + std::to_string(state.getNY()) + ".");
    OPENSIM_THROW_IF_FRMOBJ(   lambdaGuess.size() != 0
                            && lambdaGuess.size() != state.getNMultipliers(),
        Exception, "Length of lambdaGuess argument was "
                   + std::to_string(lambdaGuess.size())
                   + " but should have been either zero or the same as the"
                   + " number of acceleration-level constraints m="
                   + std::to_string(state.getNMultipliers()) + ".");
    
    
    residuals.resize(state.getNY());
    if (state.getNY() == 0) return;
    
    // Multibody states.
    // =================
    if (state.getNQ() > 0) {
    
        // N u - qdot
        // ----------
        Vector qDot; // TODO = state.getQDot() TODO profile?
        getMatterSubsystem().calcQDot(state, state.getU(), qDot);
        VectorView qResiduals = residuals(state.getQStart(), state.getNQ());
        if (yDotGuess.size() == 0) {
            qResiduals = qDot; // interpret qDotGuess as 0.
        } else {
            const VectorView& qDotGuess = yDotGuess(0, state.getNQ());
            qResiduals = qDot - qDotGuess;
        }
        // TODO does realizing to Velocity set QDot?
        // TODO do we want to use QDot here?? I think so, otherwise
        // we are recomputing something that should already be cached for us.
        // The alternatives:
        // TODO getMatterSubsystem().calcQDot(...)
        
        // M u_dot - f
        // -----------
        Vector uDotGuess;
        if (yDotGuess.size() > 0) {
            uDotGuess.viewAssign(yDotGuess(state.getUStart(), state.getNU()));
        }
        InverseDynamicsSolver idSolver(*this);
        VectorView uResiduals = residuals(state.getUStart(), state.getNU());
        // TODO is there an unnecessary copy here, and is it expensive?
        uResiduals = idSolver.solve(state, uDotGuess, lambdaGuess);
    }
    
    
    // Auxiliary states
    // ================
    if (state.getNZ() > 0) {
        // We can't expect the auxiliary states to handle empty guess vectors.
        // So if the guess vectors are empty, we turn them into
        // appropriately-sized vectors of 0.
        Vector yDotGuessSpace;
        Vector lambdaGuessSpace;
        const Vector* yDotGuessp;
        const Vector* lambdaGuessp;
        
        if (state.getNY() == 0 || yDotGuess.size() != 0) {
            yDotGuessp = &yDotGuess;
        } else {
            yDotGuessSpace = Vector(state.getNY(), 0.0);
            yDotGuessp = &yDotGuessSpace;
        }
        if (state.getNMultipliers() == 0 || lambdaGuess.size() != 0) {
            lambdaGuessp = &lambdaGuess;
        } else {
            lambdaGuessSpace = Vector(state.getNMultipliers(), 0.0);
            lambdaGuessp = &lambdaGuessSpace;
        }
        
        // Compute residuals for auxiliary state variables.
        for (const auto& comp : getComponentList()) {
            comp.calcImplicitResidualsInternal(state, yDotGuess, lambdaGuess,
                                               residuals);
        }
    }
}
    

//_____________________________________________________________________________
/**
 * Add any Component derived from ModelComponent to the Model.
 */
void Model::addModelComponent(ModelComponent* component)
{
    if(component){
        upd_ComponentSet().adoptAndAppend(component);
        finalizeFromProperties();
    }
}

//_____________________________________________________________________________
/*
 * Add a body to the Model.
 */
void Model::addBody(OpenSim::Body* body)
{
    if (body){
        updBodySet().adoptAndAppend(body);
        finalizeFromProperties();
    }
}

//_____________________________________________________________________________
/*
* Add a Frame to the Model.
*/
void Model::addFrame(OpenSim::Frame* frame)
{
    if (frame){
        updFrameSet().adoptAndAppend(frame);
        finalizeFromProperties();
    }
}
//_____________________________________________________________________________
/*
* Add a Marker to the Model.
*/
void Model::addMarker(OpenSim::Marker* marker)
{
    if (marker){
        updMarkerSet().adoptAndAppend(marker);
        finalizeFromProperties();
    }
}

//_____________________________________________________________________________
/*
* Add a joint to the Model.
*/
void Model::addJoint(Joint* joint)
{
    if (joint){
        updJointSet().adoptAndAppend(joint);
        finalizeFromProperties();
        updCoordinateSet().populate(*this);
    }
}

//_____________________________________________________________________________
/**
 * Add a constraint to the Model.
 */
void Model::addConstraint(OpenSim::Constraint *constraint)
{
    if(constraint){
        updConstraintSet().adoptAndAppend(constraint);
        finalizeFromProperties();
    }
}

//_____________________________________________________________________________
/**
 * Add a force to the Model.
 */
void Model::addForce(OpenSim::Force *force)
{
    if(force){
        updForceSet().adoptAndAppend(force);
        finalizeFromProperties();
    }
}

//_____________________________________________________________________________
/**
 * Add a probe to the Model.
 */
void Model::addProbe(OpenSim::Probe *probe)
{
    if(probe){
        updProbeSet().adoptAndAppend(probe);
        finalizeFromProperties();
    }
}

//_____________________________________________________________________________
/**
 * Remove a probe from the Model. Probe will be deleted as well since model owns it
 */
void Model::removeProbe(OpenSim::Probe *aProbe)
{
    disconnect();
    updProbeSet().remove(aProbe);
    finalizeFromProperties();
}

//_____________________________________________________________________________
/**
 * Add a contact geometry to the Model.
 */
void Model::addContactGeometry(OpenSim::ContactGeometry *contactGeometry)
{
    if (contactGeometry) {
        updContactGeometrySet().adoptAndAppend(contactGeometry);
        finalizeFromProperties();
    }
}

//_____________________________________________________________________________
/**
 * Add a controller to the Model.
 */
void Model::addController(Controller *aController)
{
    if (aController) {
        updControllerSet().adoptAndAppend(aController);
        finalizeFromProperties();
    }
}
//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
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
    connect(*this);
}

//_____________________________________________________________________________
/**
 * Perform some clean up functions that are normally done from the destructor
 * however this gives the GUI a way to proactively do the cleaning without waiting for garbage
 * collection to do the actual cleanup.
 */
void Model::cleanup()
{
    disconnect();
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
    Measure_<Vector>::Result controlsCache = Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem().getMeasure(_modelControlsIndex));
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
    ComponentList<const Component> allComps = getComponentList();
    ComponentList<Component>::const_iterator iter = allComps.begin();
    while (iter != allComps.end()){
        //std::string cn = iter->getConcreteClassName();
        //std::cout << cn << ":" << iter->getName() << std::endl;
        iter->generateDecorations(fixed, hints, state, appendToThis);
        iter++;
    }
}

void Model::equilibrateMuscles(SimTK::State& state)
{
    getMultibodySystem().realize(state, Stage::Velocity);

    bool failed = false;
    string errorMsg = "";

    auto muscles = getComponentList<Muscle>();

    for (auto& muscle : muscles) {
        if (!muscle.isDisabled(state)){
            try{
                muscle.equilibrate(state);
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
* Get the total number of frames in the model (not including Bodies).
*
* @return Number of frames.
*/
int Model::getNumFrames() const
{
    return  getFrameSet().getSize();
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
        cout << "Model.removeAnalysis:  ERROR- NULL analysis.\n" << endl;
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
        cout << "Model.removeController:  ERROR- NULL controller.\n" << endl;
    }

    upd_ControllerSet().remove(aController);
}



//==========================================================================
// OPERATIONS
//==========================================================================
//--------------------------------------------------------------------------
// SCALE
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Scale the model
 *
 * @param aScaleSet the set of XYZ scale factors for the bodies
 * @param aFinalMass the mass that the scaled model should have
 * @param aPreserveMassDist whether or not the masses of the
 *        individual bodies should be scaled with the body scale factors.
 * @return Whether or not scaling was successful.
 */
bool Model::scale(SimTK::State& s, const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
    int i;

    // 1. Save the current pose of the model, then put it in a
    //    default pose, so pre- and post-scale muscle lengths
    //    can be found.
    SimTK::Vector savedConfiguration = s.getY();
    applyDefaultConfiguration(s);
    // 2. For each Actuator, call its preScale method so it
    //    can calculate and store its pre-scale length in the
    //    current position, and then call its scale method to
    //    scale all of the muscle properties except tendon and
    //    fiber length.
    for (i = 0; i < get_ForceSet().getSize(); i++)
    {
        PathActuator* act = dynamic_cast<PathActuator*>(&get_ForceSet().get(i));
        if( act ) {
            act->preScale(s, aScaleSet);
            act->scale(s, aScaleSet);
        }
        // Do ligaments as well for now until a general mechanism is introduced. -Ayman 5/15
        else {
            Ligament* ligament = dynamic_cast<Ligament*>(&get_ForceSet().get(i));
            if (ligament){
                ligament->preScale(s, aScaleSet);
                ligament->scale(s, aScaleSet);
            }
        }
    }
    // 3. Scale the rest of the model
    bool returnVal = updSimbodyEngine().scale(s, aScaleSet, aFinalMass, aPreserveMassDist);

    // 4. If the dynamics engine was scaled successfully,
    //    call each Muscle's postScale method so it
    //    can calculate its post-scale length in the current
    //    position and then scale the tendon and fiber length
    //    properties.


    if (returnVal)
    {
        for (i = 0; i < get_ForceSet().getSize(); i++) {
            PathActuator* act = dynamic_cast<PathActuator*>(&get_ForceSet().get(i));
            if( act ) {
                act->postScale(s, aScaleSet);
            }
            // Do ligaments as well for now until a general mechanism is introduced. -Ayman 5/15
            else {
                Ligament* ligament = dynamic_cast<Ligament*>(&get_ForceSet().get(i));
                if (ligament){
                    ligament->postScale(s, aScaleSet);
                }
            }
        }

        // Changed the model after scaling path actuators. Have to recreate system!
        s = initSystem();

        // 5. Put the model back in whatever pose it was in.
        s.updY() = savedConfiguration;
        getMultibodySystem().realize( s, SimTK::Stage::Velocity );
    }

    return returnVal;
}


//=============================================================================
// PRINT
//=============================================================================
//_____________________________________________________________________________
/**
 * Print some basic information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::printBasicInfo(std::ostream &aOStream) const
{
    aOStream<<"               MODEL: "<<getName()<<std::endl;
    aOStream<<"         coordinates: "<<getCoordinateSet().getSize()<<std::endl;
    aOStream<<"              forces: "<<getForceSet().getSize()<<std::endl;
    aOStream<<"           actuators: "<<getActuators().getSize()<<std::endl;
    aOStream<<"             muscles: "<<getMuscles().getSize()<<std::endl;
    aOStream<<"            analyses: "<<getNumAnalyses()<<std::endl;
    aOStream<<"              probes: "<<getProbeSet().getSize()<<std::endl;
    aOStream<<"              bodies: "<<getBodySet().getSize()<<std::endl;
    aOStream<<"              joints: "<<((OpenSim::Model*)this)->getJointSet().getSize()<<std::endl;
    aOStream<<"         constraints: "<<getConstraintSet().getSize()<<std::endl;
    aOStream<<"             markers: "<<getMarkerSet().getSize()<<std::endl;
    aOStream<<"         controllers: "<<getControllerSet().getSize()<<std::endl;
    aOStream<<"  contact geometries: "<< getContactGeometrySet().getSize() << std::endl;
    aOStream<<"              frames: "<< getFrameSet().getSize() << std::endl;
    aOStream<<"misc modelcomponents: "<< getMiscModelComponentSet().getSize() << std::endl;

}
//_____________________________________________________________________________
/**
 * Print detailed information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::printDetailedInfo(const SimTK::State& s, std::ostream &aOStream) const
{
    //int i;

    aOStream << "MODEL: " << getName() << std::endl;

    aOStream << std::endl;
    aOStream << "numStates = " << s.getNY() << std::endl;
    aOStream << "numCoordinates = " << getNumCoordinates() << std::endl;
    aOStream << "numSpeeds = " << getNumSpeeds() << std::endl;
    aOStream << "numActuators = " << getActuators().getSize() << std::endl;
    aOStream << "numBodies = " << getNumBodies() << std::endl;
    aOStream << "numConstraints = " << getConstraintSet().getSize() << std::endl;
    aOStream << "numProbes = " << getProbeSet().getSize() << std::endl;

    aOStream << "\nANALYSES (total: " << getNumAnalyses() << ")" << std::endl;
    for (int i = 0; i < _analysisSet.getSize(); i++)
        aOStream << "analysis[" << i << "] = " << _analysisSet.get(i).getName() << std::endl;

    aOStream << "\nBODIES (total: " << getNumBodies() << ")" << std::endl;
    const BodySet& bodySet = getBodySet();
    for(int i=0; i < bodySet.getSize(); i++) {
        const OpenSim::Body& body = bodySet.get(i);
        aOStream << "body[" + std::to_string(i) + "] = " + body.getName() + ". ";
        aOStream << "mass: " << body.get_mass() << std::endl;
        const SimTK::Inertia& inertia = body.getInertia();
        aOStream << "              moments of inertia:  " << inertia.getMoments()
            << std::endl;
        aOStream << "              products of inertia: " << inertia.getProducts()
            << std::endl;
    }

    aOStream << "\nJOINTS (total: " << getNumJoints() << ")" << std::endl;
    const JointSet& jointSet = getJointSet();
    for (int i = 0; i < jointSet.getSize(); i++) {
        const OpenSim::Joint& joint = get_JointSet().get(i);
        aOStream << "joint[" << i << "] = " << joint.getName() << ".";
        aOStream << " parent: " << joint.getParentFrame().getName() <<
            ", child: " << joint.getChildFrame().getName() << std::endl;
    }

    aOStream << "\nACTUATORS (total: " << getActuators().getSize() << ")" << std::endl;
    for (int i = 0; i < getActuators().getSize(); i++) {
         aOStream << "actuator[" << i << "] = " << getActuators().get(i).getName() << std::endl;
    }

    /*
    int n;
    aOStream<<"MODEL: "<<getName()<<std::endl;

    n = getNumBodies();
    aOStream<<"\nBODIES ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) aOStream<<"body["<<i<<"] = "<<getFrameName(i)<<std::endl;

    n = getNQ();
    aOStream<<"\nGENERALIZED COORDINATES ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) aOStream<<"q["<<i<<"] = "<<getCoordinateName(i)<<std::endl;

    n = getNU();
    aOStream<<"\nGENERALIZED SPEEDS ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) aOStream<<"u["<<i<<"] = "<<getSpeedName(i)<<std::endl;

    n = getNA();
    aOStream<<"\nACTUATORS ("<<n<<")" << std::endl;
    for(i=0;i<n;i++) aOStream<<"actuator["<<i<<"] = "<<getActuatorName(i)<<std::endl;

    n = getNP();
    aOStream<<"\nCONTACTS ("<<n<<")" << std::endl;

*/
    Array<string> stateNames = getStateVariableNames();
    aOStream<<"\nSTATES (total: "<<stateNames.getSize()<<")"<<std::endl;
    for(int i=0;i<stateNames.getSize();i++) aOStream<<"y["<<i<<"] = "<<stateNames[i]<<std::endl;
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
/**
 * Replace all markers in the model with the ones in the passed-in marker set.
 *
 * @param aMarkerSet The new marker set.
 * @return Number of markers that were successfully added to the model.
 */
int Model::replaceMarkerSet(const SimTK::State& s, MarkerSet& aMarkerSet)
{
    int i, numAdded = 0;

    // First remove all existing markers from the model.
    upd_MarkerSet().clearAndDestroy();

    // Now add the markers from aMarkerSet whose body names match bodies in the engine.
    for (i = 0; i < aMarkerSet.getSize(); i++)
    {
        // Eran: we make a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
        Marker* marker = aMarkerSet.get(i).clone();
        const string& frameName = marker->getFrameName();
        if (getFrameSet().contains(frameName))
        {
            const OpenSim::PhysicalFrame* frame = dynamic_cast<const PhysicalFrame*>(&getFrameSet().get(frameName));
            if(frame) marker->changeFrame(*frame);
            upd_MarkerSet().adoptAndAppend(marker);
            numAdded++;
        }
    }

    cout << "Replaced marker set in model " << getName() << endl;
    return numAdded;
}

//_____________________________________________________________________________
/**
 * Update all markers in the model with the ones in the
 * passed-in marker set. If the marker does not yet exist
 * in the model, it is added.
 *
 * @param aMarkerSet set of markers to be updated/added
 */
void Model::updateMarkerSet(MarkerSet& aMarkerSet)
{
    for (int i = 0; i < aMarkerSet.getSize(); i++)
    {
        Marker& updatingMarker = aMarkerSet.get(i);

        /* If there is already a marker in the model with that name,
         * update it with the parameters from the updating marker,
         * moving it to a new body if necessary.
         */
        if (updMarkerSet().contains(updatingMarker.getName()))
        {
            Marker& modelMarker = updMarkerSet().get(updatingMarker.getName());
            /* If the updating marker is on a different body, delete the
             * marker from the model and add the updating one (as long as
             * the updating marker's body exists in the model).
             */
            //if (modelMarker.getBody().getName() != updatingBodyName)
            //{
                upd_MarkerSet().remove(&modelMarker);
                // Eran: we append a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
                //upd_MarkerSet().adoptAndAppend(updatingMarker.clone());
            //}
            //else
            //{
            //  modelMarker.updateFromMarker(updatingMarker);
            //}
        }
        //else
        {
            /* The model does not contain a marker by that name. If it has
             * a body by that name, add the updating marker to the markerset.
             */
            // Eran: we append a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
            //if (getBodySet().contains(updatingBodyName))
                addMarker(updatingMarker.clone());
        }
    }

    // Todo_AYMAN: We need to call connectMarkerToModel() again to make sure the
    // _body pointers are up to date; but note that we've already called 
    // it before so we need to make sure the connectMarkerToModel() function
    // supports getting called multiple times.
    initSystem();
    cout << "Updated markers in model " << getName() << endl;
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

    cout << "Deleted " << numDeleted << " unused markers from model " << getName() << endl;

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
Vector& Model::updControls(const SimTK::State &s) const
{
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::updControls() requires an initialized Model./n" 
            "Prior call to Model::initSystem() is required.");
    }

    // direct the system shared cache 
    Measure_<Vector>::Result controlsCache = 
        Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem()
            .getMeasure(_modelControlsIndex));
    return controlsCache.updValue(s);
}

void Model::markControlsAsValid(const SimTK::State& s) const
{
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::markControlsAsValid() requires an initialized Model./n" 
            "Prior call to Model::initSystem() is required.");
    }

    Measure_<Vector>::Result controlsCache = 
        Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem()
            .getMeasure(_modelControlsIndex));
    controlsCache.markAsValid(s);
}

void Model::setControls(const SimTK::State& s, const SimTK::Vector& controls) const
{   
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::setControls() requires an initialized Model./n" 
            "Prior call to Model::initSystem() is required.");
    }

    // direct the system shared cache 
    Measure_<Vector>::Result controlsCache = 
        Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem()
        .getMeasure(_modelControlsIndex));
    controlsCache.setValue(s, controls);

    // Make sure to re-realize dynamics to make sure controls can affect forces
    // and not just derivatives
    if(s.getSystemStage() == Stage::Dynamics)
        s.invalidateAllCacheAtOrAbove(Stage::Dynamics);
}

/** Const access to controls does not invalidate dynamics */
const Vector& Model::getControls(const SimTK::State &s) const
{
    if( (!_system) || (!_modelControlsIndex.isValid()) ){
        throw Exception("Model::getControls() requires an initialized Model./n" 
            "Prior call to Model::initSystem() is required.");
    }

    // direct the system shared cache 
    Measure_<Vector>::Result controlsCache = Measure_<Vector>::Result::getAs(_system->updDefaultSubsystem().getMeasure(_modelControlsIndex));

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
    for (auto& controller : getComponentList<Controller>()) {
        if (!controller.isDisabled()) {
            controller.computeControls(s, controls);
        }
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
/**
 * Model::formStateStorage is intended to take any storage and populate stateStorage.
 * stateStorage is supposed to be a Storage with labels identical to those obtained by 
 * calling Model::getStateVariableNames(). Columns/entries found in the "originalStorage"
 * are copied to the output statesStorage. Entries not found are populated with 
 * 0s (should be default value).
 */
void Model::formStateStorage(const Storage& originalStorage, Storage& statesStorage)
{
    Array<string> rStateNames = getStateVariableNames();
    int numStates = getNumStateVariables();
    // make sure same size, otherwise warn
    if (originalStorage.getSmallestNumberOfStates() != rStateNames.getSize()){
        cout << "Number of columns does not match in formStateStorage. Found "
            << originalStorage.getSmallestNumberOfStates() << " Expected  " << rStateNames.getSize() << "." << endl;
    }
    // Create a list with entry for each desiredName telling which column in originalStorage has the data
    int* mapColumns = new int[rStateNames.getSize()];
    for(int i=0; i< rStateNames.getSize(); i++){
        // the index is -1 if not found, >=1 otherwise since time has index 0 by defn.
        int fix = originalStorage.getColumnLabels().findIndex(rStateNames[i]);
        if (fix==-1){
            // try removing the complete path name to identify the state_name in storage
            string::size_type last = rStateNames[i].rfind("/");
            string name = rStateNames[i].substr(last+1, rStateNames[i].length()-last);
            fix = originalStorage.getColumnLabels().findIndex(name);
            // this whole method is one big hack and needs to be eliminated- aseth
            // for the time being, allow the new coordinate labeling to be
            // handled from storages generated by older versions
            if (fix == -1){
                if (name == "value"){
                    // old formats did not have "/value" so remove it if here
                    name = rStateNames[i].substr(0, last);
                    last = name.rfind("/");
                    name = name.substr(last + 1, name.length());
                    fix = originalStorage.getColumnLabels().findIndex(name);
                }
                else if (name == "speed"){
                    // replace "/speed" (the latest labeling for speeds) with "_u"
                    name = rStateNames[i].substr(0, last);
                    last = name.rfind("/");
                    name = name.substr(last + 1, name.length() - last) + "_u";
                    fix = originalStorage.getColumnLabels().findIndex(name);
                }
                else {
                    // try replacing the '/' with '.' in the last connection
                    name = rStateNames[i];
                    name.replace(last, 1, ".");
                    last = name.rfind("/");
                    name = name.substr(last + 1, rStateNames[i].length() - last);
                    fix = originalStorage.getColumnLabels().findIndex(name);
                }
            }
        }
        mapColumns[i] = fix;
        if (fix==-1){
            cout << "Column "<< rStateNames[i] << " not found in formStateStorage, assuming 0." << endl;
        }
    }
    // Now cycle through and shuffle each

    for (int row =0; row< originalStorage.getSize(); row++){
        StateVector* originalVec = originalStorage.getStateVector(row);
        StateVector* stateVec = new StateVector(originalVec->getTime());
        stateVec->getData().setSize(numStates);  // default value 0f 0.
        for(int column=0; column< numStates; column++){
            double valueInOriginalStorage=0.0;
            if (mapColumns[column]!=-1)
                originalVec->getDataValue(mapColumns[column]-1, valueInOriginalStorage);

            stateVec->setDataValue(column, valueInOriginalStorage);

        }
        statesStorage.append(*stateVec);
    }
    rStateNames.insert(0, "time");
    statesStorage.setColumnLabels(rStateNames);

    delete[] mapColumns;
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
        if (mapColumns[i]==-1)
            cout << "\n Column "<< qNames[i] << " not found in formQStorage, assuming 0.\n" << endl;
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
    updFrameSet().setMemoryOwner(false);
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
    else if (typeString == "Frame")
        return getFrameSet().get(nameString);
    throw Exception("Model::getObjectByTypeAndName: no object of type " + typeString +
        " and name "+nameString+" was found in the model.");

}

//------------------------------------------------------------------------------
//          REALIZE THE SYSTEM TO THE REQUIRED COMPUTATIONAL STAGE
//------------------------------------------------------------------------------
void Model::realizeTime(const SimTK::State& state) const
{
    getSystem().realize(state, Stage::Time);
}

void Model::realizePosition(const SimTK::State& state) const
{
    getSystem().realize(state, Stage::Position);
}

void Model::realizeVelocity(const SimTK::State& state) const
{
    getSystem().realize(state, Stage::Velocity);
}

void Model::realizeDynamics(const SimTK::State& state) const
{
    getSystem().realize(state, Stage::Dynamics);
}

void Model::realizeAcceleration(const SimTK::State& state) const
{
    getSystem().realize(state, Stage::Acceleration);
}

void Model::realizeReport(const SimTK::State& state) const
{
    getSystem().realize(state, Stage::Report);
}


/**
 * Compute the derivatives of the generalized coordinates and speeds.
 */
void Model::computeStateVariableDerivatives(const SimTK::State &s) const
{
    try {
        realizeAcceleration(s);
    }
    catch (const std::exception& e){
        string exmsg = e.what();
        throw Exception(
            "Model::computeStateVariableDerivatives: failed. See: "+exmsg);
    }
}

/**
 * Get the total mass of the model
 *
 * @return the mass of the model
 */
double Model::getTotalMass(const SimTK::State &s) const
{
    return getMatterSubsystem().calcSystemMass(s);
}
/**
 * getInertiaAboutMassCenter
 *
 */
SimTK::Inertia Model::getInertiaAboutMassCenter(const SimTK::State &s) const
{
    SimTK::Inertia inertia = getMatterSubsystem().calcSystemCentralInertiaInGround(s);

    return inertia;
}
/**
 * Return the position vector of the system mass center, measured from the Ground origin, and expressed in Ground.
 *
 */
SimTK::Vec3 Model::calcMassCenterPosition(const SimTK::State &s) const
{
    getMultibodySystem().realize(s, Stage::Position);
    return getMatterSubsystem().calcSystemMassCenterLocationInGround(s);    
}
/**
 * Return the velocity vector of the system mass center, measured from the Ground origin, and expressed in Ground.
 *
 */
SimTK::Vec3 Model::calcMassCenterVelocity(const SimTK::State &s) const
{
    getMultibodySystem().realize(s, Stage::Velocity);
    return getMatterSubsystem().calcSystemMassCenterVelocityInGround(s);    
}
/**
 * Return the acceleration vector of the system mass center, measured from the Ground origin, and expressed in Ground.
 *
 */
SimTK::Vec3 Model::calcMassCenterAcceleration(const SimTK::State &s) const
{
    getMultibodySystem().realize(s, Stage::Acceleration);
    return getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s);    
}

/**
* Construct outputs
*
**/

