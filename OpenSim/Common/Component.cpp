/* -------------------------------------------------------------------------- *
 *                            OpenSim: Component.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Michael Sherman                                      *
 * Contributor(s): Ayman Habib, F. C. Anderson                                *
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

// INCLUDES
#include "Component.h"
#include "OpenSim/Common/IO.h"
#include "XMLDocument.h"

#include <regex>
#include <unordered_map>
#include <unordered_set>

namespace OpenSim {

//==============================================================================
//                            COMPONENT MEASURE
//==============================================================================
// Every OpenSim::Component is associated with a Simbody Measure of type
// ComponentMeasure defined here. This provides a full set of realize()
// methods for performing computations with System resources that are maintained
// at the Component base level, such as calculating state derivatives.

template <class T>
class ComponentMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(ComponentMeasure, SimTK::Measure_<T>);

    ComponentMeasure(SimTK::Subsystem& sub,
                          const OpenSim::Component& mc)
    :   SimTK::Measure_<T>(sub, new Implementation(mc),
                    SimTK::AbstractMeasure::SetHandle()) {}

    SimTK_MEASURE_HANDLE_POSTSCRIPT(ComponentMeasure, SimTK::Measure_<T>);
};


template <class T>
class ComponentMeasure<T>::Implementation
:   public SimTK::Measure_<T>::Implementation {
public:
    // Don't allocate a value cache entry since this measure's value is
    // just a dummy.
    explicit Implementation(const Component& c)
    :   SimTK::Measure_<T>::Implementation(0), _Component(c) {}

    // Implementations of Measure_<T>::Implementation virtual methods.

    Implementation* cloneVirtual() const override final
    {   return new Implementation(*this); }

    int getNumTimeDerivativesVirtual() const override final {return 0;}
    SimTK::Stage getDependsOnStageVirtual(int order) const override final
    {   return SimTK::Stage::Empty; }

    const T& getUncachedValueVirtual
       (const SimTK::State& s, int derivOrder) const override final
    {   return this->getValueZero(); }

    void realizeMeasureTopologyVirtual(SimTK::State& s) const override final
    {   _Component.extendRealizeTopology(s); }
    void realizeMeasureModelVirtual(SimTK::State& s) const override final
    {   _Component.extendRealizeModel(s); }
    void realizeMeasureInstanceVirtual(const SimTK::State& s)
        const override final
    {   _Component.extendRealizeInstance(s); }
    void realizeMeasureTimeVirtual(const SimTK::State& s) const override final
    {   _Component.extendRealizeTime(s); }
    void realizeMeasurePositionVirtual(const SimTK::State& s)
        const override final
    {   _Component.extendRealizePosition(s); }
    void realizeMeasureVelocityVirtual(const SimTK::State& s)
        const override final
    {   _Component.extendRealizeVelocity(s); }
    void realizeMeasureDynamicsVirtual(const SimTK::State& s)
        const override final
    {   _Component.extendRealizeDynamics(s); }
    void realizeMeasureAccelerationVirtual(const SimTK::State& s)
        const override final
    {   _Component.extendRealizeAcceleration(s); }
    void realizeMeasureReportVirtual(const SimTK::State& s)
        const override final
    {   _Component.extendRealizeReport(s); }

private:
    const Component& _Component;
};


//==============================================================================
//                              COMPONENT
//==============================================================================
Component::Component() : Object()
{
    constructProperty_components();
}

Component::Component(const std::string& fileName, bool updFromXMLNode)
:   Object(fileName, updFromXMLNode)
{
    constructProperty_components();
}

Component::Component(SimTK::Xml::Element& element) : Object(element)
{
    constructProperty_components();
}

bool Component::isComponentInOwnershipTree(const Component* subcomponent) const {
    //get to the root Component
    const Component* root = this;
    while (root->hasOwner()) {
        root = &(root->getOwner());
    }
    // if the root has no immediate subcomponents do not bother
    // checking if the subcomponent is in the ownership tree
    if ((root->getNumImmediateSubcomponents() > 0)) {
        auto components = root->getComponentList<Component>();
        for (auto& c : components) {
            if (subcomponent == &c) return true;
        }
    }
    return false;
}

void Component::addComponent(Component* subcomponent)
{
    OPENSIM_THROW_IF(isComponentInOwnershipTree(subcomponent),
                     ComponentAlreadyPartOfOwnershipTree,
                      subcomponent->getName(), getName());

    updProperty_components().adoptAndAppendValue(subcomponent);
    finalizeFromProperties();

    prependComponentPathToConnecteePath(*subcomponent);

    // allow the derived Component to perform secondary operations
    // in response to the inclusion of the subcomponent
    extendAddComponent(subcomponent);
}

std::unique_ptr<Component> Component::extractComponent(Component* subcomponent)
{
    auto& componentsProp = updProperty_components();

    // Try to find `subcomponent` in the `components` property.
    int idx = -1;
    for (int i = 0; i < componentsProp.size(); ++i) {
        if (&componentsProp[i] == subcomponent) {
            idx = i;
            break;
        }
    }
    if (idx == -1) {
        return nullptr;  // Not found.
    }

    // Perform removal
    std::unique_ptr<Component> rv = componentsProp.extractValueAtIndex(idx);
    finalizeFromProperties();
    return rv;
}

bool Component::removeComponent(Component* subcomponent)
{
    return extractComponent(subcomponent) != nullptr;  // `std::unique_ptr<Component>` handles destruction
}

void Component::prependComponentPathToConnecteePath(
        Component& subcomponent) {
    const std::string compPath = subcomponent.getAbsolutePathString();
    const Component& root = subcomponent.getRoot();
    for (auto& comp : subcomponent.updComponentList()) {
        for (auto& it : comp._socketsTable) {
            // Only apply prepend logic if the socket connection is within the
            // added subcomponent. To do this, check if the connection is *not*
            // available in the root component. For list sockets, we assume that
            // checking the first connectee path is sufficient, since all
            // connectees in the list must be from the same component.
            if (it.second->getNumConnectees() > 0 &&
                    !root.hasComponent(it.second->getConnecteePath(0))) {
                it.second->prependComponentPathToConnecteePath(compPath);
            }
        }
        for (auto& it : comp._inputsTable) {
            it.second->prependComponentPathToConnecteePath(compPath);
        }
    }
}

void Component::finalizeFromProperties()
{
    reset();

    // last opportunity to modify Object names based on properties
    if (!hasOwner()) {
        // only call when Component is root since method is recursive
        makeObjectNamesConsistentWithProperties();
    }

    // TODO use a flag to set whether we are lenient on having nameless
    // Components. For backward compatibility we need to be able to
    // handle nameless components so assign them their class name
    // - aseth
    if (getName().empty()) {
        setName(IO::Lowercase(getConcreteClassName()));
    }

    OPENSIM_THROW_IF( getName().empty(), ComponentHasNoName,
                      getConcreteClassName() );

    ComponentPath cp;
    OPENSIM_THROW_IF( !cp.isLegalPathElement(getName()), InvalidComponentName,
        getName(), cp.getInvalidChars(), getConcreteClassName());

    for (auto& comp : _memberSubcomponents) {
        comp->setOwner(*this);
    }
    for (auto& comp : _adoptedSubcomponents) {
        comp->setOwner(*this);
    }

    // Provide sockets, inputs, and outputs with a pointer to its component
    // (this) so that they can invoke the component's methods.
    for (auto& it : _socketsTable) {
        it.second->setOwner(*this);
        // Let the Socket handle any errors in the connectee_name property.
        it.second->checkConnecteePathProperty();
    }
    for (auto& it : _inputsTable) {
        it.second->setOwner(*this);
        // Let the Socket handle any errors in the connectee_name property.
        it.second->checkConnecteePathProperty();
    }
    for (auto& it : _outputsTable) {
        it.second->setOwner(*this);
    }

    markPropertiesAsSubcomponents();
    componentsFinalizeFromProperties();

    // The following block is used to ensure that deserialized names of
    // Components are unique so they can be used to unambiguously locate
    // and connect all loaded Components. If a duplicate is encountered,
    // it is assigned a unique name.
    std::unordered_set<std::string> names;
    names.reserve(getNumImmediateSubcomponents());

    std::string uniqueName;
    forEachImmediateSubcomponent([&](Component& sub)
    {
        const std::string& name = sub.getName();

        // reset duplicate count and search name
        int count = 0;

        // temp variable to hold the unique name used to rename a duplicate
        uniqueName = name;

        // while the name is still not unique keep incrementing the count
        while (names.find(uniqueName) != names.cend()) {
            // In the future this should become an Exception
            //OPENSIM_THROW(SubcomponentsWithDuplicateName, getName(), searchName);
            // for now, rename the duplicately named subcomponent
            // but first test the uniqueness of the name (the while condition)
            uniqueName = name + "_" + std::to_string(count++);
        }

        if (count > 0) { // if a duplicate
            // Warn of the problem
            log_warn("{} '{}' has subcomponents with duplicate name '{}'. "
                     "The duplicate is being renamed to '{}'.",
                     getConcreteClassName(), getName(), name, uniqueName);

            // Now rename the subcomponent with its verified unique name
            sub.setName(uniqueName);
        }

        // keep track of unique names
        names.insert(uniqueName);
    });
    // End of duplicate finding and renaming.

    extendFinalizeFromProperties();
    setObjectIsUpToDateWithProperties();
}

// Base class implementation of virtual method.
// Call finalizeFromProperties on all subcomponents
void Component::componentsFinalizeFromProperties() const
{
    forEachImmediateSubcomponent([](const Component& subcomponent)
    {
        const_cast<Component&>(subcomponent).finalizeFromProperties();
    });
}

// Base class implementation of non-virtual finalizeConnections method.
void Component::finalizeConnections(Component& root)
{
    if (!isObjectUpToDateWithProperties()){
        // if edits occur between construction and connect() this is
        // the last chance to finalize before addToSystem.
        finalizeFromProperties();
    }

    for (auto& it : _socketsTable) {
        auto& socket = it.second;
        try {
            socket->finalizeConnection(root);
        }
        catch (const std::exception& x) {
            OPENSIM_THROW_FRMOBJ(Exception, "Failed to connect Socket '" +
                socket->getName() + "' of type " +
                socket->getConnecteeTypeName() +
                " (details: " + x.what() + ").");
        }
    }

    for (auto& it : _inputsTable) {
        auto& input = it.second;
        try {
            input->finalizeConnection(root);
        }
        catch (const std::exception& x) {
            OPENSIM_THROW_FRMOBJ(Exception, "Failed to connect Input '" +
                input->getName() + "' of type " + input->getConnecteeTypeName()
                + " (details: " + x.what() + ").");
        }
    }

    // Allow derived Components to handle/check their connections and also
    // override the order in which its subcomponents are ordered when
    // adding subcomponents to the System
    extendFinalizeConnections(root);

    // Allow subcomponents to form their connections
    componentsFinalizeConnections(root);

    // Forming connections changes the Socket which is a property
    // Remark as upToDate.
    setObjectIsUpToDateWithProperties();
}

// invoke connect on all (sub)components of this component
void Component::componentsFinalizeConnections(Component& root)
{
    // enable the subcomponents the opportunity to connect themselves
    forEachImmediateSubcomponent([&root](Component& subcomponent)
    {
        subcomponent.finalizeConnections(root);
    });
}

void Component::clearConnections()
{
    // First give the subcomponents the opportunity to disconnect themselves
    forEachImmediateSubcomponent([](Component& subcomponent)
    {
        subcomponent.clearConnections();
    });

    //Now cycle through and disconnect all sockets for this component
    for (auto& it : _socketsTable) {
        it.second->disconnect();
    }

    // Must also clear the input's connections.
    for (auto& it : _inputsTable) {
        it.second->disconnect();
    }

    //now clear all the stored system indices from this component
    reset();
}

void Component::addToSystem(SimTK::MultibodySystem& system) const
{
    // If being asked to be added to the same System that it is already
    // a part, there is nothing to be done.
    if (hasSystem() && (&getSystem() == &system)) {
        return;
    }
    baseAddToSystem(system);
    extendAddToSystem(system);
    componentsAddToSystem(system);
    extendAddToSystemAfterSubcomponents(system);
}

// Base class implementation of virtual method.
// Every Component owns an underlying SimTK::Measure
// which is a ComponentMeasure<T> and is added to the System's default
// subsystem. That measure is used only for the side effect of its realize()
// methods being called; its value is not used.
void Component::baseAddToSystem(SimTK::MultibodySystem& system) const
{
    if (!isObjectUpToDateWithProperties()) {
        std::string msg = "Component " + getConcreteClassName() + "::" + getName();
        msg += " cannot extendAddToSystem until it is up-to-date with its properties.";

        throw Exception(msg);
    }

    // Clear cached list of all related StateVariables if any from a previous
    // System.
    _allStateVariables.clear();

    // Briefly get write access to the Component to record some
    // information associated with the System; that info is const after this.
    Component* mutableThis = const_cast<Component *>(this);
    mutableThis->_system = system;

    // Allocate the ComponentMeasure, point it to this Component for
    // making realize() calls, and add it to the system's default subsystem.
    ComponentMeasure<double> mcMeasure(system.updDefaultSubsystem(), *this);
    mutableThis->_simTKcomponentIndex = mcMeasure.getSubsystemMeasureIndex();
}

void Component::componentsAddToSystem(SimTK::MultibodySystem& system) const
{
    // If _orderedSubcomponents is specified, then use this Component's
    // specification for the order in which subcomponents are added. At a
    // minimum the order for all immediate subcomponents must be specified.
    if (_orderedSubcomponents.size() >= getNumImmediateSubcomponents()) {
        for (const auto& compRef : _orderedSubcomponents) {
            compRef->addToSystem(system);
        }
    }
    else if (_orderedSubcomponents.size() == 0) {
        // Otherwise, invoke on all immediate subcomponents in tree order
        forEachImmediateSubcomponent([&system](const Component& subcomponent)
        {
            subcomponent.addToSystem(system);
        });
    }
    else {
        OPENSIM_THROW_FRMOBJ(Exception,
            "_orderedSubcomponents specified, but its size does not reflect the "
            "the number of immediate subcomponents. Verify that you have included "
            "all immediate subcomponents in the ordered list."
        )
    }
}

void Component::initStateFromProperties(SimTK::State& state) const
{
    extendInitStateFromProperties(state);
    componentsInitStateFromProperties(state);
}

void Component::componentsInitStateFromProperties(SimTK::State& state) const
{
    forEachImmediateSubcomponent([&state](const Component& subcomponent)
    {
        subcomponent.initStateFromProperties(state);
    });
}

void Component::setPropertiesFromState(const SimTK::State& state)
{
    extendSetPropertiesFromState(state);
    componentsSetPropertiesFromState(state);
}

void Component::componentsSetPropertiesFromState(const SimTK::State& state)
{
    forEachImmediateSubcomponent([&state](Component& subcomponent)
    {
        subcomponent.setPropertiesFromState(state);
    });
}

// Base class implementation of virtual method. Note that we're not handling
// subcomponents here; this method gets called from extendRealizeAcceleration()
// which will be invoked for each (sub) component by its own ComponentMeasure.
void Component::computeStateVariableDerivatives(const SimTK::State& s) const
{
    int nsv = getNumStateVariablesAddedByComponent();
    if(nsv > 0){
        int nasv = 0;
        std::map<std::string, StateVariableInfo>::const_iterator it;
        for(it = _namedStateVariableInfo.begin();
            it != _namedStateVariableInfo.end(); ++it){
                const StateVariable& sv = *it->second.stateVariable;
                const AddedStateVariable *asv =
                    dynamic_cast<const AddedStateVariable *>(&sv);
                if(asv) nasv++;
        }
        if(nasv > 0){
            std::stringstream msg;
            msg << "Component " + getConcreteClassName()+"::"+getName();
            msg << " added " << nasv << " state variables and ";
            msg << " must specify their derivatives." << std::endl;

            throw Exception(msg.str());
        }
    }
}


void
Component::
addModelingOption(const std::string& moName,
    int maxFlagValue, bool allocate) const
{
    // don't add modeling option if there is another state with the same
    // name for this component
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.find(moName);
    if(it != _namedModelingOptionInfo.end())
        throw Exception("Component::addModelingOption: Modeling option '"
              + moName + "' already exists.");
    // assign a "slot" for a modeling option by name
    // modeling option index will be invalid by default
    // upon allocation during realizeTopology the index will be set
    _namedModelingOptionInfo[moName] =
        ModelingOptionInfo(maxFlagValue, allocate);
}

void Component::addStateVariable(const std::string&  stateVariableName,
                                 const SimTK::Stage& invalidatesStage,
                                 bool isHidden) const
{
    if ((invalidatesStage < SimTK::Stage::Position) ||
            (invalidatesStage > SimTK::Stage::Dynamics)) {
        throw Exception("Component::addStateVariable: invalidatesStage "
                        "must be Position, Velocity or Dynamics.");
    }
    // Allocate space for a new state variable
    AddedStateVariable* asv =
        new AddedStateVariable(stateVariableName, *this, invalidatesStage, isHidden);
    // Add it to the Component and let it take ownership
    addStateVariable(asv);
}


void Component::addStateVariable(Component::StateVariable*  stateVariable) const
{
    const std::string& stateVariableName = stateVariable->getName();
    // don't add state if there is another state variable with the same name
    // for this component
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(stateVariableName);
    if(it != _namedStateVariableInfo.end()){
        throw Exception("Component::addStateVariable: State variable '" +
            stateVariableName + "' already exists.");
    }

    int order = (int)_namedStateVariableInfo.size();

    // assign a "slot" for a state variable by name
    // state variable index will be invalid by default
    // upon allocation during realizeTopology the index will be set
    _namedStateVariableInfo[stateVariableName] =
        StateVariableInfo(stateVariable, order);

    const AddedStateVariable* asv =
        dynamic_cast<const Component::AddedStateVariable *>(stateVariable);
    // Now automatically add a cache variable to hold the derivative
    // to enable a similar interface for setting and getting the derivatives
    // based on the creator specified state name
    if(asv){
        addCacheVariable(
                stateVariableName + "_deriv", 0.0, SimTK::Stage::Dynamics);
    }

}


void
Component::
addDiscreteVariable(const std::string& dvName,
    SimTK::Stage invalidatesStage, bool allocate) const
{
    // Don't add discrete var if there is another discrete variable with the
    // same name for this component.
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(dvName);
    if(it != _namedDiscreteVariableInfo.end()) {
        throw Exception("Component::addDiscreteVariable: discrete variable '" +
            dvName + "' already exists.");
    }
    // Assign "slots" for the discrete variables by name.
    // Discrete variable indices will be invalid by default.
    // Upon allocation during realizeTopology, the indices will be set.
    _namedDiscreteVariableInfo[dvName] =
        DiscreteVariableInfo(invalidatesStage, allocate);
}


Array<std::string>
Component::
getModelingOptionNames() const {
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    Array<std::string> moNames = getModelingOptionNamesAddedByComponent();

    for (int i = 0; i < moNames.size(); ++i) {
        moNames[i] = (getAbsolutePathString() + "/" + moNames[i]);
    }

    for (auto& comp : getComponentList<Component>()) {
        const std::string& pathName = comp.getAbsolutePathString();
        Array<std::string> subMONames =
            comp.getModelingOptionNamesAddedByComponent();
        for (int i = 0; i < subMONames.size(); ++i) {
            moNames.append(pathName + "/" + subMONames[i]);
        }
    }

    return moNames;
}

Array<std::string>
Component::
getModelingOptionNamesAddedByComponent() const {
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.begin();

    Array<std::string> names("", (int)_namedModelingOptionInfo.size());

    int i = 0;
    while (it != _namedModelingOptionInfo.end()) {
        names[i] = it->first;
        ++it;
        ++i;
    }
    return names;
}

int Component::
getModelingOption(const SimTK::State& s, const std::string& path) const
{
    if (path.find(ComponentPath::separator()) == std::string::npos) {
        // `path` is just the name of a modeling option on this component, so
        // directly look it up

        const auto it = _namedModelingOptionInfo.find(path);
        if (it != _namedModelingOptionInfo.end()) {
            return SimTK::Value<int>::downcast(
                s.getDiscreteVariable(it->second.ssIndex,
                    it->second.moIndex)).get();
        }
        else {
            OPENSIM_THROW(VariableNotFound, getName(), path);
        }
    }
    else {
        // `path` might be a path to some other component, so parse the
        // string as a path and defer to the path-based implementation (slower)
        return getModelingOption(s, ComponentPath{path});
    }
}

int Component::
getModelingOption(const SimTK::State& s, const ComponentPath& path) const
{
    // traverse to the component that owns the modeling option and return its
    // value

    std::string moName;
    const Component* owner = resolveVariableNameAndOwner(path, moName);

    const auto it = owner->_namedModelingOptionInfo.find(moName);
    if(it != owner->_namedModelingOptionInfo.end()) {
        return SimTK::Value<int>::downcast(
            s.getDiscreteVariable(it->second.ssIndex,
                it->second.moIndex)).get();
    } else {
        OPENSIM_THROW(VariableNotFound, getName(), moName);
        return -1;
    }
}

void Component::
setModelingOption(SimTK::State& s, const std::string& path, int flag) const
{
    // unlike `getModelingOption`, there's no need to check if it's path-y or
    // not, because this function isn't called as often during a simulation
    setModelingOption(s, ComponentPath{path}, flag);
}

void Component::
setModelingOption(SimTK::State& s, const ComponentPath& path, int flag) const
{
    // traverse to the component that owns the modeling option and set its value

    std::string moName;
    const Component* owner = resolveVariableNameAndOwner(path, moName);
    const auto it = owner->_namedModelingOptionInfo.find(moName);

    if(it != owner->_namedModelingOptionInfo.end()) {
        if(flag > it->second.maxOptionValue){
            OPENSIM_THROW(ModelingOptionMaxExceeded, getName(), moName, flag,
                it->second.maxOptionValue);
        }
        SimTK::Value<int>::downcast(
            s.updDiscreteVariable(it->second.ssIndex,
                it->second.moIndex)).upd() = flag;
    } else {
        OPENSIM_THROW(VariableNotFound, getName(), moName);
    }
}

unsigned Component::printComponentsMatching(const std::string& substring) const
{
    auto components = getComponentList();
    components.setFilter(ComponentFilterAbsolutePathNameContainsString(substring));
    unsigned count = 0;
    for (const auto& comp : components) {
        log_cout(comp.getAbsolutePathString());
        ++count;
    }
    return count;
}

int Component::getNumStateVariables() const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    //Get the number of state variables added (or exposed) by this Component
    int ns = getNumStateVariablesAddedByComponent();

    // And then include the states of its subcomponents
    forEachImmediateSubcomponent([&ns](const Component& subcomponent)
    {
        ns += subcomponent.getNumStateVariables();
    });

    return ns;
}


const Component& Component::getOwner() const
{
    if (!hasOwner()) {
        std::string msg = "Component '" + getName() + "'::getOwner(). " +
            "Has no owner assigned.\n" +
            "Make sure the component was added to the Model " +
            "(or another component).";
        throw Exception(msg);
    }
    return _owner.getRef();
}

bool Component::hasOwner() const
{
    return !_owner.empty();
}

const Component& Component::getRoot() const {
    const Component* root = this;
    while (root->hasOwner()) {
        root = &root->getOwner();
    }
    return *root;
}

void Component::setOwner(const Component& owner)
{
    if (&owner == this) {
        std::string msg = "Component '" + getName() + "'::setOwner(). " +
            "Attempted to set itself as its owner.";
        throw Exception(msg);
    }
    else if (_owner.get() == &owner) {
        return;
    }

    _owner.reset(&owner);
}

std::string Component::getAbsolutePathString() const
{
    if (!hasOwner()) return "/";
    std::string absPathName("/" + getName());

    const Component* up = this;

    while (up && up->hasOwner()) {
        up = &up->getOwner();
        if (up->hasOwner())
            absPathName.insert(0, "/" + up->getName());
    }

    return absPathName;

}

ComponentPath Component::getAbsolutePath() const
{
    if (!hasOwner()) return ComponentPath({}, true);

    std::vector<std::string> pathVec;
    pathVec.push_back(getName());

    const Component* up = this;

    while (up && up->hasOwner()) {
        up = &up->getOwner();
        if (up->hasOwner())
            pathVec.insert(pathVec.begin(), up->getName());
    }

    return ComponentPath(pathVec, true);
}

std::string Component::getRelativePathString(const Component& wrt) const
{
    return getRelativePath(wrt).toString();
}

ComponentPath Component::getRelativePath(const Component& wrt) const
{
    ComponentPath thisP = getAbsolutePath();
    ComponentPath wrtP = wrt.getAbsolutePath();

    return thisP.formRelativePath(wrtP);
}

const Component::StateVariable* Component::
    traverseToStateVariable(const std::string& pathName) const
{
    return traverseToStateVariable(ComponentPath{pathName});
}

const Component::StateVariable* Component::traverseToStateVariable(
        const ComponentPath& path) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    const StateVariable* found = nullptr;
    if (path.getNumPathLevels() == 1) {
        // There was no slash. The state variable should be in this component.
        auto it = _namedStateVariableInfo.find(path.toString());
        if (it != _namedStateVariableInfo.end()) {
            return it->second.stateVariable.get();
        }
    } else if (path.getNumPathLevels() > 1) {
        const auto& compPath = path.getParentPath();
        const Component* comp = traversePathToComponent<Component>(compPath);
        if (comp) {
            // This is the leaf of the path:
            const auto& varName = path.getComponentName();
            found = comp->traverseToStateVariable(varName);
        }
    }

    return found;
}

// Get the names of "continuous" state variables maintained by the Component
// and its subcomponents.
Array<std::string> Component::getStateVariableNames() const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    Array<std::string> stateNames = getStateVariableNamesAddedByComponent();

    for (int i = 0; i < stateNames.size(); ++i) {
        stateNames[i] = (getAbsolutePathString() + "/" + stateNames[i]);
    }

    for (auto& comp : getComponentList<Component>()) {
        const std::string& pathName = comp.getAbsolutePathString();
        Array<std::string> subStateNames =
            comp.getStateVariableNamesAddedByComponent();
        for (int i = 0; i < subStateNames.size(); ++i) {
            stateNames.append(pathName + "/" + subStateNames[i]);
        }
    }

    return stateNames;
}

// Get the value of a state variable allocated by this Component.
double Component::
    getStateVariableValue(const SimTK::State& s, const std::string& name) const
{
    if (name.find(ComponentPath::separator()) == std::string::npos) {
        // `name` is just the name of a state variable on this component, so
        // directly look it up

        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

        auto it = _namedStateVariableInfo.find(name);
        if (it != _namedStateVariableInfo.end()) {
            return it->second.stateVariable->getValue(s);
        }

        std::stringstream msg;
        msg << "Component::getStateVariableValue: ERR- state named '" << name << "' not found in " << getName() << " of type " << getConcreteClassName();
        OPENSIM_THROW_FRMOBJ(Exception, std::move(msg).str());
    }
    else {
        // `name` might be a path to some other component, so parse the
        // string as a path and defer to the path-based implementation (slower)
        return getStateVariableValue(s, ComponentPath{name});
    }
}

// Get the value of a state variable allocated by this Component.
double Component::
    getStateVariableValue(const SimTK::State& s, const ComponentPath& path) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    // find the state variable with this component or its subcomponents
    const StateVariable* rsv = traverseToStateVariable(path);
    if (rsv) {
        return rsv->getValue(s);
    }

    std::stringstream msg;
    msg << "Component::getStateVariableValue: ERR- state named '" << path.toString()
        << "' not found in " << getName() << " of type " << getConcreteClassName();
    throw Exception(msg.str(),__FILE__,__LINE__);

    return SimTK::NaN;
}

// Get the value of a state variable derivative computed by this Component.
double Component::
    getStateVariableDerivativeValue(const SimTK::State& state,
                                const std::string& name) const
{
    if (name.find(ComponentPath::separator()) == std::string::npos) {
        // `name` is just the name of a state variable on this component, so
        // directly look it up

        OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

        computeStateVariableDerivatives(state);

        auto it = _namedStateVariableInfo.find(name);
        if (it != _namedStateVariableInfo.end()) {
            return it->second.stateVariable->getDerivative(state);
        }

        std::stringstream msg;
        msg << "Component::getStateVariableDerivativeValue: ERR- variable name '" << name << "' not found.\n ";
        msg << getName() << " of type " << getConcreteClassName() << " has " << getNumStateVariables() << " states.";
        OPENSIM_THROW_FRMOBJ(Exception, std::move(msg).str());
    }
    else {
        // `name` might be a path to some other component, so parse the
        // string as a path and defer to the path-based implementation (slower)
        return getStateVariableDerivativeValue(state, ComponentPath{name});
    }
}

double Component::
        getStateVariableDerivativeValue(const SimTK::State& state, const ComponentPath& path) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    computeStateVariableDerivatives(state);

    const StateVariable* rsv = traverseToStateVariable(path);
    if (rsv) {
        return rsv->getDerivative(state);
    }

    std::stringstream msg;
    msg << "Component::getStateVariableDerivative: ERR- variable name '" << path
        << "' not found.\n "
        << getName() << " of type " << getConcreteClassName()
        << " has " << getNumStateVariables() << " states.";
    throw Exception(msg.str(),__FILE__,__LINE__);
    return SimTK::NaN;
}

// Set the value of a state variable allocated by this Component given its name
// for this component.
void Component::setStateVariableValue(
        SimTK::State& s, const std::string& name, double value) const {
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    // find the state variable
    const StateVariable* rsv = traverseToStateVariable(name);

    if(rsv){ // find required rummaging through the state variable names
            return rsv->setValue(s, value);
    }

    std::stringstream msg;
    msg << "Component::setStateVariable: ERR- state named '" << name
        << "' not found in " << getName() << " of type "
        << getConcreteClassName() << ".\n";
    throw Exception(msg.str(),__FILE__,__LINE__);
}

bool Component::isAllStatesVariablesListValid() const
{
    int nsv = getNumStateVariables();
    // Consider the list of all StateVariables to be valid if all of
    // the following conditions are true:
    // 1. Component is up-to-date with its Properties
    // 2. a System has been associated with the list of StateVariables
    // 3. The list of all StateVariables is correctly sized (initialized)
    // 4. The System associated with the StateVariables is the current System
    // TODO: Enable the isObjectUpToDateWithProperties() check when computing
    // the path of the GeomtryPath does not involve updating its PathPointSet.
    // This change dirties the GeometryPath which is a property of a Muscle which
    // is property of the Model. Therefore, during integration the Model is not
    // up-to-date and this causes a rebuilding of the cached StateVariables list.
    // See GeometryPath::computePath() for the corresponding TODO that must be
    // addressed before we can re-enable the isObjectUpToDateWithProperties
    // check.
    // It has been verified that the adding Components will invalidate the state
    // variables associated with the Model and force the list to be rebuilt.
    bool valid = //isObjectUpToDateWithProperties() &&                  // 1.
        !_statesAssociatedSystem.empty() &&                             // 2.
        (int)_allStateVariables.size() == nsv &&                        // 3.
        getSystem().isSameSystem(_statesAssociatedSystem.getRef());     // 4.

    return valid;
}


// Get all values of the state variables allocated by this Component. Includes
// state variables allocated by its subcomponents.
SimTK::Vector Component::
    getStateVariableValues(const SimTK::State& state) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    int nsv = getNumStateVariables();
    // if the StateVariables are invalid (see above) rebuild the list
    if (!isAllStatesVariablesListValid()) {
        _statesAssociatedSystem.reset(&getSystem());
        _allStateVariables.clear();
        _allStateVariables.resize(nsv);
        Array<std::string> names = getStateVariableNames();
        for (int i = 0; i < nsv; ++i)
            _allStateVariables[i].reset(traverseToStateVariable(names[i]));
    }

    SimTK::Vector stateVariableValues(nsv, SimTK::NaN);
    for(int i=0; i<nsv; ++i){
        stateVariableValues[i]= _allStateVariables[i]->getValue(state);
    }

    return stateVariableValues;
}

// Set all values of the state variables allocated by this Component. Includes
// state variables allocated by its subcomponents.
void Component::
    setStateVariableValues(SimTK::State& state,
                           const SimTK::Vector& values) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    int nsv = getNumStateVariables();

    SimTK_ASSERT_ALWAYS(values.size() == nsv,
        "Component::setStateVariableValues() number values does not match the "
        "number of state variables.");

    // if the StateVariables are invalid (see above) rebuild the list
    if (!isAllStatesVariablesListValid()) {
        _statesAssociatedSystem.reset(&getSystem());
        _allStateVariables.clear();
        _allStateVariables.resize(nsv);
        Array<std::string> names = getStateVariableNames();
        for (int i = 0; i < nsv; ++i)
            _allStateVariables[i].reset(traverseToStateVariable(names[i]));
    }

    for(int i=0; i<nsv; ++i){
        _allStateVariables[i]->setValue(state, values[i]);
    }
}

// Set the derivative of a state variable computed by this Component by name.
void Component::setStateVariableDerivativeValue(const SimTK::State& state,
        const std::string& name, double value) const {
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(name);

    if(it != _namedStateVariableInfo.end()) {
        const StateVariable& sv = *it->second.stateVariable;
        sv.setDerivative(state, value);
    }
    else{
        std::stringstream msg;
        msg << "Component::setStateVariableDerivative: ERR- name '" << name
            << "' not found.\n "
            << getName() << " of type " << getConcreteClassName()
            << " has " << getNumStateVariables() << " states.";
        throw Exception(msg.str(),__FILE__,__LINE__);
    }
}

Array<std::string> Component::getDiscreteVariableNames() const {
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    Array<std::string> dvNames = getDiscreteVariableNamesAddedByComponent();

    for (int i = 0; i < dvNames.size(); ++i) {
        dvNames[i] = (getAbsolutePathString() + "/" + dvNames[i]);
    }

    for (auto& comp : getComponentList<Component>()) {
        const std::string& pathName = comp.getAbsolutePathString();
        Array<std::string> subDVNames =
            comp.getDiscreteVariableNamesAddedByComponent();
        for (int i = 0; i < subDVNames.size(); ++i) {
            dvNames.append(pathName + "/" + subDVNames[i]);
        }
    }

    return dvNames;
}

Array<std::string> Component::getDiscreteVariableNamesAddedByComponent() const {
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.begin();

    Array<std::string> names("", (int)_namedDiscreteVariableInfo.size());

    int i = 0;
    while (it != _namedDiscreteVariableInfo.end()) {
        names[i] = it->first;
        ++it;
        ++i;
    }
    return names;
}

const Component*
Component::
resolveVariableNameAndOwner(const ComponentPath& path,
    std::string& variableName) const
{
    if(path.empty()) OPENSIM_THROW(EmptyComponentPath, getName());
    size_t nLevels = path.getNumPathLevels();
    variableName = path.getSubcomponentNameAtLevel(nLevels - 1);
    const Component* owner = this;
    if (nLevels > 1) {
        // Need to traverse to the owner of the DV based on the path.
        const ComponentPath& ownerPath = path.getParentPath();
        owner = traversePathToComponent<Component>(ownerPath);
        if (owner == nullptr) {
            OPENSIM_THROW(VariableOwnerNotFoundOnSpecifiedPath, getName(),
                variableName, ownerPath.toString());
        }
    }
    return owner;
}

const SimTK::AbstractValue&
Component::
getDiscreteVariableAbstractValue(const SimTK::State& s,
    const std::string& path) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    // Resolve the name of the DV and its owner.
    std::string dvName{""};
    const Component* owner =
        resolveVariableNameAndOwner(path, dvName);

    // Find the variable.
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = owner->_namedDiscreteVariableInfo.find(dvName);

    if (it != owner->_namedDiscreteVariableInfo.end()) {
        return s.getDiscreteVariable(it->second.ssIndex, it->second.dvIndex);
    } else {
        OPENSIM_THROW(VariableNotFound, getName(), dvName);
    }
}

SimTK::AbstractValue&
Component::
updDiscreteVariableAbstractValue(SimTK::State& s,
    const std::string& path) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    // Resolve the name of the DV and its owner.
    std::string dvName{""};
    const Component* owner =
        resolveVariableNameAndOwner(path, dvName);

    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = owner->_namedDiscreteVariableInfo.find(dvName);

    if (it != owner->_namedDiscreteVariableInfo.end()) {
        return s.updDiscreteVariable(it->second.ssIndex, it->second.dvIndex);
    } else {
        OPENSIM_THROW(VariableNotFound, getName(), dvName);
    }
}


SimTK::CacheEntryIndex Component::getCacheVariableIndex(const std::string& name) const
{
    auto it = this->_namedCacheVariables.find(name);

    if (it != this->_namedCacheVariables.end()) {
        return it->second.index();
    }

    std::stringstream msg;
    msg << "Cache variable with name '" << name << "' not found: maybe the cache variable was not allocated with `Component::addCacheVariable`?";

    OPENSIM_THROW_FRMOBJ(Exception, msg.str());
}

bool Component::isCacheVariableValid(const SimTK::State& state, const std::string& name) const
{
    const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
    const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(name);
    return subsystem.isCacheValueRealized(state, idx);
}

void Component::markCacheVariableValid(const SimTK::State& state, const std::string& name) const
{
    const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
    const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(name);
    subsystem.markCacheValueRealized(state, idx);
}

void Component::markCacheVariableInvalid(const SimTK::State& state, const std::string& name) const
{
    const SimTK::DefaultSystemSubsystem& subsystem = this->getDefaultSubsystem();
    const SimTK::CacheEntryIndex idx = this->getCacheVariableIndex(name);
    subsystem.markCacheValueNotRealized(state, idx);
}

bool Component::constructOutputForStateVariable(const std::string& name)
{
    auto func = [name](const Component* comp,
                       const SimTK::State& s, const std::string&,
                       double& result) -> void {
        result = comp->getStateVariableValue(s, name);
    };
    return constructOutput<double>(name, func, SimTK::Stage::Model);
}

// helper method to specify the order of subcomponents.
void Component::setNextSubcomponentInSystem(const Component& sub) const
{
    auto it =
        std::find(_orderedSubcomponents.begin(), _orderedSubcomponents.end(),
            SimTK::ReferencePtr<const Component>(&sub));
    if (it == _orderedSubcomponents.end()) {
        _orderedSubcomponents.push_back(SimTK::ReferencePtr<const Component>(&sub));
    }
}

void Component::updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
{
    // During deserialization, some components' clone() may
    // finalizeFromProperties(). Upating from XML can then cause stale pointers.
    // We must make sure to clear any pointers to properties that may exist
    // in this component.
    reset();
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30500) {
            // In 3.3 and earlier, spaces in names were tolerated. Spaces are
            // no longer acceptable in Component names.
            if (node.hasAttribute("name")) {
                auto name = node.getRequiredAttribute("name").getValue();
                if (name.find_first_of("\n\t ") < std::string::npos) {
                    log_warn("{} name '{}' contains whitespace.",
                            getConcreteClassName(), name);
                    name.erase(
                        std::remove_if(name.begin(), name.end(), ::isspace),
                        name.end());
                    node.setAttributeValue("name", name);
                    log_warn("It was renamed to '{}'.", name);
                }
            }
            else { // As 4.0 all Components must have a name. If none, assign one.
                // Note: in finalizeFromProperties(), the Component will ensure
                // that names are unique by travesing its list of subcomponents
                // and renaming any duplicates.
                node.setAttributeValue("name",
                    IO::Lowercase(getConcreteClassName()));
            }
        }
        if (versionNumber < 30508) {
            // Here's an example of the change this function might make:
            // Previous: <connectors>
            //               <Connector_PhysicalFrame_ name="parent">
            //                   <connectee_name>...</connectee_name>
            //               </Connector_PhysicalFrame_>
            //           </connectors>
            // New:      <connector_parent_connectee_name>...
            //           </connector_parent_connectee_name>
            //
            XMLDocument::updateConnectors30508(node);
        }

        if(versionNumber < 30510) {
            // Before -- <connector_....> ... </connector_...>
            // After  -- <socket_...> ... </socket_...>
            for(auto iter = node.element_begin();
                iter != node.element_end();
                ++iter) {
                std::string oldName{"connector"};
                std::string newName{"socket"};
                auto tagname = iter->getElementTag();
                auto pos = tagname.find(oldName);
                if(pos != std::string::npos) {
                    tagname.replace(pos, oldName.length(), newName);
                    iter->setElementTag(tagname);
                }
            }

        }
        if (versionNumber <= 30516) {
            static const std::regex s_ConnecteeNamePattern("(socket_|input_)(.*)(_connectee_name)");
            static const std::regex s_ConnecteeNamesPattern("(input_)(.*)(_connectee_names)");

            // Rename xml tags for socket_*_connectee_name to socket_*
            std::string connecteeNameString = "_connectee_name";
            for (auto iter = node.element_begin();
                iter != node.element_end();
                ++iter) {
                auto tagname = iter->getElementTag();
                if (std::regex_match(tagname, s_ConnecteeNamePattern)) {
                    auto pos = tagname.find(connecteeNameString);
                    if (pos != std::string::npos) {
                        tagname.replace(pos, connecteeNameString.length(), "");
                        iter->setElementTag(tagname);
                    }
                }
                else if (std::regex_match(tagname, s_ConnecteeNamesPattern)) {
                    auto pos = tagname.find(connecteeNameString);
                    if (pos != std::string::npos) {
                        tagname.replace(pos, connecteeNameString.length()+1, "");
                        iter->setElementTag(tagname);
                    }
                }
            }
        }
    }
    Super::updateFromXMLNode(node, versionNumber);
}

// mark components owned as properties as subcomponents
void Component::markPropertiesAsSubcomponents()
{
    // Method can be invoked for either constructing a new Component
    // or the properties have been modified. In the latter case
    // we must make sure that pointers to old properties are cleared
    _propertySubcomponents.clear();

    // Now mark properties that are Components as subcomponents
    //loop over all its properties
    for (int i = 0; i < getNumProperties(); ++i) {
        auto& prop = getPropertyByIndex(i);
        // check if property is of type Object
        if (prop.isObjectProperty()) {
            // a property is a list so cycle through its contents
            for (int j = 0; j < prop.size(); ++j) {
                const Object& obj = prop.getValueAsObject(j);
                // if the object is a Component mark it
                if (const Component* comp = dynamic_cast<const Component*>(&obj) ) {
                    markAsPropertySubcomponent(comp);
                }
                else {
                    // otherwise it may be a Set (of objects), and
                    // would prefer to do something like this to test:
                    //  Set<Component>* objects = dynamic_cast<Set<Component>*>(obj)
                    // Instead we can see if the object has a property called
                    // "objects" which is a PropertyObjArray used by Set<T>.
                    // knowing the object Type is useful for debugging
                    // and it could be used to strengthen the test (e.g. scan
                    // for "Set" in the type name).
                    std::string objType = obj.getConcreteClassName();
                    if (obj.hasProperty("objects")) {
                        // get the PropertyObjArray if the object has one
                        auto& objectsProp = obj.getPropertyByName("objects");
                        // loop over the objects in the PropertyObjArray
                        for (int k = 0; k < objectsProp.size(); ++k) {
                            const Object& obj = objectsProp.getValueAsObject(k);
                            // if the object is a Component mark it
                            if (const Component* comp = dynamic_cast<const Component*>(&obj) )
                                markAsPropertySubcomponent(comp);
                        } // loop over objects and mark it if it is a component
                    } // end if property is a Set with "objects" inside
                } // end of if/else property value is an Object or something else
            } // loop over the property list
        } // end if property is an Object
    } // loop over properties
}

// mark a Component as a subcomponent of this one. If already a
// subcomponent, it is not added to the list again.
void Component::markAsPropertySubcomponent(const Component* component)
{
    // Only add if the component is not already a part of this Component
    SimTK::ReferencePtr<Component> compRef(const_cast<Component*>(component));
    auto it =
        std::find(_propertySubcomponents.begin(), _propertySubcomponents.end(), compRef);
    if ( it == _propertySubcomponents.end() ){
        // Must reconstruct the reference pointer in place in order
        // to invoke move constructor from SimTK::Array::push_back
        // otherwise it will copy and reset the Component pointer to null.
        _propertySubcomponents.push_back(
            SimTK::ReferencePtr<Component>(const_cast<Component*>(component)));
    }
    else{
        auto compPath = component->getAbsolutePathString();
        auto foundPath = it->get()->getAbsolutePathString();
        OPENSIM_THROW( ComponentAlreadyPartOfOwnershipTree,
                       component->getName(), getName());
    }

    compRef->setOwner(*this);
}

// Include another Component as a subcomponent of this one. If already a
// subcomponent, it is not added to the list again.
void Component::adoptSubcomponent(Component* subcomponent)
{
    OPENSIM_THROW_IF(subcomponent->hasOwner(),
        ComponentAlreadyPartOfOwnershipTree,
        subcomponent->getName(), this->getName());

    //get the top-level component
    const Component* top = this;
    while (top->hasOwner())
        top = &top->getOwner();

    // cycle through all components from the top level component
    // down to verify the component is not already in the tree
    for (auto& comp : top->getComponentList<Component>()) {
        OPENSIM_THROW_IF(subcomponent->hasOwner(),
            ComponentAlreadyPartOfOwnershipTree,
            subcomponent->getName(), comp.getName());
    }

    subcomponent->setOwner(*this);
    _adoptedSubcomponents.push_back(SimTK::ClonePtr<Component>(subcomponent));
}

std::vector<SimTK::ReferencePtr<const Component>>
    Component::getImmediateSubcomponents() const
{
    std::vector<SimTK::ReferencePtr<const Component>> rv;
    rv.reserve(getNumImmediateSubcomponents());
    forEachImmediateSubcomponent([&rv](const Component& c)
    {
        rv.emplace_back(&c);
    });
    return rv;
}

void Component::forEachImmediateSubcomponent(const std::function<void(const Component&)> callback) const
{
    for (unsigned int i = 0; i < _memberSubcomponents.size(); ++i) {
        callback(*_memberSubcomponents[i]);
    }
    for (unsigned int i = 0; i < _propertySubcomponents.size(); ++i) {
        callback(*_propertySubcomponents[i]);
    }
    for (unsigned int i = 0; i < _adoptedSubcomponents.size(); ++i) {
        callback(*_adoptedSubcomponents[i]);
    }
}

void Component::forEachImmediateSubcomponent(const std::function<void(Component&)> callback)
{
    for (unsigned int i = 0; i < _memberSubcomponents.size(); ++i) {
        callback(*_memberSubcomponents[i]);
    }
    for (unsigned int i = 0; i < _propertySubcomponents.size(); ++i) {
        callback(*_propertySubcomponents[i]);
    }
    for (unsigned int i = 0; i < _adoptedSubcomponents.size(); ++i) {
        callback(*_adoptedSubcomponents[i]);
    }
}

const Component* Component::findImmediateSubcomponentByName(const std::string& name) const
{
    for (const auto& memberSubcomponent : _memberSubcomponents) {
        if (memberSubcomponent->getName() == name) {
            return memberSubcomponent.get();
        }
    }
    for (const auto& propertySubcomponent : _propertySubcomponents) {
        if (propertySubcomponent->getName() == name) {
            return propertySubcomponent.get();
        }
    }
    for (const auto& adoptedSubcomponent : _adoptedSubcomponents) {
        if (adoptedSubcomponent->getName() == name) {
            return adoptedSubcomponent.get();
        }
    }
    return nullptr;
}

size_t Component::getNumMemberSubcomponents() const
{
    return _memberSubcomponents.size();
}

size_t Component::getNumPropertySubcomponents() const
{
    return _propertySubcomponents.size();
}

size_t Component::getNumAdoptedSubcomponents() const
{
    return _adoptedSubcomponents.size();
}



int Component::getStateIndex(const std::string& name) const
{
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(name);

    if(it != _namedStateVariableInfo.end()) {
        return it->second.stateVariable->getVarIndex();
    } else {
        std::stringstream msg;
        msg << "Component::getStateVariableSystemIndex: ERR- name '"
            << name << "' not found.\n "
            << "for component '"<< getName() << "' of type "
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
        return SimTK::InvalidIndex;
    }
}

SimTK::SystemYIndex Component::
getStateVariableSystemIndex(const std::string& stateVariableName) const
{
    //const SimTK::State& s = getSystem().getDefaultState();

    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(stateVariableName);

    if(it != _namedStateVariableInfo.end()){
        return it->second.stateVariable->getSystemYIndex();
    }

    // Otherwise we have to search through subcomponents
    SimTK::SystemYIndex yix;

    for (const auto& propertySubcomponent : _propertySubcomponents) {
        yix = propertySubcomponent->getStateVariableSystemIndex(stateVariableName);
        if(yix.isValid()){
            return yix;
        }
    }

    if(!(yix.isValid())){
        throw Exception(getConcreteClassName()
            + "::getStateVariableSystemIndex : state variable "
            + stateVariableName+" has an invalid index.");
    }

    return yix;
}


void
Component::
getModelingOptionIndexes(const std::string& moName,
    SimTK::SubsystemIndex& ssIndex,
    SimTK::DiscreteVariableIndex& moIndex) const
{
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.find(moName);
    OPENSIM_THROW_IF(it == _namedModelingOptionInfo.end(),
        VariableNotFound, getName(), moName);
    ssIndex = it->second.ssIndex;
    moIndex = it->second.moIndex;
}

void
Component::
initializeModelingOptionIndexes(const std::string& moName,
    const SimTK::SubsystemIndex ssIndex,
    const SimTK::DiscreteVariableIndex& moIndex) const
{
    std::map<std::string, ModelingOptionInfo>::iterator it;
    it = _namedModelingOptionInfo.find(moName);
    OPENSIM_THROW_IF(it == _namedModelingOptionInfo.end(),
        VariableNotFound, getName(), moName);
    it->second.ssIndex = ssIndex;
    it->second.moIndex = moIndex;
}

void
Component::
getDiscreteVariableIndexes(const std::string& dvName,
    SimTK::SubsystemIndex& ssIndex,
    SimTK::DiscreteVariableIndex& dvIndex) const
{
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(dvName);
    OPENSIM_THROW_IF(it == _namedDiscreteVariableInfo.end(),
        VariableNotFound, getName(), dvName);
    ssIndex = it->second.ssIndex;
    dvIndex = it->second.dvIndex;
}

void
Component::
initializeDiscreteVariableIndexes(const std::string& dvName,
    const SimTK::SubsystemIndex ssIndex,
    const SimTK::DiscreteVariableIndex& dvIndex) const
{
    std::map<std::string, DiscreteVariableInfo>::iterator it;
    it = _namedDiscreteVariableInfo.find(dvName);
    OPENSIM_THROW_IF(it == _namedDiscreteVariableInfo.end(),
        VariableNotFound, getName(), dvName);
    it->second.ssIndex = ssIndex;
    it->second.dvIndex = dvIndex;
}

const Component* Component::traversePathToComponent(const ComponentPath& path) const
{
    const Component* current = path.isAbsolute() ? &getRoot() : this;
    const size_t numPathLevels = path.getNumPathLevels();

    for (size_t i = 0; current && i < numPathLevels; ++i) {
        const auto& pathElement = path.getSubcomponentNameAtLevel(i);

        current = pathElement == ".." ?
            current->_owner.get() :
            current->findImmediateSubcomponentByName(pathElement);
    }

    return current;
}

Array<std::string> Component::
getStateVariableNamesAddedByComponent() const
{
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.begin();

    Array<std::string> names("",(int)_namedStateVariableInfo.size());

    while(it != _namedStateVariableInfo.end()){
        names[it->second.order] = it->first;
        it++;
    }
    return names;
}

//------------------------------------------------------------------------------
//                            REALIZE TOPOLOGY
//------------------------------------------------------------------------------
// This is the base class implementation of a virtual method that can be
// overridden by derived model components, but they *must* invoke
// Super::extendRealizeTopology() as the first line in the overriding method so
// that this code is executed before theirs.
// This method is invoked from the ComponentMeasure associated with this
// Component.
// Note that subcomponent realize() methods will be invoked by their own
// ComponentMeasures, so we do not need to forward to subcomponents here.
void Component::extendRealizeTopology(SimTK::State& s) const
{
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();

    // Allocate Modeling Options
    for (auto& kv : _namedModelingOptionInfo) {
        ModelingOptionInfo& moi = kv.second;
        if (!moi.allocate) continue;
        moi.ssIndex = subSys.getMySubsystemIndex();
        moi.moIndex = subSys.allocateDiscreteVariable(
            s, SimTK::Stage::Instance, new SimTK::Value<int>(0));
    }

    // Allocate Continuous State Variables
    SimTK::Vector zInit(1, 0.0);
    for (auto& kv : _namedStateVariableInfo) {
        const StateVariable& sv = *kv.second.stateVariable;
        const AddedStateVariable* asv =
                dynamic_cast<const AddedStateVariable*>(&sv);

        if (asv) { // add index information for added state variables
            // make mutable just to update system allocated index ONLY!
            AddedStateVariable* masv = const_cast<AddedStateVariable*>(asv);
            masv->setVarIndex(subSys.allocateZ(s, zInit));
            masv->setSubsystemIndex(
                    getDefaultSubsystem().getMySubsystemIndex());
        }
    }

    // Allocate Discrete State Variables
    for (auto& kv : _namedDiscreteVariableInfo) {
        DiscreteVariableInfo& dvi = kv.second;

        // Do not allocate if the discrete state is allocated outside of class
        // Component. This case is encountered when a native Simbody object,
        // wrapped as an OpenSim Component, posseses discrete states of its
        // own. In such a case, the derived Component is responsible for
        // initializing the discrete state index, as well as its Subsystem.
        // See initializeDiscreteVariableIndices().
        if (!dvi.allocate) continue;
        dvi.ssIndex = subSys.getMySubsystemIndex();
        dvi.dvIndex = subSys.allocateDiscreteVariable(s,
            dvi.invalidatesStage, new SimTK::Value<double>(0.0));
    }

    // allocate cache entry in the state
    //
    // BEWARE: the cache variables *must* be inserted into the SimTK::state
    //         in a deterministic order.
    //
    //         The reason this is important is because:
    //
    //         - some downstream code will take copies of the `SimTK::State`
    //           and expect indicies into the new state to also be able to
    //           index into the copy (they shouldn't do this, but do, and
    //           this code hardens against it)
    //
    //         - to callers, it *feels* like the copy should be interchangable
    //           if the component is *logically* the same - the same component
    //           with the same cache vars etc. *should* produce the same state,
    //           right?
    //
    //         - but `unordered_map` has no iteration order guarantees, so even
    //           the exact same component, at the same memory address, that is
    //           merely re-initialized, and calls `addCacheVariable` in the
    //           exact same order can still iterate the cache variables in
    //           a different order and (ultimately) produce an incompatible
    //           SimTK::State
    //
    //         - this is because re-initialization does not necessarily
    //           reconstruct the backing containers. They may only have been
    //           `.clear()`ed, which *may* hold onto memory, which *may* affect
    //           (internal) insertion logic
    //
    //         - the safest thing to assume is that the iteration order of
    //           `unordered_map` is non-deterministic. It isn't, but *essentially*
    //           is, because there are plenty of non-obvious ways to affect its
    //           iteration order
    {
        std::vector<std::reference_wrapper<const std::string>> keys;
        keys.reserve(this->_namedCacheVariables.size());
        for (auto& p : this->_namedCacheVariables) {
            keys.emplace_back(p.first);
        }

        std::sort(keys.begin(), keys.end(), [](const std::string& a, const std::string& b) {
            return a < b;
        });

        for (const std::string& k : keys) {
            StoredCacheVariable& cv = this->_namedCacheVariables.at(k);
            cv.maybeUninitIndex = subSys.allocateLazyCacheEntry(s, cv.dependsOnStage, cv.value->clone());
        }
    }
}


//------------------------------------------------------------------------------
//                         REALIZE ACCELERATION
//------------------------------------------------------------------------------
// Base class implementation of virtual method.
// Collect this component's state variable derivatives.
void Component::extendRealizeAcceleration(const SimTK::State& s) const
{
    // don't bother computing derivatives if the component has no state variables
    if(getNumStateVariablesAddedByComponent() > 0) {
        const SimTK::Subsystem& subSys = getDefaultSubsystem();

        // evaluate and set component state derivative values (in cache)
        computeStateVariableDerivatives(s);

        std::map<std::string, StateVariableInfo>::const_iterator it;

        for (it = _namedStateVariableInfo.begin();
             it != _namedStateVariableInfo.end(); ++it)
        {
            const StateVariable& sv = *it->second.stateVariable;
            const AddedStateVariable* asv =
                dynamic_cast<const AddedStateVariable*>(&sv);
            if(asv)
                // set corresponding system derivative value from
                // cached value
                subSys.updZDot(s)[SimTK::ZIndex(asv->getVarIndex())] =
                        asv->getDerivative(s);
        }
    }
}

SimTK::MultibodySystem& Component::updSystem() const
{
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);
    return _system.getRef();
}

//------------------------------------------------------------------------------
//                         OTHER REALIZE METHODS
//------------------------------------------------------------------------------
// Base class implementations of these virtual methods do nothing now but
// could do something in the future. Users must still invoke Super::realizeXXX()
// as the first line in their overrides to ensure future compatibility.
void Component::extendRealizeModel(SimTK::State& state) const {}
void Component::extendRealizeInstance(const SimTK::State& state) const {}
void Component::extendRealizeTime(const SimTK::State& state) const {}
void Component::extendRealizePosition(const SimTK::State& state) const {}
void Component::extendRealizeVelocity(const SimTK::State& state) const {}
void Component::extendRealizeDynamics(const SimTK::State& state) const {}
void Component::extendRealizeReport(const SimTK::State& state) const {}


//override virtual methods
double Component::AddedStateVariable::getValue(const SimTK::State& state) const
{
    SimTK::ZIndex zix(getVarIndex());
    if(getSubsysIndex().isValid() && zix.isValid()){
        const SimTK::Vector& z = getOwner().getDefaultSubsystem().getZ(state);
        return z[SimTK::ZIndex(zix)];
    }

    std::stringstream msg;
    msg << "Component::AddedStateVariable::getValue: ERR- variable '"
        << getName() << "' is invalid for component " << getOwner().getName()
        << " of type " << getOwner().getConcreteClassName() <<".";
    throw Exception(msg.str(),__FILE__,__LINE__);
    return SimTK::NaN;
}

void Component::AddedStateVariable::setValue(SimTK::State& state, double value) const
{
    SimTK::ZIndex zix(getVarIndex());
    if(getSubsysIndex().isValid() && zix.isValid()){
        SimTK::Vector& z = getOwner().getDefaultSubsystem().updZ(state);
        z[SimTK::ZIndex(zix)] = value;
        return;
    }

    std::stringstream msg;
    msg << "Component::AddedStateVariable::setValue: ERR- variable '"
        << getName() << "' is invalid for component " << getOwner().getName()
        << " of type " << getOwner().getConcreteClassName() <<".";
    throw Exception(msg.str(),__FILE__,__LINE__);
}

static std::string const& derivativeName(const std::string& baseName) {
    // this function is called *a lot* (e.g. millions of times in a sim), so we
    // use TLS to cache the (potentially, heap-allocated) derivative name
    //
    // although it is cleared each time this is called, clearing a string does
    // not deallocate its heap

    thread_local std::string derivativeName;
    derivativeName.clear();
    derivativeName += baseName;
    derivativeName += "_deriv";

    return derivativeName;
}

double Component::AddedStateVariable::
    getDerivative(const SimTK::State& state) const
{
    return getOwner().getCacheVariableValue<double>(state, derivativeName(getName()));
}

void Component::AddedStateVariable::
    setDerivative(const SimTK::State& state, double deriv) const
{
    getOwner().setCacheVariableValue<double>(state, derivativeName(getName()), deriv);
}


void Component::printSocketInfo() const {
    std::string str = fmt::format("Sockets for component {} of type [{}] along "
                                  "with connectee paths:", getName(),
                                  getConcreteClassName());
    if (getNumSockets() == 0)
        str += " none";
    log_cout(str);


    size_t maxlenTypeName{}, maxlenSockName{};
    for(const auto& sock : _socketsTable) {
        maxlenTypeName = std::max(maxlenTypeName,
                                  sock.second->getConnecteeTypeName().length());
        maxlenSockName = std::max(maxlenSockName,
                                  sock.second->getName().length());
    }
    maxlenTypeName += 6;
    maxlenSockName += 1;

    for (const auto& it : _socketsTable) {
        const auto& socket = it.second;
        // Right-justify the connectee type names and socket names.
        str = fmt::format("{:>{}} {:>{}} : ",
                          fmt::format("[{}]", socket->getConnecteeTypeName()),
                          maxlenTypeName,
                          socket->getName(), maxlenSockName);
        if (socket->getNumConnectees() == 0) {
            str += "no connectees";
        } else {
            std::vector<std::string> connecteePaths;
            for (unsigned i = 0; i < socket->getNumConnectees(); ++i) {
                connecteePaths.push_back(socket->getConnecteePath(i));
            }
            // Join the connectee paths with spaces in between.
            str += fmt::format("{}", fmt::join(connecteePaths, " "));
        }
        log_cout(str);
    }
}

void Component::printInputInfo() const {
    std::string str = fmt::format("Inputs for component {} of type [{}] along "
                                  "with connectee paths:",
                                  getName(), getConcreteClassName());
    if (getNumInputs() == 0)
        str += " none";
    log_cout(str);

    size_t maxlenTypeName{}, maxlenInputName{};
    for(const auto& input : _inputsTable) {
        maxlenTypeName = std::max(maxlenTypeName,
                                input.second->getConnecteeTypeName().length());
        maxlenInputName = std::max(maxlenInputName,
                                input.second->getName().length());
    }
    maxlenTypeName += 6;
    maxlenInputName += 1;

    for (const auto& it : _inputsTable) {
        const auto& input = it.second;
        // Right-justify the connectee type names and input names.
        str = fmt::format("{:>{}} {:>{}} : ",
                          fmt::format("[{}]", input->getConnecteeTypeName()),
                          maxlenTypeName,
                          input->getName(), maxlenInputName);
        if (input->getNumConnectees() == 0 ||
            (input->getNumConnectees() == 1 && input->getConnecteePath().empty())) {
            str += "no connectees";
        } else {
            std::vector<std::string> connecteePaths;
            for (unsigned i = 0; i < input->getNumConnectees(); ++i) {
                connecteePaths.push_back(input->getConnecteePath(i));
                // TODO as is, requires the input connections to be satisfied.
                // std::cout << " (alias: " << input.getAlias(i) << ") ";
            }
            // Join the connectee paths with spaces in between.
            str += fmt::format("{}", fmt::join(connecteePaths, " "));
        }
        log_cout(str);
    }
}

void Component::printSubcomponentInfo() const {
    printSubcomponentInfo<Component>();
}

void Component::printOutputInfo(const bool includeDescendants) const {

    // Do not display header for Components with no outputs.
    if (getNumOutputs() > 0) {
        std::string msg = fmt::format("Outputs from {} [{}]",
                                      getAbsolutePathString(),
                                      getConcreteClassName());
        msg += "\n" + std::string(msg.size(), '=');
        log_cout(msg);

        const auto& outputs = getOutputs();
        size_t maxlen{};
        for(const auto& output : outputs)
            maxlen = std::max(maxlen, output.second->getTypeName().length());
        maxlen += 6;

        for(const auto& output : outputs) {
            const auto& name = output.second->getTypeName();
            log_cout("{:>{}}  {}",
                     fmt::format("[{}]", output.second->getTypeName()),
                     maxlen, output.first);
        }
        log_cout("");
    }

    if (includeDescendants) {
        for (const Component& thisComp : getComponentList<Component>()) {
            // getComponentList() returns all descendants (i.e.,
            // children, grandchildren, etc.) so set includeDescendants=false
            // when calling on thisComp.
            thisComp.printOutputInfo(false);
        }
    }
}

void Component::initComponentTreeTraversal(const Component &root) const {
    // Going down the tree, this node is followed by all its children.
    // The last child's successor (next) is the parent's successor.

    const size_t nmsc = _memberSubcomponents.size();
    const size_t npsc = _propertySubcomponents.size();
    const size_t nasc = _adoptedSubcomponents.size();

    if (!hasOwner()) {
        // If this isn't the root component and it has no owner, then
        // this is an orphan component and we likely failed to call
        // finalizeFromProperties() on the root OR this is a clone that
        // has not been added to the root (in which case would have an owner).
        if (this != &root) {
            OPENSIM_THROW(ComponentIsAnOrphan, getName(),
                getConcreteClassName());
        }
        // if the root (have no owner) and have no components
        else if (!(nmsc + npsc + nasc)) {
            OPENSIM_THROW(ComponentIsRootWithNoSubcomponents,
                getName(), getConcreteClassName());
        }
    }

    const Component* last = nullptr;
    for (unsigned int i = 0; i < nmsc; i++) {
        if (i == nmsc - 1) {
            _memberSubcomponents[i]->_nextComponent = _nextComponent.get();
            last = _memberSubcomponents[i].get();
        }
        else {
            _memberSubcomponents[i]->_nextComponent =
                _memberSubcomponents[i + 1].get();
            last = _memberSubcomponents[i + 1].get();
        }
    }
    if (npsc) {
        if (last)
            last->_nextComponent = _propertySubcomponents[0].get();

        for (unsigned int i = 0; i < npsc; i++) {
            if (i == npsc - 1) {
                _propertySubcomponents[i]->_nextComponent =
                    _nextComponent.get();
                last = _propertySubcomponents[i].get();
            }
            else {
                _propertySubcomponents[i]->_nextComponent =
                    _propertySubcomponents[i + 1].get();
                last = _propertySubcomponents[i + 1].get();
            }
        }
    }
    if (nasc) {
        if (last)
            last->_nextComponent = _adoptedSubcomponents[0].get();

        for (unsigned int i = 0; i <nasc; i++) {
            if (i == nasc - 1) {
                _adoptedSubcomponents[i]->_nextComponent = _nextComponent.get();
            }
            else {
                _adoptedSubcomponents[i]->_nextComponent
                    = _adoptedSubcomponents[i + 1].get();
            }
        }
    }

    // recurse to handle children of subcomponents
    forEachImmediateSubcomponent([&root](const Component& c)
    {
        c.initComponentTreeTraversal(root);
    });
}


void Component::clearStateAllocations()
{
    _namedModelingOptionInfo.clear();
    _namedStateVariableInfo.clear();
    _namedDiscreteVariableInfo.clear();
    _namedCacheVariables.clear();
}

void Component::reset()
{
    _system.reset();
    _simTKcomponentIndex.invalidate();
    clearStateAllocations();

    _propertySubcomponents.clear();
    _adoptedSubcomponents.clear();
    resetSubcomponentOrder();
}

void Component::warnBeforePrint() const {
    if (!isObjectUpToDateWithProperties()) return;
    std::string message;
    auto checkIfConnecteePathIsSet =
            [](const Component& comp, std::string& message) {
        for (const auto& it : comp._socketsTable) {
            const auto& socket = it.second;
            if (socket->isConnected() &&
                ((socket->isListSocket() &&
                  socket->getNumConnectees() == 0) ||
                 (!socket->isListSocket() &&
                         socket->getConnecteePath().empty()))) {
                // TODO: Improve this condition by making sure the connectee
                // name is correct.
                message += "  Socket '" + socket->getName() + "' in " +
                           comp.getConcreteClassName() + " at " +
                           comp.getAbsolutePathString() + "\n";
            }
        }
    };
    if (getNumImmediateSubcomponents() == 0) {
        checkIfConnecteePathIsSet(*this, message);
    } else {
        for (const auto& comp : getComponentList()) {
            checkIfConnecteePathIsSet(comp, message);
        }
    }
    if (!message.empty()) {
        std::stringstream buffer;
        buffer << "Warning in " << getConcreteClassName()
                << "::print(): The following connections are not finalized "
                   "and will not appear in the resulting XML file. "
                   "Call finalizeConnections() before print().\n"
                   "To ignore, set the debug level to at least 1 "
                   "(e.g, by calling Object::setDebugLevel(1)) first.\n"
                << message << std::endl;
        OPENSIM_THROW_FRMOBJ(Exception, buffer.str());
    }
}

} // end of namespace OpenSim
