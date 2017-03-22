/* -------------------------------------------------------------------------- *
 *                            OpenSim: Component.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Michael Sherman                                      *
 * Contributor(s): Ayman Habib                                                *
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
#include <unordered_map>

using namespace SimTK;

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

void Component::addComponent(Component* subcomponent)
{
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
            if (subcomponent == &c) {
                OPENSIM_THROW(ComponentAlreadyPartOfOwnershipTree,
                    subcomponent->getName(), getName());
            }
        }
    }

    updProperty_components().adoptAndAppendValue(subcomponent);
    finalizeFromProperties();

    // allow the derived Component to perform secondary operations
    // in response to the inclusion of the subcomponent
    extendAddComponent(subcomponent);
}

void Component::finalizeFromProperties()
{
    reset();

    // TODO use a flag to set whether we are lenient on having nameless
    // Components. For backward compatibility we need to be able to 
    // handle nameless components so assign them their class name
    // - aseth
    if (getName().empty()) {
        setName(IO::Lowercase(getConcreteClassName()) + "_");
    }

    OPENSIM_THROW_IF( getName().empty(), ComponentHasNoName,
                      getConcreteClassName() );

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
        it.second->checkConnecteeNameProperty();
    }
    for (auto& it : _inputsTable) {
        it.second->setOwner(*this);
        // Let the Socket handle any errors in the connectee_name property.
        it.second->checkConnecteeNameProperty();
    }
    for (auto& it : _outputsTable) {
        it.second->setOwner(*this);
    }

    markPropertiesAsSubcomponents();
    componentsFinalizeFromProperties();
    extendFinalizeFromProperties();
    setObjectIsUpToDateWithProperties();
}

// Base class implementation of virtual method.
// Call finalizeFromProperties on all subcomponents
void Component::componentsFinalizeFromProperties() const
{
    for (auto& comp : _memberSubcomponents) {
        const_cast<Component*>(comp.get())
            ->finalizeFromProperties();
    }
    for (auto& comp : _propertySubcomponents) {
        const_cast<Component*>(comp.get())
            ->finalizeFromProperties();
    }
    for (auto& comp : _adoptedSubcomponents) {
        const_cast<Component*>(comp.get())
            ->finalizeFromProperties();
    }
}

// Base class implementation of non-virtual finalizeConnections method.
void Component::finalizeConnections(Component &root)
{
    if (!isObjectUpToDateWithProperties()){
        // if edits occur between construction and connect() this is
        // the last chance to finalize before addToSystem.
        finalizeFromProperties();
    }

    for (auto& it : _socketsTable) {
        auto& socket = it.second;
        socket->disconnect();
        try {
            socket->findAndConnect(root);
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

        if (!input->isListSocket() && input->getConnecteeName(0).empty()) {
            // TODO When we support verbose/debug logging we should include
            // message about unspecified Outputs but generally this OK
            // if the Input's value is not required.
            /**
            std::cout << getConcreteClassName() << "'" << getName() << "'";
            std::cout << "::connect() Input<" << input.getConnecteeTypeName();
            std::cout << ">`" << input.getName();
            std::cout << "' Output has not been specified." << std::endl;
            */
            continue;
        }

        input->disconnect();
        try {
            input->findAndConnect(root);
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
    extendConnect(root);

    // Allow subcomponents to form their connections
    componentsConnect(root);

    // Forming connections changes the Socket which is a property
    // Remark as upToDate.
    setObjectIsUpToDateWithProperties();
}

// invoke connect on all (sub)components of this component
void Component::componentsConnect(Component& root) 
{
    // enable the subcomponents the opportunity to connect themselves
    for (unsigned int i = 0; i<_memberSubcomponents.size(); ++i) {
        _memberSubcomponents[i].upd()->finalizeConnections(root);
    }
    for(unsigned int i=0; i<_propertySubcomponents.size(); ++i){
        _propertySubcomponents[i].get()->finalizeConnections(root);
    }
    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); ++i) {
        _adoptedSubcomponents[i].upd()->finalizeConnections(root);
    }
}

void Component::clearConnections()
{
    // First give the subcomponents the opportunity to disconnect themselves
    for (unsigned int i = 0; i<_memberSubcomponents.size(); i++) {
        _memberSubcomponents[i]->clearConnections();
    }
    for (unsigned int i = 0; i<_propertySubcomponents.size(); i++){
        _propertySubcomponents[i]->clearConnections();
    }
    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); i++) {
        _adoptedSubcomponents[i]->clearConnections();
    }

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
        auto mySubcomponents = getImmediateSubcomponents();
        for (const auto& compRef : mySubcomponents) {
            compRef->addToSystem(system);
        }
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
    for (unsigned int i = 0; i<_memberSubcomponents.size(); ++i)
        _memberSubcomponents[i]->initStateFromProperties(state);
    for (unsigned int i = 0; i<_propertySubcomponents.size(); ++i)
        _propertySubcomponents[i]->initStateFromProperties(state);
    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); ++i)
        _adoptedSubcomponents[i]->initStateFromProperties(state);
}

void Component::setPropertiesFromState(const SimTK::State& state)
{
    extendSetPropertiesFromState(state);
    componentsSetPropertiesFromState(state);
}

void Component::componentsSetPropertiesFromState(const SimTK::State& state)
{
    for (unsigned int i = 0; i<_memberSubcomponents.size(); ++i)
        _memberSubcomponents[i]->setPropertiesFromState(state);
    for (unsigned int i = 0; i<_propertySubcomponents.size(); ++i)
        _propertySubcomponents[i]->setPropertiesFromState(state);
    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); ++i)
        _adoptedSubcomponents[i]->setPropertiesFromState(state);
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


void Component::
addModelingOption(const std::string& optionName, int maxFlagValue) const 
{
    // don't add modeling option if there is another state with the same  
    // name for this component
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.find(optionName);
    if(it != _namedModelingOptionInfo.end())
        throw Exception("Component::addModelingOption: Modeling option '"
              + optionName + "' already exists.");
    // assign a "slot" for a modeling option by name
    // modeling option index will be invalid by default
    // upon allocation during realizeTopology the index will be set
    _namedModelingOptionInfo[optionName] = ModelingOptionInfo(maxFlagValue);
}

void Component::addStateVariable(const std::string&  stateVariableName,
                                 const SimTK::Stage& invalidatesStage,
                                 bool isHidden) const
{
    if( (invalidatesStage < Stage::Position) ||
        (invalidatesStage > Stage::Dynamics)) {
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
        addCacheVariable(stateVariableName+"_deriv", 0.0, Stage::Dynamics);
    }

}


void Component::addDiscreteVariable(const std::string&  discreteVariableName, 
                                    SimTK::Stage        invalidatesStage) const
{
    // don't add discrete var if there is another discrete variable with the 
    // same name for this component
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(discreteVariableName);
    if(it != _namedDiscreteVariableInfo.end()){
        throw Exception("Component::addDiscreteVariable: discrete variable '" + 
            discreteVariableName + "' already exists.");
    }
    // assign "slots" for the discrete variables by name
    // discrete variable indices will be invalid by default
    // upon allocation during realizeTopology the indices will be set
    _namedDiscreteVariableInfo[discreteVariableName] = 
        DiscreteVariableInfo(invalidatesStage);
}

// Get the value of a ModelingOption flag for this Component.
int Component::
getModelingOption(const SimTK::State& s, const std::string& name) const
{
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.find(name);

    if(it != _namedModelingOptionInfo.end()) {
        SimTK::DiscreteVariableIndex dvIndex = it->second.index;
        return SimTK::Value<int>::downcast(
            getDefaultSubsystem().getDiscreteVariable(s, dvIndex)).get();
    } else {
        std::stringstream msg;
        msg << "Component::getModelingOption: ERR- name '" << name 
            << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
        return -1;
    }
}

// Set the value of a discrete variable allocated by this Component by name.
void Component::
setModelingOption(SimTK::State& s, const std::string& name, int flag) const
{
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.find(name);

    if(it != _namedModelingOptionInfo.end()) {
        SimTK::DiscreteVariableIndex dvIndex = it->second.index;
        if(flag > it->second.maxOptionValue){
            std::stringstream msg;
            msg << "Component::setModelingOption: "<< name 
                << " flag cannot exceed "<< it->second.maxOptionValue <<".\n ";
        throw Exception(msg.str(),__FILE__,__LINE__);
        }

        SimTK::Value<int>::downcast(
            getDefaultSubsystem().updDiscreteVariable(s, dvIndex)).upd() = flag;
    } else {
        std::stringstream msg;
        msg << "Component::setModelingOption: modeling option " << name 
            << " not found.\n ";
        throw Exception(msg.str(),__FILE__,__LINE__);
    }
}

unsigned Component::printComponentsMatching(const std::string& substring) const
{
    auto components = getComponentList();
    components.setFilter(ComponentFilterAbsolutePathNameContainsString(substring));
    unsigned count = 0;
    for (const auto& comp : components) {
        std::cout << comp.getAbsolutePathName() << std::endl;
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
    for (unsigned int i = 0; i<_memberSubcomponents.size(); i++)
        ns += _memberSubcomponents[i]->getNumStateVariables();

    for(unsigned int i=0; i<_propertySubcomponents.size(); i++)
        ns += _propertySubcomponents[i]->getNumStateVariables();

    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); i++)
        ns += _adoptedSubcomponents[i]->getNumStateVariables();

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

std::string Component::getAbsolutePathName() const
{
    std::vector<std::string> pathVec;
    pathVec.push_back(getName());

    const Component* up = this;

    while (up && up->hasOwner()) {
        up = &up->getOwner();
        pathVec.insert(pathVec.begin(), up->getName());
    }
    // The root must have a leading '/' 
    ComponentPath path(pathVec, true);

    return path.toString();
}

std::string Component::getRelativePathName(const Component& wrt) const
{
    ComponentPath thisP(getAbsolutePathName());
    ComponentPath wrtP(wrt.getAbsolutePathName());

    return thisP.formRelativePath(wrtP).toString();
}

const Component::StateVariable* Component::
    findStateVariable(const std::string& name) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    // Split the prefix from the varName (part of string past the last "/")
    // In the case where no "/" is found, prefix = name.
    std::string::size_type back = name.rfind("/");
    std::string prefix = name.substr(0, back);

    // In the case where no "/" is found, this assigns varName = name.
    // When "/" is not found, back = UINT_MAX. Then, back + 1 = 0.
    // Subtracting by UINT_MAX is effectively adding by 1, so the next line
    // should work in all cases except if name.length() = UINT_MAX.
    std::string varName = name.substr(back + 1, name.length() - back);

    // first assume that the state variable named belongs to this
    // top level component
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(varName);

    if (it != _namedStateVariableInfo.end()) {
        return it->second.stateVariable.get();
    }

    const StateVariable* found = nullptr;
    const Component* comp = traversePathToComponent<Component>(prefix);

    if (comp) {
        found = comp->findStateVariable(varName);
    }

    // Path not given or could not find it along given path name
    // Now try complete search.
    if (!found) {
        for (unsigned int i = 0; i < _propertySubcomponents.size(); ++i) {
            comp = _propertySubcomponents[i]->findComponent(prefix, &found);
            if (found) {
                return found;
            }
            if (comp) {
                return comp->findStateVariable(varName);
            }
        }
    }

    return found;
}

// Get the names of "continuous" state variables maintained by the Component and
// its subcomponents.
Array<std::string> Component::getStateVariableNames() const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    Array<std::string> names = getStateVariablesNamesAddedByComponent();

/** TODO: Use component iterator  like below
    for (int i = 0; i < stateNames.size(); ++i) {
        stateNames[i] = (getAbsolutePathName() + "/" + stateNames[i]);
    }

    for (auto& comp : getComponentList<Component>()) {
        const std::string& pathName = comp.getAbsolutePathName();// *this);
        Array<std::string> subStateNames = 
            comp.getStateVariablesNamesAddedByComponent();
        for (int i = 0; i < subStateNames.size(); ++i) {
            stateNames.append(pathName + "/" + subStateNames[i]);
        }
    }
*/

    // Include the states of its subcomponents
    for (unsigned int i = 0; i<_memberSubcomponents.size(); i++) {
        Array<std::string> subnames = _memberSubcomponents[i]->getStateVariableNames();
        int nsubs = subnames.getSize();
        const std::string& subCompName = _memberSubcomponents[i]->getName();
        std::string::size_type front = subCompName.find_first_not_of(" \t\r\n");
        std::string::size_type back = subCompName.find_last_not_of(" \t\r\n");
        std::string prefix = "";
        if (back >= front) // have non-whitespace name
            prefix = subCompName + "/";
        for (int j = 0; j<nsubs; ++j) {
            names.append(prefix + subnames[j]);
        }
    }
    for(unsigned int i=0; i<_propertySubcomponents.size(); i++){
        Array<std::string> subnames = _propertySubcomponents[i]->getStateVariableNames();
        int nsubs = subnames.getSize();
        const std::string& subCompName =  _propertySubcomponents[i]->getName();
        // TODO: We should implement checks that names do not have whitespace at the time 
        // they are assigned and not here where it is a waste of time - aseth
        std::string::size_type front = subCompName.find_first_not_of(" \t\r\n");
        std::string::size_type back = subCompName.find_last_not_of(" \t\r\n");
        std::string prefix = "";
        if(back >= front) // have non-whitespace name
            prefix = subCompName+"/";
        for(int j =0; j<nsubs; ++j){
            names.append(prefix+subnames[j]);
        }
    }

    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); i++) {
        Array<std::string> subnames = _adoptedSubcomponents[i]->getStateVariableNames();
        int nsubs = subnames.getSize();
        const std::string& subCompName = _adoptedSubcomponents[i]->getName();
        std::string::size_type front = subCompName.find_first_not_of(" \t\r\n");
        std::string::size_type back = subCompName.find_last_not_of(" \t\r\n");
        std::string prefix = "";
        if (back >= front) // have non-whitespace name
            prefix = subCompName + "/";
        for (int j = 0; j<nsubs; ++j) {
            names.append(prefix + subnames[j]);
        }
    }

    return names;
}

// Get the value of a state variable allocated by this Component.
double Component::
    getStateVariableValue(const SimTK::State& s, const std::string& name) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    // find the state variable with this component or its subcomponents
    const StateVariable* rsv = findStateVariable(name);
    if (rsv) {
        return rsv->getValue(s);
    }
    
    std::stringstream msg;
    msg << "Component::getStateVariable: ERR- state named '" << name 
        << "' not found in " << getName() << " of type " << getConcreteClassName();
    throw Exception(msg.str(),__FILE__,__LINE__);

    return SimTK::NaN;
}

// Get the value of a state variable derivative computed by this Component.
double Component::
    getStateVariableDerivativeValue(const SimTK::State& state, 
                                const std::string& name) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    computeStateVariableDerivatives(state);
    
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(name);

    if(it != _namedStateVariableInfo.end()) {
        return it->second.stateVariable->getDerivative(state);
    } 
    else{
        // otherwise find the component that variable belongs to
        const StateVariable* rsv = findStateVariable(name);
        if (rsv) {
            return rsv->getDerivative(state);
        }
    }

    std::stringstream msg;
    msg << "Component::getStateVariableDerivative: ERR- variable name '" << name 
        << "' not found.\n " 
        << getName() << " of type " << getConcreteClassName() 
        << " has " << getNumStateVariables() << " states.";
    throw Exception(msg.str(),__FILE__,__LINE__);
    return SimTK::NaN;
}

// Set the value of a state variable allocated by this Component given its name
// for this component.
void Component::
    setStateVariableValue(State& s, const std::string& name, double value) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    // find the state variable
    const StateVariable* rsv = findStateVariable(name);

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
        _allStateVariables.size() == nsv &&                             // 3.
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
            _allStateVariables[i].reset(findStateVariable(names[i]));
    }

    Vector stateVariableValues(nsv, SimTK::NaN);
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

    SimTK_ASSERT(values.size() == nsv,
        "Component::setStateVariableValues() number values does not match the "
        "number of state variables.");

    // if the StateVariables are invalid (see above) rebuild the list 
    if (!isAllStatesVariablesListValid()) {
        _statesAssociatedSystem.reset(&getSystem());
        _allStateVariables.clear();
        _allStateVariables.resize(nsv);
        Array<std::string> names = getStateVariableNames();
        for (int i = 0; i < nsv; ++i)
            _allStateVariables[i].reset(findStateVariable(names[i]));
    }

    for(int i=0; i<nsv; ++i){
        _allStateVariables[i]->setValue(state, values[i]);
    }
}

// Set the derivative of a state variable computed by this Component by name.
void Component::
    setStateVariableDerivativeValue(const State& state, 
                               const std::string& name, double value) const
{
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

// Get the value of a discrete variable allocated by this Component by name.
double Component::
getDiscreteVariableValue(const SimTK::State& s, const std::string& name) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(name);

    if(it != _namedDiscreteVariableInfo.end()) {
        SimTK::DiscreteVariableIndex dvIndex = it->second.index;
        return SimTK::Value<double>::downcast(
            getDefaultSubsystem().getDiscreteVariable(s, dvIndex)).get();
    } else {
        std::stringstream msg;
        msg << "Component::getDiscreteVariable: ERR- name '" << name 
            << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
        return SimTK::NaN;
    }
}

// Set the value of a discrete variable allocated by this Component by name.
void Component::
setDiscreteVariableValue(SimTK::State& s, const std::string& name, double value) const
{
    // Must have already called initSystem.
    OPENSIM_THROW_IF_FRMOBJ(!hasSystem(), ComponentHasNoSystem);

    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(name);

    if(it != _namedDiscreteVariableInfo.end()) {
        SimTK::DiscreteVariableIndex dvIndex = it->second.index;
        SimTK::Value<double>::downcast(
            getDefaultSubsystem().updDiscreteVariable(s, dvIndex)).upd() = value;
    } else {
        std::stringstream msg;
        msg << "Component::setDiscreteVariable: ERR- name '" << name 
            << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
    }
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
    if (versionNumber < XMLDocument::getLatestVersion()) {
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
        auto compPath = component->getAbsolutePathName();
        auto foundPath = it->get()->getAbsolutePathName();
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
    std::vector<SimTK::ReferencePtr<const Component>> mySubcomponents;
    for (auto& compRef : _memberSubcomponents) {
        mySubcomponents.push_back(
            SimTK::ReferencePtr<const Component>(compRef.get()) );
    }
    for (auto& compRef : _propertySubcomponents) {
        mySubcomponents.push_back(
            SimTK::ReferencePtr<const Component>(compRef.get()) );
    }
    for (auto& compRef : _adoptedSubcomponents) {
        mySubcomponents.push_back(
            SimTK::ReferencePtr<const Component>(compRef.get()) );
    }
    return mySubcomponents;
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



const int Component::getStateIndex(const std::string& name) const
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

    for(unsigned int i = 0; i < _propertySubcomponents.size(); ++i) {
        yix = _propertySubcomponents[i]->getStateVariableSystemIndex(stateVariableName);
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

const SimTK::DiscreteVariableIndex Component::
getDiscreteVariableIndex(const std::string& name) const
{
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(name);

    return it->second.index;
}

const SimTK::CacheEntryIndex Component::
getCacheVariableIndex(const std::string& name) const
{
    std::map<std::string, CacheInfo>::const_iterator it;
    it = _namedCacheVariableInfo.find(name);

    return it->second.index;
}

Array<std::string> Component::
getStateVariablesNamesAddedByComponent() const
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
    
    Component *mutableThis = const_cast<Component*>(this);

    // Allocate Modeling Option
    if(_namedModelingOptionInfo.size()>0){
        std::map<std::string, ModelingOptionInfo>::iterator it;
        for (it = (mutableThis->_namedModelingOptionInfo).begin(); 
             it !=_namedModelingOptionInfo.end(); ++it)
        {
            ModelingOptionInfo& moi = it->second;
            moi.index = subSys.allocateDiscreteVariable
               (s, SimTK::Stage::Instance, new SimTK::Value<int>(0));
        }
    }

    // Allocate Continuous State Variables
    if(_namedStateVariableInfo.size()>0){
        SimTK::Vector zInit(1, 0.0);
        std::map<std::string, StateVariableInfo>::iterator it;
        for (it = (mutableThis->_namedStateVariableInfo).begin(); 
             it != _namedStateVariableInfo.end(); ++it)
        {
            const StateVariable& sv = *it->second.stateVariable;
            const AddedStateVariable* asv 
                = dynamic_cast<const AddedStateVariable *>(&sv);

            if(asv){// add index information for added state variables
                // make mutable just to update system allocated index ONLY!
                AddedStateVariable* masv = const_cast<AddedStateVariable*>(asv);
                masv->setVarIndex(subSys.allocateZ(s, zInit));
                masv->setSubsystemIndex(getDefaultSubsystem().getMySubsystemIndex());
            }
        }
    }

    // Allocate Discrete State Variables
    if(_namedDiscreteVariableInfo.size()>0){
        std::map<std::string, DiscreteVariableInfo>::iterator it;
        for (it = (mutableThis->_namedDiscreteVariableInfo).begin(); 
             it != _namedDiscreteVariableInfo.end(); ++it)
        {
            DiscreteVariableInfo& dvi = it->second;
            dvi.index = subSys.allocateDiscreteVariable
               (s, dvi.invalidatesStage, new SimTK::Value<double>(0.0));
        }
    }

    // Allocate Cache Entry in the State
    if(_namedCacheVariableInfo.size()>0){
        std::map<std::string, CacheInfo>::iterator it;
        for (it = (mutableThis->_namedCacheVariableInfo).begin(); 
             it != _namedCacheVariableInfo.end(); ++it){
            CacheInfo& ci = it->second;
            ci.index = subSys.allocateLazyCacheEntry
               (s, ci.dependsOnStage, ci.prototype->clone());
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
                subSys.updZDot(s)[ZIndex(asv->getVarIndex())] =
                    asv->getDerivative(s);
        }
    }
}

const SimTK::MultibodySystem& Component::getSystem() const
{
    if (!hasSystem()){
        std::string msg = getConcreteClassName()+"::getSystem() ";
        msg += getName() + " has no reference to a System.\n";
        msg += "Make sure you added the Component to the Model and ";
        msg += "called Model::initSystem(). ";
        throw Exception(msg, __FILE__, __LINE__);
    }
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
    ZIndex zix(getVarIndex());
    if(getSubsysIndex().isValid() && zix.isValid()){
        const SimTK::Vector& z = getOwner().getDefaultSubsystem().getZ(state);
        return z[ZIndex(zix)];
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
    ZIndex zix(getVarIndex());
    if(getSubsysIndex().isValid() && zix.isValid()){
        SimTK::Vector& z = getOwner().getDefaultSubsystem().updZ(state);
        z[ZIndex(zix)] = value;
        return;
    }

    std::stringstream msg;
    msg << "Component::AddedStateVariable::setValue: ERR- variable '" 
        << getName() << "' is invalid for component " << getOwner().getName() 
        << " of type " << getOwner().getConcreteClassName() <<".";
    throw Exception(msg.str(),__FILE__,__LINE__);
}

double Component::AddedStateVariable::
    getDerivative(const SimTK::State& state) const
{
    return getOwner().getCacheVariableValue<double>(state, getName()+"_deriv");
}

void Component::AddedStateVariable::
    setDerivative(const SimTK::State& state, double deriv) const
{
    return getOwner().setCacheVariableValue<double>(state, getName()+"_deriv", deriv);
}


void Component::printSocketInfo() const {
    std::cout << "Sockets for component " << getName() << " of type ["
              << getConcreteClassName() << "] along with connectee names:";
    if (getNumSockets() == 0)
        std::cout << " none";
    std::cout << std::endl;

    size_t maxlenTypeName{}, maxlenSockName{};
    for(const auto& sock : _socketsTable) {
        maxlenTypeName = std::max(maxlenTypeName,
                                  sock.second->getConnecteeTypeName().length());
        maxlenSockName = std::max(maxlenSockName,
                                  sock.second->getName().length());
    }
    maxlenTypeName += 4;
    maxlenSockName += 1;
    
    for (const auto& it : _socketsTable) {
        const auto& socket = it.second;
        std::cout << std::string(maxlenTypeName -
                                 socket->getConnecteeTypeName().length(), ' ')
                  << "[" << socket->getConnecteeTypeName() << "]"
                  << std::string(maxlenSockName -
                                 socket->getName().length(), ' ')
                  << socket->getName() << " : ";
        if (socket->getNumConnectees() == 0) {
            std::cout << "no connectees" << std::endl;
        } else {
            for (unsigned i = 0; i < socket->getNumConnectees(); ++i) {
                std::cout << socket->getConnecteeName(i) << " ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;
}

void Component::printInputInfo() const {
    std::cout << "Inputs for component " << getName() << " of type ["
              << getConcreteClassName() << "] along with connectee names:";
    if (getNumInputs() == 0)
        std::cout << " none";
    std::cout << std::endl;

    size_t maxlenTypeName{}, maxlenInputName{};
    for(const auto& input : _inputsTable) {
        maxlenTypeName = std::max(maxlenTypeName,
                                input.second->getConnecteeTypeName().length());
        maxlenInputName = std::max(maxlenInputName,
                                input.second->getName().length());
    }
    maxlenTypeName += 4;
    maxlenInputName += 1;

    for (const auto& it : _inputsTable) {
        const auto& input = it.second;
        std::cout << std::string(maxlenTypeName -
                                 input->getConnecteeTypeName().length(), ' ')
                  << "[" << input->getConnecteeTypeName() << "]"
                  << std::string(maxlenInputName -
                                 input->getName().length(), ' ')
                  << input->getName() << " : ";
        if (input->getNumConnectees() == 0) {
            std::cout << "no connectees" << std::endl;
        } else {
            for (unsigned i = 0; i < input->getNumConnectees(); ++i) {
                std::cout << input->getConnecteeName(i) << " ";
                // TODO as is, requires the input connections to be satisfied. 
                // std::cout << " (alias: " << input.getAlias(i) << ") ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;
}

void Component::printSubcomponentInfo() const {
    printSubcomponentInfo<Component>();
}

void Component::printOutputInfo(const bool includeDescendants) const {
    using ValueType = std::pair<std::string, SimTK::ClonePtr<AbstractOutput>>;

    // Do not display header for Components with no outputs.
    if (getNumOutputs() > 0) {
        const std::string msg = "Outputs from " + getAbsolutePathName() +
            " [" + getConcreteClassName() + "]";
        std::cout << msg << "\n" << std::string(msg.size(), '=') << std::endl;

        const auto& outputs = getOutputs();
        size_t maxlen{};
        for(const auto& output : outputs)
            maxlen = std::max(maxlen, output.second->getTypeName().length());
        maxlen += 2;
        
        for(const auto& output : outputs) {
            const auto& name = output.second->getTypeName();
            std::cout << std::string(maxlen - name.length(), ' ');
            std::cout << "[" << name  << "]  " << output.first << std::endl;
        }
        std::cout << std::endl;
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
    for (unsigned int i = 0; i < nmsc; ++i) {
        _memberSubcomponents[i]->initComponentTreeTraversal(root);
    }
    for (unsigned int i = 0; i < npsc; ++i) {
        _propertySubcomponents[i]->initComponentTreeTraversal(root);
    }
    for (unsigned int i = 0; i < nasc; ++i) {
        _adoptedSubcomponents[i]->initComponentTreeTraversal(root);
    }
}

} // end of namespace OpenSim
