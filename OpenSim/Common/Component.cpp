/* -------------------------------------------------------------------------- *
 *                            OpenSim: Component.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
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
#include "OpenSim/Common/Component.h"
#include "OpenSim/Common/Set.h"
#include "OpenSim/Common/IO.h"

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
    constructProperty_connectors();
    constructProperty_inputs();
    constructProperty_components();
}

Component::Component(const std::string& fileName, bool updFromXMLNode)
:   Object(fileName, updFromXMLNode)
{
    constructProperty_connectors();
    constructProperty_inputs();
    constructProperty_components();
}

Component::Component(SimTK::Xml::Element& element) : Object(element)
{
    constructProperty_connectors();
    constructProperty_inputs();
    constructProperty_components();
}

void Component::addComponent(Component* comp) {
    //get to the root Component
    const Component* root = this;
    while (root->hasParent()) {
        root = &(root->getParent());
    }

    auto components = root->getComponentList<Component>();
    for (auto& c : components) {
        if (comp == &c) {
            OPENSIM_THROW( ComponentAlreadyPartOfOwnershipTree,
                comp->getName(), getName());
        }
    }

    updProperty_components().adoptAndAppendValue(comp);
    finalizeFromProperties();
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
        comp->setParent(*this);
    }
    for (auto& comp : _adoptedSubcomponents) {
        comp->setParent(*this);
    }

    // Provide each Input and Output with a pointer to its component (this) so 
    // that it can invoke its methods.
    // TODO if we implement custom copy constructor and assignment methods,
    // then this could be moved there (and then Output could take owner as an
    // argument to its constructor).
    if (getProperty_connectors().size() != _connectorsTable.size()) {
        std::stringstream msg;
        msg << "Expected " << _connectorsTable.size()
            << " Connectors but got " << getProperty_connectors().size() << ".";
        OPENSIM_THROW_FRMOBJ(Exception, msg.str());
    }
    for (int icx = 0; icx < getProperty_connectors().size(); ++icx){
        AbstractConnector& connector = upd_connectors(icx);
        connector.initialize(*this);
        _connectorsTable[connector.getName()] = icx;
    }
    
    if (getProperty_inputs().size() != _inputsTable.size()) {
        std::stringstream msg;
        msg << "Expected " << _inputsTable.size()
            << " Inputs but got " << getProperty_inputs().size() << ".";
        OPENSIM_THROW_FRMOBJ(Exception, msg.str());
    }
    for (int iix = 0; iix < getProperty_inputs().size(); ++iix) {
        AbstractInput& input = upd_inputs(iix);
        // The inputsTable holds an std::pair, whose second entry is `isList`.
        input.initialize(*this, _inputsTable[input.getName()].second);
        _inputsTable[input.getName()].first = iix;
    }
    for (auto& it : _outputsTable) {
        it.second->setOwner(*this);
    }

    markPropertiesAsSubcomponents();
    extendFinalizeFromProperties();
    componentsFinalizeFromProperties();
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
        comp->finalizeFromProperties();
    }
    for (auto& comp : _adoptedSubcomponents) {
        const_cast<Component*>(comp.get())
            ->finalizeFromProperties();
    }
}

// Base class implementation of non virtual connect method.
void Component::connect(Component &root)
{
    if (!isObjectUpToDateWithProperties()){
        // if edits occur between construction and connect() this is
        // the last chance to finalize before addToSystm.
        finalizeFromProperties();
    }

    // rebuilding the connectors table, which was emptied by clearStateAllocations
    for (int icx = 0; icx < getProperty_connectors().size(); ++icx){
        AbstractConnector& connector = upd_connectors(icx);
        connector.disconnect();
        try {
            connector.findAndConnect(root);
        }
        catch (const std::exception& x) {
            throw Exception(getConcreteClassName() + "'" + getName() +"'"
                "::connect() \nFailed to connect Connector<" +
                connector.getConnecteeTypeName() + "> '" + connector.getName() +
                "' within " + root.getConcreteClassName() + " '" + root.getName() +
                "' (details: " + x.what() + ").");
        }
    }

    for (int iix = 0; iix < getProperty_inputs().size(); ++iix) {
        AbstractInput& input = upd_inputs(iix);

        if (!input.isListConnector() && input.getConnecteeName(0).empty()) {
            std::cout << getConcreteClassName() << "'" << getName() << "'";
            std::cout << "::connect() Input<" << input.getConnecteeTypeName();
            std::cout << ">`" << input.getName();
            std::cout << "' Output has not been specified." << std::endl;
            continue;
        }

        input.disconnect();
        try {
            input.findAndConnect(root);
        }
        catch (const std::exception& x) {
            throw Exception(getConcreteClassName() + "'" + getName() + "'"
                "::connect() \nFailed to connect Input<" +
                input.getConnecteeTypeName() + "> '" + input.getName() +
                "' within " + root.getConcreteClassName() + " '" +
                root.getName() + "' (details: " + x.what() + ").");
        }
    }

    // Allow derived Components to handle/check their connections
    extendConnect(root);

    componentsConnect(root);

    // Forming connections changes the Connector which is a property
    // Remark as upToDate.
    setObjectIsUpToDateWithProperties();
}

// invoke connect on all (sub)components of this component
void Component::componentsConnect(Component& root) 
{
    // enable the subcomponents the opportunity to connect themselves
    for (unsigned int i = 0; i<_memberSubcomponents.size(); ++i) {
        _memberSubcomponents[i].upd()->connect(root);
    }
    for(unsigned int i=0; i<_propertySubcomponents.size(); ++i){
        _propertySubcomponents[i].get()->connect(root);
    }
    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); ++i) {
        _adoptedSubcomponents[i].upd()->connect(root);
    }
}

void Component::disconnect()
{
    // First give the subcomponents the opportunity to disconnect themselves
    for (unsigned int i = 0; i<_memberSubcomponents.size(); i++) {
        _memberSubcomponents[i]->disconnect();
    }
    for (unsigned int i = 0; i<_propertySubcomponents.size(); i++){
        _propertySubcomponents[i]->disconnect();
    }
    for (unsigned int i = 0; i<_adoptedSubcomponents.size(); i++) {
        _adoptedSubcomponents[i]->disconnect();
    }

    //Now cycle through and disconnect all connectors for this component
    for (const auto& it : _connectorsTable) {
        upd_connectors(it.second).disconnect();
    }
    
    // Must also clear the input's connections.
    for (const auto& it : _inputsTable) {
        upd_inputs(it.second.first).disconnect();
    }

    //now clear all the stored system indices from this component
    reset();
}

void Component::addToSystem(SimTK::MultibodySystem& system) const
{
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
    for (unsigned int i = 0; i<_memberSubcomponents.size(); ++i)
        _memberSubcomponents[i]->addToSystem(system);
    for (unsigned int i = 0; i<_propertySubcomponents.size(); ++i)
        _propertySubcomponents[i]->addToSystem(system);
    for(unsigned int i=0; i<_adoptedSubcomponents.size(); ++i)
        _adoptedSubcomponents[i]->addToSystem(system);
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


int Component::getNumStateVariables() const
{
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


const Component& Component::getParent() const 
{
    if (!hasParent()) {
        std::string msg = "Component '" + getName() + "'::getParent(). " +
            "Has no parent Component assigned.\n" +
            "Make sure the component was added to the Model (or its parent).";
        throw Exception(msg);
    }
    return _parent.getRef();
}

bool Component::hasParent() const
{
    return !_parent.empty();
}

void Component::setParent(const Component& parent)
{
    if (&parent == this) {
        std::string msg = "Component '" + getName() + "'::setParent(). " +
            "Attempted to set itself as its parent.";
        throw Exception(msg);
    }
    else if (_parent.get() == &parent) {
        return;
    }

    _parent.reset(&parent);
}

std::string Component::getFullPathName() const
{
    std::string pathName = getName();
    const Component* up = this;

    while (up && up->hasParent()) {
        up = &up->getParent();
        pathName = up->getName() + "/" + pathName;
    }
    // The root must have a leading '/' 
    pathName = "/" + pathName;

    return pathName;
}

std::string Component::getRelativePathName(const Component& wrt) const
{
    std::string thisP = getFullPathName();
    std::string wrtP = wrt.getFullPathName();

    IO::TrimWhitespace(thisP);
    IO::TrimWhitespace(wrtP);

    size_t ni = thisP.length();
    size_t nj = wrtP.length();
    // the limit on the common substring size is the smallest of the two
    size_t limit = ni <= nj ? ni : nj;
    size_t ix = 1, jx = 0;
    size_t last = 0;

    // Loop through the paths to pick up a bigger substring that matches
    while ( !thisP.compare(0, ix, wrtP, 0, ix) && (ix < std::string::npos) ) {
        last = ix;
        // step ahead to the next node if there is one
        ix = thisP.find('/', last+1);
        // step along both strings
        jx = wrtP.find('/', last+1);

        // if no more nodes for this path try complete match of final node name
        if (ix == std::string::npos && last < limit)
            ix = ni;

        // if no more nodes for wrt path try complete match of final node name
        if (jx == std::string::npos && last < limit)
            jx = nj;

        // verify that the both paths are in sync
        if (ix != jx) { // if not break to use the current last index of match
            break;
        }
     }

    // Include the delimiter in the common string iff we are not at the end of string
    // and not at the root
    if ((0 < last) && (last < limit) && thisP[last] == '/') {
        last = last + 1;
    }

    std::string common = thisP.substr(0, last);
    std::string stri = thisP.substr(last, ni-last);
    std::string strj = wrtP.substr(last, nj-last);

    std::string prefix  = "";

    // the whole wrt path is part of the common path
    if (strj.empty() && stri[0] == '/') {
        prefix = ".";
    }
    // if there is a remainder in the wrtP then move at least one
    else if (!strj.empty() && strj[0] != '/') {
        prefix += "../";
    }

    last = 0;
    // process all the '/' nodes in the wrtP up to the common path
    while ((jx = strj.find('/', last)) < std::string::npos) {
        prefix += "../";
        last = jx + 1;
    }

    // return the relative path 
    return prefix + stri;
}

const AbstractConnector* Component::findConnector(const std::string& name) const
{
    const AbstractConnector* found = nullptr;

    std::map<std::string, int>::const_iterator it;
    it = _connectorsTable.find(name);

    if (it != _connectorsTable.end()) {
        const AbstractConnector& absConnector = get_connectors(it->second);
        found = &absConnector;
    }
    else {
        std::string::size_type back = name.rfind("/");
        std::string prefix = name.substr(0, back);
        std::string conName = name.substr(back + 1, name.length() - back);

        const Component* component = findComponent(prefix);
        if (component){
            found = component->findConnector(conName);
        }
    }
    return found;
}

const AbstractInput* Component::findInput(const std::string& name) const
{
    const AbstractInput* found = nullptr;
    
    const auto it = _inputsTable.find(name);
    
    if (it != _inputsTable.end()) {
        const AbstractInput& absInput = get_inputs(it->second.first);
        found = &absInput;
    }
    else {
        std::string::size_type back = name.rfind("/");
        std::string prefix = name.substr(0, back);
        std::string inputName = name.substr(back + 1, name.length() - back);
        
        const Component* component = findComponent(prefix);
        if (component) {
            found = component->findInput(inputName);
        }
    }
    return found;
}

const Component::StateVariable* Component::
    findStateVariable(const std::string& name) const
{
    // first assume that the state variable named belongs to this
    // top level component
    std::string::size_type back = name.rfind("/");
    std::string prefix = name.substr(0, back);
    std::string varName = name.substr(back + 1, name.length() - back);

    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(varName);

    if (it != _namedStateVariableInfo.end()) {
        return it->second.stateVariable.get();
    }

    const StateVariable* found = nullptr;
    const Component* comp = traversePathToComponent<Component>(prefix);

    if (comp){
        found = comp->findStateVariable(varName);
    }

    // Path not given or could not find it along given path name
    // Now try complete search.
    if (!found) {
        for (unsigned int i = 0; i < _propertySubcomponents.size(); ++i){
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
    Array<std::string> names = getStateVariablesNamesAddedByComponent();

/** TODO: Use component iterator  like below
    for (int i = 0; i < stateNames.size(); ++i) {
        stateNames[i] = (getFullPathName() + "/" + stateNames[i]);
    }

    for (auto& comp : getComponentList<Component>()) {
        const std::string& pathName = comp.getFullPathName();// *this);
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
        if (back > front) // have non-whitespace name
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
        if(back > front) // have non-whitespace name
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
        if (back > front) // have non-whitespace name
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
    // This change dirties the GeometryPath which is aproperty of a Muscle which
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
    setStateVariableValues(SimTK::State& state, const SimTK::Vector& values)
{
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
        auto& prop = updPropertyByIndex(i);
        // check if property is of type Object
        if (prop.isObjectProperty()) {
            // a property is a list so cycle through its contents
            for (int j = 0; j < prop.size(); ++j) {
                Object& obj = prop.updValueAsObject(j);
                // if the object is a Component mark it
                if (Component* comp = dynamic_cast<Component*>(&obj) ) {
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
                        auto& objectsProp = obj.updPropertyByName("objects");
                        // loop over the objects in the PropertyObjArray
                        for (int k = 0; k < objectsProp.size(); ++k) {
                            Object& obj = objectsProp.updValueAsObject(k);
                            // if the object is a Component mark it
                            if (Component* comp = dynamic_cast<Component*>(&obj) )
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
void Component::markAsPropertySubcomponent(Component* component)
{
    // Only add if the component is not already a part of this Component
    // So, add if empty
    SimTK::ReferencePtr<Component> compRef(component);
    if (_propertySubcomponents.empty() ){
        _propertySubcomponents.push_back(SimTK::ReferencePtr<Component>(component));
    }
    else{ //otherwise check that it isn't a part of the component already
        auto it =
            std::find(_propertySubcomponents.begin(), _propertySubcomponents.end(), compRef);
        if ( it == _propertySubcomponents.end() ){
            _propertySubcomponents.push_back(SimTK::ReferencePtr<Component>(component));
        }
        else{
            auto compPath = component->getFullPathName();
            auto foundPath = it->get()->getFullPathName();
            OPENSIM_THROW( ComponentAlreadyPartOfOwnershipTree,
                           component->getName(), getName());
        }
    }

    component->setParent(*this);
}


// Include another Component as a subcomponent of this one. If already a
// subcomponent, it is not added to the list again.
void Component::adoptSubcomponent(Component* subcomponent)
{
    OPENSIM_THROW_IF(subcomponent->hasParent(),
        ComponentAlreadyPartOfOwnershipTree,
        subcomponent->getName(), this->getName());

    //get the top-level component
    const Component* top = this;
    while (top->hasParent())
        top = &top->getParent();

    // cycle through all components from the top level component
    // down to verify the component is not already in the tree
    for (auto& comp : top->getComponentList<Component>()) {
        OPENSIM_THROW_IF(subcomponent->hasParent(),
            ComponentAlreadyPartOfOwnershipTree,
            subcomponent->getName(), comp.getName());
    }

    subcomponent->setParent(*this);
    _adoptedSubcomponents.push_back(SimTK::ClonePtr<Component>(subcomponent));
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


void Component::dumpSubcomponents(int depth) const
{
    std::string tabs;
    for (int t = 0; t < depth; ++t) {
        tabs += "\t";
    }

    std::cout << tabs << getConcreteClassName();
    std::cout << " '" << getName() << "'" << std::endl;
    for (size_t i = 0; i < _memberSubcomponents.size(); ++i) {
        _memberSubcomponents[int(i)]->dumpSubcomponents(depth + 1);
    }
    for (size_t i = 0; i < _propertySubcomponents.size(); ++i) {
        _propertySubcomponents[int(i)]->dumpSubcomponents(depth + 1);
    }
    for (size_t i = 0; i < _adoptedSubcomponents.size(); ++i) {
        _adoptedSubcomponents[int(i)]->dumpSubcomponents(depth + 1);
    }
}

void Component::dumpConnections() const {
    std::cout << "Connectors for " << getConcreteClassName() << " '"
              << getName() << "':";
    if (getNumConnectors() == 0) std::cout << " none";
    std::cout << std::endl;
    for (int icx = 0; icx < getProperty_connectors().size(); ++icx){
        const auto& connector = get_connectors(icx);
        std::cout << "  " << connector.getConnecteeTypeName() << " '"
                  << connector.getName() << "': ";
        if (connector.getNumConnectees() == 0) {
            std::cout << "no connectees" << std::endl;
        } else {
            for (unsigned i = 0; i < connector.getNumConnectees(); ++i) {
                std::cout << connector.getConnecteeName(i) << " ";
            }
            std::cout << std::endl;
        }
    }
    
    std::cout << "Inputs for " << getConcreteClassName() << " '"
              << getName() << "':";
    if (getNumInputs() == 0) std::cout << " none";
    std::cout << std::endl;
    for (int iix = 0; iix < getProperty_inputs().size(); ++iix) {
        const auto& input = get_inputs(iix);
        std::cout << "  " << input.getConnecteeTypeName() << " '"
                  << input.getName() << "': ";
        if (input.getNumConnectees() == 0) {
            std::cout << "no connectees" << std::endl;
        } else {
            for (unsigned i = 0; i < input.getNumConnectees(); ++i) {
                std::cout << input.getConnecteeName(i) << " ";
                // TODO as is, requires the input connections to be satisfied. 
                // std::cout << " (annotation: " << input.getAnnotation(i) 
                //           << ") ";
            }
            std::cout << std::endl;
        }
    }
}


void Component::initComponentTreeTraversal(const Component &root) const {
    // Going down the tree, node is followed by all its
    // children in order, last child's successor is the parent's successor.
    const Component* last = nullptr;
    for (unsigned int i = 0; i < _memberSubcomponents.size(); i++) {
        if (i == _memberSubcomponents.size() - 1) {
            // use parent's sibling if any
            if (this == &root) // only to be safe if root changes
                _memberSubcomponents[i]->_nextComponent = nullptr;
            else {
                _memberSubcomponents[i]->_nextComponent =
                    _nextComponent.get();
            }
            last = _memberSubcomponents[i].get();
        }
        else {
            _memberSubcomponents[i]->_nextComponent =
                _memberSubcomponents[i + 1].get();
            last = _memberSubcomponents[i + 1].get();
        }
    }
    if (size_t npsc = _propertySubcomponents.size()) {
        if (last)
            last->_nextComponent = _propertySubcomponents[0].get();

        for (unsigned int i = 0; i < npsc; i++) {
            if (i == npsc - 1) {
                // use parent's sibling if any
                if (this == &root) // only to be safe if root changes
                    _propertySubcomponents[i]->_nextComponent = nullptr;
                else {
                    _propertySubcomponents[i]->_nextComponent =
                        _nextComponent.get();
                }
                last = _propertySubcomponents[i].get();
            }
            else {
                _propertySubcomponents[i]->_nextComponent =
                    _propertySubcomponents[i + 1].get();
                last = _propertySubcomponents[i + 1].get();
            }
        }
    }
    if (size_t nasc = _adoptedSubcomponents.size()) {
        if (last)
            last->_nextComponent = _adoptedSubcomponents[0].get();

        for (unsigned int i = 0; i <nasc; i++) {
            if (i == nasc - 1) {
                // use parent's sibling if any
                if (this == &root) // only to be safe if root changes
                    _adoptedSubcomponents[i]->_nextComponent = nullptr;
                else
                    _adoptedSubcomponents[i]->_nextComponent =
                    _nextComponent.get();
            }
            else {
                _adoptedSubcomponents[i]->_nextComponent
                    = _adoptedSubcomponents[i + 1].get();
            }
        }
    }

    // recurse to handle children of subcomponents
    for (unsigned int i = 0; i < _memberSubcomponents.size(); i++) {
        _memberSubcomponents[i]->initComponentTreeTraversal(root);
    }
    for (unsigned int i = 0; i < _propertySubcomponents.size(); i++) {
        _propertySubcomponents[i]->initComponentTreeTraversal(root);
    }
    for (unsigned int i = 0; i < _adoptedSubcomponents.size(); i++) {
        _adoptedSubcomponents[i]->initComponentTreeTraversal(root);
    }
}


} // end of namespace OpenSim
