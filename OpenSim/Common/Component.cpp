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
//#include "OpenSim/Common/ComponentOutput.h"

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

    Implementation* cloneVirtual() const FINAL_11
    {   return new Implementation(*this); }

    int getNumTimeDerivativesVirtual() const FINAL_11 {return 0;}
    SimTK::Stage getDependsOnStageVirtual(int order) const FINAL_11
    {   return SimTK::Stage::Empty; }
       
    const T& getUncachedValueVirtual
       (const SimTK::State& s, int derivOrder) const FINAL_11
    {   return this->getValueZero(); }

    void realizeMeasureTopologyVirtual(SimTK::State& s) const FINAL_11
    {   _Component.realizeTopology(s); }
    void realizeMeasureModelVirtual(SimTK::State& s) const FINAL_11
    {   _Component.realizeModel(s); }
    void realizeMeasureInstanceVirtual(const SimTK::State& s) const FINAL_11
    {   _Component.realizeInstance(s); }
    void realizeMeasureTimeVirtual(const SimTK::State& s) const FINAL_11
    {   _Component.realizeTime(s); }
    void realizeMeasurePositionVirtual(const SimTK::State& s) const FINAL_11
    {   _Component.realizePosition(s); }
    void realizeMeasureVelocityVirtual(const SimTK::State& s) const FINAL_11
    {   _Component.realizeVelocity(s); }
    void realizeMeasureDynamicsVirtual(const SimTK::State& s) const FINAL_11
    {   _Component.realizeDynamics(s); }
    void realizeMeasureAccelerationVirtual(const SimTK::State& s) const FINAL_11
    {   _Component.realizeAcceleration(s); }
    void realizeMeasureReportVirtual(const SimTK::State& s) const FINAL_11
    {   _Component.realizeReport(s); }

private:
    const Component& _Component;
};


//==============================================================================
//                              COMPONENT
//==============================================================================
Component::Component() : Object()
{
    constructProperty_connectors();
    finalizeFromProperties();
}

Component::Component(const std::string& fileName, bool updFromXMLNode)
:   Object(fileName, updFromXMLNode)
{
    constructProperty_connectors();
    finalizeFromProperties();
}

Component::Component(SimTK::Xml::Element& element) 
:   Object(element)
{
    constructProperty_connectors();
    finalizeFromProperties();
}

Component::Component(const Component& source) : Object(source)
{
    //Object copy will handle pthe propeties table.
    //But need to copy Component specific property inidices.
    copyProperty_connectors(source);
    finalizeFromProperties();
}

Component& Component::operator=(const Component &component)
{
    // Object handles assignment of all properties
    Super::operator=(component);
    finalizeFromProperties();
    return *this;
}


void Component::finalizeFromProperties()
{
    reset();
    clearComponents();
    extendFinalizeFromProperties();
    componentsFinalizeFromProperties();
    setObjectIsUpToDateWithProperties();
}

// Base class implementation of virtual method.
// Call extendFinalizeFromProperties on all components
void Component::componentsFinalizeFromProperties() const
{
    for (unsigned int i = 0; i<_components.size(); i++){
        _components[i]->finalizeFromProperties();
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

    reset();

    // rebuilding the connectors table, which was emptied by reset
    for (int ix = 0; ix < getProperty_connectors().size(); ++ix){
        AbstractConnector& connector = upd_connectors(ix);
        connector.disconnect();

        const Component* connectTo = root.findComponent(connector.get_connected_to_name());
        if (connectTo){
            connector.connect(*connectTo);
            //cout << getConcreteClassName() << " '" << getName();
            //cout << "' connected to: " << ci->get_connected_to_name() << endl;
        }
        else{
            throw Exception(getConcreteClassName() + "::connect() Could not find component '"
                + connector.get_connected_to_name() + "' to satisfy Connector<" +
                connector.getConnectedToTypeName() + "> '" + connector.getName() + "'.");
        }
        //is connected or an exception was thrown
    }

    // Allow derived Components to handle/check their connections
    extendConnect(root);

    componentsConnect(root);

    // Forming connections changes the Connector which is a property
    // Remark as upToDate.
    setObjectIsUpToDateWithProperties();
}


// Call connect on all components and find unconnected Connectors a
void Component::componentsConnect(Component& root) const
{
    // First give the subcomponents the opportunity to connect themselves
    for(unsigned int i=0; i<_components.size(); i++){
        _components[i]->connect(root);
    }
}

void Component::disconnect()
{
    // First give the subcomponents the opportunity to disconnect themselves
    for (unsigned int i = 0; i<_components.size(); i++){
        _components[i]->disconnect();
    }

    //Now cycle through and disconnect all connectors for this component
    std::map<std::string, int>::const_iterator it;
    for (it = _connectorsTable.begin(); it != _connectorsTable.end(); ++it){
        upd_connectors(it->second).disconnect();
    }

    //now clear all the stored system indices from this component
    reset();
}

void Component::addToSystem(SimTK::MultibodySystem& system) const
{
    baseAddToSystem(system);
    extendAddToSystem(system);
    componentsAddToSystem(system);
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
    // Invoke same method on subcomponents. TODO: is this right? The 
    // subcomponents add themselves to the system before the parent component.
    for(unsigned int i=0; i<_components.size(); i++)
        _components[i]->addToSystem(system);
}

void Component::initStateFromProperties(SimTK::State& state) const
{
    extendInitStateFromProperties(state);
    componentsInitStateFromProperties(state);
}

void Component::componentsInitStateFromProperties(SimTK::State& state) const
{
    for(unsigned int i=0; i < _components.size(); i++)
        _components[i]->initStateFromProperties(state);
}

void Component::setPropertiesFromState(const SimTK::State& state)
{
    extendSetPropertiesFromState(state);
    componentsSetPropertiesFromState(state);
}

void Component::componentsSetPropertiesFromState(const SimTK::State& state)
{
    for(unsigned int i=0; i < _components.size(); i++)
        _components[i]->setPropertiesFromState(state);
}

// Base class implementation of virtual method. Note that we're not handling
// subcomponents here; this method gets called from realizeAcceleration()
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
            msg	<< " added " << nasv << " state variables and ";
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
    _namedStateVariableInfo[stateVariableName] = StateVariableInfo(stateVariable, order);

    // If the StateVariable is not hidden, create an Output for this
    // StateVariable's value. We do this with an AddedStateVariable since
    // only AddedStateVariable's have a stage.
    if(!stateVariable->isHidden()){
        // StateVariable values are of type double. Also, StateVariable's
        // always have a value after they've been created, and they don't
        // depend on anything. So, their dependsOn Stage is Model.
        const_cast<Component*>(this)->constructOutput<double>(
                stateVariableName,
                std::bind(&Component::StateVariable::getValue,
                    stateVariable,
                    std::placeholders::_1),
                Stage::Model);
    }
                
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
    // assign "slots" for the the discrete variables by name
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
    for(unsigned int i=0; i<_components.size(); i++)
        ns += _components[i]->getNumStateVariables();

    return ns;
}

const Component& Component::getComponent(const std::string& name) const
{  
    const Component* found = findComponent(name);
    if(!found){
        std::string msg = "Component::getComponent() could not find component '";
        msg += name + "' from Component '" + getName();
        throw Exception(msg);
    }
    return *found;
}

Component& Component::updComponent(const std::string& name) const
{
    const Component* found = findComponent(name);
    if(!found){
        std::string msg = "Component::updComponent() could not find component '";
        msg += name + "' from Component '" + getName();
        throw Exception(msg);
    }
    return *const_cast<Component *>(found); 
}


const Component* Component::findComponent(const std::string& name,
    const StateVariable** rsv) const
{
    const Component* found = NULL;
    std::string::size_type front = name.find("/");
    std::string subname = name;
    std::string remainder = "";

    // Follow the provided path
    if (front < name.length()){
        subname = name.substr(0, front);
        remainder = name.substr(front + 1, name.length() - front);
    }

    for (unsigned int i = 0; i < _components.size(); ++i){
        if (_components[i]->getName() == subname){
            // if not the end of the path keep drilling
            if (remainder.length()){
                // keep traversing the components till we find the component
                found = _components[i]->findComponent(remainder, rsv);
                if (found)
                    return found;
            }
            else{
                return _components[i];
            }
        }
    }

    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(name);
    if (it != _namedStateVariableInfo.end()){
        if (rsv){
            *rsv = it->second.stateVariable.get();
        }
        return this;
    }

    // Path not given or could not find it along given path name
    // Now try complete search.
    if (!found) {
        for (unsigned int i = 0; i < _components.size(); ++i){
            found = _components[i]->findComponent(name, rsv);
            if (found)
                return found;
        }
    }

    return found;
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
    const Component* comp = findComponent(prefix, &found);

    if (comp){
        found = comp->findStateVariable(varName);
    }

    // Path not given or could not find it along given path name
    // Now try complete search.
    if (!found) {
        for (unsigned int i = 0; i < _components.size(); ++i){
            comp = _components[i]->findComponent(prefix, &found);
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
    // Include the states of its subcomponents
    for(unsigned int i=0; i<_components.size(); i++){
        Array<std::string> subnames = _components[i]->getStateVariableNames();
        int nsubs = subnames.getSize();
        const std::string& subCompName =  _components[i]->getName();
        std::string::size_type front = subCompName.find_first_not_of(" \t\r\n");
        std::string::size_type back = subCompName.find_last_not_of(" \t\r\n");
        std::string prefix = "";
        if(back > front) // have non-whitespace name
            prefix = subCompName+"/";
        for(int j =0; j<nsubs; ++j){
            names.append(prefix+subnames[j]);
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

// Set the value of a state variable allocated by this Component given its index
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

// Get all values of the state variables allocated by this Component. Includes
// state variables allocated by its subcomponents.
SimTK::Vector Component::
    getStateVariableValues(const SimTK::State& state) const
{
    int nsv = getNumStateVariables();
    Array<std::string> names = getStateVariableNames();

    Vector stateVariableValues(nsv, SimTK::NaN);
    for(int i=0; i<nsv; ++i){
        stateVariableValues[i]=getStateVariableValue(state, names[i]);
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
        "Component::setStateVariableValues() number values does not match number of state variables."); 
    Array<std::string> names = getStateVariableNames();

    Vector stateVariableValues(nsv, SimTK::NaN);
    for(int i=0; i<nsv; ++i){
        setStateVariableValue(state, names[i], values[i]);
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

/*
// Specifiy a member function of the state implemented by this component to be
// an Output.
template <typename T>
void Component::addOutput(const std::string& name,
                    const std::function<T(const SimTK::State&)> outputFunction,
                    const SimTK::Stage& dependsOn)
*/

// Include another Component as a subcomponent of this one. If already a
// subcomponent, it is not added to the list again.
void Component::addComponent(Component *aComponent)
{
    // Only add if the Component is not already a part of the model
    // So, add if empty
    if ( _components.empty() ){
        _components.push_back(aComponent);
    }
    else{ //otherwise check that it isn't apart of the component already		
        SimTK::Array_<Component *>::iterator it =
            std::find(_components.begin(), _components.end(), aComponent);
        if ( it == _components.end() ){
            _components.push_back(aComponent);
        }
        else{
            std::string msg = "ERROR- " +getConcreteClassName()+"::addComponent() '"
                + getName() + "' already has '" + aComponent->getName() +
                    "' as a subcomponent.";
            throw Exception(msg, __FILE__, __LINE__);
        }
    }
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
    const SimTK::State& s = getSystem().getDefaultState();

    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(stateVariableName);
    
    if(it != _namedStateVariableInfo.end()){
        return it->second.stateVariable->getSystemYIndex();
    }

    // Otherwise we have to search through subcomponents
    SimTK::SystemYIndex yix; 

    for(unsigned int i = 0; i < _components.size(); ++i) {
        yix = _components[i]->getStateVariableSystemIndex(stateVariableName);
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
// Super::realizeTopology() as the first line in the overriding method so that
// this code is executed before theirs.
// This method is invoked from the ComponentMeasure associated with this
// Component.
// Note that subcomponent realize() methods will be invoked by their own
// ComponentMeasures, so we do not need to forward to subcomponents here.
void Component::realizeTopology(SimTK::State& s) const
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
void Component::realizeAcceleration(const SimTK::State& s) const
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
                // set corresponing system derivative value from
                // cached value
                subSys.updZDot(s)[ZIndex(asv->getVarIndex())] =
                    asv->getDerivative(s);
        }
    }
}

//------------------------------------------------------------------------------
//                         OTHER REALIZE METHODS
//------------------------------------------------------------------------------
// Base class implementations of these virtual methods do nothing now but
// could do something in the future. Users must still invoke Super::realizeXXX()
// as the first line in their overrides to ensure future compatibility.
void Component::realizeModel(SimTK::State& state) const {}
void Component::realizeInstance(const SimTK::State& state) const {}
void Component::realizeTime(const SimTK::State& state) const {}
void Component::realizePosition(const SimTK::State& state) const {}
void Component::realizeVelocity(const SimTK::State& state) const {}
void Component::realizeDynamics(const SimTK::State& state) const {}
void Component::realizeReport(const SimTK::State& state) const {}


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



} // end of namespace OpenSim
