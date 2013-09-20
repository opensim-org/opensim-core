/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ModelComponent.cpp                        *
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
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"

using namespace SimTK;

namespace OpenSim {

//==============================================================================
//                         MODEL COMPONENT MEASURE
//==============================================================================
// Every OpenSim::ModelComponent is associated with a Simbody Measure of type
// ModelComponentMeasure defined here. This provides a full set of realize()
// methods for performing computations with System resources that are maintained
// at the ModelComponent base level, such as calculating state derivatives.

template <class T>
class ModelComponentMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(ModelComponentMeasure, SimTK::Measure_<T>);

    ModelComponentMeasure(SimTK::Subsystem& sub, 
                          const OpenSim::ModelComponent& mc)
    :   SimTK::Measure_<T>(sub, new Implementation(mc), 
                    SimTK::AbstractMeasure::SetHandle()) {}

    SimTK_MEASURE_HANDLE_POSTSCRIPT(ModelComponentMeasure, SimTK::Measure_<T>);
};


template <class T>
class ModelComponentMeasure<T>::Implementation 
:   public SimTK::Measure_<T>::Implementation {
public:
    // Don't allocate a value cache entry since this measure's value is
    // just a dummy.
    explicit Implementation(const ModelComponent& mc)
    :   SimTK::Measure_<T>::Implementation(0), _modelComponent(mc) {}

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
    {   _modelComponent.realizeTopology(s); }
    void realizeMeasureModelVirtual(SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeModel(s); }
    void realizeMeasureInstanceVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeInstance(s); }
    void realizeMeasureTimeVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeTime(s); }
    void realizeMeasurePositionVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizePosition(s); }
    void realizeMeasureVelocityVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeVelocity(s); }
    void realizeMeasureDynamicsVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeDynamics(s); }
    void realizeMeasureAccelerationVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeAcceleration(s); }
    void realizeMeasureReportVirtual(const SimTK::State& s) const FINAL_11
    {   _modelComponent.realizeReport(s); }

private:
    const ModelComponent& _modelComponent;
};


//==============================================================================
//                             MODEL COMPONENT
//==============================================================================
ModelComponent::ModelComponent() : _model(NULL) 
{
}

ModelComponent::ModelComponent(const std::string& fileName, bool updFromXMLNode)
:   Object(fileName, updFromXMLNode), _model(NULL)
{
}

ModelComponent::ModelComponent(SimTK::Xml::Element& element) 
:   Object(element), _model(NULL)
{
}

// Don't copy any of the base class data members; they all get set later.
ModelComponent::ModelComponent(const ModelComponent& source) 
:   Object(source), _model(NULL)
{
}

// Don't copy any of the base class data members; clear them instead.
ModelComponent& ModelComponent::operator=(const ModelComponent& source)
{
    if (&source != this) { 
        Object::operator=(source);
        setNull();
    }
    return *this;
}

const Model& ModelComponent::getModel() const
{
    if(_model==NULL)
        throw Exception("ModelComponent::getModel(): component does not "
                        "belong to a model."); 
    return *_model;
}

Model& ModelComponent::updModel()
{
    if(_model==NULL)
        throw Exception("ModelComponent::updModel(): component does not "
                        "belong to a model."); 
    return *_model;
}

// Base class implementation of virtual method.
void ModelComponent::connectToModel(Model& model)
{
    _model = &model;
    model._modelComponents.append(this);
    
    _simTKcomponentIndex.invalidate();
    clearStateAllocations();

    for(unsigned int i=0; i<_subComponents.size(); i++)
        _subComponents[i]->connectToModel(model);
}

// Base class implementation of virtual method.
// Every ModelComponent owns an underlying SimTK::Measure 
// which is a ModelComponentMeasure<T> and is added to the System's default
// subsystem. That measure is used only for the side effect of its realize()
// methods being called; its value is not used.
void ModelComponent::addToSystem(SimTK::MultibodySystem& system) const
{
    // Briefly get write access to the ModelComponent to record some
    // information associated with the System; that info is const after this.
    ModelComponent* mutableThis = const_cast<ModelComponent *>(this);

    // Allocate the ModelComponentMeasure, point it to this ModelComponent for 
    // making realize() calls, and add it to the system's default subsystem. 
    ModelComponentMeasure<double> mcMeasure(system.updDefaultSubsystem(), *this);
    mutableThis->_simTKcomponentIndex = mcMeasure.getSubsystemMeasureIndex();

    // Invoke same method on subcomponents. TODO: is this right? The 
    // subcomponents add themselves to the system before the parent component.
    for(unsigned int i=0; i<_subComponents.size(); i++)
        _subComponents[i]->addToSystem(system);
}

// Base class implementation of virtual method.
void ModelComponent::initStateFromProperties(SimTK::State& state) const {
    for(unsigned int i=0; i < _subComponents.size(); i++)
        _subComponents[i]->initStateFromProperties(state);
};

// Base class implementation of virtual method.
void ModelComponent::setPropertiesFromState(const SimTK::State& state) {
    for(unsigned int i=0; i < _subComponents.size(); i++)
        _subComponents[i]->setPropertiesFromState(state);
};

// Base class implementation of virtual method. Note that we're not handling
// subcomponents here; this method gets called from realizeAcceleration()
// which will be invoked for each subcomponent by its own ModelComponentMeasure.
SimTK::Vector ModelComponent::
computeStateVariableDerivatives(const SimTK::State& s) const
{   return SimTK::Vector(0); };

// Base class implementation of virtual method.
void ModelComponent::generateDecorations
    (bool                                        fixed, 
    const ModelDisplayHints&                    hints,
    const SimTK::State&                         state,
    SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const 
{
    for(unsigned int i=0; i < _subComponents.size(); i++)
        _subComponents[i]->
            generateDecorations(fixed,hints,state,appendToThis);   
}

void ModelComponent::
addModelingOption(const std::string& optionName, int maxFlagValue) const 
{
    // don't add modeling option there is another state with the same name for 
    // this component
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.find(optionName);
    if(it != _namedModelingOptionInfo.end())
        throw Exception("ModelComponent::addModelingOption: Modeling option '"
              + optionName + "' already exists.");
    // assign a "slot" for a modeling option by name
    // modeling option index will be invalid by default
    // upon allocation during realizeTopology the index will be set
    _namedModelingOptionInfo[optionName] = ModelingOptionInfo(maxFlagValue);
}

void ModelComponent::
addStateVariable(const std::string& stateVariableName, 
                 SimTK::Stage       invalidatesStage) const
{
    if(   (invalidatesStage < Stage::Position) 
       || (invalidatesStage > Stage::Dynamics))
    {
        throw Exception("ModelComponent::addStateVariable: invalidatesStage "
                        "must be Position, Velocity or Dynamics.");
    }

    // don't add state if there is another state variable with the same name 
    // for this component
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(stateVariableName);
    if(it != _namedStateVariableInfo.end()){
        throw Exception("ModelComponent::addStateVariable: State variable '" + 
            stateVariableName + "' already exists.");
    }

	int order = (int)_namedStateVariableInfo.size();
    // assign a "slot" for a state variable by name
    // state variable index will be invalid by default
    // upon allocation during realizeTopology the index will be set
    _namedStateVariableInfo[stateVariableName] = 
        StateVariableInfo(invalidatesStage, order);
}

void ModelComponent::
addDiscreteVariable(const std::string&  discreteVariableName, 
                    SimTK::Stage        invalidatesStage) const
{
    // don't add discrete var if there is another discrete variable with the 
    // same name for this component
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(discreteVariableName);
    if(it != _namedDiscreteVariableInfo.end()){
        throw Exception("ModelComponent::addDiscreteVariable: discrete variable '" + 
            discreteVariableName + "' already exists.");
    }
    // assign "slots" for the the discrete variables by name
    // discrete variable indices will be invalid by default
    // upon allocation during realizeTopology the indices will be set
    _namedDiscreteVariableInfo[discreteVariableName] = 
        DiscreteVariableInfo(invalidatesStage);
}

/*  Get the value of a ModelingOption flag for this ModelComponent. 
 *
 * @param state  the State for which to set the value
 * @return  flag  integer value for modeling option
 */
int ModelComponent::
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
        msg << "ModelComponent::getModelingOption: ERR- name '" << name 
            << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
        return -1;
    }
}
/* Set the value of a discrete variable allocated by this ModelComponent by name. 
 *
 * @param state  the State in which to set the flag
 */
void ModelComponent::
setModelingOption(SimTK::State& s, const std::string& name, int flag) const
{
    std::map<std::string, ModelingOptionInfo>::const_iterator it;
    it = _namedModelingOptionInfo.find(name);

    if(it != _namedModelingOptionInfo.end()) {
        SimTK::DiscreteVariableIndex dvIndex = it->second.index;
        if(flag > it->second.maxOptionValue){
            std::stringstream msg;
            msg << "ModelComponent::setModelingOption: "<< name 
                << " flag cannot exceed "<< it->second.maxOptionValue <<".\n ";
        throw Exception(msg.str(),__FILE__,__LINE__);
        }

        SimTK::Value<int>::downcast(
            getDefaultSubsystem().updDiscreteVariable(s, dvIndex)).upd() = flag;
    } else {
        std::stringstream msg;
        msg << "ModelComponent::setModelingOption: modeling option " << name 
            << " not found.\n ";
        throw Exception(msg.str(),__FILE__,__LINE__);
    }
}


int ModelComponent::getNumStateVariables() const
{
    int ns = getNumStateVariablesAddedByModelComponent(); //+ numStatesOfUnderlyingComponent
    // Include the states of its subcomponents
    for(unsigned int i=0; i<_subComponents.size(); i++)
        ns += _subComponents[i]->getNumStateVariables();

    return ns;
}

/*
 * Get the names of "continuous" state variables maintained by the ModelComponent
 * and its subcomponents
 */
Array<std::string> ModelComponent::getStateVariableNames() const
{
    Array<std::string> names = getStateVariablesNamesAddedByModelComponent();
    // Include the states of its subcomponents
    for(unsigned int i=0; i<_subComponents.size(); i++)
        names.append(_subComponents[i]->getStateVariableNames());

    return names;
}

/* Get the value of a state variable allocated by this ModelComponent.  
 *
 * param state   the State for which to get the value
 * param name    the name of the state variable  */
double ModelComponent::
getStateVariable(const SimTK::State& s, const std::string& name) const
{
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(name);

    if(it != _namedStateVariableInfo.end()) {
        const StateVariableInfo& svi = it->second;
        if(svi.invalidatesStage == Stage::Dynamics){
            const SimTK::Vector& z = getDefaultSubsystem().getZ(s);
            return z[ZIndex(svi.index)];
        }
        else if(svi.invalidatesStage == Stage::Velocity){
            const SimTK::Vector& u = getDefaultSubsystem().getU(s);
            return u[UIndex(svi.index)];
        }
        else if(svi.invalidatesStage == Stage::Position){
            const SimTK::Vector& q = getDefaultSubsystem().getQ(s);
            return q[QIndex(svi.index)];
        }	
    } 

    std::stringstream msg;
    msg << "ModelComponent::getStateVariable: ERR- variable name '" << name 
        << "' not found.\n " 
        << getName() << " of type " << getConcreteClassName() 
        << " has " << getNumStateVariables() << " states.";
    throw Exception(msg.str(),__FILE__,__LINE__);
    return SimTK::NaN;
}


/* Set the value of a state variable allocated by this ModelComponent given its
 * index for this component.
 *
 * param state   the State for which to set the value
 * param name    name of the state variable 
 * param value   the value to set */
void ModelComponent::
setStateVariable(SimTK::State& s, const std::string& name, double value) const
{
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(name);

    if(it != _namedStateVariableInfo.end()) {
        const StateVariableInfo& svi = it->second;
        if(svi.invalidatesStage == Stage::Dynamics){
            SimTK::Vector& z = getDefaultSubsystem().updZ(s);
            z[ZIndex(svi.index)] = value;
        }
        else if(svi.invalidatesStage == Stage::Velocity){
            SimTK::Vector& u = getDefaultSubsystem().updU(s);
            u[UIndex(svi.index)] = value;
        }
        else if(svi.invalidatesStage == Stage::Position){
            SimTK::Vector& q = getDefaultSubsystem().updQ(s);
            q[QIndex(svi.index)] = value;
        }	
    } 
    else{
        std::stringstream msg;
        msg << "ModelComponent::setStateVariable: ERR- name '" << name 
            << "' not found.\n " 
            << getName() << " of type " << getConcreteClassName() 
            << " has " << getNumStateVariables() << " states.";
        throw Exception(msg.str(),__FILE__,__LINE__);
    }
}
// Get/Set for Discrete Variables

/* Get the value of a discrete variable allocated by this ModelComponent by name. 
 *
 * param state  the State for which to set the value
 * param name   the name of the state variable 
 * param value  the value to set */
double ModelComponent::
getDiscreteVariable(const SimTK::State& s, const std::string& name) const
{
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(name);

    if(it != _namedDiscreteVariableInfo.end()) {
        SimTK::DiscreteVariableIndex dvIndex = it->second.index;
        return SimTK::Value<double>::downcast(
            getDefaultSubsystem().getDiscreteVariable(s, dvIndex)).get();
    } else {
        std::stringstream msg;
        msg << "ModelComponent::getDiscreteVariable: ERR- name '" << name 
            << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
        return SimTK::NaN;
    }
}

/* Set the value of a discrete variable allocated by this ModelComponent by name. 
 *
 * @param state  the State for which to set the value
 * @param name   the name of the state variable 
 * @param value  the value to set  */
void ModelComponent::
setDiscreteVariable(SimTK::State& s, const std::string& name, double value) const
{
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(name);

    if(it != _namedDiscreteVariableInfo.end()) {
        SimTK::DiscreteVariableIndex dvIndex = it->second.index;
        SimTK::Value<double>::downcast(
            getDefaultSubsystem().updDiscreteVariable(s, dvIndex)).upd() = value;
    } else {
        std::stringstream msg;
        msg << "ModelComponent::setDiscreteVariable: ERR- name '" << name 
            << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
    }
}

/** Include another ModelComponent as a subcomponent of this one. 
    If already a subcomponent it is not added to the list again. */
void ModelComponent::includeAsSubComponent(ModelComponent *aComponent)
{
    if(_model && _model->isValidSystem())
        throw Exception("ModelComponent: Cannot include subcomponent after addToSystem().");
    
    // Only keep if unique
    SimTK::Array_<ModelComponent *>::iterator it = std::find(_subComponents.begin(), _subComponents.end(), aComponent);
    if(it == _subComponents.end()) 
        _subComponents.push_back(aComponent);
}

const int ModelComponent::getStateIndex(const std::string& name) const
{
    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(name);

    if(it != _namedStateVariableInfo.end()) {
        return it->second.index;
    } else {
        std::stringstream msg;
        msg << "ModelComponent::getStateVariableSystemIndex: ERR- name '" 
            << name << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
        return SimTK::InvalidIndex;
    }
}

SimTK::SystemYIndex ModelComponent::
getStateVariableSystemIndex(const std::string& stateVariableName) const
{
    const SimTK::State& s = _model->getWorkingState();

    std::map<std::string, StateVariableInfo>::const_iterator it;
    it = _namedStateVariableInfo.find(stateVariableName);
    bool isFound = (it != _namedStateVariableInfo.end());

    // Check subcomponents, if necessary.
    unsigned int idx = 0;
    while(idx<_subComponents.size() && !isFound) {
        it = _subComponents[idx]->_namedStateVariableInfo.find(stateVariableName);
        if(it != _subComponents[idx]->_namedStateVariableInfo.end())
            isFound = true;
        ++idx;
    }

    if(!isFound) {
        std::stringstream msg;
        msg << "ModelComponent::getStateVariableSystemIndex: ERR- name '" 
            << stateVariableName << "' not found.\n " 
            << "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
        throw Exception(msg.str(),__FILE__,__LINE__);
    } 
    
    const StateVariableInfo& svi = it->second;

    SimTK::SystemYIndex ix;
    if(svi.invalidatesStage == Stage::Dynamics)
        ix = SystemYIndex(s.getZStart() + getDefaultSubsystem().getZStart(s)
                                        + ZIndex(svi.index));
    else if(svi.invalidatesStage == Stage::Velocity)
        ix = SystemYIndex(s.getUStart() + getDefaultSubsystem().getUStart(s)
                                        + UIndex(svi.index));
    else if(svi.invalidatesStage == Stage::Position)
        ix = SystemYIndex(s.getQStart() + getDefaultSubsystem().getQStart(s)
                                        + QIndex(svi.index));
    
    if(!(ix.isValid()))
        throw Exception(getConcreteClassName()
            + "::getStateVariableSystemIndex : state variable "
            + stateVariableName+" not found."); 
    return ix;
}

const SimTK::DiscreteVariableIndex ModelComponent::
getDiscreteVariableIndex(const std::string& name) const
{
    std::map<std::string, DiscreteVariableInfo>::const_iterator it;
    it = _namedDiscreteVariableInfo.find(name);

    return it->second.index;
}

const SimTK::CacheEntryIndex ModelComponent::
getCacheVariableIndex(const std::string& name) const
{
    std::map<std::string, CacheInfo>::const_iterator it;
    it = _namedCacheVariableInfo.find(name);

    return it->second.index;
}

Array<std::string> ModelComponent::
getStateVariablesNamesAddedByModelComponent() const
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


const SimTK::DefaultSystemSubsystem& ModelComponent::
getDefaultSubsystem() const
{   return getModel().getDefaultSubsystem(); }

const SimTK::DefaultSystemSubsystem& ModelComponent::
updDefaultSubsystem()
{   return updModel().updDefaultSubsystem(); }


//------------------------------------------------------------------------------
//                            REALIZE TOPOLOGY
//------------------------------------------------------------------------------
// This is the base class implementation of a virtual method that can be
// overridden by derived model components, but they *must* invoke
// Super::realizeTopology() as the first line in the overriding method so that
// this code is executed before theirs.
// This method is invoked from the ModelComponentMeasure associated with this
// ModelComponent.
// Note that subcomponent realize() methods will be invoked by their own
// ModelComponentMeasures, so we do not need to forward to subcomponents here.
void ModelComponent::realizeTopology(SimTK::State& s) const
{
    const SimTK::System&    sys = getModel().getMultibodySystem();
    const SimTK::Subsystem& subSys = sys.getDefaultSubsystem();
    
    
    ModelComponent *mutableThis = const_cast<ModelComponent*>(this);

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
            StateVariableInfo& svi = it->second;
            if(svi.invalidatesStage == Stage::Dynamics)
                svi.index = subSys.allocateZ(s, zInit);
            else if(svi.invalidatesStage == Stage::Velocity)
                svi.index = subSys.allocateU(s, zInit);
            else if(svi.invalidatesStage == Stage::Position)
                svi.index = subSys.allocateQ(s, zInit);
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
void ModelComponent::realizeAcceleration(const SimTK::State& s) const
{
    if(_namedStateVariableInfo.size()>0) {
        const SimTK::System&    sys = getModel().getMultibodySystem();
        const SimTK::Subsystem& subSys =  getModel().getMultibodySystem().getDefaultSubsystem();

        SimTK::Vector derivs = computeStateVariableDerivatives(s);
    
        if(derivs.size() != _namedStateVariableInfo.size())
            throw Exception("ModelComponent: number of derivatives does not match number of state variables.");

        std::map<std::string, StateVariableInfo>::const_iterator it;

        for (it = _namedStateVariableInfo.begin(); 
             it != _namedStateVariableInfo.end(); ++it)
        {
            const StateVariableInfo& svi = it->second;
            if(svi.invalidatesStage == Stage::Dynamics)
                subSys.updZDot(s)[ZIndex(svi.index)] = derivs[svi.order];
            else if(svi.invalidatesStage == Stage::Velocity)
                subSys.updUDot(s)[UIndex(svi.index)] = derivs[svi.order];
            else if(svi.invalidatesStage == Stage::Position)
                subSys.updQDot(s)[QIndex(svi.index)] = derivs[svi.order];
        }
    }
}

//------------------------------------------------------------------------------
//                         OTHER REALIZE METHODS
//------------------------------------------------------------------------------
// Base class implementations of these virtual methods do nothing now but
// could do something in the future. Users must still invoke Super::realizeXXX()
// as the first line in their overrides to ensure future compatibility.
void ModelComponent::realizeModel(SimTK::State& state) const {}
void ModelComponent::realizeInstance(const SimTK::State& state) const {}
void ModelComponent::realizeTime(const SimTK::State& state) const {}
void ModelComponent::realizePosition(const SimTK::State& state) const {}
void ModelComponent::realizeVelocity(const SimTK::State& state) const {}
void ModelComponent::realizeDynamics(const SimTK::State& state) const {}
void ModelComponent::realizeReport(const SimTK::State& state) const {}

} // end of namespace OpenSim
