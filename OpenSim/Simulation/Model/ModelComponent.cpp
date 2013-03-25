/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ModelComponent.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

// INCLUDES
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"

using namespace SimTK;

namespace OpenSim {


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

void ModelComponent::connectToModel(Model& model)
{
    _model = &model;
    model._modelComponents.append(this);
    
    _simTKcomponentIndex.invalidate();
    clearStateAllocations();

    for(unsigned int i=0; i<_subComponents.size(); i++)
        _subComponents[i]->connectToModel(model);
}

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
    
    if(it == _namedStateVariableInfo.end()) {
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

    // Now do the same for subcomponents
    for(unsigned int i=0; i< _subComponents.size(); i++)
        _subComponents[i]->realizeTopology(s);
}



//------------------------------------------------------------------------------
//                         REALIZE ACCELERATION
//------------------------------------------------------------------------------
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

    // Now do the same for subcomponents
    for(unsigned int i=0; i< _subComponents.size(); i++)
        _subComponents[i]->realizeAcceleration(s);
}

} // end of namespace OpenSim
