// ModelComponent.cpp
// Authors: Ajay Seth, Peter Eastman
/*
 * Copyright (c) 2009, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// INCLUDES
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"

using namespace SimTK;

namespace OpenSim {

//=============================================================================
// Begin class ModelComponentMeasure
//=============================================================================



ModelComponent::ModelComponent() : _model(NULL), _rep(NULL)
{
}

ModelComponent::ModelComponent(const std::string& aFileName, bool aUpdateFromXMLNode) : Object(aFileName, aUpdateFromXMLNode),
		_model(NULL), _rep(NULL)
{
}

ModelComponent::ModelComponent(const XMLDocument* aDocument): Object(aDocument), _model(NULL), _rep(NULL)
{
}

ModelComponent::ModelComponent(SimTK::Xml::Element& aNode) : Object(aNode), _model(NULL), _rep(NULL)
{
}

ModelComponent::ModelComponent(const ModelComponent& copy) : Object(copy), _model(NULL), _rep(NULL)
{
}

ModelComponent::~ModelComponent()
{
	//Delete info for allocation of variables
	delete _rep;
}

ModelComponent& ModelComponent::operator=(const ModelComponent &aModelComponent)
{
	Object::operator=(aModelComponent);
	return(*this);
}

const Model& ModelComponent::getModel() const
{
    return *_model;
}

Model& ModelComponent::updModel()
{
    return *_model;
}

void ModelComponent::setup(Model& model)
{
	_model = &model;
	model._modelComponents.append(this);
	delete _rep;
	_rep = new ModelComponentRep(*this);

	for(unsigned int i=0; i<_subComponents.size(); i++)
		_subComponents[i]->setup(model);
}

// Default behavior is that ModelComponent owns an underlying SimTK::Measure 
// which is a ModelComponentMeasure<T>. For simplicity, we are specifying
// T as SimTK::Real but a more flexible mechanism could be developed to 
// specify according to a specified prototype (similar to how CacheVariables
// are being specified).
void ModelComponent::createSystem(SimTK::MultibodySystem& system) const
{
	if(!(getIndexOfSubsystemForAllocations().isValid())){
		ModelComponent* mutableThis = const_cast<ModelComponent *>(this);
		mutableThis->setIndexOfSubsystemForAllocations(system.getDefaultSubsystem().getMySubsystemIndex());
		ModelComponentMeasure<SimTK::Real> mcMeasure(system.updDefaultSubsystem(), *_rep);
		_rep->_simTKcomponentIndex = mcMeasure.getSubsystemMeasureIndex();
	}
	for(unsigned int i=0; i<_subComponents.size(); i++)
		_subComponents[i]->createSystem(system);
}

void ModelComponent::setIndexOfSubsystemForAllocations(SimTK::SubsystemIndex subsysIndex)
{
	_rep->_indexOfSubsystemForAllocations = subsysIndex;
	
	for(unsigned int i=0; i<_subComponents.size(); i++)
		_subComponents[i]->setIndexOfSubsystemForAllocations(subsysIndex);
}

const SimTK::SubsystemIndex ModelComponent::getIndexOfSubsystemForAllocations() const
{
	return _rep->_indexOfSubsystemForAllocations;
}


void ModelComponent::addModelingOption(const Array<std::string> &optionFlagNames)
{
	if((int(_rep->_indexOfSubsystemForAllocations) == SimTK::InvalidIndex) ||
		_model->getMultibodySystem().systemTopologyHasBeenRealized())
			throw Exception("ModelComponent: Modeling Option can only be added during createSystem().");

	_rep->_optionFlagNames.resize(optionFlagNames.getSize());
	for(int i=0; i<optionFlagNames.getSize(); i++)
		_rep->_optionFlagNames[i] = optionFlagNames[i];
}

void ModelComponent::addStateVariables(const Array<std::string> &stateVariableNames)
{
	if((int(_rep->_indexOfSubsystemForAllocations) == SimTK::InvalidIndex) ||
		_model->getMultibodySystem().systemTopologyHasBeenRealized())
			throw Exception("ModelComponent: State Variables can only be added during createSystem().");
	// assign "slots" for the the state variables by name
	// state variable indices (ZIndex) will be invalid by default
	// upon allocation during realizeTopology the indices will be set
	for(int i=0; i<stateVariableNames.getSize(); i++)
		_rep->_namedStateVariableIndices[stateVariableNames[i]];

}

void ModelComponent::addDiscreteVariables(const Array<std::string> &discreteVariableNames, 
											const SimTK::Stage &dependentOnStage)
{
	if((int(_rep->_indexOfSubsystemForAllocations) == SimTK::InvalidIndex) ||
		_model->getMultibodySystem().systemTopologyHasBeenRealized())
			throw Exception("ModelComponent: Discrete Variables can only be added during createSystem().");
	// assign "slots" for the the discrete variables by name
	// discrete variable indices will be invalid by default
	// upon allocation during realizeTopology the indices will be set
	for(int i=0; i<discreteVariableNames.getSize(); i++){
		ModelComponentRep::DiscreteVariableInfo *dvi = new ModelComponentRep::DiscreteVariableInfo;
		dvi->dependentOnStage = dependentOnStage;
		_rep->_namedDiscreteVariableInfo[discreteVariableNames[i]] = dvi;
	}
}

/*  Get the value of a ModelingOption flag for this ModelComponent. 
 *
 * @param state  the State for which to set the value
 * @return  flag  integer value for modeling option
 */
int ModelComponent::getModelingOption(const SimTK::State& s) const
{
	SimTK::DiscreteVariableIndex dvIndex = _rep->_modelingOptionIndex;

	return( SimTK::Value<int>::downcast(s.getDiscreteVariable(getIndexOfSubsystemForAllocations(), dvIndex)).get());
}
/* Set the value of a discrete variable allocated by this ModelComponent by name. 
 *
 * @param state  the State in which to set the flag
 */
void ModelComponent::setModelingOption(SimTK::State& s, int flag) const
{
	if((flag < 0 ) || (flag > int(_rep->_optionFlagNames.size())-1))
			throw Exception("ModelComponent: modeling option flag does not correspond available options.");
	
	SimTK::DiscreteVariableIndex dvIndex = _rep->_modelingOptionIndex;
	SimTK::Value<int>::downcast(s.updDiscreteVariable(getIndexOfSubsystemForAllocations(), dvIndex)).upd() = flag;
}


int ModelComponent::getNumStateVariables() const
{
	int ns = _rep->getNumStateVariablesAddedByModelComponent(); //+ numStatesOfUnderlyingComponent
	// Include the states of its subcomponents
	for(unsigned int i=0; i<_subComponents.size(); i++)
		ns += _subComponents[i]->getNumStateVariables();

	return ns;
}

/**
* Get the names of "continuous" state variables maintained by the ModelComponent
* and its subcomponents
*/
Array<std::string> ModelComponent::getStateVariableNames() const
{
	Array<std::string> names = _rep->getStateVariablesNamesAddedByModelComponent();
	// Include the states of its subcomponents
	for(unsigned int i=0; i<_subComponents.size(); i++)
		names.append(_subComponents[i]->getStateVariableNames());

	return names;
}

/* Get the value of a state variable allocated by this ModelComponent.  
 *
 * param state   the State for which to get the value
 * param name    the name of the state variable  */
double ModelComponent::getStateVariable(const SimTK::State &s, const std::string &name) const
{
	std::map<std::string,SimTK::ZIndex>::const_iterator it;
	it = _rep->_namedStateVariableIndices.find(name);

	if(it != _rep->_namedStateVariableIndices.end()) {
		const SimTK::Vector& z = s.getZ(getIndexOfSubsystemForAllocations());
		return z[it->second];
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::getStateVariable: ERR- variable name '" << name << "' not found.\n " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
		return SimTK::NaN;
	}
}


/* Set the value of a state variable allocated by this ModelComponent given its index
 * for this component.
 *
 * param state   the State for which to set the value
 * param name    name of the state variable 
 * param value   the value to set */
void ModelComponent::setStateVariable(SimTK::State &s, const std::string &name, double value) const
{
	std::map<std::string,SimTK::ZIndex>::const_iterator it;
	it = _rep->_namedStateVariableIndices.find(name);

	if(it != _rep->_namedStateVariableIndices.end()) {
		SimTK::Vector& z = s.updZ(getIndexOfSubsystemForAllocations());
		z[it->second] = value;
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::setStateVariable: ERR- name not found.\n " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}
// Get/Set for Discrete Variables

/* Get the value of a discrete variable allocated by this ModelComponent by name. 
 *
 * param state  the State for which to set the value
 * param name   the name of the state variable 
 * param value  the value to set */
double ModelComponent::getDiscreteVariable(const SimTK::State &s, const std::string &name) const
{
	std::map<std::string, ModelComponentRep::DiscreteVariableInfo*>::const_iterator it;
	it = _rep->_namedDiscreteVariableInfo.find(name);

	if(it != _rep->_namedDiscreteVariableInfo.end()) {
		SimTK::DiscreteVariableIndex dvIndex = it->second->index;
		return( SimTK::Value<double>::downcast(s.getDiscreteVariable(getIndexOfSubsystemForAllocations(), dvIndex)).get());
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::getDiscreteVariable: ERR- name not found.\n " 
			<< "for component '"<< getName() << "' of type " << getType();
		throw( Exception(msg.str(),__FILE__,__LINE__) );
		return SimTK::NaN;
	}
}

/* Set the value of a discrete variable allocated by this ModelComponent by name. 
 *
 * @param state  the State for which to set the value
 * @param name   the name of the state variable 
 * @param value  the value to set  */
void ModelComponent::setDiscreteVariable(SimTK::State &s, const std::string &name, double value) const
{
	std::map<std::string, ModelComponentRep::DiscreteVariableInfo*>::const_iterator it;
	it = _rep->_namedDiscreteVariableInfo.find(name);

	if(it != _rep->_namedDiscreteVariableInfo.end()) {
		SimTK::DiscreteVariableIndex dvIndex = it->second->index;
		SimTK::Value<double>::downcast(s.updDiscreteVariable(getIndexOfSubsystemForAllocations(), dvIndex)).upd() = value;
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::setDiscreteVariable: ERR- name not found.\n " 
			<< "for component '"<< getName() << "' of type " << getType();
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

/** Include another ModelComponent as a subcomponent of this one. 
    If already a subcomponent it is not added to the list again. */
void ModelComponent::includeAsSubComponent(ModelComponent *aComponent)
{
	if(_model && _model->isValidSystem())
		throw Exception("ModelComponent: Cannot include subcomponent after createSystem().");
	
	// Only keep if unique
	SimTK::Array_<ModelComponent *>::iterator it = std::find(_subComponents.begin(), _subComponents.end(), aComponent);
	if(it == _subComponents.end()) 
		_subComponents.push_back(aComponent);
}

const SimTK::ZIndex ModelComponent::getZIndex(const std::string &name) const
{
	std::map<std::string, SimTK::ZIndex>::const_iterator it;
	it = _rep->_namedStateVariableIndices.find(name);
	return it->second;
}

SimTK::SystemYIndex ModelComponent::getStateVariableSystemIndex(const std::string &stateVariableName) const
{
	const SimTK::State &s = _model->getMultibodySystem().getDefaultState();
	SimTK::SystemYIndex ix(s.getZStart()+ s.getZStart(getIndexOfSubsystemForAllocations())+ getZIndex(stateVariableName));
	if(!(ix.isValid()))
		throw Exception(getType()+"::getStateVariableSystemIndex : state variable "+stateVariableName+" not found."); 
	return ix;
}

const SimTK::DiscreteVariableIndex ModelComponent::getDiscreteVariableIndex(const std::string &name) const
{
	std::map<std::string, ModelComponentRep::DiscreteVariableInfo*>::const_iterator it;
	it = _rep->_namedDiscreteVariableInfo.find(name);

	return it->second->index;
}

const SimTK::CacheEntryIndex ModelComponent::getCacheVariableIndex(const std::string &name) const
{
	std::map<std::string, ModelComponentRep::CacheInfo*>::const_iterator it;
	it = _rep->_namedCacheVariableInfo.find(name);

	return it->second->index;
}

//===========================================================================================
//==============================  ModelComponentRep =========================================
ModelComponentRep::ModelComponentRep(ModelComponent &mc): _modelComponent(mc)
{
	_namedDiscreteVariableInfo.clear();
	_namedCacheVariableInfo.clear();
}

ModelComponentRep::~ModelComponentRep()
{
	std::map<std::string, DiscreteVariableInfo*>::iterator it;
	for(it = _namedDiscreteVariableInfo.begin(); it!=_namedDiscreteVariableInfo.end();){
		delete it->second;
		_namedDiscreteVariableInfo.erase(it++);
	}

	std::map<std::string, CacheInfo*>::iterator ic;
	for(ic = _namedCacheVariableInfo.begin(); ic!=_namedCacheVariableInfo.end();){
		delete ic->second;
		_namedCacheVariableInfo.erase(ic++);
	}
}

Array<std::string> ModelComponentRep::getStateVariablesNamesAddedByModelComponent() const
{
	std::map<std::string,SimTK::ZIndex>::const_iterator it;
	it = _namedStateVariableIndices.begin();
	
	Array<std::string> names;

	while(it != _namedStateVariableIndices.end()){
		names.append(it->first);
		it++;
	}
	return names;
}


void ModelComponentRep::realizeTopology(SimTK::State &s) const
{
	SimTK::System &sys = _modelComponent.updModel().updMultibodySystem();
	SimTK::Subsystem &subSys = sys.updSubsystem(_indexOfSubsystemForAllocations);

	ModelComponentRep *mutableThis = const_cast<ModelComponentRep *>(this);

	// Allocate Modeling Option
	if(_optionFlagNames.size()>0)
		mutableThis->_modelingOptionIndex = subSys.allocateDiscreteVariable(s, SimTK::Stage::Instance, new SimTK::Value<int>(0));

	// Allocate Continuous State Variables
	if(_namedStateVariableIndices.size()>0){
		SimTK::Vector zInit(1, 0.0);
		std::map<std::string,SimTK::ZIndex>::iterator it;
		for(it = (mutableThis->_namedStateVariableIndices).begin(); it!=_namedStateVariableIndices.end(); it++){
			// should set to default value and not assume 0.0
			it->second = subSys.allocateZ(s, zInit);
		}
	}

	// Allocate Discrete State Variables
	if(_namedDiscreteVariableInfo.size()>0){
		std::map<std::string, DiscreteVariableInfo*>::iterator it;
		for(it = (mutableThis->_namedDiscreteVariableInfo).begin(); it!=_namedDiscreteVariableInfo.end(); it++){
			DiscreteVariableInfo *dvi = it->second;
			dvi->index = subSys.allocateDiscreteVariable(s, dvi->dependentOnStage, new SimTK::Value<double>(0.0));
		}
	}

	// Allocate Cache Entry in the State
	if(_namedCacheVariableInfo.size()>0){
		std::map<std::string, CacheInfo*>::iterator it;
		for(it = (mutableThis->_namedCacheVariableInfo).begin(); it!=_namedCacheVariableInfo.end(); it++){
			CacheInfo *ci = it->second;
			ci->index = subSys.allocateLazyCacheEntry(s, ci->lowestValidStage, ci->prototype->clone());
		}
	}

	// Now do the same for subcomponents
	for(unsigned int i=0; i< _modelComponent._subComponents.size(); i++)
		_modelComponent._subComponents[i]->getRep()->realizeTopology(s);
}

void ModelComponentRep::realizeAcceleration(const SimTK::State& s) const
{
	SimTK::System &sys = _modelComponent.updModel().updMultibodySystem();
	SimTK::Subsystem &subSys = sys.updSubsystem(_indexOfSubsystemForAllocations);

	SimTK::Vector derivs = _modelComponent.computeStateVariableDerivatives(s);
	
	if(derivs.size() != _namedStateVariableIndices.size())
		throw Exception("ModelComponent: number of derivatives does not match number of state variables.");

	if(_namedStateVariableIndices.size()>0){
		std::map<std::string,SimTK::ZIndex>::const_iterator it;
		int cnt = 0;
		for(it = _namedStateVariableIndices.begin(); it!=_namedStateVariableIndices.end(); it++){
			subSys.updZDot(s)[it->second] = derivs[cnt++];
		}
	}

	// Now do the same for subcomponents
	for(unsigned int i=0; i< _modelComponent._subComponents.size(); i++)
		_modelComponent._subComponents[i]->getRep()->realizeAcceleration(s);
}

} // end of namespace OpenSim
