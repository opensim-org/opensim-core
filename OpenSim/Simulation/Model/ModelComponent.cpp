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


ModelComponent::ModelComponent() : _model(NULL), _rep(NULL)
{
}

ModelComponent::ModelComponent(const std::string& aFileName, bool aUpdateFromXMLNode) : Object(aFileName, aUpdateFromXMLNode),
		_model(NULL), _rep(NULL)
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
    if(_model==NULL)
		throw Exception("ModelComponent::getModel(): component does not belong to a model."); 
	return *_model;
}

Model& ModelComponent::updModel()
{
	if(_model==NULL)
		throw Exception("ModelComponent::updModel(): component does not belong to a model."); 
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


void ModelComponent::addModelingOption(const std::string &optionName, int maxFlagValue) const
{
	if((int(_rep->_indexOfSubsystemForAllocations) == SimTK::InvalidIndex) ||
		_model->getMultibodySystem().systemTopologyHasBeenRealized())
			throw Exception("ModelComponent::addModelingOption: Modeling option can only be added during createSystem().");
	
	// don't add modeling option there is another state with the same name for this component
	std::map<std::string, ModelComponentRep::ModelingOptionInfo*>::const_iterator it;
	it = _rep->_namedModelingOptionInfo.find(optionName);
	if(it != _rep->_namedModelingOptionInfo.end())
		throw Exception("ModelComponent::addModelingOption: Modeling option '"+optionName+"' already exists.");
	// assign a "slot" for a modeling option by name
	// modeling option index will be invalid by default
	// upon allocation during realizeTopology the index will be set
	ModelComponentRep::ModelingOptionInfo *moi = new ModelComponentRep::ModelingOptionInfo;
	moi->maxOptionValue = maxFlagValue;
	_rep->_namedModelingOptionInfo[optionName] = moi;
}

void ModelComponent::addStateVariable(const std::string &stateVariableName, const SimTK::Stage &dependentOnStage) const
{
	if((int(_rep->_indexOfSubsystemForAllocations) == SimTK::InvalidIndex) ||
		_model->getMultibodySystem().systemTopologyHasBeenRealized())
			throw Exception("ModelComponent::addStateVariable: State Variable can only be added during createSystem().");

	if((dependentOnStage < Stage::Position) || (dependentOnStage > Stage::Dynamics)){
		throw Exception("ModelComponent::addStateVariable: dependentOnStage can only be Position, Velocity or Dynamics.");
	}

	// don't add state if there is another state variable with the same name for this component
	std::map<std::string, ModelComponentRep::StateVariableInfo*>::const_iterator it;
	it = _rep->_namedStateVariableInfo.find(stateVariableName);
	if(it != _rep->_namedStateVariableInfo.end()){
		throw Exception("ModelComponent::addStateVariable: State variable '" + 
			stateVariableName + "' already exists.");
	}
	// assign a "slot" for a state variable by name
	// state variable index will be invalid by default
	// upon allocation during realizeTopology the index will be set
	ModelComponentRep::StateVariableInfo *svi = new ModelComponentRep::StateVariableInfo;
	svi->dependentOnStage = dependentOnStage;
	_rep->_namedStateVariableInfo[stateVariableName] = svi;
}

void ModelComponent::addDiscreteVariable(const std::string &discreteVariableName, 
											const SimTK::Stage &dependentOnStage) const
{
	if((int(_rep->_indexOfSubsystemForAllocations) == SimTK::InvalidIndex) ||
		_model->getMultibodySystem().systemTopologyHasBeenRealized())
			throw Exception("ModelComponent: Discrete Variables can only be added during createSystem().");
	
	// don't add discrete var if there is another discrete variable with the same name for this component
	std::map<std::string, ModelComponentRep::DiscreteVariableInfo*>::const_iterator it;
	it = _rep->_namedDiscreteVariableInfo.find(discreteVariableName);
	if(it != _rep->_namedDiscreteVariableInfo.end()){
		throw Exception("ModelComponent::addDiscreteVariable: discrete variable '" + 
			discreteVariableName + "' already exists.");
	}
	// assign "slots" for the the discrete variables by name
	// discrete variable indices will be invalid by default
	// upon allocation during realizeTopology the indices will be set
	ModelComponentRep::DiscreteVariableInfo *dvi = new ModelComponentRep::DiscreteVariableInfo;
	dvi->dependentOnStage = dependentOnStage;
	_rep->_namedDiscreteVariableInfo[discreteVariableName] = dvi;
}

/*  Get the value of a ModelingOption flag for this ModelComponent. 
 *
 * @param state  the State for which to set the value
 * @return  flag  integer value for modeling option
 */
int ModelComponent::getModelingOption(const SimTK::State& s, const std::string &name) const
{
	std::map<std::string, ModelComponentRep::ModelingOptionInfo*>::const_iterator it;
	it = _rep->_namedModelingOptionInfo.find(name);

	if(it != _rep->_namedModelingOptionInfo.end()) {
		SimTK::DiscreteVariableIndex dvIndex = it->second->index;
		return( SimTK::Value<int>::downcast(s.getDiscreteVariable(getIndexOfSubsystemForAllocations(), dvIndex)).get());
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::getModelingOption: ERR- name '" << name << "' not found.\n " 
			<< "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
		throw( Exception(msg.str(),__FILE__,__LINE__) );
		return -1;
	}
}
/* Set the value of a discrete variable allocated by this ModelComponent by name. 
 *
 * @param state  the State in which to set the flag
 */
void ModelComponent::setModelingOption(SimTK::State& s, const std::string &name, int flag) const
{
	std::map<std::string, ModelComponentRep::ModelingOptionInfo*>::const_iterator it;
	it = _rep->_namedModelingOptionInfo.find(name);

	if(it != _rep->_namedModelingOptionInfo.end()) {
		SimTK::DiscreteVariableIndex dvIndex = it->second->index;
		if(flag > it->second->maxOptionValue){
			std::stringstream msg;
			msg << "ModelComponent::setModelingOption: "<< name << " flag cannot exceed "<< it->second->maxOptionValue <<".\n ";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
		}

		SimTK::Value<int>::downcast(s.updDiscreteVariable(getIndexOfSubsystemForAllocations(), dvIndex)).upd() = flag;
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::setModelingOption: modeling option "<< name <<" not found.\n ";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
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
	std::map<std::string, ModelComponentRep::StateVariableInfo*>::const_iterator it;
	it = _rep->_namedStateVariableInfo.find(name);

	if(it != _rep->_namedStateVariableInfo.end()) {
		ModelComponentRep::StateVariableInfo* svi = it->second;
		if(svi->dependentOnStage == Stage::Dynamics){
			const SimTK::Vector& z = s.getZ(getIndexOfSubsystemForAllocations());
			return z[ZIndex(svi->index)];
		}
		else if(svi->dependentOnStage == Stage::Velocity){
			const SimTK::Vector& u = s.getU(getIndexOfSubsystemForAllocations());
			return u[UIndex(svi->index)];
		}
		else if(svi->dependentOnStage == Stage::Position){
			const SimTK::Vector& q = s.getQ(getIndexOfSubsystemForAllocations());
			return q[QIndex(svi->index)];
		}	
	} 

	std::stringstream msg;
	msg << "ModelComponent::getStateVariable: ERR- variable name '" << name << "' not found.\n " 
		<< getName() << " of type " << getConcreteClassName() 
        << " has " << getNumStateVariables() << " states.";
	throw( Exception(msg.str(),__FILE__,__LINE__) );
	return SimTK::NaN;
}


/* Set the value of a state variable allocated by this ModelComponent given its index
 * for this component.
 *
 * param state   the State for which to set the value
 * param name    name of the state variable 
 * param value   the value to set */
void ModelComponent::setStateVariable(SimTK::State &s, const std::string &name, double value) const
{
	std::map<std::string, ModelComponentRep::StateVariableInfo*>::const_iterator it;
	it = _rep->_namedStateVariableInfo.find(name);

	if(it != _rep->_namedStateVariableInfo.end()) {
		ModelComponentRep::StateVariableInfo* svi = it->second;
		if(svi->dependentOnStage == Stage::Dynamics){
			SimTK::Vector& z = s.updZ(getIndexOfSubsystemForAllocations());
			z[ZIndex(svi->index)] = value;
		}
		else if(svi->dependentOnStage == Stage::Velocity){
			SimTK::Vector& u = s.updU(getIndexOfSubsystemForAllocations());
			u[UIndex(svi->index)] = value;
		}
		else if(svi->dependentOnStage == Stage::Position){
			SimTK::Vector& q = s.updQ(getIndexOfSubsystemForAllocations());
			q[QIndex(svi->index)] = value;
		}	
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::setStateVariable: ERR- name '" << name << "' not found.\n " 
			 << getName() << " of type " << getConcreteClassName() 
             << " has " << getNumStateVariables() << " states.";
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
		msg << "ModelComponent::getDiscreteVariable: ERR- name '" << name << "' not found.\n " 
			<< "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
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
		msg << "ModelComponent::setDiscreteVariable: ERR- name '" << name << "' not found.\n " 
			<< "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
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

const int ModelComponent::getStateIndex(const std::string &name) const
{
	std::map<std::string, ModelComponentRep::StateVariableInfo*>::const_iterator it;
	it = _rep->_namedStateVariableInfo.find(name);

	if(it != _rep->_namedStateVariableInfo.end()) {
		return it->second->index;
	} 
	else{
		std::stringstream msg;
		msg << "ModelComponent::getStateVariableSystemIndex: ERR- name '" << name << "' not found.\n " 
			<< "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
		throw( Exception(msg.str(),__FILE__,__LINE__) );
		return SimTK::InvalidIndex;
	}
}

SimTK::SystemYIndex ModelComponent::getStateVariableSystemIndex(const std::string &stateVariableName) const
{
	const SimTK::State &s = _model->getMultibodySystem().getDefaultState();

	std::map<std::string, ModelComponentRep::StateVariableInfo*>::const_iterator it;
	it = _rep->_namedStateVariableInfo.find(stateVariableName);
	
	if(it == _rep->_namedStateVariableInfo.end()) {
		std::stringstream msg;
		msg << "ModelComponent::getStateVariableSystemIndex: ERR- name '" << stateVariableName << "' not found.\n " 
			<< "for component '"<< getName() << "' of type " 
            << getConcreteClassName();
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	} 
	
	ModelComponentRep::StateVariableInfo *svi = it->second;

	SimTK::SystemYIndex ix;
	if(svi->dependentOnStage == Stage::Dynamics)
		ix = SystemYIndex(s.getZStart()+ s.getZStart(getIndexOfSubsystemForAllocations())+ ZIndex(svi->index));
	else if(svi->dependentOnStage == Stage::Velocity)
		ix = SystemYIndex(s.getUStart()+ s.getUStart(getIndexOfSubsystemForAllocations())+ UIndex(svi->index));
	else if(svi->dependentOnStage == Stage::Position)
		ix = SystemYIndex(s.getQStart()+ s.getQStart(getIndexOfSubsystemForAllocations())+ QIndex(svi->index));
	
	if(!(ix.isValid()))
		throw Exception(getConcreteClassName()
            + "::getStateVariableSystemIndex : state variable "
            + stateVariableName+" not found."); 
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
	std::map<std::string, StateVariableInfo*>::const_iterator it;
	it = _namedStateVariableInfo.begin();
	
	Array<std::string> names;

	while(it != _namedStateVariableInfo.end()){
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
	if(_namedModelingOptionInfo.size()>0){
		std::map<std::string, ModelingOptionInfo*>::iterator it;
		for(it = (mutableThis->_namedModelingOptionInfo).begin(); it!=_namedModelingOptionInfo.end(); it++){
			ModelingOptionInfo *moi = it->second;
			moi->index = subSys.allocateDiscreteVariable(s, SimTK::Stage::Instance, new SimTK::Value<int>(0));
		}
	}

	// Allocate Continuous State Variables
	if(_namedStateVariableInfo.size()>0){
		SimTK::Vector zInit(1, 0.0);
		std::map<std::string, StateVariableInfo*>::iterator it;
		for(it = (mutableThis->_namedStateVariableInfo).begin(); it!=_namedStateVariableInfo.end(); it++){
			StateVariableInfo *svi = it->second;
			if(svi->dependentOnStage == Stage::Dynamics)
				svi->index = subSys.allocateZ(s, zInit);
			else if(svi->dependentOnStage == Stage::Velocity)
				svi->index = subSys.allocateU(s, zInit);
			else if(svi->dependentOnStage == Stage::Position)
				svi->index = subSys.allocateQ(s, zInit);
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
	
	if(derivs.size() != _namedStateVariableInfo.size())
		throw Exception("ModelComponent: number of derivatives does not match number of state variables.");

	if(_namedStateVariableInfo.size()>0){
		std::map<std::string, StateVariableInfo*>::const_iterator it;
		int cnt = 0;
		for(it = _namedStateVariableInfo.begin(); it!=_namedStateVariableInfo.end(); it++){
			StateVariableInfo *svi = it->second;
			if(svi->dependentOnStage == Stage::Dynamics)
				subSys.updZDot(s)[ZIndex(svi->index)] = derivs[cnt++];
			else if(svi->dependentOnStage == Stage::Velocity)
				subSys.updUDot(s)[UIndex(svi->index)] = derivs[cnt++];
			else if(svi->dependentOnStage == Stage::Position)
				subSys.updQDot(s)[QIndex(svi->index)] = derivs[cnt++];
		}
	}

	// Now do the same for subcomponents
	for(unsigned int i=0; i< _modelComponent._subComponents.size(); i++)
		_modelComponent._subComponents[i]->getRep()->realizeAcceleration(s);
}

} // end of namespace OpenSim
