#ifndef __ModelComponent_h__
#define __ModelComponent_h__

// ModelComponent.h
// Authors: Ajay Seth, Peter Eastman, Ayman Habib
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Common/Object.h"
#include "SimTKsimbody.h"

namespace OpenSim {

class Model;
class ModelComponent;
#ifndef SWIG
//=============================================================================
// Begin class ModelComponentRep
//=============================================================================
class ModelComponentRep
{
protected:
	ModelComponent& _modelComponent;

	// Structures to hold related info about discrete variables and cache entries to be added
	struct DiscreteVariableInfo {
		SimTK::DiscreteVariableIndex index;
		SimTK::Stage dependentOnStage;
		DiscreteVariableInfo(): index(SimTK::InvalidIndex), dependentOnStage(SimTK::Stage::Empty) {};
    };

	struct CacheInfo {
		SimTK::CacheEntryIndex index;
		SimTK::AbstractValue *prototype;
		SimTK::Stage lowestValidStage;
		CacheInfo(): index(SimTK::InvalidIndex), prototype(NULL), lowestValidStage(SimTK::Stage::Empty) {};
		~CacheInfo() {delete prototype; prototype=NULL;}
    };

	// Underlying subsystem containing the allocated variables, defaults to
	// SimTK::System.defaultSubsystem if not overridden by subclass
	SimTK::SubsystemIndex _indexOfSubsystemForAllocations;
	// Underlying SimTK custom measure ModelComponentMeasure, which implements
	// the realizations in the subsystem by calling private concrete methods on
	// the ModelComponent. Subclasses are free to use this and/or any other
	// component, such as a SimTK::Force for the same purposes.
	SimTK::MeasureIndex _simTKcomponentIndex;

	// Names of modeling option flags of the ModelComponent
	SimTK::Array_<std::string> _optionFlagNames;
	// Index of the modeling option integer flag in the state
	SimTK::DiscreteVariableIndex _modelingOptionIndex;

	// Map names of continuous state variables of the ModelComponent to their underlying SimTK indices
	std::map<std::string, SimTK::ZIndex, std::less<std::string> > _namedStateVariableIndices;
	// Map names of discrete variables of the ModelComponent to their underlying SimTK indices
	std::map<std::string, DiscreteVariableInfo*, std::less<std::string> > _namedDiscreteVariableInfo;
	// Map names of cache entries of the ModelComponent to their individual cache information
	std::map<std::string, CacheInfo*, std::less<std::string> > _namedCacheVariableInfo;

	/** Constructor */
	ModelComponentRep(ModelComponent &mc);
	/** Destructor */
	~ModelComponentRep();

public:
	/** Get the number of continuous states that the ModelComponent added to the underlying
	    computational system. It does not include the number of states already built-in by
		the SimTK::System level component. Should query it and add to this to obtain the total
		number of states managed by this ModelComponent */
	int getNumStateVariablesAddedByModelComponent() const {return _namedStateVariableIndices.size();}
	void realizeTopology(SimTK::State &s) const;
	void realizeModel(SimTK::State& s) const;
    void realizeInstance(const SimTK::State& s) const;
    void realizeTime(const SimTK::State& s) const;
    void realizePosition(const SimTK::State& s) const;
    void realizeVelocity(const SimTK::State& s) const;
    void realizeDynamics(const SimTK::State& s) const;
    void realizeAcceleration(const SimTK::State& s) const;
    void realizeReport(const SimTK::State& s) const;
	friend class ModelComponent;
};
//==================================
// end class ModelComponentRep
//==================================
#endif


//=============================================================================
// Begin class ModelComponent
//=============================================================================
/**
 * This is the base class for any component which can be added to a Model.  It
 * defines a set of methods which a component must implement to be integrated
 * into the underlying system.
 *
 * The primary responsibility of a ModelComponent is to add its contributions
 * to the underlying SimTK::System by implementing createSystem().
 *
 * Additional methods provide support for adding modeling options, state and
 * cache variables.
 */

class OSIMSIMULATION_API ModelComponent : public Object
{
protected:
	Model* _model;


//=============================================================================
// METHODS
//=============================================================================
public:
    template <class T> friend class ModelComponentSet;
	ModelComponent();
	ModelComponent(const std::string& aFileName, bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;
	ModelComponent(const XMLDocument* aDocument);
	ModelComponent(DOMElement* aNode);
	ModelComponent(const ModelComponent& copy);
    virtual ~ModelComponent();

	/** Assignment operator to copy contents of an existing component */
	ModelComponent& operator=(const ModelComponent &aModelComponent);

    /**
     * Get the Model this object is part of.
     */
    const Model& getModel() const;
    /**
     * Get a modifiable reference to the Model this object is part of.
     */
    Model& updModel();

	/**
	 * In case the ModelCompoenent has a visual representation, override this method to update 
	 * the representation, typically this's done by recomputing anchor points and positions based 
	 * on transforms obtained from current state
	 */
	virtual void updateDisplayer(const SimTK::State& s) {};

	/**
     * Get the value of a ModelingOption flag for this ModelComponent.
     *
     * @param state  the State for which to set the value
     * @return  flag  integer value for modeling option
     */
	virtual int getModelingOption(const SimTK::State& state) const;
	/**
     * Set the value of a discrete variable allocated by this ModelComponent by name.
     *
     * @param state  the State in which to set the flag
     */
	virtual void setModelingOption(SimTK::State& state, int flag) const;


	/**
     * Gets the number of "Continuous" state variables maintained by the ModelComponent
     * If the ModelComponent defines any that are of interest to the user, names should also be given
	 */
    virtual int getNumStateVariables() const;

    /**
     * Get the name of a state variable allocated by this ModelComponent.  The default implementation
     * throws an exception, so subclasses that allocate state variables must override it.
     *
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     */
	virtual std::string getStateVariableName(int index) const
	{
		throw Exception("This ModelComponent has no state variables");
	}
    /**
     * Get the YIndex of a state variable allocated by this ModelComponent.  The default implementation
     * throws an exception, so subclasses that allocate state variables must override it.
     *
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     */
	virtual int getStateVariableYIndex(int index) const
	{
		throw Exception("This ModelComponent has no state variables");
	}

    /**
     * Get the value of a state variable allocated by this ModelComponent by index.
	 * TODO: This should be deprecated to use name only to avoid any confusion.
     *
     * @param state   the State for which to get the value
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     */
	virtual double getStateVariable(const SimTK::State& state, int index) const;
    /**
     * Get the value of a state variable allocated by this ModelComponent.
     *
     * @param state   the State for which to get the value
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     */
	virtual double getStateVariable(const SimTK::State& state, const std::string &name) const;

    /**
     * Set the value of a state variable allocated by this ModelComponent.
     *
     * @param state   the State for which to set the value
     * @param index   the index of the state variable (0 to getNumStateVariables()-1)
     * @param value   the value to set
     */
	virtual void setStateVariable(SimTK::State& state, int index, double value) const;

	/**
     * Set the value of a state variable allocated by this ModelComponent by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param value  the value to set
     */
	virtual void setStateVariable(SimTK::State& state, const std::string &name, double value) const;

	/**
     * Get the value of a discrete variable allocated by this ModelComponent by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @param value  the value to get
     */
	virtual double getDiscreteVariable(const SimTK::State& state, const std::string &name) const;
	/**
     * Set the value of a discrete variable allocated by this ModelComponent by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the dsicrete variable
     * @param value  the value to set
     */
	virtual void setDiscreteVariable(SimTK::State& state, const std::string &name, double value) const;

	/**
     * Get the value of a cache variable allocated by this ModelComponent by name.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the cache variable
     * @return T	 the variable's cache value
     */
	template<typename T> const T& getCacheVariable(const SimTK::State& state, const std::string &name) const
	{
		std::map<std::string, ModelComponentRep::CacheInfo*>::const_iterator it;
		it = _rep->_namedCacheVariableInfo.find(name);

		if(it != _rep->_namedCacheVariableInfo.end()) {
			SimTK::CacheEntryIndex ceIndex = it->second->index;
			return( SimTK::Value<T>::downcast(state.getCacheEntry(getIndexOfSubsystemForAllocations(), ceIndex)).get());
		}
		else{
			std::stringstream msg;
			msg << "ModelComponent::getCacheVariable: ERR- name not found.\n "
				<< "for component '"<< getName() << "' of type " << getType();
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}
	/**
     * Obtain a writable cache variable value allocated by this ModelComponent by name.
	 * Do not forget to mark the cache value as valid after updating, otherwise it will
	 * force a reevaluation if the evaluation method is monitoring the validity of the
	 * cache value.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @return value  the variable's cache value to update
     */
	template<typename T> T& updCacheVariable(const SimTK::State& state, const std::string &name) const
	{
		std::map<std::string, ModelComponentRep::CacheInfo*>::const_iterator it;
		it = _rep->_namedCacheVariableInfo.find(name);

		if(it != _rep->_namedCacheVariableInfo.end()) {
			SimTK::CacheEntryIndex ceIndex = it->second->index;
			return SimTK::Value<T>::downcast(state.updCacheEntry( getIndexOfSubsystemForAllocations(), ceIndex)).upd();
		}
		else{
			std::stringstream msg;
			msg << "ModelComponent::updCacheVariable: ERR- '" << name << "' name not found.\n "
				<< "for component '"<< getName() << "' of type " << getType();
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}

	/**
     * After updating a cache variable value allocated by this ModelComponent, you can
	 * mark its value as valid, which will not change until the realization stage falls
	 * below the minimum set at the time the cache variable was created. If not marked
     * as valid, the evaluation method monitoring this flag will force a re-evaluation
	 * rather that just reading the value from the cache.
	 *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     */
	void markCacheVariableValid(const SimTK::State& state, const std::string &name) const
	{
		std::map<std::string, ModelComponentRep::CacheInfo*>::const_iterator it;
		it = _rep->_namedCacheVariableInfo.find(name);

		if(it != _rep->_namedCacheVariableInfo.end()) {
			SimTK::CacheEntryIndex ceIndex = it->second->index;
			state.markCacheValueRealized(getIndexOfSubsystemForAllocations(), ceIndex);
		}
		else{
			std::stringstream msg;
			msg << "ModelComponent::markCacheVariableValid: ERR- name not found.\n "
				<< "for component '"<< getName() << "' of type " << getType();
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}

	/**
     * Enable the component performing a costly evaluation method to monitor the validity
	 * of the cache variable value and to use this flag to  force a re-evaluation only
	 * when necessary.
	 *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     */
	bool isCacheVariableValid(const SimTK::State& state, const std::string &name) const
	{
		std::map<std::string, ModelComponentRep::CacheInfo*>::const_iterator it;
		it = _rep->_namedCacheVariableInfo.find(name);

		if(it != _rep->_namedCacheVariableInfo.end()) {
			SimTK::CacheEntryIndex ceIndex = it->second->index;
			return state.isCacheValueRealized(getIndexOfSubsystemForAllocations(), ceIndex);
		}
		else{
			std::stringstream msg;
			msg << "ModelComponent::isCacheVariableValid: ERR- name not found.\n "
				<< "for component '"<< getName() << "' of type " << getType();
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}

	/**
     *  Set cache variable value allocated by this ModelComponent by name.
	 *  All cache entries are lazily evaluated (on a need basis) so a set
	 *  also marks the cache as valid.
     *
     * @param state  the State for which to set the value
     * @param name   the name of the state variable
     * @return value  the variable's cache value to update
     */
	template<typename T> void setCacheVariable(const SimTK::State& state, const std::string &name, T &value) const
	{
		std::map<std::string, ModelComponentRep::CacheInfo*>::const_iterator it;
		it = _rep->_namedCacheVariableInfo.find(name);

		if(it != _rep->_namedCacheVariableInfo.end()) {
			SimTK::CacheEntryIndex ceIndex = it->second->index;
			SimTK::Value<T>::downcast(state.updCacheEntry( getIndexOfSubsystemForAllocations(), ceIndex)).upd() = value;
			state.markCacheValueRealized(getIndexOfSubsystemForAllocations(), ceIndex);
		}
		else{
			std::stringstream msg;
			msg << "ModelComponent::setCacheVariable: ERR- name not found.\n "
				<< "for component '"<< getName() << "' of type " << getType();
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}	
	}

	/**
     * Include another ModelComponent as a Subcomponent of this ModelComponent.
	 * ModelComponent methods (e.g. createSystem(), initState(), ...) are therefore
     * invoked on Subcomponents when called on the parent. Realization is also
	 * performed automatically on subcomponents. This ModelComponent does not
	 * take ownership of the subcomponent.
	 */
	void includeAsSubComponent(ModelComponent *aComponent);

protected:
    /**
     * This is called after the Model has been constructed from an XML file.
	 * Set the Model this object is part of and then so do required initialization,
	 * such as looking up references to other objects in the Model (which might not
	 * have existed yet when this object was instantiated).
	 * Override this method as necessary but always call the parent class setup to
	 * ensure all members (belonging to parent and child) are initialized.
     */
	virtual void setup(Model& model);

    /**
     * This is called when a SimTK System is being created for the Model.  It must be implemented
     * to add appropriate elements to the System corresponding to this object. Methods for
	 * adding modeling options, state variables and their derivatives, discrete variables, as well
	 * must be called with createSystem() only.
     *
     * @param system   the System being created
     */
    virtual void createSystem(SimTK::MultibodySystem& system) const;

    /**
     * This is called after a SimTK System and State have been created for the Model.  It must
     * be implementd to set initial values of state variables.
     *
     * @param state    the State to initialize
     */
	virtual void initState(SimTK::State& state) const {
		for(unsigned int i=0; i < _subComponents.size(); i++)
			_subComponents[i]->initState(state);
	};

    /**
     * Set all default values for this object to match those in a specified State.  It must be
     * implemented/overriden to set any default values defined by each subclass.
     *
     * @param state    the State from which to take values that should become the defaults for this object
     */
	virtual void setDefaultsFromState(const SimTK::State& state) {
		for(unsigned int i=0; i < _subComponents.size(); i++)
			_subComponents[i]->setDefaultsFromState(state);
	};

	/** Set the index for the subsystem to which new variables (modeling option, state, cache)
	    corresponding to the ModelComponent will be added. This method is intended to override
		use of the defaultSubsystem by concrete subclasses. */
	void setIndexOfSubsystemForAllocations(SimTK::SubsystemIndex subsysIndex);
	const SimTK::SubsystemIndex getIndexOfSubsystemForAllocations() const;

	/**
	 * Helper methods for implementing concrete ModelComponents. These methods
	 * can only be called inside of createSystem() and are useful for creating
	 * the underlying SimTK::System level variables that are used for computing
	 * values of interest.
	 **/

	/** Add a modeling option (integer flag) used by this ModelComponent,
	    with each option identified by a label.  Modeling options enable the model
		component to be configured differently in order to represent different operating modes.
		For example, in may be useful to ignore excitation-activation dynamics of a muscle, or
		to apply a known muscle force, which could be modeling options. */
	void addModelingOption(const Array<std::string> &optionFlagNames);

	/** Add continuous system state variables belonging to this ModelComponent.
	    The number of states is defined by the number of stateVariableNames, and each
		state variable is assigned the corresponding name.*/
	void addStateVariables(const Array<std::string> &stateVariableNames);

	/** If state variables are continuous, then computeStateVariableDerivatives() must be
		implemented so that the evolution of the state variable can be determined.
		Otherwise, state variables are intepreted as discrete variables and must be
		supplied to the model component.
		Override to return a Vector of the same size as the number of state variables
		defined, otherwise returns empty (no derivatives are defined). */
	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State &s) const
	{ return SimTK::Vector(0); };

	/** Add system discrete variables belonging to this ModelComponent.
	    The number of discrete variables is specified  by the number of discreteVariableNames,
		and each allocated discrete variable is assigned the corresponding name.
		Each variable is allocated as SimTK::Real and is dependent on the same Stage.
		Use repeated calls with a different Stage for variables that are dependent on
		other Stages */
	void addDiscreteVariables(const Array<std::string> &discreteVariableNames,
							  const SimTK::Stage &dependentOnStage);

	/** Add a state cache variable belonging to this ModelComponent.
	    Cache variables are typically computed from the state and provide
		convenience and/or efficiency by holding on to them in memory (cache) and
		using the cached value subsequently rather than recomputing.  Once the state
		changes, the cache variable values become invalid and have to be
		recomputed based on the current state.
		@param cacheVariableName is the name of the variable
		@param variablePrototype is a prototype of the object (type) to be cached
		@param the lowest computational stage at which the cache variable is valid 	*/
	template<typename T> void addCacheVariable(	const std::string &cacheVariableName,
					const T& variablePrototype, const SimTK::Stage &lowestValidStage)
	{
		if(int(_rep->_indexOfSubsystemForAllocations) == SimTK::InvalidIndex)
			throw Exception("ModelComponent: Cache Variables can only be added during createSystem().");

		ModelComponentRep::CacheInfo *ci = new ModelComponentRep::CacheInfo;
		// Note, cache index is invalid until the actual allocation occurs during realizeTopology
		ci->prototype = new SimTK::Value<T>(variablePrototype);
		ci->lowestValidStage = lowestValidStage;
		_rep->_namedCacheVariableInfo[cacheVariableName] = ci;
	}

	/** Get the index of a ModelComponent's conintuous state variable in the Subsystem for
		allocations. This method is intended for derived ModelComponents that may need direct
		access to its underlying Subsystem.*/
	const SimTK::ZIndex getZIndex(const std::string &name) const;

	/** Get the index of a ModelComponent's discrete variable in the Subsystem for allocations.
		This method is intended for derived ModelComponents that may need direct access
		to its underlying Subsystem.*/
	const SimTK::DiscreteVariableIndex getDiscreteVariableIndex(const std::string &name) const;

	/** Get the index of a ModelComponent's cache variable in the Subsystem for allocations.
		This method is intended for derived ModelComponents that may need direct access
		to its underlying Subsystem.*/
	const SimTK::CacheEntryIndex getCacheVariableIndex(const std::string &name) const;

	const ModelComponentRep *getRep() {return _rep; };
	ModelComponentRep *updRep() {return _rep; };

private:

	// maintain pointes to subcomponents so we can invoke them automatically
	SimTK::Array_<ModelComponent *> _subComponents;

	// Representative implementation of ModelComponent for SimTK::System level allocations
	ModelComponentRep* _rep;

	friend class ModelComponentRep;
//=============================================================================
};	// END of class ModelComponent
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ModelComponent_h__

