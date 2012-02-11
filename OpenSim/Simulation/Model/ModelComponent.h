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

/** @file
 * This defines the abstract ModelComponent class, which is used to add computational
 * components to the underlying SimTK::System (MultibodySystem). It specifies
 * the interface that components must satisfy in order to be part of the system
 * and provides a series of helper methods for adding variables (state, discrete,
 * cache, ...) to the underlying system. As such, ModelComponent handles all of
 * the bookkeeping for variable indices and provides conveniece access via 
 * variable names.
 *
 * All components, Bodies, Joints, Coordinates, Constraints, Forces, Controllers, .. 
 * and even Model itself, are ModelComponents. Each component is "connected" to a
 * SimTK::Subsystem and by default this is the System's DefaultSubsystem.
 */


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Common/Object.h"
#include "SimTKsimbody.h"

namespace OpenSim {

class Model;
class ModelComponent;

/** @cond **/ // hide from Doxygen
#ifndef SWIG

//=============================================================================
// Begin class ModelComponentRep
//=============================================================================
class ModelComponentRep 
{
protected:
	ModelComponent& _modelComponent;

	// Structure to hold modeling option information
	struct ModelingOptionInfo {
		int maxOptionValue;
		SimTK::DiscreteVariableIndex index;
		ModelingOptionInfo(): maxOptionValue(0), index(SimTK::InvalidIndex) {};
    };

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

	// Map names of modeling options for the ModelComponent to their underlying
    // SimTK indices.
	std::map<std::string, ModelingOptionInfo*>      _namedModelingOptionInfo;
	// Map names of continuous state variables of the ModelComponent to their 
    // underlying SimTK indices.
	std::map<std::string, SimTK::ZIndex>            _namedStateVariableIndices;
	// Map names of discrete variables of the ModelComponent to their underlying
    // SimTK indices.
	std::map<std::string, DiscreteVariableInfo*>    _namedDiscreteVariableInfo;
	// Map names of cache entries of the ModelComponent to their individual 
    // cache information.
	std::map<std::string, CacheInfo*>               _namedCacheVariableInfo;

	explicit ModelComponentRep(ModelComponent &mc);
	~ModelComponentRep();

public:

	/** Get the number of continuous states that the ModelComponent added to the underlying
	    computational system. It does not include the number of states already built-in by
		the SimTK::System level component. Should query it and add to this to obtain the total
		number of states managed by this ModelComponent */
	int getNumStateVariablesAddedByModelComponent() const {return _namedStateVariableIndices.size();}
	Array<std::string> getStateVariablesNamesAddedByModelComponent() const;

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
/** @endcond **/

//=============================================================================
// Begin class ModelComponent
//=============================================================================
/**
 * This defines the abstract ModelComponent class, which is used to add computational
 * components to the underlying SimTK::System (MultibodySystem). It specifies
 * the interface that components must satisfy in order to be part of the system
 * and provides a series of helper methods for adding variables (state, discrete,
 * cache, ...) to the underlying system. As such, ModelComponent handles all of
 * the bookkeeping of system indices and provides convenience access to variable 
 * values via their names as strings. 
 *
 * The MultibodySystem and its State are defined by Simbody (ref ...). Briefly,
 * a System can be thought of the equations that define the mathematical behavior
 * of a model. The State is a collection of all the variables that uniquely
 * define the unknowns in the system equations. These would be joint coordinates,
 * their speeds, accelerations as well other variables that govern system dynamics 
 * (e.g. muscle activation and fiber-length variables that dictate muscle force). 
 * These variables are called continuous state variables in Simbody, but  are more
 * simply referred to as <em>StateVariables</em> in OpenSim.  ModelComponent provides 
 * services to define and access its StateVariables and specify their dynamics
 * (derivatives with respect to time) that are automatically and simultaneously
 * integrated with the MultibodySystem dynamics.
 *
 * There are other types of "State" variables such as a flag (or option) that 
 * enables a component to be disabled or for a muscle force to be overridden and 
 * and these are identified as <em>ModelingOptions</em> since they fundamentally 
 * change the modeled dynamics of the component. ModelComponent provides services
 * that enable developers of components to define additional ModelingOptions.
 *
 * Often a component requires input from an outside source (precompuetd data from 
 * a file, another program, or interaction from a user) in which case these
 * variables do not have dynamics (differential eqns.) known to the component, but 
 * are necessary to describe the dynamical "state" of the system. An example, is
 * a throttle component (a "controller" that provides an actuator (e.g. a motor) with
 * a control signal (a voltage or current)) which it gets direct input from the user 
 * (via a joystick, key press, etc..). The throttle controls the motor torque output 
 * and therefore the behavior of the model. The input by the user to the throttle
 * the motor (the controls) is necesary to specify the model dynamics at any instant
 * and therefore are considered part of the State in Simbody as Discrete State Variables.  
 * In OpenSim they are simplify referred to as \e DiscreteVariables. The ModelComponent 
 * provides services to enable developers of components to define and access its 
 * DiscreteVariables.
 *
 * Fast and efficient simulations also require compututationally expensive calculations
 * to be performed only when necessary. Often the result of a expensive calculation can
 * be reused many times over, while the variables it is dependent on remain fixed. The
 * concept of holding onto these values is called caching and the variables that hold
 * these values are call <em>CacheVariables</em>. It is important to note, that cache 
 * variables are not state variables. Cache variables can always be recomputed excactly
 * from the State. OpenSim uses the Simbody infrastructure to manage cache variables and
 * their validity. ModelComponent provides a simplified interface to define and 
 * access CacheVariables.
 *
 * Many modeling and simulation codes put the onus on users and component creators to
 * manage the validity of cache variables, which is likely to lead to undetectable 
 * errors where cache values are stale (calculated based on past state variable values).
 * Simbody, on the other hand, provides a more strict infrastructure to make it easy to
 * exploit the efficiencies of caching while reducing the risks of validity errors. 
 * To do this, Simbody employs the concept of computational stages. To "realize" a model's
 * system a particular stage is to perform all the computations necessary to evaluate the 
 * cached quantities up to and including the stage specified. Simbody utilizes 
 * nine realization stages (<tt>SimTK::Stage::</tt>)
 *
 * -# \c Topology       finalize System with "slots" for most variables (above)
 * -# \c %Model         specify modeling choices
 * -# \c Instance       specify modifiable model parameters
 * -# \c Time           compute time dependent quantities
 * -# \c Position       compute position dependent quantities	
 * -# \c Velocity       compute velocity dependent quantities
 * -# \c Dynamics       compute system applied forces and dependent quantities	
 * -# \c Acceleration   compute system accelerations and all other derivatives
 * -# \c Report         compute quantities for reporting/output
 *  
 * The ModelComponent interface is automatically invoked by the System and its realizations.
 * Component users and most developers need not concern themselves with
 * \c Topology, \c %Model or \c Instance stages. That interaction is managed by ModelComponent 
 * when component creators implement createSystem() and use the services provded ModelComponent.
 * Component creators do need to determine and specify stage dependencies for Discrete  
 * and CacheVariables that they add to their components. For example, the throttle 
 * controller reads its value from user input and it is valid for all calculations as
 * long as time does not change. If the simulation (via numerical integration) steps
 * forward (or backward for a trial step) and updates the state, the control from
 * a previous state (time) should be invalid and an error generated for trying to access
 * the DiscreteVariable for the control value. To do this one specifies the "dependsOn" stage
 * (e.g. <tt>SimTK::Stage::Time</tt>) for a DiscreteVariable when the variable is added to
 * the ModelComponent. If the control is fixed (not changing with time) then the lowest
 * valid stage could just be Instance.
 *
 * Similar principles apply to CacheVariables, which requires a "lowestValid" stage to 
 * be specified when a CacheVariable is added to the component. In this case, the cache
 * variable augments the State (unlike a discrete state variable, which is a part
 * of the State) for the convenience of not recomputing a value unless necesary.
 * Accessing the cache variable at a stage lower than the lowestValid specified will trigger
 * an exception. It is up to the component to update the value of the cache variable.
 * ModelComponent provides methods to check if the cache is valid, update its value and 
 * mark as valid. 

 * All components: Bodies, Joints, Coordinates, Constraints, Forces, Controllers, .. 
 * and even Model itself, are ModelComponents. Each component is "connected" to a
 * SimTK::Subsystem and by default this is the System's DefaultSubsystem.
 *
 * The primary responsibility of a ModelComponent is to add its computational 
 * representation(s) to the underlying SimTK::System by implementing
 * createSystem().
 *
 * Additional methods provide support for adding modeling options, state and
 * cache variables.
 *
 * Public methods enable access to component variables via their names.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API ModelComponent : public Object
{
protected:
	/** The model this component belongs to. */
	Model* _model;


//=============================================================================
// METHODS
//=============================================================================
public:
    template <class T> friend class ModelComponentSet;
	// Default constructor
	ModelComponent();
	// Construct ModelComponent from a an XML file
	ModelComponent(const std::string& aFileName, bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;
	// Construct ModelComponent from a specific node in an XML document
	ModelComponent(SimTK::Xml::Element& aNode);
	// Construct ModelComponent with its contents copied from another ModelComponent
	ModelComponent(const ModelComponent& copy);
	// Destructor
    virtual ~ModelComponent();

	/** Assignment operator to copy contents of an existing component */
	ModelComponent& operator=(const ModelComponent &aModelComponent);

    /**
     * Get a const reference to the Model this component is part of.
     */
    const Model& getModel() const;
    /**
     * Get a modifiable reference to the Model this component is part of.
     */
    Model& updModel();

	/**
	 * In case the ModelComponent has a visual representation (VisualObject), override this method  
	 * to update it. This is typically done by recomputing anchor points and positions based 
	 * on transforms obtained from current state.
	 */
	virtual void updateDisplayer(const SimTK::State& s) {};

	/**
     * Get the number of "Continuous" state variables maintained by the ModelComponent
	 * and its specified subcomponents
	 */
    virtual int getNumStateVariables() const;

	/**
     * Get the names of "continuous" state variables maintained by the ModelComponent
	 * and its subcomponents
	 */
    virtual Array<std::string> getStateVariableNames() const;

   /**
     * Get the System Index of a state variable allocated by this ModelComponent.  
     * Returns an InvalidIndex if no state variable with the name provided is found.
     * @param stateVariableName   the name of the state variable 
     */
	virtual SimTK::SystemYIndex getStateVariableSystemIndex(const std::string &stateVariableName) const;

   /** @name ModelComponent State Access methods
	 * Get and set modeling option, state, discrete and/or cache variables in the State
	 */ 
	//@{
    
	/**
     * Get a ModelingOption flag for this ModelComponent by name.
     * The flag is an integer corresponding to the index of modelingOptionNames used 
	 * add the modeling option to the component. @see addModelingOption
	 *
     * @param state  the State in which to set the modeling option
	 * @param name   the name (string) of the modeling option of interest
     * @return flag  integer value for modeling option
     */
	virtual int getModelingOption(const SimTK::State& state, const std::string &name) const;

	/**
     * Set the value of a ModelingOption flag for this ModelComponent.
	 * if the integer value exceeds the number of option names used to
	 * define the options, an exception is thrown.
     *
     * @param state  the State in which to set the flag
	 * @param name   the name (string) of the modeling option of interest
	 * @param flag   the desired flag (integer) value specifying the modeling option
     */
	virtual void setModelingOption(SimTK::State& state, const std::string &name, int flag) const;

    /**
     * Get the value of a state variable allocated by this ModelComponent.
     *
     * @param state   the State for which to get the value
     * @param name    the name (string) of the state variable of interest
     */
	virtual double getStateVariable(const SimTK::State& state, const std::string &name) const;

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
     * @param state   the State for which to set the value
     * @param name    the name of the state variable
     * @return value  the discrete variable value
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
     * @return T	 const reference to the cache variable's value
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
     * @return value modifiable reference to the cache variable's value
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
     * Enables the to monitor the validity of the cache variable value using the
	 * returned flag. For components performing a costly evaluation, use this method to 
	 * force a re-evaluation cache variable value only when necessary (returns false).
	 *
     * @param state  the State in which the cache value resides
     * @param name   the name of the cache variable
	 * @return bool  whether the cache variable value is valid or not
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
     * @param value  the variable's cache new value to set
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
	// End of Model Component State Accessors.
    //@} 

protected:

	/** @name ModelComponent Interface
	 * The interface ensures that deserialization, resolution of inter-connections and dependicies,
	 * are performed systematically and prior to system creation. createSystem() is responsible
	 * These methods are virtual and must be implemented by subclasses of ModelComponents.
	*/ 
	//@{

    /**
     * setup() is automatically called on all components prior to system creation. 
	 * Set the Model this component is part of and then perform any necessary initialization,
	 * such as looking up references to other objects in the Model (to verify that they exist,
	 * which is not guaranteed when a component is instantiated or constructed from XML).
	 * Override this method as necessary but always call the parent class setup to
	 * ensure all members (including those defined by the parent class) are initialized.
	 * @param model   the model this component is a part of
     */
	virtual void setup(Model& model);

    /**
     * createSystem() is called when the SimTK::System is being created for the Model.  It must be 
     * implemented in order to add appropriate SimTK elements to the System corresponding to this component. 
	 * Helper methods for adding modeling option, state variables and their derivatives, discrete variables,
	 * etc., are available and can be called within createSystem() only.
	 * @see addModelingOption()
	 * @see addStateVariables()
	 * @see addDiscreteVariables()
	 * @see addCacheVariable()
     *
     * @param system   the System being created
     */
    virtual void createSystem(SimTK::MultibodySystem& system) const;

    /**
     * This is called after a SimTK::System and State have been created for the Model.  It is
     * implementd to set initial values of state variables from defaults (typical) or by  
     * any calculation that is not dependent on the state (i.e. function of property values).
     * @param state    the State to initialize
     */
	virtual void initState(SimTK::State& state) const {
		for(unsigned int i=0; i < _subComponents.size(); i++)
			_subComponents[i]->initState(state);
	};

    /**
     * Assign new default values for this component to match those in a specified State.  It must be
     * implemented/overriden to set default values to states defined by each subclass. @note Defaults 
	 * are usually properties of the component, and these properties can be updated to match an
	 * existing state. Thus, state variable values can persist as part of the model component and be
	 * serialized as a property.
     *
     * @param state    the State from which the default values are obtained (calculated) for this component
     */
	virtual void setDefaultsFromState(const SimTK::State& state) {
		for(unsigned int i=0; i < _subComponents.size(); i++)
			_subComponents[i]->setDefaultsFromState(state);
	};

	// End of Model Component Interface (virtuals).
    //@} 

	/** @name ModelComponent System Creation and Access Methods
	 * These methods support implementing concrete ModelComponents. Add methods
	 * can only be called inside of createSystem() and are useful for creating
	 * the underlying SimTK::System level variables that are used for computing
	 * values of interest.
	 * @warning Accessors for System indices are intended for component internal use only.
	 **/

	//@{

	/** Set the index for the subsystem to which new variables (modeling option, state, cache)
	    corresponding to the ModelComponent will be added. This method is intended to override
		use of the defaultSubsystem by concrete subclasses. */
	void setIndexOfSubsystemForAllocations(SimTK::SubsystemIndex subsysIndex);
	const SimTK::SubsystemIndex getIndexOfSubsystemForAllocations() const;

	/**
     * Include another ModelComponent as a Subcomponent of this ModelComponent.
	 * ModelComponent methods (e.g. createSystem(), initState(), ...) are therefore
     * invoked on Subcomponents when called on the parent. Realization is also
	 * performed automatically on subcomponents. This ModelComponent does not
	 * take ownership of the subcomponent.
	 */
	void includeAsSubComponent(ModelComponent *aComponent);

	/** Add a modeling option (integer flag) used by this ModelComponent.
	    Each modeling option identified by its own optionName.  Modeling options enable the model
		component to be configured differently in order to represent different operating modes.
		For example, if two modes of operation are necessary (mode off and mode on) then specify
		optionName, "mode" with maxFlagValue = 1. Subsequent gets will return 0 or 1 and set will
		only accept 0 and 1 as acceptable values. Changing the value of a model option, invalidates
		Instance and above realization Stages.*/
	void addModelingOption(const std::string &optionName, int maxFlagValue) const;

	/** Add continuous system state variables belonging to this ModelComponent.
	    The number of states is defined by the number of stateVariableNames, and each
		state variable is assigned the corresponding name.*/
	void addStateVariables(const Array<std::string> &stateVariableNames) const;

	/** If state variables are continuous, then computeStateVariableDerivatives() must be
		implemented so that the evolution of the state variable can be determined.
		Otherwise, state variables are intepreted as discrete variables and must be
		supplied to the model component.
		Override to return a Vector of the same size as the number of state variables defined
		and in order of getStateVariableNames(). Default returns empty (no derivatives are defined). */
	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State &s) const
	{ return SimTK::Vector(0); };

	/** Add system discrete variables belonging to this ModelComponent.
	    The number of discrete variables is specified  by the number of discreteVariableNames,
		and each allocated discrete variable is assigned the corresponding name.
		Each variable is allocated as SimTK::Real and is dependent on the same Stage.
		Use repeated calls with a different Stage for variables that are dependent on
		other Stages */
	void addDiscreteVariables(const Array<std::string> &discreteVariableNames,
							  const SimTK::Stage &dependentOnStage) const;

	/** Add a state cache variable belonging to this ModelComponent.
	    Cache variables are typically computed from the state and provide
		convenience and/or efficiency by holding on to them in memory (cache) and
		using the cached value subsequently rather than recomputing.  Once the state
		changes, the cache variable values automatically become invalid and have to be
		recomputed based on the current state. The cache is also invalidated if the
		realization stage falls below the lowest valid stage. For example, a body's momementum,
		which is dependent on position and velocity states, should have velocity as its lowestValidStage.
		That is if velocity (or any stage below) has to be realized then the cache is invalidated.
		But, if dynamics (andy anything above) is realized (the is velocity remains valid) then the cache
		variable is valid and does not have to be recomputed.

		@param cacheVariableName	is the name of the variable
		@param variablePrototype	is a prototype of the object (type) to be cached
		@param lowestValidStage		the lowest computational stage at which the cache variable is valid */
	template<typename T> void addCacheVariable(	const std::string &cacheVariableName,
					const T& variablePrototype, const SimTK::Stage &lowestValidStage) const
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

	// End of System Creation and Access Methods.
    //@} 

	/** @cond **/ // hide from Doxygen
	const ModelComponentRep *getRep() {return _rep; }
	ModelComponentRep *updRep() {return _rep; }
	/** @endcond **/

private:

	// maintain pointes to subcomponents so we can invoke them automatically
	SimTK::Array_<ModelComponent *> _subComponents;

	// Representative implementation of ModelComponent for SimTK::System level allocations
	ModelComponentRep* _rep;

	/** @cond **/ // hide from Doxygen
	friend class ModelComponentRep;
	/** @endcond **/
//=============================================================================
};	// END of class ModelComponent
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


/** @cond **/ // hide from Doxygen
/**
	The default underlying SimTK component for an OpenSim::ModelComponent is a
	SimTK::Measure.  In particular, it is a ModelComponentMeasure defined here.
*/

namespace SimTK {

template <class T>
class ModelComponentMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(ModelComponentMeasure, SimTK::Measure_<T>);

	ModelComponentMeasure(SimTK::Subsystem& sub, const OpenSim::ModelComponentRep& mcRep)
    :   Measure_<T>(sub, new Implementation(mcRep), AbstractMeasure::SetHandle()) {}

    SimTK_MEASURE_HANDLE_POSTSCRIPT(ModelComponentMeasure, SimTK::Measure_<T>);
};

template <class T>
class ModelComponentMeasure<T>::Implementation : public SimTK::Measure_<T>::Implementation
{
public:
	Implementation(const OpenSim::ModelComponentRep& mcRep)
    :   SimTK::Measure_<T>::Implementation(1), _modelCompRep(mcRep) {}

    // Implementations of virtual methods.
    Implementation* cloneVirtual() const {return new Implementation(*this);}

	virtual void realizeMeasureTopologyVirtual(State& s) const {_modelCompRep.realizeTopology(s);}
    virtual int getNumTimeDerivativesVirtual() const {return 0;}
	Stage getDependsOnStageVirtual(int order) const {   return Stage::Acceleration; }
	   
	void calcCachedValueVirtual(const State& s, int derivOrder, T& value) const
    {
        SimTK_ASSERT1_ALWAYS(derivOrder==0,
            "ModelComponentMeasure::Implementation::calcCachedValueVirtual():"
            " derivOrder %d seen but only 0 allowed.", derivOrder);
        value = 0;
    }

	virtual void realizeMeasureAccelerationVirtual(const State& s) const {_modelCompRep.realizeAcceleration(s);}


private:
    bool presumeValidAtDependsOnStage;
	const OpenSim::ModelComponentRep& _modelCompRep;
	Array_<CacheEntryIndex> derivIx;
};

} // end namespace SimTK
/** @endcond **/

#endif // __ModelComponent_h__

