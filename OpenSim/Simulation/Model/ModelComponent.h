#ifndef OPENSIM_MODEL_COMPONENT_H_
#define OPENSIM_MODEL_COMPONENT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ModelComponent.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib, Michael Sherman                         *
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

/** @file
 * This defines the abstract ModelComponent class, which is used to add 
 * computational components to the underlying SimTK::System (MultibodySystem). 
 * It specifies the interface that components must satisfy in order to be part 
 * of the system and provides a series of helper methods for adding variables 
 * (state, discrete, cache, ...) to the underlying system. As such, 
 * ModelComponent handles all of the bookkeeping for variable indices and  
 * provides convenience access via variable names.
 *
 * All OpenSim components: Bodies, Joints, Coordinates, Constraints, Forces,   
 * Actuators, Controllers, etc. and Model itself, are ModelComponents. Each  
 * component is composed of one or more underlying Simbody multibody system 
 * elements (MobilizedBody, Constraint, Force, or Measure) which are part of 
 * a SimTK::Subsystem and by default this is the System's DefaultSubsystem.
 * The SimTK::Subsystem is where new variables are allocated.
 */


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Geometry.h>
#include "Simbody.h"

namespace OpenSim {

class Model;
//==============================================================================
//                            MODEL COMPONENT
//==============================================================================
/**
 * This defines the abstract ModelComponent class, which is used to specify 
 * components of a musculoskeletal model and the elements they add to the 
 * underlying computational SimTK::System (MultibodySystem). A ModelComponent is 
 * an OpenSim::Component and therefore has the capabilities to add nececessry
 * system resources to the System and to manage access to those resources (@see
 * Component)
 *
 * Bodies, Joints, Coordinates, Constraints, Forces, Actuators, Controllers, 
 * and even Model itself, are ModelComponents. Each component is "connected" to
 * a model and an underlying SimTK::Subsystem, which by default is the 
 * System's DefaultSubsystem.
 *
 * The primary responsibility of a ModelComponent is to add its computational 
 * representation(s) of physical musculoskeletal structures to the underlying
 * SimTK::System by implementing extendAddToSystem().
 *
 * Additional methods provide support for adding modeling options, state and
 * cache variables (@see Component).
 *
 * Public methods enable access to component variables via their names.
 *
 * @author Ajay Seth, Michael Sherman
 */

class OSIMSIMULATION_API ModelComponent : public Component {
OpenSim_DECLARE_ABSTRACT_OBJECT(ModelComponent, Component);
//==============================================================================
// PROPERTIES
//==============================================================================
public:
    OpenSim_DECLARE_LIST_PROPERTY(geometry, Geometry,
        "List of Geometry that is attached to this Model Component.");
//==============================================================================
// METHODS
//==============================================================================
public:
    /** Default constructor **/
    ModelComponent();
    /** Construct ModelComponent from an XML file. **/
    ModelComponent(const std::string& aFileName, 
                   bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;
    /** Construct ModelComponent from a specific node in an XML document. **/
    explicit ModelComponent(SimTK::Xml::Element& aNode);

    // compiler default copy costructor and assignment operator

    /** Destructor is virtual to allow concrete model component cleanup. **/
    virtual ~ModelComponent() {}

    /** Connect this ModelComponent to its aggregate- a  Model */
    void connectToModel(Model& model);

    /**
     * Get a const reference to the Model this component is part of.
     */
    const Model& getModel() const;
    /**
     * Get a modifiable reference to the Model this component is part of.
     */
    Model& updModel();
    /**
     * returns the number of published Geometry objects 
     * maintained by the ModelComponent.
     */
    int getNumGeometry() const {
        return getProperty_geometry().size();
    }
    /**
     * Copy the passed in Geometry and add the copy to the visualization of 
     * this ModelComponent as subcomponent.
     */
    void addGeometry(OpenSim::Geometry& aGeometry);

    void extendFinalizeFromProperties();
protected:
template <class T> friend class ModelComponentSet;

    /** @name           ModelComponent Basic Interface
    The interface ensures that deserialization, resolution of inter-connections,
    and handling of dependencies are performed systematically and prior to 
    system creation, followed by allocation of necessary System resources. These 
    methods are virtual and may be implemented by subclasses of 
    ModelComponents. 
    
    @note Every implementation of virtual method xxx(args) must begin
    with the line "Super::xxx(args);" to ensure that the parent class methods
    execute before the child class method, starting with ModelComponent::xxx()
    and going down. 
    
    The base class implementations here do two things: (1) take care of any
    needs of the %ModelComponent base class itself, and then (2) ensure that the 
    corresponding calls are made to any subcomponents that have been specified 
    by derived %ModelComponent objects, via calls to the 
    includeAsSubComponent() method. So assuming that your concrete 
    %ModelComponent and all intermediate classes from which it derives properly 
    follow the requirement of calling the Super class method first, the order of
    operations enforced here for a call to a single method will be
      -# %ModelComponent base class computations
      -# calls to that same method for \e all subcomponents
      -# calls to that same method for intermediate %ModelComponent-derived 
         objects' computations, in order down from %ModelComponent, and
      -# finally a call to that method for the bottom-level concrete class. 

    You should consider this ordering when designing a %ModelComponent. In 
    particular the fact that all your subcomponents will be invoked before you
    are may be surprising. **/ 
    //@{

    /** Perform any necessary initializations required to connect the 
    component into the Model, and check for error conditions. extendConnectToModel() 
    is invoked on all components to complete construction of a Model, prior to
    creating a Simbody System to represent it computationally. It may also be
    invoked at times just for its error-checking side effects.
    
    If you override this method, be sure to invoke the base class method first, 
    using code like this:
    @code
    void MyComponent::extendConnectToModel(Model& model) {
        Super::extendConnectToModel(model); // invoke parent class method
        // ... your code goes here
    }
    @endcode

    Note that this method is expected to check for modeling errors and should
    throw an OpenSim::Exception if there is something wrong. For example, if
    your model component references another object by name, you should verify
    that it exists in the supplied Model, which is not guaranteed since 
    components may be independently instantiated or constructed from XML files.

    @param[in,out]  model   The Model currently being constructed to which this
                            %ModelComponent should be connected. **/
    virtual void extendConnectToModel(Model& model) {};

    /** Perform initialization on the passed in Geometry before adding it to
     the list of subcomponents and Properties of this ModelComponent. For 
     example, Frames can set the frame name on the Geometry to themselves. 

     @param[in,out]  geometry to be added to current ModelComponent   
     **/
    virtual void extendAddGeometry(OpenSim::Geometry& geometry) {};

    // End of Model Component Basic Interface (protected virtuals).
    //@} 

    /** @name     ModelComponent System Creation and Access Methods
     * These methods support implementing concrete ModelComponents. Add methods
     * can only be called inside of extendAddToSystem() and are useful for creating
     * the underlying SimTK::System level variables that are used for computing
     * values of interest.
     * @warning Accessors for System indices are intended for component internal use only.
     **/

    //@{

    // End of System Creation and Access Methods.
    //@} 


private:
    /** Satisfy the general Component interface, but this is not part of the
    * ModelComponent interface. ModelComponent::extendConnect() ensures that
    * extendConnectToModel() on ModelComponent subcomponents are invoked. **/
    void extendConnect(Component& root) override final;

    const SimTK::DefaultSystemSubsystem& getDefaultSubsystem() const;
    const SimTK::DefaultSystemSubsystem& updDefaultSubsystem();

    // Clear out all the data fields in the base class. There should be one
    // line here for each data member below.
    void setNull() {
        _model = NULL;
    }
protected:
    /** The model this component belongs to. */
    // TODO: this should be private; all components should use getModel()
    // and updModel() to get access. This is just a reference; don't delete!
    SimTK::ReferencePtr<Model> _model;

//==============================================================================
};  // END of class ModelComponent
//==============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MODEL_COMPONENT_H_

